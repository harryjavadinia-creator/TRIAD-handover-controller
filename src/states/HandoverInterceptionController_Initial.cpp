#include "HandoverInterceptionController_Initial.h"
#include "../HandoverInterceptionController.h"

#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>

namespace
{
double clampScalar(double x, double lo, double hi)
{
  return std::max(lo, std::min(hi, x));
}
}

void HandoverInterceptionController_Initial::configure(
    const mc_rtc::Configuration & config)
{
  if(config.has("jointTol")) { config("jointTol", jointTol_); }
  if(config.has("jointVelTol")) { config("jointVelTol", jointVelTol_); }
  if(config.has("stableDwell")) { config("stableDwell", stableDwell_); }
  if(config.has("maxJointSpeed")) { config("maxJointSpeed", maxJointSpeed_); }
  if(config.has("maxJointAcceleration"))
  {
    config("maxJointAcceleration", maxJointAcceleration_);
  }
  if(config.has("maxIter")) { config("maxIter", maxIter_); }

  if(config.has("hardwareGripperCommissioning"))
  {
    const auto commissioning = config("hardwareGripperCommissioning");
    if(commissioning.has("enabled"))
    {
      commissioning("enabled", hardwareCommissioningEnabled_);
    }
    if(commissioning.has("targetPercent"))
    {
      commissioning("targetPercent", commissioningTargetPercent_);
    }
    if(commissioning.has("openTolerancePercent"))
    {
      commissioning("openTolerancePercent", commissioningOpenTolerancePercent_);
    }
    if(commissioning.has("targetTolerancePercent"))
    {
      commissioning("targetTolerancePercent", commissioningTargetTolerancePercent_);
    }
    if(commissioning.has("velocityTolerancePercent"))
    {
      commissioning("velocityTolerancePercent", commissioningVelocityTolerancePercent_);
    }
    if(commissioning.has("openDwell"))
    {
      commissioning("openDwell", commissioningOpenDwell_);
    }
    if(commissioning.has("targetDwell"))
    {
      commissioning("targetDwell", commissioningTargetDwell_);
    }
    if(commissioning.has("holdDuration"))
    {
      commissioning("holdDuration", commissioningHoldDuration_);
    }
    if(commissioning.has("armDriftLimit"))
    {
      commissioning("armDriftLimit", commissioningArmDriftLimit_);
    }
    if(commissioning.has("returnOpen"))
    {
      commissioning("returnOpen", commissioningReturnOpen_);
    }
  }

  commissioningTargetPercent_ =
      std::max(0.0, std::min(100.0, commissioningTargetPercent_));
  commissioningOpenTolerancePercent_ =
      std::max(0.05, commissioningOpenTolerancePercent_);
  commissioningTargetTolerancePercent_ =
      std::max(0.05, commissioningTargetTolerancePercent_);
  commissioningVelocityTolerancePercent_ =
      std::max(0.05, commissioningVelocityTolerancePercent_);
  commissioningOpenDwell_ = std::max(0.05, commissioningOpenDwell_);
  commissioningTargetDwell_ = std::max(0.05, commissioningTargetDwell_);
  commissioningHoldDuration_ = std::max(0.0, commissioningHoldDuration_);
  commissioningArmDriftLimit_ =
      std::max(0.001, commissioningArmDriftLimit_);
}

void HandoverInterceptionController_Initial::start(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  iter_ = 0;
  stableTime_ = 0.0;
  qRef_.clear();
  qdotRef_.clear();
  commissioningPhase_ = CommissioningPhase::OpenSettle;
  commissioningPhaseTime_ = 0.0;
  commissioningStableTime_ = 0.0;
  commissioningMaxArmDrift_ = 0.0;
  commissioningCompletionLogged_ = false;
  commissioningFaultLogged_ = false;

  ctl.detachObject();
  ctl.setGripperClosureAuthorized(hardwareCommissioningEnabled_);
  ctl.invalidateSelectedCandidate();
  ctl.deactivateToolTask();

  for(const auto & kv : ctl.readyPosture())
  {
    try
    {
      const int idx = ctl.robot().mb().jointIndexByName(kv.first);
      if(idx >= 0 && static_cast<size_t>(idx) < ctl.robot().mbc().q.size()
         && !ctl.robot().mbc().q[static_cast<size_t>(idx)].empty())
      {
        qRef_[kv.first] = {ctl.robot().mbc().q[static_cast<size_t>(idx)][0]};
        qdotRef_[kv.first] = 0.0;
      }
    }
    catch(const std::exception &)
    {
    }
  }

  ctl.commandReadyArmPosture(qRef_);
  ctl.setGripperJointPriority(hardwareCommissioningEnabled_);
  ctl.commandGripper(0.0);

  if(hardwareCommissioningEnabled_)
  {
    mc_rtc::log::warning(
        "[HardwareGripperCommissioning] ARMED target={:.1f}% returnOpen={} armTarget=frozen_current postureDriftLimit={:.4f}rad; no planning, Cartesian reach, acquisition, transfer or retreat can start",
        commissioningTargetPercent_, commissioningReturnOpen_,
        commissioningArmDriftLimit_);
  }
  else
  {
    mc_rtc::log::warning(
        "[Initial] bounded readiness trajectory: qdotMax={:.2f}rad/s qddotMax={:.2f}rad/s2; gripper open; {}",
        maxJointSpeed_, maxJointAcceleration_, ctl.gripperActuationStatus());
  }
}

void HandoverInterceptionController_Initial::advanceReference(
    HandoverInterceptionController & ctl)
{
  const double dt = std::max(1e-6, ctl.controlDt());
  const double aMax = std::max(1e-6, maxJointAcceleration_);
  const double vMax = std::max(1e-6, maxJointSpeed_);

  for(const auto & goal : ctl.readyPosture())
  {
    auto it = qRef_.find(goal.first);
    if(it == qRef_.end() || it->second.empty() || goal.second.empty()) { continue; }

    double & q = it->second[0];
    double & v = qdotRef_[goal.first];
    const double error = goal.second[0] - q;
    const double brakeSpeed = std::sqrt(std::max(0.0, 2.0 * aMax * std::abs(error)));
    const double vTarget = (error >= 0.0 ? 1.0 : -1.0)
                         * std::min(vMax, brakeSpeed);
    v += clampScalar(vTarget - v, -aMax * dt, aMax * dt);

    double qNext = q + v * dt;
    if((goal.second[0] - q) * (goal.second[0] - qNext) <= 0.0)
    {
      qNext = goal.second[0];
      v = 0.0;
    }
    q = qNext;
  }
  ctl.commandReadyArmPosture(qRef_);
}

bool HandoverInterceptionController_Initial::homePostureReached(
    const HandoverInterceptionController & ctl) const
{
  const auto & robot = ctl.robot();
  double maxAbsErr = 0.0;
  double maxAbsVel = 0.0;

  for(const auto & kv : ctl.readyPosture())
  {
    const int index = robot.mb().jointIndexByName(kv.first);
    if(index < 0 || robot.mbc().q[index].empty()) { return false; }
    maxAbsErr = std::max(maxAbsErr,
                         std::abs(robot.mbc().q[index][0] - kv.second[0]));
    if(!robot.mbc().alpha[index].empty())
    {
      maxAbsVel = std::max(maxAbsVel, std::abs(robot.mbc().alpha[index][0]));
    }
  }

  if(iter_ % 200 == 1)
  {
    mc_rtc::log::info(
        "[Initial] maxJointErr={:.4f} maxJointVel={:.4f} refSpeedMax={:.3f} tol={:.4f} velTol={:.4f}",
        maxAbsErr, maxAbsVel, maxJointSpeed_, jointTol_, jointVelTol_);
  }
  return maxAbsErr < jointTol_ && maxAbsVel < jointVelTol_;
}


double HandoverInterceptionController_Initial::commissioningArmDrift(
    const HandoverInterceptionController & ctl) const
{
  const auto & robot = ctl.robot();
  double maxAbsDrift = 0.0;
  for(const auto & kv : qRef_)
  {
    if(kv.second.empty()) { continue; }
    try
    {
      const int index = robot.mb().jointIndexByName(kv.first);
      if(index < 0 || static_cast<size_t>(index) >= robot.mbc().q.size()
         || robot.mbc().q[static_cast<size_t>(index)].empty())
      {
        return std::numeric_limits<double>::infinity();
      }
      maxAbsDrift = std::max(
          maxAbsDrift,
          std::abs(robot.mbc().q[static_cast<size_t>(index)][0] - kv.second[0]));
    }
    catch(const std::exception &)
    {
      return std::numeric_limits<double>::infinity();
    }
  }
  return maxAbsDrift;
}

bool HandoverInterceptionController_Initial::runHardwareGripperCommissioning(
    HandoverInterceptionController & ctl)
{
  const double dt = std::max(1e-6, ctl.controlDt());
  ctl.deactivateToolTask();
  ctl.commandReadyArmPosture(qRef_);
  ctl.setGripperJointPriority(true);
  ctl.setGripperClosureAuthorized(true);

  const double armDrift = commissioningArmDrift(ctl);
  commissioningMaxArmDrift_ = std::max(commissioningMaxArmDrift_, armDrift);

  if(!std::isfinite(armDrift) || armDrift > commissioningArmDriftLimit_)
  {
    commissioningPhase_ = CommissioningPhase::FaultHold;
    if(!commissioningFaultLogged_)
    {
      commissioningFaultLogged_ = true;
      mc_rtc::log::error(
          "[HardwareGripperCommissioning] FAULT arm drift {:.5f}rad exceeded {:.5f}rad; freezing the physical gripper at its measured aperture and refusing all further commissioning motion",
          armDrift, commissioningArmDriftLimit_);
    }
  }

  if(!ctl.physicalGripperBridgeEnabled()
     || !ctl.physicalGripperCommandEnabled())
  {
    ctl.commandGripper(ctl.measuredGripperClosure());
    if(iter_ % 500 == 1)
    {
      mc_rtc::log::error(
          "[HardwareGripperCommissioning] blocked: physicalBridge.enabled={} commandEnabled={}; both must be true only for this bounded test",
          ctl.physicalGripperBridgeEnabled(),
          ctl.physicalGripperCommandEnabled());
    }
    return false;
  }

  if(!ctl.physicalGripperFeedbackValid())
  {
    ctl.commandGripper(ctl.measuredGripperClosure());
    commissioningStableTime_ = 0.0;
    if(iter_ % 500 == 1)
    {
      mc_rtc::log::error(
          "[HardwareGripperCommissioning] blocked: physical Robotiq feedback invalid; command held at measured aperture");
    }
    return false;
  }

  const double maxPercent = std::max(1e-6, ctl.physicalGripperMaxPercent());
  const double targetPercent =
      std::max(0.0, std::min(maxPercent, commissioningTargetPercent_));
  const double targetClosure = targetPercent / maxPercent;
  const double measuredPercent = ctl.physicalGripperMeasuredPercent();
  const double measuredVelocity =
      ctl.physicalGripperMeasuredVelocityPercent();

  auto stableAt = [&](double target, double tolerance)
  {
    return std::abs(measuredPercent - target) <= tolerance
        && std::abs(measuredVelocity)
               <= commissioningVelocityTolerancePercent_;
  };

  double commandedClosure = ctl.measuredGripperClosure();
  switch(commissioningPhase_)
  {
    case CommissioningPhase::OpenSettle:
      commandedClosure = 0.0;
      if(stableAt(0.0, commissioningOpenTolerancePercent_))
      {
        commissioningStableTime_ += dt;
      }
      else
      {
        commissioningStableTime_ = 0.0;
      }
      if(commissioningStableTime_ >= commissioningOpenDwell_)
      {
        commissioningPhase_ = CommissioningPhase::CloseToTarget;
        commissioningStableTime_ = 0.0;
        commissioningPhaseTime_ = 0.0;
        mc_rtc::log::success(
            "[HardwareGripperCommissioning] open baseline stable measured={:.2f}% velocity={:.2f}%/s; starting bounded target {:.2f}%",
            measuredPercent, measuredVelocity, targetPercent);
      }
      break;

    case CommissioningPhase::CloseToTarget:
      commandedClosure = targetClosure;
      if(stableAt(targetPercent, commissioningTargetTolerancePercent_))
      {
        commissioningStableTime_ += dt;
      }
      else
      {
        commissioningStableTime_ = 0.0;
      }
      if(commissioningStableTime_ >= commissioningTargetDwell_)
      {
        commissioningPhase_ = CommissioningPhase::HoldTarget;
        commissioningStableTime_ = 0.0;
        commissioningPhaseTime_ = 0.0;
        mc_rtc::log::success(
            "[HardwareGripperCommissioning] target stable measured={:.2f}% target={:.2f}% velocity={:.2f}%/s maxArmDrift={:.5f}rad",
            measuredPercent, targetPercent, measuredVelocity,
            commissioningMaxArmDrift_);
      }
      break;

    case CommissioningPhase::HoldTarget:
      commandedClosure = targetClosure;
      commissioningPhaseTime_ += dt;
      if(commissioningPhaseTime_ >= commissioningHoldDuration_)
      {
        commissioningPhase_ = commissioningReturnOpen_
            ? CommissioningPhase::ReturnOpen
            : CommissioningPhase::CompleteHold;
        commissioningStableTime_ = 0.0;
        commissioningPhaseTime_ = 0.0;
        if(commissioningReturnOpen_)
        {
          mc_rtc::log::warning(
              "[HardwareGripperCommissioning] bounded target hold complete; returning to open");
        }
      }
      break;

    case CommissioningPhase::ReturnOpen:
      commandedClosure = 0.0;
      if(stableAt(0.0, commissioningOpenTolerancePercent_))
      {
        commissioningStableTime_ += dt;
      }
      else
      {
        commissioningStableTime_ = 0.0;
      }
      if(commissioningStableTime_ >= commissioningOpenDwell_)
      {
        commissioningPhase_ = CommissioningPhase::CompleteHold;
        commissioningStableTime_ = 0.0;
      }
      break;

    case CommissioningPhase::CompleteHold:
      commandedClosure = commissioningReturnOpen_ ? 0.0 : targetClosure;
      if(!commissioningCompletionLogged_)
      {
        commissioningCompletionLogged_ = true;
        mc_rtc::log::success(
            "[HardwareGripperCommissioning] PASS target={:.2f}% finalMeasured={:.2f}% returnOpen={} maxArmDrift={:.5f}rad; controller remains in Initial and cannot enter handover planning",
            targetPercent, measuredPercent, commissioningReturnOpen_,
            commissioningMaxArmDrift_);
      }
      break;

    case CommissioningPhase::FaultHold:
      commandedClosure = ctl.measuredGripperClosure();
      break;
  }

  ctl.commandGripper(commandedClosure);

  if(iter_ % 200 == 1)
  {
    mc_rtc::log::info(
        "[HardwareGripperCommissioning] phase={} command={:.3f} target={:.2f}% measured={:.2f}% velocity={:.2f}%/s armDrift={:.5f}/{:.5f}rad feedbackValid=true",
        static_cast<int>(commissioningPhase_), commandedClosure, targetPercent,
        measuredPercent, measuredVelocity, armDrift,
        commissioningArmDriftLimit_);
  }
  return false;
}

bool HandoverInterceptionController_Initial::run(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ++iter_;
  if(hardwareCommissioningEnabled_)
  {
    return runHardwareGripperCommissioning(ctl);
  }

  ctl.setGripperClosureAuthorized(false);

  ctl.deactivateToolTask();
  advanceReference(ctl);
  ctl.setGripperJointPriority(false);
  ctl.commandGripper(0.0);

  if(ctl.physicalGripperBridgeEnabled()
     && !ctl.physicalGripperFeedbackValid())
  {
    stableTime_ = 0.0;
    if(iter_ % 500 == 1)
    {
      mc_rtc::log::error(
          "[Initial] hardware gripper feedback invalid; holding before planning and suppressing physical gripper commands");
    }
    return false;
  }

  if(homePostureReached(ctl)) { stableTime_ += ctl.controlDt(); }
  else { stableTime_ = 0.0; }

  if(stableTime_ >= stableDwell_)
  {
    if(ctl.physicalGripperBridgeEnabled()
       && !ctl.physicalGripperCommandEnabled())
    {
      if(iter_ % 1000 == 1)
      {
        mc_rtc::log::success(
            "[Initial] staged hardware hold: ready posture stable and physical gripper feedback valid; commandEnabled=false prevents transition to handover execution");
      }
      return false;
    }
    mc_rtc::log::success(
        "[Initial] bounded ready posture stable and gripper open; proceeding to complete capture planning");
    output("OK");
    return true;
  }

  if(iter_ >= maxIter_)
  {
    mc_rtc::log::error(
        "[Initial] readiness trajectory did not converge within {} cycles",
        maxIter_);
    output("FAIL");
    return true;
  }
  return false;
}

void HandoverInterceptionController_Initial::teardown(
    mc_control::fsm::Controller &)
{
  mc_rtc::log::info("[Initial] teardown");
}

EXPORT_SINGLE_STATE("HandoverInterceptionController_Initial",
                    HandoverInterceptionController_Initial)
