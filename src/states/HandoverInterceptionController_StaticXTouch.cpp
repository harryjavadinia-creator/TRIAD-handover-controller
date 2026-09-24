#include "HandoverInterceptionController_StaticXTouch.h"
#include "../HandoverInterceptionController.h"

#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

Eigen::Vector3d HandoverInterceptionController_StaticXTouch::readVector3(
    const mc_rtc::Configuration & cfg,
    const std::string & key,
    const Eigen::Vector3d & fallback)
{
  if(!cfg.has(key)) { return fallback; }
  std::vector<double> values;
  cfg(key, values);
  if(values.size() != 3)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[StaticXTouch] {} must contain exactly three values", key);
  }
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

double HandoverInterceptionController_StaticXTouch::smoothStep(double u)
{
  u = std::max(0.0, std::min(1.0, u));
  return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
}

double HandoverInterceptionController_StaticXTouch::smoothStepVelocity(double u)
{
  u = std::max(0.0, std::min(1.0, u));
  return 30.0 * u * u * (1.0 - u) * (1.0 - u);
}

void HandoverInterceptionController_StaticXTouch::configure(
    const mc_rtc::Configuration & config)
{
  if(config.has("giverRobot")) { config("giverRobot", giverRobot_); }
  if(config.has("giverToolFrame")) { config("giverToolFrame", giverToolFrame_); }
  objectTipOffsetWorld_ = readVector3(
      config, "objectTipOffsetWorld", objectTipOffsetWorld_);
  if(config.has("contactGap")) { config("contactGap", contactGap_); }
  if(config.has("giverDuration")) { config("giverDuration", giverDuration_); }
  if(config.has("receiverDuration")) { config("receiverDuration", receiverDuration_); }
  if(config.has("settleDwell")) { config("settleDwell", settleDwell_); }
  if(config.has("settleTimeout")) { config("settleTimeout", settleTimeout_); }
  if(config.has("positionTolerance")) { config("positionTolerance", positionTolerance_); }
  if(config.has("velocityTolerance")) { config("velocityTolerance", velocityTolerance_); }
  if(config.has("maximumGiverTravel")) { config("maximumGiverTravel", maximumGiverTravel_); }
  if(config.has("maximumReceiverTravel")) { config("maximumReceiverTravel", maximumReceiverTravel_); }
  if(config.has("giverStiffness")) { config("giverStiffness", giverStiffness_); }
  if(config.has("giverWeight")) { config("giverWeight", giverWeight_); }
  if(config.has("receiverStiffness")) { config("receiverStiffness", receiverStiffness_); }
  if(config.has("receiverWeight")) { config("receiverWeight", receiverWeight_); }
  if(config.has("logEvery")) { config("logEvery", logEvery_); }

  contactGap_ = std::max(0.005, contactGap_);
  giverDuration_ = std::max(2.0, giverDuration_);
  receiverDuration_ = std::max(2.0, receiverDuration_);
  settleDwell_ = std::max(0.1, settleDwell_);
  settleTimeout_ = std::max(settleDwell_ + 0.5, settleTimeout_);
  positionTolerance_ = std::max(0.002, positionTolerance_);
  velocityTolerance_ = std::max(0.002, velocityTolerance_);
  maximumGiverTravel_ = std::max(0.05, maximumGiverTravel_);
  maximumReceiverTravel_ = std::max(0.05, maximumReceiverTravel_);
  giverStiffness_ = std::max(1.0, giverStiffness_);
  giverWeight_ = std::max(100.0, giverWeight_);
  receiverStiffness_ = std::max(1.0, receiverStiffness_);
  receiverWeight_ = std::max(100.0, receiverWeight_);
  logEvery_ = std::max(1, logEvery_);
}

void HandoverInterceptionController_StaticXTouch::start(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  iter_ = 0;
  phaseTime_ = 0.0;
  stableTime_ = 0.0;
  havePrevious_ = false;
  giverSpeed_ = 0.0;
  receiverSpeed_ = 0.0;

  if(!ctl.robots().hasRobot(giverRobot_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[StaticXTouch] giver robot not loaded: {}", giverRobot_);
  }
  auto & giver = ctl.robots().robot(giverRobot_);
  if(!giver.hasFrame(giverToolFrame_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[StaticXTouch] giver frame not found: {}/{}",
        giverRobot_, giverToolFrame_);
  }

  giverTask_ = std::make_shared<mc_tasks::TransformTask>(
      giver.frame(giverToolFrame_), giverStiffness_, giverWeight_);
  giverStart_ = giver.frame(giverToolFrame_).position();
  giverTask_->target(giverStart_);
  ctl.solver().addTask(giverTask_);
  giverTaskActive_ = true;

  ctl.detachObject();
  ctl.invalidateSelectedCandidate();
  ctl.setGripperClosureAuthorized(false);
  ctl.setGripperJointPriority(false);
  ctl.commandGripper(0.0);
  ctl.activateToolTask();
  ctl.setToolTaskGains(receiverStiffness_, receiverWeight_);
  receiverStartMouth_ = ctl.actualMouthPose();
  ctl.commandMouthTarget(receiverStartMouth_);

  const Eigen::Vector3d giverTipStart =
      giverStart_.translation() + objectTipOffsetWorld_;
  if(giverTipStart.x() <= receiverStartMouth_.translation().x() + 2.0 * contactGap_)
  {
    fail(ctl, "model ordering invalid: Robot-B object tip is not in front of Robot-A mouth along +X");
    return;
  }

  // Static X rendezvous is derived from the measured startup poses. This avoids
  // hard-coding a guessed world contact coordinate. Robot B first places the
  // assumed bottle tip at the X-midpoint and aligns it to Robot A's current Y/Z.
  touchPointWorld_ = receiverStartMouth_.translation();
  touchPointWorld_.x() = 0.5 * (
      receiverStartMouth_.translation().x() + giverTipStart.x());

  const Eigen::Vector3d giverTargetPosition =
      touchPointWorld_ - objectTipOffsetWorld_;
  const Eigen::Vector3d receiverTargetPosition =
      touchPointWorld_ - Eigen::Vector3d(contactGap_, 0.0, 0.0);

  giverTarget_ = sva::PTransformd(giverStart_.rotation(), giverTargetPosition);
  receiverTargetMouth_ = sva::PTransformd(
      receiverStartMouth_.rotation(), receiverTargetPosition);
  holdGiver_ = giverTarget_;
  holdReceiverMouth_ = receiverTargetMouth_;

  const double giverTravel =
      (giverTarget_.translation() - giverStart_.translation()).norm();
  const double receiverTravel =
      (receiverTargetMouth_.translation()
       - receiverStartMouth_.translation()).norm();
  if(giverTravel > maximumGiverTravel_)
  {
    fail(ctl, "Robot-B preposition exceeds configured travel bound");
    return;
  }
  if(receiverTravel > maximumReceiverTravel_)
  {
    fail(ctl, "Robot-A X approach exceeds configured travel bound");
    return;
  }
  if(touchPointWorld_.z() < 0.15)
  {
    fail(ctl, "derived touch point is too close to the table/ground");
    return;
  }

  phase_ = Phase::MoveGiver;
  mc_rtc::log::warning(
      "[StaticXTouch ARMED] SIMPLE MODE ONLY: Robot B preposition -> hold; Robot A open-mouth X approach -> {:.3f}m gap -> hold. No planning, grasp, transfer or retreat.",
      contactGap_);
  mc_rtc::log::warning(
      "[StaticXTouch GEOMETRY] B base model yaw=180deg; assumed object-tip offset world=[{:.3f},{:.3f},{:.3f}]m",
      objectTipOffsetWorld_.x(), objectTipOffsetWorld_.y(), objectTipOffsetWorld_.z());
  mc_rtc::log::success(
      "[StaticXTouch TARGETS] touch=[{:.3f},{:.3f},{:.3f}] Btool=[{:.3f},{:.3f},{:.3f}] Amouth=[{:.3f},{:.3f},{:.3f}] travelB={:.3f}m travelA={:.3f}m",
      touchPointWorld_.x(), touchPointWorld_.y(), touchPointWorld_.z(),
      giverTargetPosition.x(), giverTargetPosition.y(), giverTargetPosition.z(),
      receiverTargetPosition.x(), receiverTargetPosition.y(), receiverTargetPosition.z(),
      giverTravel, receiverTravel);
}

void HandoverInterceptionController_StaticXTouch::commandGiver(
    const sva::PTransformd & target,
    const Eigen::Vector3d & velocityWorld)
{
  if(!giverTask_) { return; }
  giverTask_->target(target);
  (void)velocityWorld;
  // Keep this temporary static test deliberately simple: the task target is
  // quintic, while body-frame feed-forward references remain zero.
  giverTask_->refVelB(sva::MotionVecd(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
  giverTask_->refAccel(sva::MotionVecd(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
}

void HandoverInterceptionController_StaticXTouch::updateMeasuredSpeeds(
    HandoverInterceptionController & ctl)
{
  const Eigen::Vector3d pB = ctl.robots().robot(giverRobot_)
      .frame(giverToolFrame_).position().translation();
  const Eigen::Vector3d pA = ctl.actualMouthPose().translation();
  if(!havePrevious_)
  {
    previousGiverPosition_ = pB;
    previousReceiverPosition_ = pA;
    havePrevious_ = true;
    return;
  }
  const double dt = std::max(1e-6, ctl.controlDt());
  giverSpeed_ = (pB - previousGiverPosition_).norm() / dt;
  receiverSpeed_ = (pA - previousReceiverPosition_).norm() / dt;
  previousGiverPosition_ = pB;
  previousReceiverPosition_ = pA;
}

void HandoverInterceptionController_StaticXTouch::fail(
    HandoverInterceptionController & ctl,
    const std::string & reason)
{
  phase_ = Phase::Failed;
  if(giverTask_)
  {
    holdGiver_ = ctl.robots().robot(giverRobot_)
        .frame(giverToolFrame_).position();
    commandGiver(holdGiver_);
  }
  holdReceiverMouth_ = ctl.actualMouthPose();
  ctl.commandMouthTarget(holdReceiverMouth_);
  mc_rtc::log::error(
      "[StaticXTouch FAILURE] {}. Both robots hold measured poses; no retry.",
      reason);
}

bool HandoverInterceptionController_StaticXTouch::run(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ++iter_;
  updateMeasuredSpeeds(ctl);
  ctl.setGripperClosureAuthorized(false);
  ctl.commandGripper(0.0);

  if(phase_ == Phase::Failed)
  {
    commandGiver(holdGiver_);
    ctl.commandMouthTarget(holdReceiverMouth_);
    return false;
  }
  if(phase_ == Phase::Hold)
  {
    commandGiver(holdGiver_);
    ctl.commandMouthTarget(holdReceiverMouth_);
    if(iter_ % static_cast<uint64_t>(logEvery_) == 1)
    {
      const double gap =
          (touchPointWorld_ - ctl.actualMouthPose().translation()).norm();
      mc_rtc::log::info(
          "[StaticXTouch HOLD] mouth-to-touch={:.4f}m giverSpeed={:.4f}m/s receiverSpeed={:.4f}m/s",
          gap, giverSpeed_, receiverSpeed_);
    }
    return false;
  }

  const double dt = std::max(1e-6, ctl.controlDt());
  phaseTime_ += dt;

  if(phase_ == Phase::MoveGiver)
  {
    ctl.commandMouthTarget(receiverStartMouth_);
    const double u = std::min(1.0, phaseTime_ / giverDuration_);
    const double s = smoothStep(u);
    const double dsdt = smoothStepVelocity(u) / giverDuration_;
    const Eigen::Vector3d delta = giverTarget_.translation() - giverStart_.translation();
    const sva::PTransformd target(
        giverStart_.rotation(), giverStart_.translation() + s * delta);
    commandGiver(target, delta * dsdt);
    if(u >= 1.0)
    {
      phase_ = Phase::SettleGiver;
      phaseTime_ = 0.0;
      stableTime_ = 0.0;
      commandGiver(giverTarget_);
      mc_rtc::log::warning("[StaticXTouch B-GATE] Robot B reference reached; measured settle gate active");
    }
    return false;
  }

  if(phase_ == Phase::SettleGiver)
  {
    commandGiver(giverTarget_);
    ctl.commandMouthTarget(receiverStartMouth_);
    const double error = giverTask_ ? giverTask_->eval().norm() : 1e9;
    const bool stable = error <= positionTolerance_
        && giverSpeed_ <= velocityTolerance_;
    stableTime_ = stable ? stableTime_ + dt : 0.0;
    if(stableTime_ >= settleDwell_)
    {
      phase_ = Phase::MoveReceiver;
      phaseTime_ = 0.0;
      stableTime_ = 0.0;
      mc_rtc::log::success(
          "[StaticXTouch B-READY] error={:.4f}m speed={:.4f}m/s; Robot A starts one direct X approach",
          error, giverSpeed_);
    }
    else if(phaseTime_ >= settleTimeout_)
    {
      fail(ctl, "Robot-B measured settle gate timed out");
    }
    return false;
  }

  if(phase_ == Phase::MoveReceiver)
  {
    commandGiver(giverTarget_);
    const double u = std::min(1.0, phaseTime_ / receiverDuration_);
    const double s = smoothStep(u);
    const Eigen::Vector3d delta = receiverTargetMouth_.translation()
        - receiverStartMouth_.translation();
    const sva::PTransformd target(
        receiverStartMouth_.rotation(),
        receiverStartMouth_.translation() + s * delta);
    ctl.commandMouthTarget(target);
    if(u >= 1.0)
    {
      phase_ = Phase::SettleReceiver;
      phaseTime_ = 0.0;
      stableTime_ = 0.0;
      ctl.commandMouthTarget(receiverTargetMouth_);
      mc_rtc::log::warning("[StaticXTouch A-GATE] Robot A reference reached; measured settle gate active");
    }
    return false;
  }

  if(phase_ == Phase::SettleReceiver)
  {
    commandGiver(giverTarget_);
    ctl.commandMouthTarget(receiverTargetMouth_);
    const double error =
        (receiverTargetMouth_.translation()
         - ctl.actualMouthPose().translation()).norm();
    const bool stable = error <= positionTolerance_
        && receiverSpeed_ <= velocityTolerance_;
    stableTime_ = stable ? stableTime_ + dt : 0.0;
    if(stableTime_ >= settleDwell_)
    {
      phase_ = Phase::Hold;
      holdGiver_ = giverTarget_;
      holdReceiverMouth_ = receiverTargetMouth_;
      mc_rtc::log::success(
          "[StaticXTouch COMPLETE-HOLD] simple static X near-touch admitted error={:.4f}m speed={:.4f}m/s configuredGap={:.3f}m; gripper remains open",
          error, receiverSpeed_, contactGap_);
    }
    else if(phaseTime_ >= settleTimeout_)
    {
      fail(ctl, "Robot-A measured settle gate timed out");
    }
    return false;
  }

  return false;
}

void HandoverInterceptionController_StaticXTouch::teardown(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  if(giverTask_ && giverTaskActive_)
  {
    ctl.solver().removeTask(giverTask_);
    giverTaskActive_ = false;
  }
}

EXPORT_SINGLE_STATE("HandoverInterceptionController_StaticXTouch",
                    HandoverInterceptionController_StaticXTouch)
