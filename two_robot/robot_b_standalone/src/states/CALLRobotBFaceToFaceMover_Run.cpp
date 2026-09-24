#include "CALLRobotBFaceToFaceMover_Run.h"
#include "../CALLRobotBFaceToFaceMover.h"

#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>

void CALLRobotBFaceToFaceMover_Run::configure(
    const mc_rtc::Configuration & config)
{
  if(config.has("logEvery")) { config("logEvery", logEvery_); }
  logEvery_ = std::max<uint64_t>(1, logEvery_);
}

void CALLRobotBFaceToFaceMover_Run::updateMeasuredMotion(
    CALLRobotBFaceToFaceMover & ctl)
{
  const sva::PTransformd currentPose = ctl.actualToolPose();
  const Eigen::Vector3d currentPosition = currentPose.translation();
  const Eigen::Matrix3d currentRotation = currentPose.rotation();

  if(!havePreviousActualPose_)
  {
    previousActualPosition_ = currentPosition;
    previousActualRotation_ = currentRotation;
    filteredActualVelocity_.setZero();
    filteredActualAngularSpeed_ = 0.0;
    havePreviousActualPose_ = true;
    return;
  }

  const double dt = std::max(1e-6, ctl.controlDt());
  const Eigen::Vector3d rawLinear =
      (currentPosition - previousActualPosition_) / dt;
  const Eigen::Matrix3d deltaRotation =
      currentRotation * previousActualRotation_.transpose();
  const double rawAngularSpeed =
      CALLRobotBFaceToFaceMover::rotationAngle(deltaRotation) / dt;
  const double alpha = ctl.velocityFilterAlpha();

  filteredActualVelocity_ =
      (1.0 - alpha) * filteredActualVelocity_ + alpha * rawLinear;
  filteredActualAngularSpeed_ =
      (1.0 - alpha) * filteredActualAngularSpeed_
      + alpha * rawAngularSpeed;

  previousActualPosition_ = currentPosition;
  previousActualRotation_ = currentRotation;
}

sva::PTransformd CALLRobotBFaceToFaceMover_Run::interpolatedPrepositionPose(
    double s) const
{
  Eigen::Quaterniond qTarget = prepositionTargetQuaternion_;
  if(prepositionStartQuaternion_.dot(qTarget) < 0.0)
  {
    qTarget.coeffs() *= -1.0;
  }
  Eigen::Quaterniond q = prepositionStartQuaternion_.slerp(s, qTarget);
  q.normalize();

  const Eigen::Vector3d p =
      prepositionStartPose_.translation()
      + s * (startPose_.translation() - prepositionStartPose_.translation());
  return sva::PTransformd(q.toRotationMatrix(), p);
}

void CALLRobotBFaceToFaceMover_Run::start(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CALLRobotBFaceToFaceMover &>(ctl_);

  iter_ = 0;
  prepositionElapsed_ = 0.0;
  prepositionDuration_ = 0.0;
  prepositionSettleElapsed_ = 0.0;
  prepositionStableTime_ = 0.0;
  elapsed_ = 0.0;
  settleElapsed_ = 0.0;
  terminalStableTime_ = 0.0;
  maximumObservedTrackingError_ = 0.0;
  maximumPrepositionPositionError_ = 0.0;
  havePreviousActualPose_ = false;
  filteredActualVelocity_.setZero();
  filteredActualAngularSpeed_ = 0.0;

  ctl.clearTrigger();
  ctl.activateToolTask();
  initialHoldPose_ = ctl.actualToolPose();
  holdPose_ = initialHoldPose_;
  ctl.commandToolPose(holdPose_);
  updateMeasuredMotion(ctl);
  if(!ctl.motionEnabled())
  {
    phase_ = Phase::DisabledHold;
    mc_rtc::log::warning(
        "[RobotB DISABLED HOLD] motionEnabled=false. Holding the current tool pose; no preposition or scenario motion will execute.");
    return;
  }

  std::string reason;
  if(!ctl.configuredTrajectorySafe(reason))
  {
    enterFailure(ctl, "unsafe or incomplete scenario configuration: " + reason);
    return;
  }

  prepositionStartPose_ = ctl.actualToolPose();
  startPose_ = sva::PTransformd(
      prepositionStartPose_.rotation(), ctl.scenarioStartPose().translation());
  prepositionStartQuaternion_ =
      Eigen::Quaterniond(prepositionStartPose_.rotation());
  prepositionTargetQuaternion_ = Eigen::Quaterniond(startPose_.rotation());
  prepositionStartQuaternion_.normalize();
  prepositionTargetQuaternion_.normalize();

  const double distance =
      (startPose_.translation() - prepositionStartPose_.translation()).norm();
  const double angle = CALLRobotBFaceToFaceMover::rotationAngle(
      startPose_.rotation() * prepositionStartPose_.rotation().transpose());

  // Quintic smooth-step has maximum ds/du = 1.875.
  const double translationDuration =
      1.875 * distance / ctl.prepositionMaximumLinearSpeed();
  const double rotationDuration =
      1.875 * angle / ctl.prepositionMaximumAngularSpeed();
  prepositionDuration_ = std::max(
      ctl.prepositionMinimumDuration(),
      std::max(translationDuration, rotationDuration));

  if(prepositionDuration_ > ctl.prepositionTimeout())
  {
    enterFailure(ctl, "required preposition duration exceeds configured timeout");
    return;
  }

  phase_ = Phase::Prepositioning;
  const Eigen::Vector3d p0 = prepositionStartPose_.translation();
  const Eigen::Vector3d p1 = startPose_.translation();
  mc_rtc::log::success(
      "[RobotB PREPOSITION START] scenario={} current=[{:.4f},{:.4f},{:.4f}] "
      "predefinedStart=[{:.4f},{:.4f},{:.4f}] distance={:.4f}m "
      "orientationDistance={:.4f}rad duration={:.3f}s. "
      "Robot B moves to the predefined scenario start before READY.",
      ctl.scenarioName(), p0.x(), p0.y(), p0.z(), p1.x(), p1.y(), p1.z(),
      distance, angle, prepositionDuration_);
}

void CALLRobotBFaceToFaceMover_Run::enterFailure(
    CALLRobotBFaceToFaceMover & ctl,
    const std::string & reason)
{
  phase_ = Phase::Failed;
  holdPose_ = ctl.actualToolPose();
  ctl.commandToolPose(holdPose_);
  ctl.clearTrigger();
  mc_rtc::log::error(
      "[RobotB FAILURE] {}. Holding current physical tool pose; "
      "no retry and no fallback motion.", reason);
}

bool CALLRobotBFaceToFaceMover_Run::run(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CALLRobotBFaceToFaceMover &>(ctl_);
  ++iter_;
  updateMeasuredMotion(ctl);

  if(phase_ == Phase::DisabledHold)
  {
    ctl.commandToolPose(holdPose_);
    if(ctl.triggerPresent())
    {
      ctl.clearTrigger();
      mc_rtc::log::error(
          "[RobotB] trigger ignored because motionEnabled=false; disabled hold remains active");
    }
    return false;
  }

  if(phase_ == Phase::Prepositioning)
  {
    prepositionElapsed_ += ctl.controlDt();
    const double u = std::min(1.0, prepositionElapsed_ / prepositionDuration_);
    const double s = CALLRobotBFaceToFaceMover::smoothStep(u);
    const double dsdt =
        CALLRobotBFaceToFaceMover::smoothStepVelocity(u) / prepositionDuration_;
    const double d2sdt2 =
        CALLRobotBFaceToFaceMover::smoothStepAcceleration(u)
        / (prepositionDuration_ * prepositionDuration_);

    targetPose_ = interpolatedPrepositionPose(s);
    const Eigen::Vector3d delta =
        startPose_.translation() - prepositionStartPose_.translation();
    ctl.commandToolPoseWithWorldMotion(
        targetPose_, delta * dsdt, delta * d2sdt2);

    const sva::PTransformd actualPose = ctl.actualToolPose();
    const double positionError =
        (targetPose_.translation() - actualPose.translation()).norm();
    const double orientationError =
        CALLRobotBFaceToFaceMover::rotationAngle(
            targetPose_.rotation() * actualPose.rotation().transpose());
    maximumPrepositionPositionError_ =
        std::max(maximumPrepositionPositionError_, positionError);

    if(!ctl.targetInsideWorkspace(targetPose_.translation()))
    {
      enterFailure(ctl, "preposition target left configured workspace");
      return false;
    }

    if(iter_ % logEvery_ == 1)
    {
      const Eigen::Vector3d a = actualPose.translation();
      const Eigen::Vector3d q = targetPose_.translation();
      mc_rtc::log::info(
          "[RobotB PREPOSITION] t={:.3f}/{:.3f}s actual=[{:.3f},{:.3f},{:.3f}] "
          "target=[{:.3f},{:.3f},{:.3f}] positionError={:.4f}m "
          "orientationError={:.4f}rad speed={:.4f}m/s angularSpeed={:.4f}rad/s",
          prepositionElapsed_, prepositionDuration_,
          a.x(), a.y(), a.z(), q.x(), q.y(), q.z(),
          positionError, orientationError, filteredActualVelocity_.norm(),
          filteredActualAngularSpeed_);
    }

    if(prepositionElapsed_ >= prepositionDuration_)
    {
      phase_ = Phase::PrepositionSettling;
      prepositionSettleElapsed_ = 0.0;
      prepositionStableTime_ = 0.0;
      holdPose_ = startPose_;
      ctl.commandToolPose(holdPose_);
      mc_rtc::log::warning(
          "[RobotB START-GATE] predefined scenario start reached by reference; "
          "measured admission begins target=[{:.4f},{:.4f},{:.4f}] "
          "tolerances=[pos:{:.4f}m,ori:{:.4f}rad,v:{:.4f}m/s,w:{:.4f}rad/s] "
          "dwell={:.3f}s",
          holdPose_.translation().x(), holdPose_.translation().y(),
          holdPose_.translation().z(), ctl.prepositionPositionTolerance(),
          ctl.prepositionOrientationTolerance(),
          ctl.prepositionLinearVelocityTolerance(),
          ctl.prepositionAngularVelocityTolerance(),
          ctl.prepositionStableDuration());
    }
    return false;
  }

  if(phase_ == Phase::PrepositionSettling)
  {
    prepositionSettleElapsed_ += ctl.controlDt();
    ctl.commandToolPose(holdPose_);

    const sva::PTransformd actualPose = ctl.actualToolPose();
    const double positionError =
        (holdPose_.translation() - actualPose.translation()).norm();
    const double orientationError =
        CALLRobotBFaceToFaceMover::rotationAngle(
            holdPose_.rotation() * actualPose.rotation().transpose());
    const double linearSpeed = filteredActualVelocity_.norm();
    const double angularSpeed = filteredActualAngularSpeed_;
    const bool stable =
        positionError <= ctl.prepositionPositionTolerance()
        && orientationError <= ctl.prepositionOrientationTolerance()
        && linearSpeed <= ctl.prepositionLinearVelocityTolerance()
        && angularSpeed <= ctl.prepositionAngularVelocityTolerance();

    prepositionStableTime_ = stable
        ? prepositionStableTime_ + ctl.controlDt()
        : 0.0;

    if(iter_ % logEvery_ == 1)
    {
      mc_rtc::log::info(
          "[RobotB START-GATE] t={:.3f}s positionError={:.4f}m "
          "orientationError={:.4f}rad speed={:.4f}m/s angularSpeed={:.4f}rad/s "
          "stable={} dwell={:.3f}/{:.3f}s",
          prepositionSettleElapsed_, positionError, orientationError,
          linearSpeed, angularSpeed, stable, prepositionStableTime_,
          ctl.prepositionStableDuration());
    }

    if(prepositionStableTime_ >= ctl.prepositionStableDuration())
    {
      phase_ = Phase::Ready;
      ctl.clearTrigger();
      ctl.commandToolPose(holdPose_);
      const Eigen::Vector3d a = actualPose.translation();
      mc_rtc::log::success(
          "[RobotB READY] scenario={} at predefined start actual=[{:.4f},{:.4f},{:.4f}] "
          "positionError={:.4f}m orientationError={:.4f}rad speed={:.4f}m/s "
          "angularSpeed={:.4f}rad/s stableDwell={:.3f}s "
          "maxPrepositionTrackingError={:.4f}m trigger={}. "
          "Waiting for one manual scenario-motion trigger.",
          ctl.scenarioName(), a.x(), a.y(), a.z(), positionError,
          orientationError, linearSpeed, angularSpeed,
          prepositionStableTime_, maximumPrepositionPositionError_,
          ctl.triggerFile());
      return false;
    }

    if(prepositionSettleElapsed_ >= ctl.prepositionTimeout())
    {
      enterFailure(ctl, "scenario start pose/orientation/velocity gate timed out");
      return false;
    }
    return false;
  }

  if(phase_ == Phase::Ready)
  {
    ctl.commandToolPose(startPose_);
    if(!ctl.triggerPresent()) { return false; }
    ctl.clearTrigger();

    if(!ctl.motionEnabled())
    {
      enterFailure(ctl, "motion was disabled after READY");
      return false;
    }

    std::string reason;
    if(!ctl.configuredTrajectorySafe(reason))
    {
      enterFailure(ctl, "unsafe configured trajectory at trigger: " + reason);
      return false;
    }

    targetPose_ = startPose_;
    elapsed_ = 0.0;
    settleElapsed_ = 0.0;
    terminalStableTime_ = 0.0;
    maximumObservedTrackingError_ = 0.0;

    const Eigen::Vector3d vB = ctl.robotBVelocity();
    const double effectiveTime = ctl.constantVelocityDuration()
        + 0.5 * ctl.decelerationDuration();
    const Eigen::Vector3d finalPosition =
        startPose_.translation() + vB * effectiveTime;

    phase_ = Phase::Executing;
    mc_rtc::log::success(
        "[RobotB START] scenario={} predefinedStart=[{:.4f},{:.4f},{:.4f}] "
        "final=[{:.4f},{:.4f},{:.4f}] constant={:.3f}s decel={:.3f}s "
        "prepositioned=true feedforward=true terminalAdmission=true "
        "manualSynchronization=true noRobotCommunication=true",
        ctl.scenarioName(), startPose_.translation().x(),
        startPose_.translation().y(), startPose_.translation().z(),
        finalPosition.x(), finalPosition.y(), finalPosition.z(),
        ctl.constantVelocityDuration(), ctl.decelerationDuration());
    return false;
  }

  if(phase_ == Phase::Executing)
  {
    elapsed_ += ctl.controlDt();
    const double tc = ctl.constantVelocityDuration();
    const double td = ctl.decelerationDuration();
    const Eigen::Vector3d vB = ctl.robotBVelocity();

    double effectiveTime = 0.0;
    double velocityScale = 0.0;
    double accelerationScale = 0.0;
    const char * segment = "constant";

    if(elapsed_ <= tc)
    {
      effectiveTime = elapsed_;
      velocityScale = 1.0;
    }
    else if(td > 1e-9 && elapsed_ < tc + td)
    {
      segment = "deceleration";
      const double u = (elapsed_ - tc) / td;
      effectiveTime = tc + td
          * CALLRobotBFaceToFaceMover::decelerationIntegral(u);
      velocityScale =
          CALLRobotBFaceToFaceMover::decelerationVelocityScale(u);
      accelerationScale =
          CALLRobotBFaceToFaceMover::decelerationAccelerationScale(u) / td;
    }
    else
    {
      segment = "terminal";
      effectiveTime = tc + 0.5 * td;
      velocityScale = 0.0;
      accelerationScale = 0.0;
    }

    targetPose_ = sva::PTransformd(
        startPose_.rotation(), startPose_.translation() + vB * effectiveTime);

    if(!ctl.targetInsideWorkspace(targetPose_.translation()))
    {
      enterFailure(ctl, "runtime target left configured workspace");
      return false;
    }

    ctl.commandToolPoseWithWorldMotion(
        targetPose_, vB * velocityScale, vB * accelerationScale);

    const Eigen::Vector3d actual = ctl.actualToolPose().translation();
    const double positionError =
        (targetPose_.translation() - actual).norm();
    maximumObservedTrackingError_ =
        std::max(maximumObservedTrackingError_, positionError);

    if(elapsed_ >= ctl.trackingErrorGrace()
       && positionError > ctl.maximumTrackingError())
    {
      enterFailure(ctl, "tracking error exceeded configured bound");
      return false;
    }

    if(iter_ % logEvery_ == 1)
    {
      const Eigen::Vector3d q = targetPose_.translation();
      mc_rtc::log::info(
          "[RobotB EXECUTE] t={:.3f}s segment={} "
          "actual=[{:.3f},{:.3f},{:.3f}] "
          "target=[{:.3f},{:.3f},{:.3f}] "
          "vRef={:.4f}m/s vActual={:.4f}m/s positionError={:.4f}m",
          elapsed_, segment,
          actual.x(), actual.y(), actual.z(),
          q.x(), q.y(), q.z(),
          (vB * velocityScale).norm(), filteredActualVelocity_.norm(),
          positionError);
    }

    if(elapsed_ >= tc + td)
    {
      phase_ = Phase::Settling;
      settleElapsed_ = 0.0;
      terminalStableTime_ = 0.0;
      holdPose_ = targetPose_;
      ctl.commandToolPose(holdPose_);
      mc_rtc::log::warning(
          "[RobotB TERMINAL] reference motion finished; measured admission started "
          "target=[{:.3f},{:.3f},{:.3f}] tolerances=[pos:{:.4f}m,vel:{:.4f}m/s] "
          "dwell={:.3f}s timeout={:.3f}s",
          holdPose_.translation().x(), holdPose_.translation().y(),
          holdPose_.translation().z(), ctl.terminalPositionTolerance(),
          ctl.terminalVelocityTolerance(), ctl.terminalStableDuration(),
          ctl.terminalTimeout());
    }
    return false;
  }

  if(phase_ == Phase::Settling)
  {
    settleElapsed_ += ctl.controlDt();
    ctl.commandToolPose(holdPose_);

    const Eigen::Vector3d actual = ctl.actualToolPose().translation();
    const double positionError =
        (holdPose_.translation() - actual).norm();
    const double speed = filteredActualVelocity_.norm();
    const bool stable =
        positionError <= ctl.terminalPositionTolerance()
        && speed <= ctl.terminalVelocityTolerance();

    terminalStableTime_ = stable
        ? terminalStableTime_ + ctl.controlDt()
        : 0.0;

    if(iter_ % logEvery_ == 1)
    {
      mc_rtc::log::info(
          "[RobotB TERMINAL] t={:.3f}s actual=[{:.3f},{:.3f},{:.3f}] "
          "target=[{:.3f},{:.3f},{:.3f}] positionError={:.4f}m "
          "speed={:.4f}m/s stable={} dwell={:.3f}/{:.3f}s",
          settleElapsed_, actual.x(), actual.y(), actual.z(),
          holdPose_.translation().x(), holdPose_.translation().y(),
          holdPose_.translation().z(), positionError, speed, stable,
          terminalStableTime_, ctl.terminalStableDuration());
    }

    if(terminalStableTime_ >= ctl.terminalStableDuration())
    {
      phase_ = Phase::Holding;
      ctl.commandToolPose(holdPose_);
      mc_rtc::log::success(
          "[RobotB HOLD] terminal gate passed finalActual=[{:.4f},{:.4f},{:.4f}] "
          "finalTarget=[{:.4f},{:.4f},{:.4f}] finalError={:.4f}m "
          "finalSpeed={:.4f}m/s stableDwell={:.3f}s "
          "maxTrackingError={:.4f}m. Robot B keeps holding; "
          "it does not release or retreat.",
          actual.x(), actual.y(), actual.z(),
          holdPose_.translation().x(), holdPose_.translation().y(),
          holdPose_.translation().z(), positionError, speed,
          terminalStableTime_, maximumObservedTrackingError_);
      return false;
    }

    if(settleElapsed_ >= ctl.terminalTimeout())
    {
      enterFailure(ctl, "terminal pose/velocity gate timed out");
      return false;
    }
    return false;
  }

  ctl.commandToolPose(holdPose_);
  return false;
}

void CALLRobotBFaceToFaceMover_Run::teardown(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CALLRobotBFaceToFaceMover &>(ctl_);
  ctl.clearToolTaskReferenceMotion();
  mc_rtc::log::info("[RobotB] controller state teardown");
}

EXPORT_SINGLE_STATE("CALLRobotBFaceToFaceMover_Run",
                    CALLRobotBFaceToFaceMover_Run)
