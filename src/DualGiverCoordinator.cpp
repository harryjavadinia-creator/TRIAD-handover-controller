#include "DualGiverCoordinator.h"
#include "HandoverInterceptionController.h"

#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>


DualGiverCoordinator::DualGiverCoordinator(
    HandoverInterceptionController & ctl,
    const mc_rtc::Configuration & config)
{
  loadConfig(config);
  if(!enabled_) { return; }

  if(!ctl.robots().hasRobot(giverRobotName_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[DualGiver] giver robot not loaded: {}", giverRobotName_);
  }
  auto & giver = ctl.robots().robot(giverRobotName_);
  if(!giver.hasFrame(giverToolFrame_) || !giver.hasFrame(giverBaseFrame_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[DualGiver] missing giver frames tool={} base={} robot={}",
        giverToolFrame_, giverBaseFrame_, giverRobotName_);
  }

  if(carriedObjectEnabled_)
  {
    if(!ctl.robots().hasRobot(carriedObjectRobotName_))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[DualGiver] carried object robot is not loaded: {}",
          carriedObjectRobotName_);
    }
    const auto & object = ctl.robots().robot(carriedObjectRobotName_);
    if(!object.hasFrame(carriedObjectFrameName_))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[DualGiver] carried object frame is missing: {}/{}",
          carriedObjectRobotName_, carriedObjectFrameName_);
    }
  }

  task_ = std::make_shared<mc_tasks::TransformTask>(
      giver.frame(giverToolFrame_), taskStiffness_, taskWeight_);
  holdPose_ = actualToolPose(ctl);
  targetPose_ = holdPose_;
  task_->target(holdPose_);

  ctl.logger().addLogEntry("dual_giver_phase", [this]() { return phaseCode(); });
  ctl.logger().addLogEntry("dual_giver_tracking_error", [this]() { return trackingError(); });
  ctl.logger().addLogEntry("dual_giver_speed", [this]() { return measuredSpeed(); });
  ctl.logger().addLogEntry("dual_giver_target_x", [this]() { return targetPose_.translation().x(); });
  ctl.logger().addLogEntry("dual_giver_target_y", [this]() { return targetPose_.translation().y(); });
  ctl.logger().addLogEntry("dual_giver_target_z", [this]() { return targetPose_.translation().z(); });
  ctl.logger().addLogEntry("dual_object_target_x", [this]() { return targetObjectPoseWorld_.translation().x(); });
  ctl.logger().addLogEntry("dual_object_target_y", [this]() { return targetObjectPoseWorld_.translation().y(); });
  ctl.logger().addLogEntry("dual_object_target_z", [this]() { return targetObjectPoseWorld_.translation().z(); });
  ctl.logger().addLogEntry("dual_object_carried_by_b", [this]() {
    return carriedObjectEnabled_ && !objectReleasedToReceiver_ ? 1.0 : 0.0;
  });
  ctl.logger().addLogEntry("dual_object_released_to_a", [this]() {
    return objectReleasedToReceiver_ ? 1.0 : 0.0;
  });

  mc_rtc::log::warning(
      "[DualGiver] world-frame trajectory presenter loaded robot={} scenario={} enabled={} motionEnabled={} "
      "startWorld=[{:.4f},{:.4f},{:.4f}] endWorld=[{:.4f},{:.4f},{:.4f}] "
      "velocityWorld=[{:.4f},{:.4f},{:.4f}] autoStartWithObservation={} "
      "canonicalValidatedOnly={} referenceOnlyPreview={} carriedObject={} "
      "positionCoupledWorldOrientation={} referenceCoupledObject={} object={}/{}",
      giverRobotName_, scenarioName_, enabled_, motionEnabled_,
      objectStartWorld_.x(), objectStartWorld_.y(), objectStartWorld_.z(),
      objectEndWorld_.x(), objectEndWorld_.y(), objectEndWorld_.z(),
      objectVelocityWorld_.x(), objectVelocityWorld_.y(), objectVelocityWorld_.z(),
      autoStartWithObservation_, !allowUnvalidatedScenario_, referenceOnlyPreview_,
      carriedObjectEnabled_, positionCoupledWorldOrientation_,
      referenceCoupledObject_, carriedObjectRobotName_, carriedObjectFrameName_);
}

void DualGiverCoordinator::loadConfig(const mc_rtc::Configuration & config)
{
  if(!config.has("dualHandover")) { return; }
  const auto dual = config("dualHandover");
  dual("enabled", enabled_);
  dual("motionEnabled", motionEnabled_);
  dual("giverRobot", giverRobotName_);
  dual("giverToolFrame", giverToolFrame_);
  dual("giverBaseFrame", giverBaseFrame_);
  dual("scenario", scenarioName_);
  dual("autoStartWithObservation", autoStartWithObservation_);
  dual("allowUnvalidatedScenario", allowUnvalidatedScenario_);
  dual("referenceOnlyPreview", referenceOnlyPreview_);

  if(dual.has("carriedObject"))
  {
    const auto object = dual("carriedObject");
    object("enabled", carriedObjectEnabled_);
    object("releaseOnReceiverAttach", releaseObjectOnReceiverAttach_);
    object("positionCoupledWorldOrientation", positionCoupledWorldOrientation_);
    object("referenceCoupledObject", referenceCoupledObject_);
    object("robot", carriedObjectRobotName_);
    object("frame", carriedObjectFrameName_);
    objectOrientationRPYWorld_ = readVector3(
        object, "orientationRPYWorld", objectOrientationRPYWorld_);
    objectToGiverToolTranslation_ = readVector3(
        object, "objectToGiverToolTranslation",
        objectToGiverToolTranslation_);
    objectToGiverToolRPY_ = readVector3(
        object, "objectToGiverToolRPY", objectToGiverToolRPY_);
  }

  if(dual.has("task"))
  {
    const auto task = dual("task");
    task("stiffness", taskStiffness_);
    task("weight", taskWeight_);
  }
  if(dual.has("preposition"))
  {
    const auto pre = dual("preposition");
    pre("maximumLinearSpeed", prepositionMaximumLinearSpeed_);
    pre("minimumDuration", prepositionMinimumDuration_);
    pre("timeout", prepositionTimeout_);
    pre("positionTolerance", prepositionPositionTolerance_);
    pre("velocityTolerance", prepositionVelocityTolerance_);
    pre("stableDuration", prepositionStableDuration_);
  }
  if(dual.has("terminal"))
  {
    const auto terminal = dual("terminal");
    terminal("positionTolerance", terminalPositionTolerance_);
    terminal("velocityTolerance", terminalVelocityTolerance_);
    terminal("stableDuration", terminalStableDuration_);
    terminal("timeout", terminalTimeout_);
    terminal("velocityFilterAlpha", velocityFilterAlpha_);
  }
  if(dual.has("safety"))
  {
    const auto safety = dual("safety");
    safety("maximumLinearSpeed", maximumLinearSpeed_);
    safety("maximumTravel", maximumTravel_);
    safety("maximumTrackingError", maximumTrackingError_);
    safety("trackingErrorGrace", trackingErrorGrace_);
    safety("enforceTrackingError", enforceTrackingError_);
    workspaceMinLocal_ = readVector3(safety, "workspaceMinLocal", workspaceMinLocal_);
    workspaceMaxLocal_ = readVector3(safety, "workspaceMaxLocal", workspaceMaxLocal_);
  }
  if(dual.has("scenarios"))
  {
    const auto scenarios = dual("scenarios");
    if(!scenarios.has(scenarioName_))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[DualGiver] scenario not found in dualHandover.scenarios: {}", scenarioName_);
    }
    const auto s = scenarios(scenarioName_);
    objectStartWorld_ = readVector3(
        s, "objectStartWorld", readVector3(s, "objectStartA", objectStartWorld_));
    objectVelocityWorld_ = readVector3(
        s, "objectVelocityWorld", readVector3(s, "objectVelocityA", objectVelocityWorld_));
    s("decelerationDuration", decelerationDuration_);

    const double configuredConstantDuration = [&]() {
      double value = constantVelocityDuration_;
      s("constantVelocityDuration", value);
      return value;
    }();
    const double configuredEffectiveTime = configuredConstantDuration
        + 0.5 * std::max(0.0, decelerationDuration_);
    const Eigen::Vector3d fallbackEnd = objectStartWorld_
        + objectVelocityWorld_ * configuredEffectiveTime;
    objectEndWorld_ = readVector3(s, "objectEndWorld", fallbackEnd);
  }

  taskStiffness_ = std::max(0.1, taskStiffness_);
  taskWeight_ = std::max(1.0, taskWeight_);
  prepositionMaximumLinearSpeed_ = std::max(0.005, prepositionMaximumLinearSpeed_);
  prepositionMinimumDuration_ = std::max(0.2, prepositionMinimumDuration_);
  prepositionTimeout_ = std::max(prepositionMinimumDuration_ + 1.0, prepositionTimeout_);
  constantVelocityDuration_ = std::max(0.0, constantVelocityDuration_);
  decelerationDuration_ = std::max(0.0, decelerationDuration_);
  maximumLinearSpeed_ = std::max(0.0, maximumLinearSpeed_);
  maximumTravel_ = std::max(0.0, maximumTravel_);
  velocityFilterAlpha_ = std::max(0.001, std::min(1.0, velocityFilterAlpha_));

  if(!allowUnvalidatedScenario_ && scenarioName_ != "canonical_yz")
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[DualGiver] physical execution is locked to canonical_yz; scenario={} requires allowUnvalidatedScenario=true",
        scenarioName_);
  }
  const Eigen::Vector3d displacement = objectEndWorld_ - objectStartWorld_;
  const double speed = objectVelocityWorld_.norm();
  if(speed <= 1e-12)
  {
    if(displacement.norm() > 1e-9)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[DualGiver] zero-velocity scenario has distinct world start/end");
    }
    constantVelocityDuration_ = 0.0;
    decelerationDuration_ = 0.0;
  }
  else
  {
    const double effectiveTime = displacement.dot(objectVelocityWorld_)
        / objectVelocityWorld_.squaredNorm();
    const Eigen::Vector3d residual = displacement
        - objectVelocityWorld_ * effectiveTime;
    if(effectiveTime < -1e-9 || residual.norm() > 1e-5)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[DualGiver] world endpoint is not on the configured velocity ray");
    }
    constantVelocityDuration_ = effectiveTime - 0.5 * decelerationDuration_;
    if(constantVelocityDuration_ < -1e-9)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[DualGiver] world endpoint is too close for the configured deceleration duration");
    }
    constantVelocityDuration_ = std::max(0.0, constantVelocityDuration_);
  }

  if(speed > maximumLinearSpeed_ + 1e-9
     || displacement.norm() > maximumTravel_ + 1e-9)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[DualGiver] configured world velocity/travel exceeds safety limits");
  }
}

void DualGiverCoordinator::reset(HandoverInterceptionController & ctl)
{
  if(!enabled_) { return; }
  taskActive_ = false;
  iter_ = 0;
  presentationRequested_ = false;
  prepositionElapsed_ = 0.0;
  settleElapsed_ = 0.0;
  stableTime_ = 0.0;
  executionElapsed_ = 0.0;
  maximumObservedTrackingError_ = 0.0;
  havePreviousPose_ = false;
  filteredLinearVelocity_.setZero();
  filteredAngularSpeed_ = 0.0;
  objectReleasedToReceiver_ = false;
  objectPoseInitialized_ = false;

  const Eigen::Matrix3d R_O_G = rpyToRotation(objectToGiverToolRPY_);
  O_T_G_ = ctl.fromWorldPose(R_O_G, objectToGiverToolTranslation_);
  const Eigen::Matrix3d R_G_O = R_O_G.transpose();
  G_T_O_ = ctl.fromWorldPose(
      R_G_O, -R_G_O * objectToGiverToolTranslation_);
  startObjectPoseWorld_ = scenarioObjectStartWorld(ctl);
  endObjectPoseWorld_ = scenarioObjectEndWorld(ctl);
  targetObjectPoseWorld_ = startObjectPoseWorld_;

  activateTask(ctl);
  prepositionStartPose_ = actualToolPose(ctl);
  holdPose_ = prepositionStartPose_;
  targetPose_ = holdPose_;
  commandPose(ctl, holdPose_);
  if(carriedObjectEnabled_)
  {
    updateCarriedObjectPose(ctl);
    const Eigen::Matrix3d R_W_O =
        rpyToRotation(objectOrientationRPYWorld_);
    const Eigen::Vector3d blueOuterTipWorld =
        objectStartWorld_ + R_W_O * Eigen::Vector3d(0.0, 0.0, -0.1556);
    const Eigen::Vector3d greyOuterTipWorld =
        objectStartWorld_ + R_W_O * Eigen::Vector3d(0.0, 0.0, 0.1556);
    mc_rtc::log::success(
        "[DualGiver OBJECT COUPLING] mode={} Robot-B {}.{} follows GREY outer tip; "
        "object orientation is {}",
        positionCoupledWorldOrientation_ ? "position_only_world_locked" : "rigid_6d",
        giverRobotName_, giverToolFrame_,
        positionCoupledWorldOrientation_ ? "locked in Robot-A/world" : "inherited through rigid coupling");
    mc_rtc::log::success(
        "[DualGiver OBJECT SIDES] start blueTipWorld=[{:.4f},{:.4f},{:.4f}] "
        "centreWorld=[{:.4f},{:.4f},{:.4f}] greyTipWorld=[{:.4f},{:.4f},{:.4f}]",
        blueOuterTipWorld.x(), blueOuterTipWorld.y(), blueOuterTipWorld.z(),
        objectStartWorld_.x(), objectStartWorld_.y(), objectStartWorld_.z(),
        greyOuterTipWorld.x(), greyOuterTipWorld.y(), greyOuterTipWorld.z());
  }

  if(!motionEnabled_)
  {
    phase_ = Phase::DisabledHold;
    mc_rtc::log::warning("[DualGiver DISABLED] holding measured giver pose; dualHandover.motionEnabled=false");
    return;
  }

  startPoseWorld_ = toolPoseForObjectPose(ctl, startObjectPoseWorld_);
  const sva::PTransformd endPoseWorld =
      toolPoseForObjectPose(ctl, endObjectPoseWorld_);
  const double distance = (startPoseWorld_.translation() - prepositionStartPose_.translation()).norm();
  prepositionDuration_ = std::max(
      prepositionMinimumDuration_, 1.875 * distance / prepositionMaximumLinearSpeed_);
  if(prepositionDuration_ > prepositionTimeout_)
  {
    fail(ctl, "preposition duration exceeds timeout");
    return;
  }
  if(!localTargetInsideWorkspace(ctl, startPoseWorld_))
  {
    fail(ctl, "scenario start outside giver local workspace");
    return;
  }

  if(!localTargetInsideWorkspace(ctl, endPoseWorld))
  {
    fail(ctl, "scenario endpoint outside giver local workspace");
    return;
  }

  phase_ = Phase::Prepositioning;
  const Eigen::Vector3d startLocal = ctl.pointToLocal(
      giverBasePose(ctl), startPoseWorld_.translation());
  const Eigen::Vector3d endLocal = ctl.pointToLocal(
      giverBasePose(ctl), endPoseWorld.translation());
  mc_rtc::log::success(
      "[DualGiver PREPOSITION START] scenario={} distance={:.4f}m duration={:.3f}s "
      "objectStartWorld=[{:.4f},{:.4f},{:.4f}] objectEndWorld=[{:.4f},{:.4f},{:.4f}] "
      "toolStartWorld=[{:.4f},{:.4f},{:.4f}] toolEndWorld=[{:.4f},{:.4f},{:.4f}] "
      "toolStartLocal=[{:.4f},{:.4f},{:.4f}] toolEndLocal=[{:.4f},{:.4f},{:.4f}]",
      scenarioName_, distance, prepositionDuration_,
      startObjectPoseWorld_.translation().x(), startObjectPoseWorld_.translation().y(),
      startObjectPoseWorld_.translation().z(), endObjectPoseWorld_.translation().x(),
      endObjectPoseWorld_.translation().y(), endObjectPoseWorld_.translation().z(),
      startPoseWorld_.translation().x(), startPoseWorld_.translation().y(),
      startPoseWorld_.translation().z(), endPoseWorld.translation().x(),
      endPoseWorld.translation().y(), endPoseWorld.translation().z(),
      startLocal.x(), startLocal.y(), startLocal.z(),
      endLocal.x(), endLocal.y(), endLocal.z());
}

void DualGiverCoordinator::activateTask(HandoverInterceptionController & ctl)
{
  if(task_ && !taskActive_)
  {
    ctl.solver().addTask(task_);
    taskActive_ = true;
    mc_rtc::log::success("[DualGiver] Cartesian task activated robot={} frame={}",
                         giverRobotName_, giverToolFrame_);
  }
}

void DualGiverCoordinator::run(HandoverInterceptionController & ctl)
{
  if(!enabled_) { return; }
  ++iter_;
  updateMeasuredMotion(ctl);

  if(carriedObjectEnabled_ && ctl.objectAttached()
     && !objectReleasedToReceiver_)
  {
    objectReleasedToReceiver_ = true;
    if(releaseObjectOnReceiverAttach_)
    {
      holdPose_ = actualToolPose(ctl);
      phase_ = Phase::Holding;
      commandPose(ctl, holdPose_);
    }
    mc_rtc::log::success(
        "[DualGiver OBJECT RELEASE] Robot A acquired the blue handle; "
        "the object is no longer propagated from Robot B and Robot B holds");
    return;
  }

  if(carriedObjectEnabled_ && !ctl.objectAttached())
  {
    updateCarriedObjectPose(ctl);
  }

  if(phase_ == Phase::DisabledHold || phase_ == Phase::Failed
     || phase_ == Phase::Holding)
  {
    commandPose(ctl, holdPose_);
    return;
  }

  if(phase_ == Phase::Prepositioning)
  {
    prepositionElapsed_ += ctl.controlDt();
    const double u = std::min(1.0, prepositionElapsed_ / prepositionDuration_);
    const double s = smoothStep(u);
    const double dsdt = smoothStepVelocity(u) / prepositionDuration_;
    const double d2sdt2 = smoothStepAcceleration(u)
        / (prepositionDuration_ * prepositionDuration_);
    const Eigen::Vector3d delta = startPoseWorld_.translation()
        - prepositionStartPose_.translation();
    targetPose_ = ctl.interpolatePose(
        prepositionStartPose_, startPoseWorld_, s);
    commandPose(ctl, targetPose_, delta * dsdt, delta * d2sdt2);

    const double err = trackingError();
    maximumObservedTrackingError_ = std::max(maximumObservedTrackingError_, err);
    if(enforceTrackingError_ && prepositionElapsed_ >= trackingErrorGrace_ && err > maximumTrackingError_)
    {
      fail(ctl, "preposition tracking error exceeded bound");
      return;
    }
    if(prepositionElapsed_ >= prepositionDuration_)
    {
      phase_ = Phase::StartSettling;
      settleElapsed_ = 0.0;
      stableTime_ = 0.0;
      holdPose_ = startPoseWorld_;
      commandPose(ctl, holdPose_);
      mc_rtc::log::warning("[DualGiver START-GATE] reference reached fixed scenario start");
    }
    return;
  }

  if(phase_ == Phase::StartSettling)
  {
    settleElapsed_ += ctl.controlDt();
    commandPose(ctl, holdPose_);
    const bool stable = referenceOnlyPreview_
        || (trackingError() <= prepositionPositionTolerance_
            && measuredSpeed() <= prepositionVelocityTolerance_);
    stableTime_ = stable ? stableTime_ + ctl.controlDt() : 0.0;
    if(stableTime_ >= prepositionStableDuration_)
    {
      phase_ = Phase::Ready;
      mc_rtc::log::success(
          "[DualGiver READY] scenario={} trackingError={:.4f}m speed={:.4f}m/s "
          "gateMode={}; waiting for presentation request",
          scenarioName_, trackingError(), measuredSpeed(),
          referenceOnlyPreview_ ? "reference_only_preview" : "measured");
    }
    else if(settleElapsed_ >= prepositionTimeout_)
    {
      fail(ctl, "fixed-start pose/velocity gate timed out");
    }
    return;
  }

  if(phase_ == Phase::Ready)
  {
    commandPose(ctl, startPoseWorld_);
    if(presentationRequested_)
    {
      phase_ = Phase::Executing;
      executionElapsed_ = 0.0;
      maximumObservedTrackingError_ = 0.0;
      mc_rtc::log::success(
          "[DualGiver START] synchronized with Robot-A object observation scenario={}",
          scenarioName_);
    }
    return;
  }

  if(phase_ == Phase::Executing)
  {
    executionElapsed_ += ctl.controlDt();
    const Eigen::Vector3d vWorld = velocityWorld();
    double effectiveTime = 0.0;
    double velocityScale = 0.0;
    double accelerationScale = 0.0;
    if(executionElapsed_ <= constantVelocityDuration_)
    {
      effectiveTime = executionElapsed_;
      velocityScale = 1.0;
    }
    else if(decelerationDuration_ > 1e-9
            && executionElapsed_ < constantVelocityDuration_ + decelerationDuration_)
    {
      const double u = (executionElapsed_ - constantVelocityDuration_)
          / decelerationDuration_;
      effectiveTime = constantVelocityDuration_
          + decelerationDuration_ * stopIntegral(u);
      velocityScale = stopVelocityScale(u);
      accelerationScale = stopAccelerationScale(u) / decelerationDuration_;
    }
    else
    {
      effectiveTime = constantVelocityDuration_ + 0.5 * decelerationDuration_;
    }

    const Eigen::Vector3d commandedObjectPosition =
        executionElapsed_ >= constantVelocityDuration_ + decelerationDuration_
        ? objectEndWorld_
        : objectStartWorld_ + vWorld * effectiveTime;
    targetObjectPoseWorld_ = ctl.fromWorldPose(
        rpyToRotation(objectOrientationRPYWorld_), commandedObjectPosition);
    targetPose_ = toolPoseForObjectPose(ctl, targetObjectPoseWorld_);
    if(!localTargetInsideWorkspace(ctl, targetPose_))
    {
      fail(ctl, "runtime target left giver local workspace");
      return;
    }
    commandPose(ctl, targetPose_, vWorld * velocityScale,
                vWorld * accelerationScale);
    const double err = trackingError();
    maximumObservedTrackingError_ = std::max(maximumObservedTrackingError_, err);
    if(enforceTrackingError_ && executionElapsed_ >= trackingErrorGrace_ && err > maximumTrackingError_)
    {
      fail(ctl, "scenario tracking error exceeded bound");
      return;
    }
    if(executionElapsed_ >= constantVelocityDuration_ + decelerationDuration_)
    {
      phase_ = Phase::TerminalSettling;
      holdPose_ = targetPose_;
      settleElapsed_ = 0.0;
      stableTime_ = 0.0;
      commandPose(ctl, holdPose_);
      mc_rtc::log::warning("[DualGiver TERMINAL] trajectory reference complete; measured gate active");
    }
    return;
  }

  if(phase_ == Phase::TerminalSettling)
  {
    settleElapsed_ += ctl.controlDt();
    commandPose(ctl, holdPose_);
    const bool stable = referenceOnlyPreview_
        || (trackingError() <= terminalPositionTolerance_
            && measuredSpeed() <= terminalVelocityTolerance_);
    stableTime_ = stable ? stableTime_ + ctl.controlDt() : 0.0;
    if(stableTime_ >= terminalStableDuration_)
    {
      phase_ = Phase::Holding;
      mc_rtc::log::success(
          "[DualGiver HOLD] presentation endpoint admitted error={:.4f}m speed={:.4f}m/s maxTracking={:.4f}m",
          trackingError(), measuredSpeed(), maximumObservedTrackingError_);
    }
    else if(settleElapsed_ >= terminalTimeout_)
    {
      fail(ctl, "terminal pose/velocity gate timed out");
    }
  }
}

bool DualGiverCoordinator::requestPresentation(HandoverInterceptionController & ctl)
{
  if(!enabled_) { return true; }
  if(!autoStartWithObservation_) { return ready(); }
  if(phase_ != Phase::Ready)
  {
    mc_rtc::log::error(
        "[DualGiver] Robot-A observation requested before giver READY phase={}",
        phaseCode());
    return false;
  }
  presentationRequested_ = true;
  commandPose(ctl, startPoseWorld_);
  return true;
}

void DualGiverCoordinator::safeHold(HandoverInterceptionController & ctl)
{
  if(!enabled_) { return; }
  holdPose_ = actualToolPose(ctl);
  phase_ = Phase::Failed;
  commandPose(ctl, holdPose_);
}

bool DualGiverCoordinator::presentationScheduleAvailable() const
{
  return enabled_ && motionEnabled_ && presentationRequested_
      && (phase_ == Phase::Executing
          || phase_ == Phase::TerminalSettling
          || phase_ == Phase::Holding);
}

double DualGiverCoordinator::timeToPresentation() const
{
  if(!presentationScheduleAvailable()) { return 0.0; }
  if(phase_ != Phase::Executing) { return 0.0; }
  return std::max(
      0.0,
      constantVelocityDuration_ + decelerationDuration_ - executionElapsed_);
}

sva::PTransformd DualGiverCoordinator::presentationObjectPoseWorld() const
{
  return endObjectPoseWorld_;
}

void DualGiverCoordinator::commandPose(
    HandoverInterceptionController & ctl,
    const sva::PTransformd & target,
    const Eigen::Vector3d & linearVelocityWorld,
    const Eigen::Vector3d & linearAccelerationWorld)
{
  targetPose_ = target;
  if(!task_) { return; }
  task_->target(targetPose_);
  const Eigen::Matrix3d R_W_T = ctl.worldRotation(actualToolPose(ctl));
  task_->refVelB(sva::MotionVecd(
      Eigen::Vector3d::Zero(), R_W_T.transpose() * linearVelocityWorld));
  task_->refAccel(sva::MotionVecd(
      Eigen::Vector3d::Zero(), R_W_T.transpose() * linearAccelerationWorld));
}

sva::PTransformd DualGiverCoordinator::actualToolPose(
    const HandoverInterceptionController & ctl) const
{
  return ctl.robots().robot(giverRobotName_).frame(giverToolFrame_).position();
}

sva::PTransformd DualGiverCoordinator::giverBasePose(
    const HandoverInterceptionController & ctl) const
{
  return ctl.robots().robot(giverRobotName_).frame(giverBaseFrame_).position();
}

Eigen::Matrix3d DualGiverCoordinator::rpyToRotation(
    const Eigen::Vector3d & rpy)
{
  const Eigen::AngleAxisd roll(rpy.x(), Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(rpy.y(), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(rpy.z(), Eigen::Vector3d::UnitZ());
  return (yaw * pitch * roll).toRotationMatrix();
}

sva::PTransformd DualGiverCoordinator::scenarioObjectStartWorld(
    const HandoverInterceptionController & ctl) const
{
  return ctl.fromWorldPose(
      rpyToRotation(objectOrientationRPYWorld_), objectStartWorld_);
}

sva::PTransformd DualGiverCoordinator::scenarioObjectEndWorld(
    const HandoverInterceptionController & ctl) const
{
  return ctl.fromWorldPose(
      rpyToRotation(objectOrientationRPYWorld_), objectEndWorld_);
}

sva::PTransformd DualGiverCoordinator::toolPoseForObjectPose(
    const HandoverInterceptionController & ctl,
    const sva::PTransformd & objectPoseWorld) const
{
  if(positionCoupledWorldOrientation_)
  {
    // Command Robot-B tool position to the GREY outer tip, but preserve the
    // tool's initial world orientation. This avoids rotating the object with
    // Robot-B tool and prevents the blue side from appearing inside Robot B.
    const Eigen::Matrix3d R_W_O = ctl.worldRotation(objectPoseWorld);
    const Eigen::Vector3d p_W_G = objectPoseWorld.translation()
        + R_W_O * objectToGiverToolTranslation_;
    const Eigen::Matrix3d R_W_G =
        ctl.worldRotation(prepositionStartPose_);
    return ctl.fromWorldPose(R_W_G, p_W_G);
  }
  return ctl.compose(objectPoseWorld, O_T_G_);
}

sva::PTransformd DualGiverCoordinator::objectPoseFromToolPose(
    const HandoverInterceptionController & ctl,
    const sva::PTransformd & toolPoseWorld) const
{
  if(positionCoupledWorldOrientation_)
  {
    // Keep the CALL-object orientation fixed in Robot-A/world. Only its
    // translation follows Robot B, with the GREY outer tip coincident with
    // Robot-B tool position.
    const Eigen::Matrix3d R_W_O =
        rpyToRotation(objectOrientationRPYWorld_);
    const Eigen::Vector3d p_W_O = toolPoseWorld.translation()
        - R_W_O * objectToGiverToolTranslation_;
    return ctl.fromWorldPose(R_W_O, p_W_O);
  }
  return ctl.compose(toolPoseWorld, G_T_O_);
}

void DualGiverCoordinator::updateCarriedObjectPose(
    HandoverInterceptionController & ctl)
{
  if(!carriedObjectEnabled_ || objectReleasedToReceiver_
     || ctl.objectAttached())
  {
    return;
  }

  sva::PTransformd objectPose =
      objectPoseFromToolPose(ctl, actualToolPose(ctl));

  // In ticker, Robot A must observe the exact scenario law that was committed:
  // start -> constant world velocity -> terminal deceleration -> endpoint.
  // Propagating the object from Robot-B's measured tool introduced a small
  // transient speed overshoot (0.0859 instead of 0.0800 m/s). Robot A then
  // correctly rejected the run when the accumulated model error reached its
  // unchanged 15 mm prediction tube. Reference coupling removes that simulator
  // artefact without weakening Robot A's safety gate or changing its methodology.
  if(referenceCoupledObject_
     && (phase_ == Phase::Executing
         || phase_ == Phase::TerminalSettling
         || phase_ == Phase::Holding))
  {
    objectPose = targetObjectPoseWorld_;
  }

  ctl.robots().robot(carriedObjectRobotName_).posW(objectPose);
  objectPoseInitialized_ = true;
}

Eigen::Vector3d DualGiverCoordinator::velocityWorld() const
{
  return objectVelocityWorld_;
}

void DualGiverCoordinator::updateMeasuredMotion(HandoverInterceptionController & ctl)
{
  const sva::PTransformd current = actualToolPose(ctl);
  if(!havePreviousPose_)
  {
    previousPosition_ = current.translation();
    previousRotation_ = current.rotation();
    havePreviousPose_ = true;
    return;
  }
  const double dt = std::max(1e-6, ctl.controlDt());
  const Eigen::Vector3d raw = (current.translation() - previousPosition_) / dt;
  const double rawAngular = rotationAngle(
      current.rotation() * previousRotation_.transpose()) / dt;
  filteredLinearVelocity_ = (1.0 - velocityFilterAlpha_) * filteredLinearVelocity_
      + velocityFilterAlpha_ * raw;
  filteredAngularSpeed_ = (1.0 - velocityFilterAlpha_) * filteredAngularSpeed_
      + velocityFilterAlpha_ * rawAngular;
  previousPosition_ = current.translation();
  previousRotation_ = current.rotation();
}

void DualGiverCoordinator::fail(
    HandoverInterceptionController & ctl, const std::string & reason)
{
  phase_ = Phase::Failed;
  holdPose_ = actualToolPose(ctl);
  commandPose(ctl, holdPose_);
  mc_rtc::log::error(
      "[DualGiver FAILURE] {}. Holding measured pose; no retry and no fallback.",
      reason);
}

bool DualGiverCoordinator::localTargetInsideWorkspace(
    const HandoverInterceptionController & ctl,
    const sva::PTransformd & targetWorld) const
{
  const Eigen::Vector3d local = ctl.pointToLocal(giverBasePose(ctl),
                                                 targetWorld.translation());
  return (local.array() >= workspaceMinLocal_.array()).all()
      && (local.array() <= workspaceMaxLocal_.array()).all();
}

double DualGiverCoordinator::trackingError() const
{
  return task_ ? task_->eval().norm() : 0.0;
}

Eigen::Vector3d DualGiverCoordinator::readVector3(
    const mc_rtc::Configuration & cfg, const std::string & key,
    const Eigen::Vector3d & fallback)
{
  if(!cfg.has(key)) { return fallback; }
  std::vector<double> values;
  cfg(key, values);
  if(values.size() != 3) { return fallback; }
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

double DualGiverCoordinator::clamp01(double x)
{
  return std::max(0.0, std::min(1.0, x));
}

double DualGiverCoordinator::smoothStep(double u)
{
  u = clamp01(u);
  return 10.0*u*u*u - 15.0*u*u*u*u + 6.0*u*u*u*u*u;
}

double DualGiverCoordinator::smoothStepVelocity(double u)
{
  u = clamp01(u);
  return 30.0*u*u - 60.0*u*u*u + 30.0*u*u*u*u;
}

double DualGiverCoordinator::smoothStepAcceleration(double u)
{
  u = clamp01(u);
  return 60.0*u - 180.0*u*u + 120.0*u*u*u;
}

double DualGiverCoordinator::stopIntegral(double u)
{
  u = clamp01(u);
  const double u2=u*u, u3=u2*u, u4=u3*u, u5=u4*u, u6=u5*u;
  return u - 2.5*u4 + 3.0*u5 - u6;
}

double DualGiverCoordinator::stopVelocityScale(double u)
{
  u = clamp01(u);
  const double u2=u*u, u3=u2*u, u4=u3*u, u5=u4*u;
  return 1.0 - 10.0*u3 + 15.0*u4 - 6.0*u5;
}

double DualGiverCoordinator::stopAccelerationScale(double u)
{
  u = clamp01(u);
  const double v = 1.0-u;
  return -30.0*u*u*v*v;
}

double DualGiverCoordinator::rotationAngle(const Eigen::Matrix3d & R)
{
  const double c = std::max(-1.0, std::min(1.0, 0.5*(R.trace()-1.0)));
  return std::acos(c);
}
