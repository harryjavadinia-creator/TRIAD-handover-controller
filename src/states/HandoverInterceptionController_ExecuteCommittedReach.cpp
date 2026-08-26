#include "HandoverInterceptionController_ExecuteCommittedReach.h"
#include "../HandoverInterceptionController.h"

#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>
#include <limits>

void HandoverInterceptionController_ExecuteCommittedReach::configure(
    const mc_rtc::Configuration & config)
{
  if(config.has("maxLinearTrackingLead"))
  {
    config("maxLinearTrackingLead", maxLinearTrackingLead_);
  }
  if(config.has("maxAngularTrackingLead"))
  {
    config("maxAngularTrackingLead", maxAngularTrackingLead_);
  }
  if(config.has("nearLinearTrackingLead"))
  {
    config("nearLinearTrackingLead", nearLinearTrackingLead_);
  }
  if(config.has("nearAngularTrackingLead"))
  {
    config("nearAngularTrackingLead", nearAngularTrackingLead_);
  }
  if(config.has("clearanceSlowdownStart"))
  {
    config("clearanceSlowdownStart", clearanceSlowdownStart_);
  }
  if(config.has("clearanceHardMargin"))
  {
    config("clearanceHardMargin", clearanceHardMargin_);
  }
  if(config.has("minimumRuntimeClearance"))
  {
    config("minimumRuntimeClearance", minimumRuntimeClearance_);
  }
  if(config.has("minimumVelocityScale"))
  {
    config("minimumVelocityScale", minimumVelocityScale_);
  }
  if(config.has("clearanceScaleDropRate"))
  {
    config("clearanceScaleDropRate", clearanceScaleDropRate_);
  }
  if(config.has("clearanceScaleRiseRate"))
  {
    config("clearanceScaleRiseRate", clearanceScaleRiseRate_);
  }
  if(config.has("farLinearSpeed"))
  {
    config("farLinearSpeed", farLinearSpeed_);
  }
  if(config.has("nearLinearSpeed"))
  {
    config("nearLinearSpeed", nearLinearSpeed_);
  }
  if(config.has("farAngularSpeed"))
  {
    config("farAngularSpeed", farAngularSpeed_);
  }
  if(config.has("nearAngularSpeed"))
  {
    config("nearAngularSpeed", nearAngularSpeed_);
  }
  if(config.has("posTol")) { config("posTol", posTol_); }
  if(config.has("oriTol")) { config("oriTol", oriTol_); }
  if(config.has("taskStiffness")) { config("taskStiffness", taskStiffness_); }
  if(config.has("taskWeight")) { config("taskWeight", taskWeight_); }
  if(config.has("launchTimingTolerance"))
  {
    config("launchTimingTolerance", launchTimingTolerance_);
  }
  if(config.has("scheduleLatenessTolerance"))
  {
    config("scheduleLatenessTolerance", scheduleLatenessTolerance_);
  }
  if(config.has("minimumScheduledDuration"))
  {
    config("minimumScheduledDuration", minimumScheduledDuration_);
  }
  if(config.has("maximumObjectTranslationDeviation"))
  {
    config("maximumObjectTranslationDeviation",
           maximumObjectTranslationDeviation_);
  }
  if(config.has("maximumObjectRotationDeviation"))
  {
    config("maximumObjectRotationDeviation", maximumObjectRotationDeviation_);
  }
  if(config.has("logEvery")) { config("logEvery", logEvery_); }

  maxLinearTrackingLead_ = std::max(0.001, maxLinearTrackingLead_);
  maxAngularTrackingLead_ = std::max(0.001, maxAngularTrackingLead_);
  nearLinearTrackingLead_ = std::min(
      maxLinearTrackingLead_, std::max(0.001, nearLinearTrackingLead_));
  nearAngularTrackingLead_ = std::min(
      maxAngularTrackingLead_, std::max(0.001, nearAngularTrackingLead_));
  clearanceSlowdownStart_ = std::max(0.001, clearanceSlowdownStart_);
  clearanceHardMargin_ = std::min(
      clearanceSlowdownStart_ - 1e-4,
      std::max(0.0, clearanceHardMargin_));
  minimumRuntimeClearance_ = std::min(
      clearanceHardMargin_, std::max(0.0, minimumRuntimeClearance_));
  minimumVelocityScale_ = std::min(
      1.0, std::max(0.01, minimumVelocityScale_));
  clearanceScaleDropRate_ = std::max(0.01, clearanceScaleDropRate_);
  clearanceScaleRiseRate_ = std::max(0.01, clearanceScaleRiseRate_);
  farLinearSpeed_ = std::max(0.01, farLinearSpeed_);
  nearLinearSpeed_ = std::min(
      farLinearSpeed_, std::max(0.005, nearLinearSpeed_));
  farAngularSpeed_ = std::max(0.05, farAngularSpeed_);
  nearAngularSpeed_ = std::min(
      farAngularSpeed_, std::max(0.02, nearAngularSpeed_));
  posTol_ = std::max(0.001, posTol_);
  oriTol_ = std::max(0.001, oriTol_);
  launchTimingTolerance_ = std::max(0.001, launchTimingTolerance_);
  scheduleLatenessTolerance_ = std::max(launchTimingTolerance_, scheduleLatenessTolerance_);
  minimumScheduledDuration_ = std::max(0.05, minimumScheduledDuration_);
  maximumObjectTranslationDeviation_ = std::max(
      0.0, maximumObjectTranslationDeviation_);
  maximumObjectRotationDeviation_ = std::max(
      0.0, maximumObjectRotationDeviation_);
  logEvery_ = std::max<uint64_t>(1, logEvery_);
}

void HandoverInterceptionController_ExecuteCommittedReach::start(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  const auto & policy = ctl.predictiveReachPolicy();
  maxLinearTrackingLead_ = policy.maxLinearTrackingLead;
  maxAngularTrackingLead_ = policy.maxAngularTrackingLead;
  nearLinearTrackingLead_ = policy.nearLinearTrackingLead;
  nearAngularTrackingLead_ = policy.nearAngularTrackingLead;
  clearanceSlowdownStart_ = policy.clearanceSlowdownStart;
  clearanceHardMargin_ = policy.clearanceHardMargin;
  minimumRuntimeClearance_ = policy.minimumRuntimeClearance;
  minimumVelocityScale_ = policy.minimumVelocityScale;
  clearanceScaleDropRate_ = policy.clearanceScaleDropRate;
  clearanceScaleRiseRate_ = policy.clearanceScaleRiseRate;
  farLinearSpeed_ = policy.farLinearSpeed;
  nearLinearSpeed_ = policy.nearLinearSpeed;
  farAngularSpeed_ = policy.farAngularSpeed;
  nearAngularSpeed_ = policy.nearAngularSpeed;
  posTol_ = policy.positionTolerance;
  oriTol_ = policy.orientationTolerance;
  taskStiffness_ = policy.taskStiffness;
  taskWeight_ = policy.taskWeight;
  launchTimingTolerance_ = policy.launchTimingTolerance;
  scheduleLatenessTolerance_ = policy.scheduleLatenessTolerance;
  minimumScheduledDuration_ = policy.minimumScheduledDuration;
  maximumObjectTranslationDeviation_ =
      policy.maximumObjectTranslationDeviation;
  maximumObjectRotationDeviation_ = policy.maximumObjectRotationDeviation;
  iter_ = 0;
  ready_ = false;
  motionStarted_ = false;
  phaseTimingStarted_ = false;
  decelerationLogged_ = false;
  clearanceScale_ = 1.0;
  minimumObservedClearance_ = std::numeric_limits<double>::infinity();
  stateEntryTime_ = ctl.controllerTime();
  startTime_ = 0.0;

  if(!ctl.committedPlanValid() || !ctl.hasSelectedCandidate())
  {
    mc_rtc::log::error(
        "[PredictiveReach] no committed presentation plan is available");
    return;
  }

  const auto & plan = ctl.committedInterceptionPlan();
  if(!plan.presentationMode)
  {
    mc_rtc::log::error(
        "[PredictiveReach] clean controller requires presentation mode");
    return;
  }

  reachStartTime_ = plan.reachStartTime;
  reachEndTime_ = plan.standoffTime;
  const double entryLateness = stateEntryTime_ - reachStartTime_;
  if(entryLateness > launchTimingTolerance_)
  {
    mc_rtc::log::error(
        "[PredictiveReach] state entered after committed reach start lateness={:+.3f}s tolerance={:.3f}s; no motion",
        entryLateness, launchTimingTolerance_);
    return;
  }

  const double scheduledDuration = reachEndTime_ - reachStartTime_;
  if(!std::isfinite(scheduledDuration)
     || scheduledDuration < minimumScheduledDuration_)
  {
    mc_rtc::log::error(
        "[PredictiveReach] invalid committed reach duration={:.3f}s",
        scheduledDuration);
    return;
  }

  holdArmPosture_ = ctl.currentArmPosture();
  startPose_ = ctl.actualMouthPose();
  const double startAnchorError = (
      startPose_.translation() - plan.mouthAtReachStart.translation()).norm();
  const double startAnchorAngle = ctl.orientationError(
      startPose_, plan.mouthAtReachStart);
  if(startAnchorError > posTol_ || startAnchorAngle > oriTol_)
  {
    mc_rtc::log::error(
        "[PredictiveReach] physical start differs from committed start position={:.4f} orientation={:.4f}; no motion",
        startAnchorError, startAnchorAngle);
    return;
  }

  goalPose_ = ctl.committedStandoffTargetAt(plan.standoffTime);
  referencePose_ = plan.mouthAtReachStart;
  previousMouthPose_ = startPose_;
  previousSampleTime_ = stateEntryTime_;
  havePreviousSample_ = false;

  ctl.activateToolTask();
  ctl.setToolTaskGains(taskStiffness_, taskWeight_);
  ctl.commandReadyArmPosture(holdArmPosture_);
  ctl.setGripperClosureAuthorized(false);
  ctl.setGripperJointPriority(false);
  ctl.commandGripper(0.0);
  ctl.commandMouthTarget(referencePose_);

  const Eigen::Vector3d p = goalPose_.translation();
  mc_rtc::log::warning(
      "[PredictiveReach] committed state entered {:+.3f}s relative to exact reachStart; holding until t={:.3f}s then executing {:.3f}s route={} reach to [{:.3f},{:.3f},{:.3f}]",
      entryLateness, reachStartTime_, scheduledDuration, plan.transitRouteName,
      p.x(), p.y(), p.z());
  mc_rtc::log::success(
      "[MethodologyLockRuntime] phase=predictive_reach exactCommittedLaunch=true entryTimingError={:+.3f}s startPositionError={:.4f} startOrientationError={:.4f} gripperOpen=true",
      entryLateness, startAnchorError, startAnchorAngle);
  mc_rtc::log::success(
      "[VelocityGatedTerminalCapture] V5.2.5 exactReachStart=true clearanceGovernor=true slowdownStart={:.3f}m runtimeReserve={:.3f}m hardLeadCap=true leadFarNear=[{:.3f},{:.3f}]m speedFarNear=[{:.3f},{:.3f}]m/s noRetiming=true noReplanning=true",
      clearanceSlowdownStart_, minimumRuntimeClearance_,
      maxLinearTrackingLead_, nearLinearTrackingLead_,
      farLinearSpeed_, nearLinearSpeed_);
  mc_rtc::log::success(
      "[CommittedRouteBank] V5.4.0 route={} curveOffset=[{:.3f},{:.3f},{:.3f}] sharedPreviewRuntimePolicy=true immutableAfterCommit=true",
      plan.transitRouteName, plan.reachCurveOffsetWorld.x(),
      plan.reachCurveOffsetWorld.y(), plan.reachCurveOffsetWorld.z());
  ready_ = true;
}

bool HandoverInterceptionController_ExecuteCommittedReach::run(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ++iter_;

  if(!ready_ || !ctl.committedPlanValid() || !ctl.hasSelectedCandidate())
  {
    ctl.endObjectObservation(true);
    mc_rtc::log::error(
        "[PredictiveReach] committed plan unavailable during execution");
    output("FAIL");
    return true;
  }

  const auto & plan = ctl.committedInterceptionPlan();
  ctl.activateToolTask();
  ctl.setToolTaskGains(taskStiffness_, taskWeight_);
  if(!motionStarted_) { ctl.commandReadyArmPosture(holdArmPosture_); }
  ctl.setGripperClosureAuthorized(false);
  ctl.setGripperJointPriority(false);
  ctl.commandGripper(0.0);

  const double now = ctl.controllerTime();
  const double objectPositionError = ctl.committedObjectPositionErrorAt(now);
  const double objectOrientationError = ctl.committedObjectOrientationErrorAt(now);
  if(objectPositionError > maximumObjectTranslationDeviation_
     || objectOrientationError > maximumObjectRotationDeviation_)
  {
    ctl.commandMouthTarget(ctl.actualMouthPose());
    ctl.endObjectObservation(true);
    mc_rtc::log::error(
        "[PredictiveReach] object left committed prediction tube positionError={:.4f} rotationError={:.4f}; no replanning",
        objectPositionError, objectOrientationError);
    output("FAIL");
    return true;
  }

  if(!motionStarted_)
  {
    if(now < reachStartTime_)
    {
      ctl.commandMouthTarget(plan.mouthAtReachStart);
      if(iter_ % logEvery_ == 1)
      {
        mc_rtc::log::info(
            "[PredictiveReachWait] timeToExactReachStart={:+.3f}s objectModelError={:.4f} gripperMeasured={:.3f} noMotion=true",
            reachStartTime_ - now, objectPositionError,
            ctl.measuredGripperClosure());
      }
      return false;
    }

    const double launchTimingError = now - reachStartTime_;
    if(launchTimingError > launchTimingTolerance_)
    {
      ctl.commandMouthTarget(ctl.actualMouthPose());
      ctl.endObjectObservation(true);
      mc_rtc::log::error(
          "[PredictiveReach] exact committed launch missed lateness={:+.3f}s tolerance={:.3f}s; no motion",
          launchTimingError, launchTimingTolerance_);
      output("FAIL");
      return true;
    }

    startPose_ = ctl.actualMouthPose();
    const double launchAnchorError = (
        startPose_.translation() - plan.mouthAtReachStart.translation()).norm();
    const double launchAnchorAngle = ctl.orientationError(
        startPose_, plan.mouthAtReachStart);
    if(launchAnchorError > posTol_ || launchAnchorAngle > oriTol_)
    {
      ctl.commandMouthTarget(startPose_);
      ctl.endObjectObservation(true);
      mc_rtc::log::error(
          "[PredictiveReach] exact launch anchor mismatch position={:.4f} orientation={:.4f}; no motion",
          launchAnchorError, launchAnchorAngle);
      output("FAIL");
      return true;
    }

    startTime_ = now;
    referencePose_ = plan.mouthAtReachStart;
    previousMouthPose_ = startPose_;
    previousSampleTime_ = now;
    havePreviousSample_ = false;
    ctl.commandSelectedReachPosture(0.0, holdArmPosture_);
    ctl.startPhaseTiming("reach");
    phaseTimingStarted_ = true;
    motionStarted_ = true;
    mc_rtc::log::success(
        "[PredictiveReachLaunch] exact committed reach started timingError={:+.4f}s duration={:.3f}s candidate={} route={} noRetiming=true",
        launchTimingError, reachEndTime_ - reachStartTime_,
        ctl.selectedCandidateName(), plan.transitRouteName);
  }

  const double referenceTime = std::min(now, plan.standoffTime);
  auto reference = ctl.committedReferenceAt(
      referenceTime, ctl.controlDt(), false, 0.0, 0.0);
  if(now >= plan.standoffTime)
  {
    reference.mouthLinearVelocityWorld.setZero();
    reference.mouthAngularVelocityWorld.setZero();
  }
  ctl.commandSelectedReachPosture(reference.phaseProgress, holdArmPosture_);
  if(!decelerationLogged_ && now >= plan.decelerationStartTime)
  {
    decelerationLogged_ = true;
    mc_rtc::log::success(
        "[PresentationProfile] terminal deceleration started timeToPresentation={:.3f}s objectSpeedRef={:.4f}m/s",
        plan.presentationTime - now,
        reference.objectLinearVelocityWorld.norm());
  }

  const sva::PTransformd current = ctl.actualMouthPose();

  // V5.2.5: the committed geometric plan is unchanged, and the physical
  // tracking command is governed by the live non-contact clearance. This is
  // bounded terminal feedback inside the committed tube, not replanning.
  HandoverSafetyReport currentReport;
  if(!ctl.evaluateCurrentPoseSafety(currentReport, false))
  {
    ctl.commandMouthTarget(current);
    ctl.endObjectObservation(true);
    mc_rtc::log::error(
        "[PredictiveReachGovernor] current pose already unsafe clear={:.4f} limiting={}/{}; no physical retry",
        currentReport.minClearance, currentReport.sample,
        currentReport.obstacle);
    output("FAIL");
    return true;
  }

  minimumObservedClearance_ = std::min(
      minimumObservedClearance_, currentReport.minClearance);
  if(currentReport.minClearance < minimumRuntimeClearance_)
  {
    ctl.commandMouthTarget(current);
    ctl.endObjectObservation(true);
    mc_rtc::log::error(
        "[PredictiveReachGovernor] dynamic clearance reserve exhausted clear={:.4f} minimum={:.4f} limiting={}/{}; fail-safe hold",
        currentReport.minClearance, minimumRuntimeClearance_,
        currentReport.sample, currentReport.obstacle);
    output("FAIL");
    return true;
  }

  double rawClearanceScale = 1.0;
  if(currentReport.minClearance < clearanceSlowdownStart_)
  {
    const double denominator = std::max(
        1e-6, clearanceSlowdownStart_ - clearanceHardMargin_);
    const double u = std::min(
        1.0, std::max(0.0,
        (currentReport.minClearance - clearanceHardMargin_) / denominator));
    const double smooth = u * u * (3.0 - 2.0 * u);
    rawClearanceScale = minimumVelocityScale_
        + (1.0 - minimumVelocityScale_) * smooth;
  }

  const double scaleRate = rawClearanceScale < clearanceScale_
                         ? clearanceScaleDropRate_
                         : clearanceScaleRiseRate_;
  const double maximumScaleChange = scaleRate * ctl.controlDt();
  const double scaleError = rawClearanceScale - clearanceScale_;
  clearanceScale_ += std::max(
      -maximumScaleChange, std::min(maximumScaleChange, scaleError));
  clearanceScale_ = std::min(1.0, std::max(
      minimumVelocityScale_, clearanceScale_));

  const double linearSpeedLimit = nearLinearSpeed_
      + clearanceScale_ * (farLinearSpeed_ - nearLinearSpeed_);
  const double angularSpeedLimit = nearAngularSpeed_
      + clearanceScale_ * (farAngularSpeed_ - nearAngularSpeed_);
  const double linearLeadLimit = nearLinearTrackingLead_
      + clearanceScale_
        * (maxLinearTrackingLead_ - nearLinearTrackingLead_);
  const double angularLeadLimit = nearAngularTrackingLead_
      + clearanceScale_
        * (maxAngularTrackingLead_ - nearAngularTrackingLead_);

  // Rate-limit the committed reference, then apply a final hard projection
  // around the measured mouth. V5.2 could log a reference lead larger than
  // its configured cap because it advanced once more after the projection.
  // The command below is mathematically guaranteed to remain inside the live
  // lead tube on every cycle.
  const sva::PTransformd rateLimitedReference = ctl.advancePoseReference(
      referencePose_, reference.mouthPose,
      linearSpeedLimit, angularSpeedLimit);
  const sva::PTransformd nextReference = ctl.boundedPoseStep(
      current, rateLimitedReference,
      linearLeadLimit, angularLeadLimit);

  sva::PTransformd safeReference;
  HandoverSafetyReport report;
  if(!ctl.filterSafeMouthCommand(
         current, nextReference, safeReference, report, false))
  {
    ctl.commandMouthTarget(current);
    ctl.endObjectObservation(true);
    mc_rtc::log::error(
        "[PredictiveReach] runtime safety mismatch clear={:.4f} limiting={}/{}; no physical retry",
        report.minClearance, report.sample, report.obstacle);
    output("FAIL");
    return true;
  }

  const sva::PTransformd previousCommand = referencePose_;
  referencePose_ = safeReference;
  Eigen::Vector3d commandedLinearVelocityWorld = Eigen::Vector3d::Zero();
  Eigen::Vector3d commandedAngularVelocityWorld = Eigen::Vector3d::Zero();
  ctl.worldPoseTwist(
      previousCommand, referencePose_, ctl.controlDt(),
      commandedLinearVelocityWorld, commandedAngularVelocityWorld);
  const double commandedLinearSpeed = commandedLinearVelocityWorld.norm();
  if(commandedLinearSpeed > linearSpeedLimit && commandedLinearSpeed > 1e-12)
  {
    commandedLinearVelocityWorld *= linearSpeedLimit / commandedLinearSpeed;
  }
  const double commandedAngularSpeed = commandedAngularVelocityWorld.norm();
  if(commandedAngularSpeed > angularSpeedLimit && commandedAngularSpeed > 1e-12)
  {
    commandedAngularVelocityWorld *= angularSpeedLimit / commandedAngularSpeed;
  }
  ctl.commandMouthTargetWithWorldVelocity(
      referencePose_, commandedLinearVelocityWorld,
      commandedAngularVelocityWorld);

  double mouthLinearSpeed = 0.0;
  double relativeLinearSpeed = 0.0;
  if(havePreviousSample_)
  {
    const double sampleDt = std::max(1e-9, now - previousSampleTime_);
    const Eigen::Vector3d mouthVelocity =
        (current.translation() - previousMouthPose_.translation()) / sampleDt;
    mouthLinearSpeed = mouthVelocity.norm();
    relativeLinearSpeed = (
        mouthVelocity - commandedLinearVelocityWorld).norm();
  }
  else { havePreviousSample_ = true; }
  previousMouthPose_ = current;
  previousSampleTime_ = now;

  const double dist = (goalPose_.translation() - current.translation()).norm();
  const double angle = ctl.orientationError(current, goalPose_);
  const double robotTravel =
      (current.translation() - startPose_.translation()).norm();
  const double timeToPresentation = reachEndTime_ - now;

  if(iter_ % logEvery_ == 1)
  {
    mc_rtc::log::info(
        "[PredictiveReach] timeToPresentation={:+.3f}s s={:.3f} scheduledDist={:.4f} terminalDist={:.4f} angle={:.4f} robotTravel={:.4f} objectModelError={:.4f} objectSpeedRef={:.4f} ffLinear={:.4f} mouthLinear={:.4f} relativeLinear={:.4f} gripperCommand={:.3f} gripperMeasured={:.3f} refLead={:.4f} clear={:.4f} governorScale={:.3f} speedCap={:.3f} leadCap={:.4f} minClear={:.4f}",
        timeToPresentation, reference.phaseProgress,
        (reference.mouthPose.translation() - current.translation()).norm(),
        dist, angle, robotTravel, objectPositionError,
        reference.objectLinearVelocityWorld.norm(),
        commandedLinearVelocityWorld.norm(), mouthLinearSpeed,
        relativeLinearSpeed, ctl.gripperCommand(),
        ctl.measuredGripperClosure(),
        (referencePose_.translation() - current.translation()).norm(),
        report.minClearance, clearanceScale_, linearSpeedLimit,
        linearLeadLimit, minimumObservedClearance_);
  }

  const bool armAtStandoff = now >= reachEndTime_
      && dist <= posTol_ && angle <= oriTol_;
  if(armAtStandoff)
  {
    mc_rtc::log::success(
        "[PredictiveReach] future standoff reached candidate={} route={} scheduleError={:+.3f}s dist={:.4f} angle={:.4f} robotTravel={:.4f} objectSpeed={:.4f} gripperOpenMeasured={:.3f}",
        ctl.selectedCandidateName(), plan.transitRouteName,
        now - reachEndTime_, dist, angle,
        robotTravel, ctl.objectLinearVelocityEstimate().norm(),
        ctl.measuredGripperClosure());
    mc_rtc::log::success(
        "[PredictiveReachGovernor] completed committed reach minimumClearance={:.4f} finalScale={:.3f} hardLeadCap=true noRetiming=true noReplanning=true",
        minimumObservedClearance_, clearanceScale_);
    output("OK");
    return true;
  }

  if(now > reachEndTime_ + scheduleLatenessTolerance_)
  {
    ctl.commandMouthTarget(current);
    ctl.endObjectObservation(true);
    mc_rtc::log::error(
        "[PredictiveReach] missed committed presentation standoff lateness={:.3f}s dist={:.4f} angle={:.4f}",
        now - reachEndTime_, dist, angle);
    output("FAIL");
    return true;
  }

  return false;
}

void HandoverInterceptionController_ExecuteCommittedReach::teardown(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  if(phaseTimingStarted_) { ctl.finishPhaseTiming("reach"); }
  mc_rtc::log::info("[PredictiveReach] teardown");
}

EXPORT_SINGLE_STATE("HandoverInterceptionController_ExecuteCommittedReach",
                    HandoverInterceptionController_ExecuteCommittedReach)
