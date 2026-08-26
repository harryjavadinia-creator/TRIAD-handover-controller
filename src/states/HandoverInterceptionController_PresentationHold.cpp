#include "HandoverInterceptionController_PresentationHold.h"
#include "../HandoverInterceptionController.h"

#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>

void HandoverInterceptionController_PresentationHold::configure(
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
  if(config.has("posTol")) { config("posTol", posTol_); }
  if(config.has("oriTol")) { config("oriTol", oriTol_); }
  if(config.has("maximumOpenClosure"))
  {
    config("maximumOpenClosure", maximumOpenClosure_);
  }
  if(config.has("stableDwell")) { config("stableDwell", stableDwell_); }
  if(config.has("reserveMargin"))
  {
    config("reserveMargin", reserveMargin_);
  }
  if(config.has("taskStiffness"))
  {
    config("taskStiffness", taskStiffness_);
  }
  if(config.has("taskWeight")) { config("taskWeight", taskWeight_); }
  if(config.has("maximumObjectTranslationDeviation"))
  {
    config("maximumObjectTranslationDeviation",
           maximumObjectTranslationDeviation_);
  }
  if(config.has("maximumObjectRotationDeviation"))
  {
    config("maximumObjectRotationDeviation",
           maximumObjectRotationDeviation_);
  }
  if(config.has("logEvery")) { config("logEvery", logEvery_); }

  maxLinearTrackingLead_ = std::max(0.001, maxLinearTrackingLead_);
  maxAngularTrackingLead_ = std::max(0.001, maxAngularTrackingLead_);
  posTol_ = std::max(0.001, posTol_);
  oriTol_ = std::max(0.001, oriTol_);
  maximumOpenClosure_ = std::max(0.0, maximumOpenClosure_);
  stableDwell_ = std::max(0.0, stableDwell_);
  reserveMargin_ = std::max(0.0, reserveMargin_);
  maximumObjectTranslationDeviation_ = std::max(
      0.0, maximumObjectTranslationDeviation_);
  maximumObjectRotationDeviation_ = std::max(
      0.0, maximumObjectRotationDeviation_);
  logEvery_ = std::max<uint64_t>(1, logEvery_);
}

void HandoverInterceptionController_PresentationHold::start(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ready_ = false;
  iter_ = 0;
  stableTime_ = 0.0;

  if(!ctl.committedPlanValid() || !ctl.hasSelectedCandidate())
  {
    mc_rtc::log::error(
        "[PresentationHold] committed presentation plan unavailable");
    return;
  }

  const auto & plan = ctl.committedInterceptionPlan();
  if(!plan.presentationMode)
  {
    mc_rtc::log::error(
        "[PresentationHold] clean controller requires presentation mode");
    return;
  }

  holdPose_ = ctl.committedStandoffTargetAt(plan.presentationTime);
  latestReleaseTime_ = plan.acquisitionDeadlineTime
                     - plan.approachDuration
                     - plan.acquireDuration
                     - plan.confirmationDuration
                     - reserveMargin_;
  if(latestReleaseTime_ <= ctl.controllerTime())
  {
    mc_rtc::log::error(
        "[PresentationHold] no time remains for certified static terminal execution current={:.3f}s latestRelease={:.3f}s deadline={:.3f}s",
        ctl.controllerTime(), latestReleaseTime_,
        plan.acquisitionDeadlineTime);
    return;
  }

  ctl.startPhaseTiming("presentation_hold");
  ctl.activateToolTask();
  ctl.setToolTaskGains(taskStiffness_, taskWeight_);
  ctl.commandSelectedStandoffPosture();
  ctl.setGripperClosureAuthorized(false);
  ctl.setGripperJointPriority(false);
  ctl.commandGripper(0.0);
  ctl.commandMouthTarget(holdPose_);

  mc_rtc::log::warning(
      "[PresentationHold] holding the committed standoff with gripper fully open until stopped-object gates pass candidate={} latestRelease={:.3f}s deadline={:.3f}s",
      ctl.selectedCandidateName(), latestReleaseTime_,
      plan.acquisitionDeadlineTime);
  ready_ = true;
}

bool HandoverInterceptionController_PresentationHold::run(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ++iter_;

  if(!ready_ || !ctl.committedPlanValid() || !ctl.hasSelectedCandidate())
  {
    ctl.endObjectObservation(true);
    mc_rtc::log::error(
        "[PresentationHold] committed presentation plan lost");
    output("FAIL");
    return true;
  }

  const auto & plan = ctl.committedInterceptionPlan();
  const double now = ctl.controllerTime();
  const double objectPositionError = ctl.committedObjectPositionErrorAt(now);
  const double objectOrientationError =
      ctl.committedObjectOrientationErrorAt(now);
  const double objectLinearSpeed =
      ctl.objectLinearVelocityEstimate().norm();
  const double objectAngularSpeed =
      ctl.objectAngularVelocityEstimate().norm();

  if(objectPositionError > maximumObjectTranslationDeviation_
     || objectOrientationError > maximumObjectRotationDeviation_)
  {
    ctl.commandMouthTarget(ctl.actualMouthPose());
    ctl.endObjectObservation(true);
    mc_rtc::log::error(
        "[PresentationHold] object left committed presentation tube positionError={:.4f} rotationError={:.4f}; no replanning",
        objectPositionError, objectOrientationError);
    output("FAIL");
    return true;
  }

  ctl.activateToolTask();
  ctl.setToolTaskGains(taskStiffness_, taskWeight_);
  ctl.commandSelectedStandoffPosture();
  ctl.setGripperClosureAuthorized(false);
  ctl.setGripperJointPriority(false);
  ctl.commandGripper(0.0);

  const sva::PTransformd current = ctl.actualMouthPose();
  const sva::PTransformd nextReference = ctl.boundedPoseStep(
      current, holdPose_, maxLinearTrackingLead_, maxAngularTrackingLead_);
  sva::PTransformd safeReference;
  HandoverSafetyReport report;
  if(!ctl.filterSafeMouthCommand(
         current, nextReference, safeReference, report, false))
  {
    ctl.commandMouthTarget(current);
    ctl.endObjectObservation(true);
    mc_rtc::log::error(
        "[PresentationHold] standoff hold became unsafe clear={:.4f} limiting={}/{}",
        report.minClearance, report.sample, report.obstacle);
    output("FAIL");
    return true;
  }
  ctl.commandMouthTargetWithWorldVelocity(
      safeReference, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  const double dist = (holdPose_.translation() - current.translation()).norm();
  const double angle = ctl.orientationError(current, holdPose_);
  const double measuredClosure = ctl.measuredGripperClosure();
  const bool objectStopped =
      objectLinearSpeed <= ctl.presentationMaximumLinearSpeed()
      && objectAngularSpeed <= ctl.presentationMaximumAngularSpeed();
  const bool gripperOpen = measuredClosure <= maximumOpenClosure_;
  const bool gate = now >= plan.presentationTime
      && dist <= posTol_ && angle <= oriTol_
      && objectStopped && gripperOpen;

  if(gate) { stableTime_ += ctl.controlDt(); }
  else { stableTime_ = 0.0; }

  if(iter_ % logEvery_ == 1)
  {
    mc_rtc::log::info(
        "[PresentationHold] timeToLatestRelease={:+.3f}s stable={:.3f}/{:.3f}s dist={:.4f} angle={:.4f} objectLinear={:.4f} objectAngular={:.4f} modelError={:.4f} gripperCommand={:.3f} gripperMeasured={:.3f} open={} clear={:.4f}",
        latestReleaseTime_ - now, stableTime_, stableDwell_,
        dist, angle, objectLinearSpeed, objectAngularSpeed,
        objectPositionError, ctl.gripperCommand(), measuredClosure,
        gripperOpen, report.minClearance);
  }

  if(stableTime_ >= stableDwell_)
  {
    // Realize the immutable object-relative candidate at the measured stopped
    // pose, then freeze that pose. This bounded correction is not a replan.
    if(!ctl.lockStaticPresentationToCurrentObject())
    {
      ctl.commandMouthTarget(current);
      ctl.endObjectObservation(true);
      output("FAIL");
      return true;
    }
    ctl.endObjectObservation(true);
    mc_rtc::log::success(
        "[PresentationGate] stopped object, committed standoff and fully open gripper accepted candidate={} releaseTime={:.3f}s timeBeforeLatestRelease={:.3f}s dist={:.4f} angle={:.4f} objectLinear={:.4f} gripperMeasured={:.3f}",
        ctl.selectedCandidateName(), now, latestReleaseTime_ - now,
        dist, angle, objectLinearSpeed, measuredClosure);
    output("OK");
    return true;
  }

  if(now > latestReleaseTime_)
  {
    ctl.commandMouthTarget(current);
    ctl.endObjectObservation(true);
    mc_rtc::log::error(
        "[PresentationHold] presentation gates did not settle before reserved static terminal deadline dist={:.4f} angle={:.4f} objectLinear={:.4f} objectAngular={:.4f} gripperMeasured={:.3f}",
        dist, angle, objectLinearSpeed, objectAngularSpeed,
        measuredClosure);
    output("FAIL");
    return true;
  }

  return false;
}

void HandoverInterceptionController_PresentationHold::teardown(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ctl.finishPhaseTiming("presentation_hold");
  mc_rtc::log::info("[PresentationHold] teardown");
}

EXPORT_SINGLE_STATE("HandoverInterceptionController_PresentationHold",
                    HandoverInterceptionController_PresentationHold)
