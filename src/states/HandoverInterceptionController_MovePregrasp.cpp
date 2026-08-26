#include "HandoverInterceptionController_MovePregrasp.h"
#include "../HandoverInterceptionController.h"

#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
double clamp01Approach(double value)
{
  return std::max(0.0, std::min(1.0, value));
}

double projectedTranslationProgressApproach(
    const sva::PTransformd & start,
    const sva::PTransformd & goal,
    const sva::PTransformd & current)
{
  const Eigen::Vector3d path = goal.translation() - start.translation();
  const double squaredLength = path.squaredNorm();
  if(squaredLength <= 1e-12) { return 1.0; }
  return clamp01Approach(
      (current.translation() - start.translation()).dot(path)
      / squaredLength);
}

double allowanceProgressApproach(double allowance, double total)
{
  if(total <= 1e-9) { return std::numeric_limits<double>::infinity(); }
  return std::max(0.0, allowance) / total;
}


double taperedLookAheadApproach(double maximum, double minimum,
                                double remaining, double tolerance,
                                double taperDistance)
{
  const double maxValue = std::max(0.0, maximum);
  const double minValue = std::min(maxValue, std::max(0.0, minimum));
  const double start = std::max(tolerance + 1e-6, taperDistance);
  if(remaining >= start) { return maxValue; }
  const double alpha = clamp01Approach(
      (remaining - tolerance) / std::max(1e-9, start - tolerance));
  return minValue + alpha * (maxValue - minValue);
}

double smoothStepApproach(double value)
{
  const double x = clamp01Approach(value);
  return x * x * (3.0 - 2.0 * x);
}

double rampSpeedApproach(double current, double desired,
                         double acceleration, double dt)
{
  const double step = std::max(0.0, acceleration) * std::max(0.0, dt);
  return current + std::max(-step, std::min(step, desired - current));
}

/**
 * Couple translation and orientation progress only when the orientation
 * correction is materially larger than the terminal tolerance. A tiny
 * tolerance excess still has to be corrected before Acquire, but it must not
 * throttle a long translational insertion from the first cycle.
 */
double activePathProgressApproach(double linearProgress,
                                  double angularProgress,
                                  bool linearCoupled,
                                  bool angularCoupled)
{
  if(linearCoupled && angularCoupled)
  {
    return std::min(linearProgress, angularProgress);
  }
  if(linearCoupled) { return linearProgress; }
  if(angularCoupled) { return angularProgress; }
  return 1.0;
}

double activeLookAheadProgressApproach(double linearLookAheadProgress,
                                       double angularLookAheadProgress,
                                       bool linearCoupled,
                                       bool angularCoupled)
{
  if(linearCoupled && angularCoupled)
  {
    return std::min(linearLookAheadProgress, angularLookAheadProgress);
  }
  if(linearCoupled) { return linearLookAheadProgress; }
  if(angularCoupled) { return angularLookAheadProgress; }
  return 1.0;
}
}

void HandoverInterceptionController_MovePregrasp::configure(
    const mc_rtc::Configuration & config)
{
  if(config.has("farLinearSpeed")) { config("farLinearSpeed", farLinearSpeed_); }
  if(config.has("nearLinearSpeed")) { config("nearLinearSpeed", nearLinearSpeed_); }
  if(config.has("nearDistance")) { config("nearDistance", nearDistance_); }
  if(config.has("angularSpeed")) { config("angularSpeed", angularSpeed_); }
  if(config.has("linearAcceleration"))
  {
    config("linearAcceleration", linearAcceleration_);
  }
  if(config.has("angularAcceleration"))
  {
    config("angularAcceleration", angularAcceleration_);
  }
  if(config.has("spatialLinearLookAhead"))
  {
    config("spatialLinearLookAhead", spatialLinearLookAhead_);
  }
  if(config.has("spatialAngularLookAhead"))
  {
    config("spatialAngularLookAhead", spatialAngularLookAhead_);
  }
  if(config.has("minimumLinearLookAhead"))
  {
    config("minimumLinearLookAhead", minimumLinearLookAhead_);
  }
  if(config.has("minimumAngularLookAhead"))
  {
    config("minimumAngularLookAhead", minimumAngularLookAhead_);
  }
  if(config.has("lookAheadRemainingRatio"))
  {
    config("lookAheadRemainingRatio", lookAheadRemainingRatio_);
  }
  if(config.has("lookAheadTaperDistance"))
  {
    config("lookAheadTaperDistance", lookAheadTaperDistance_);
  }
  if(config.has("linearFeedforwardRatio"))
  {
    config("linearFeedforwardRatio", linearFeedforwardRatio_);
  }
  if(config.has("angularFeedforwardRatio"))
  {
    config("angularFeedforwardRatio", angularFeedforwardRatio_);
  }
  if(config.has("terminalLinearDeceleration"))
  {
    config("terminalLinearDeceleration", terminalLinearDeceleration_);
  }
  if(config.has("terminalAngularDeceleration"))
  {
    config("terminalAngularDeceleration", terminalAngularDeceleration_);
  }
  if(config.has("terminalBrakeMargin"))
  {
    config("terminalBrakeMargin", terminalBrakeMargin_);
  }
  if(config.has("terminalLinearSpeedTolerance"))
  {
    config("terminalLinearSpeedTolerance", terminalLinearSpeedTolerance_);
  }
  if(config.has("terminalAngularSpeedTolerance"))
  {
    config("terminalAngularSpeedTolerance", terminalAngularSpeedTolerance_);
  }
  if(config.has("terminalStableDwell"))
  {
    config("terminalStableDwell", terminalStableDwell_);
  }
  if(config.has("terminalVelocityFilter"))
  {
    config("terminalVelocityFilter", terminalVelocityFilter_);
  }
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
  if(config.has("centeringTol")) { config("centeringTol", centeringTol_); }
  if(config.has("maximumOpenClosure"))
  {
    config("maximumOpenClosure", maximumOpenClosure_);
  }
  if(config.has("taskStiffness")) { config("taskStiffness", taskStiffness_); }
  if(config.has("taskWeight")) { config("taskWeight", taskWeight_); }
  if(config.has("progressEps")) { config("progressEps", progressEps_); }
  if(config.has("progressLimit")) { config("progressLimit", progressLimit_); }
  if(config.has("maxIter")) { config("maxIter", maxIter_); }

  maximumOpenClosure_ = std::max(0.0, maximumOpenClosure_);
  spatialLinearLookAhead_ = std::max(0.001, spatialLinearLookAhead_);
  spatialAngularLookAhead_ = std::max(0.001, spatialAngularLookAhead_);
  minimumLinearLookAhead_ = std::min(
      spatialLinearLookAhead_, std::max(0.0005, minimumLinearLookAhead_));
  minimumAngularLookAhead_ = std::min(
      spatialAngularLookAhead_, std::max(0.001, minimumAngularLookAhead_));
  lookAheadRemainingRatio_ = std::max(
      0.05, std::min(1.0, lookAheadRemainingRatio_));
  lookAheadTaperDistance_ = std::max(
      posTol_ + 0.001, lookAheadTaperDistance_);
  linearFeedforwardRatio_ = clamp01Approach(linearFeedforwardRatio_);
  angularFeedforwardRatio_ = clamp01Approach(angularFeedforwardRatio_);
  terminalLinearDeceleration_ = std::max(0.05, terminalLinearDeceleration_);
  terminalAngularDeceleration_ = std::max(0.05, terminalAngularDeceleration_);
  terminalBrakeMargin_ = std::max(0.0, terminalBrakeMargin_);
  terminalLinearSpeedTolerance_ = std::max(0.0, terminalLinearSpeedTolerance_);
  terminalAngularSpeedTolerance_ = std::max(0.0, terminalAngularSpeedTolerance_);
  terminalStableDwell_ = std::max(0.0, terminalStableDwell_);
  terminalVelocityFilter_ = clamp01Approach(terminalVelocityFilter_);
  maxLinearTrackingLead_ = std::max(
      spatialLinearLookAhead_, std::max(0.001, maxLinearTrackingLead_));
  maxAngularTrackingLead_ = std::max(
      spatialAngularLookAhead_, std::max(0.001, maxAngularTrackingLead_));
  linearAcceleration_ = std::max(0.01, linearAcceleration_);
  angularAcceleration_ = std::max(0.01, angularAcceleration_);
}

void HandoverInterceptionController_MovePregrasp::start(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ctl.startPhaseTiming("approach");
  iter_ = 0;
  stagnantCycles_ = 0;
  bestCombinedError_ = 1e9;
  commandedLinearSpeed_ = 0.0;
  commandedAngularSpeed_ = 0.0;
  referenceProgress_ = 0.0;
  measuredProgressAnchor_ = 0.0;
  peakMeasuredLinearSpeed_ = 0.0;
  peakMeasuredAngularSpeed_ = 0.0;
  filteredLinearSpeed_ = 0.0;
  filteredAngularSpeed_ = 0.0;
  dynamicBrakeDistance_ = 0.0;
  dynamicBrakeAngle_ = 0.0;
  terminalStableTime_ = 0.0;
  linearBrakingActive_ = false;
  angularBrakingActive_ = false;
  haveVelocityEstimate_ = false;

  ctl.setGripperClosureAuthorized(false);
  ctl.activateToolTask();
  ctl.setToolTaskGains(taskStiffness_, taskWeight_);
  ctl.commandSelectedArmPosture();
  ctl.setGripperJointPriority(false);
  ctl.commandGripper(0.0);

  startPose_ = ctl.actualMouthPose();
  goalPose_ = ctl.currentMouthPregraspTarget();
  referencePose_ = startPose_;
  const Eigen::Vector3d linearPath =
      goalPose_.translation() - startPose_.translation();
  pathDistance_ = linearPath.norm();
  linearPathDirectionWorld_.setZero();
  if(pathDistance_ > 1e-9)
  {
    linearPathDirectionWorld_ = linearPath / pathDistance_;
  }
  pathAngle_ = ctl.orientationError(startPose_, goalPose_);
  linearProgressCoupled_ = pathDistance_ > posTol_;
  angularCorrectionRequired_ = pathAngle_ > oriTol_;
  angularProgressCoupled_ =
      pathAngle_ > oriTol_ + minimumAngularLookAhead_;
  Eigen::Vector3d unusedLinear;
  Eigen::Vector3d angularPath;
  ctl.worldPoseTwist(startPose_, goalPose_, 1.0, unusedLinear, angularPath);
  angularPathDirectionWorld_.setZero();
  if(angularPath.norm() > 1e-9)
  {
    angularPathDirectionWorld_ = angularPath.normalized();
  }
  previousActualPose_ = startPose_;
  ctl.commandMouthTarget(referencePose_);

  const Eigen::Vector3d p = goalPose_.translation();
  mc_rtc::log::warning(
      "[GuardedApproach] velocity-gated terminal-capture insertion {} toward [{:.3f},{:.3f},{:.3f}] far={:.2f}m/s near={:.2f}m/s lookAhead=[{:.3f}m,{:.3f}rad] feedforward=[{:.2f},{:.2f}] brake=[a:{:.2f}m/s2 margin:{:.3f}m] gate=[v:{:.3f}m/s w:{:.3f}rad/s dwell:{:.3f}s]",
      ctl.selectedCandidateName(), p.x(), p.y(), p.z(),
      farLinearSpeed_, nearLinearSpeed_, spatialLinearLookAhead_,
      spatialAngularLookAhead_, linearFeedforwardRatio_,
      angularFeedforwardRatio_, terminalLinearDeceleration_,
      terminalBrakeMargin_, terminalLinearSpeedTolerance_,
      terminalAngularSpeedTolerance_, terminalStableDwell_);
  mc_rtc::log::success(
      "[SpatialProgressContract] phase=approach linearCoupled={} angularCoupled={} angularCorrectionRequired={} pathDistance={:.4f} pathAngle={:.4f} posTol={:.4f} oriTol={:.4f} angularActivationMargin={:.4f} activeDimensionGate=true monotonicAnchor=true tangentFeedforward=true terminalVelocityGate=true modalitySeparatedBrake=true",
      linearProgressCoupled_, angularProgressCoupled_,
      angularCorrectionRequired_, pathDistance_, pathAngle_, posTol_, oriTol_,
      minimumAngularLookAhead_);
  mc_rtc::log::success(
      "[DecoupledTerminalProgress] V5.4.1 insignificant angular tolerance excess cannot activate translational braking; linear and angular brake channels are independent");
}

bool HandoverInterceptionController_MovePregrasp::run(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ++iter_;
  ctl.setGripperClosureAuthorized(false);

  if(!ctl.hasSelectedCandidate())
  {
    mc_rtc::log::error("[GuardedApproach] no certified candidate available");
    output("FAIL");
    return true;
  }

  if(ctl.committedPlanValid()
     && ctl.controllerTime() > ctl.committedAcquisitionDeadline())
  {
    mc_rtc::log::error(
        "[GuardedApproach] bounded static acquisition deadline expired before capture; no retry");
    output("FAIL");
    return true;
  }

  ctl.commandSelectedArmPosture();
  ctl.setGripperJointPriority(false);
  ctl.commandGripper(0.0);
  const sva::PTransformd current = ctl.actualMouthPose();
  Eigen::Vector3d measuredLinearVelocity;
  Eigen::Vector3d measuredAngularVelocity;
  ctl.worldPoseTwist(previousActualPose_, current, ctl.controlDt(),
                     measuredLinearVelocity, measuredAngularVelocity);
  previousActualPose_ = current;
  peakMeasuredLinearSpeed_ = std::max(
      peakMeasuredLinearSpeed_, measuredLinearVelocity.norm());
  peakMeasuredAngularSpeed_ = std::max(
      peakMeasuredAngularSpeed_, measuredAngularVelocity.norm());
  const double measuredLinearSpeed = measuredLinearVelocity.norm();
  const double measuredAngularSpeed = measuredAngularVelocity.norm();
  if(!haveVelocityEstimate_)
  {
    filteredLinearSpeed_ = measuredLinearSpeed;
    filteredAngularSpeed_ = measuredAngularSpeed;
    haveVelocityEstimate_ = true;
  }
  else
  {
    filteredLinearSpeed_ = (1.0 - terminalVelocityFilter_)
                         * filteredLinearSpeed_
                         + terminalVelocityFilter_ * measuredLinearSpeed;
    filteredAngularSpeed_ = (1.0 - terminalVelocityFilter_)
                          * filteredAngularSpeed_
                          + terminalVelocityFilter_ * measuredAngularSpeed;
  }
  const double measuredClosure = ctl.measuredGripperClosure();
  if(measuredClosure > maximumOpenClosure_)
  {
    referencePose_ = current;
    commandedLinearSpeed_ = 0.0;
    commandedAngularSpeed_ = 0.0;
    ctl.commandMouthTarget(current);
    if(iter_ % 100 == 1)
    {
      mc_rtc::log::warning(
          "[GuardedApproach] waiting at fixed standoff for fully open gripper measured={:.3f} limit={:.3f}",
          measuredClosure, maximumOpenClosure_);
    }
    return false;
  }

  const double dist = (goalPose_.translation() - current.translation()).norm();
  const double angle = ctl.orientationError(current, goalPose_);

  ctl.refreshGripperGeometry(false);
  HandoverSafetyReport captureReport;
  const bool captureSafe = ctl.evaluateCurrentClosureSafety(captureReport);
  const bool positionReady = dist <= posTol_;
  const bool orientationReady = angle <= oriTol_;
  const bool centeringReady =
      captureSafe && captureReport.padCenteringError <= centeringTol_;
  const bool entryReady = captureReport.corridorEntryClearance >= -1e-9;
  const bool palmReady = captureReport.corridorPalmClearance >= -1e-9;
  const bool axialReady = captureReport.corridorAxialClearance >= -1e-9;
  const bool axisReady = captureReport.corridorAngleClearance >= -1e-9;
  const bool linearVelocityReady =
      filteredLinearSpeed_ <= terminalLinearSpeedTolerance_;
  const bool angularVelocityReady =
      filteredAngularSpeed_ <= terminalAngularSpeedTolerance_;

  const bool terminalGeometryReady =
      positionReady && orientationReady && centeringReady;
  const bool terminalVelocityReady =
      linearVelocityReady && angularVelocityReady;

  const bool auditCandidate =
      ctl.selectedCandidateName() == "axisP_side_337deg";
  const bool nearTerminal = dist <= std::max(0.03, 3.0 * posTol_);
  if(auditCandidate && nearTerminal && (iter_ % 50 == 1))
  {
    mc_rtc::log::info(
        "[R1TerminalGateAudit] source=runtime candidate={} posReady={} oriReady={} captureSafe={} centeringReady={} entryReady={} palmReady={} axialReady={} axisReady={} linearVelocityReady={} angularVelocityReady={} geometryReady={} velocityReady={} stable={:.3f}/{:.3f}s dist={:.4f}/{:.4f} angle={:.4f}/{:.4f} centerErr={:.4f}/{:.4f} corridor=[entry:{:.4f},palm:{:.4f},axial:{:.4f},axis:{:.4f}] speed=[v:{:.4f}/{:.4f},w:{:.4f}/{:.4f}] clear={:.4f} limiting={}/{}",
        ctl.selectedCandidateName(), positionReady, orientationReady,
        captureSafe, centeringReady, entryReady, palmReady, axialReady,
        axisReady, linearVelocityReady, angularVelocityReady,
        terminalGeometryReady, terminalVelocityReady, terminalStableTime_,
        terminalStableDwell_, dist, posTol_, angle, oriTol_,
        captureReport.padCenteringError, centeringTol_,
        captureReport.corridorEntryClearance,
        captureReport.corridorPalmClearance,
        captureReport.corridorAxialClearance,
        captureReport.corridorAngleClearance, filteredLinearSpeed_,
        terminalLinearSpeedTolerance_, filteredAngularSpeed_,
        terminalAngularSpeedTolerance_, captureReport.minClearance,
        captureReport.sample, captureReport.obstacle);
  }
  if(terminalGeometryReady)
  {
    terminalStableTime_ = terminalVelocityReady
        ? terminalStableTime_ + ctl.controlDt() : 0.0;
    stagnantCycles_ = 0;
    commandedLinearSpeed_ = rampSpeedApproach(
        commandedLinearSpeed_, 0.0, terminalLinearDeceleration_,
        ctl.controlDt());
    commandedAngularSpeed_ = rampSpeedApproach(
        commandedAngularSpeed_, 0.0, terminalAngularDeceleration_,
        ctl.controlDt());

    sva::PTransformd safeGoal;
    HandoverSafetyReport holdReport;
    if(!ctl.filterSafeMouthCommand(current, goalPose_, safeGoal,
                                   holdReport, true))
    {
      ctl.commandMouthTarget(current);
      mc_rtc::log::error(
          "[TerminalCaptureGate] static capture hold rejected clear={:.4f} limiting={}/{}; fail-safe hold",
          holdReport.minClearance, holdReport.sample, holdReport.obstacle);
      output("FAIL");
      return true;
    }
    referencePose_ = safeGoal;
    ctl.commandMouthTarget(referencePose_);

    if(iter_ % 20 == 1 || terminalStableTime_ >= terminalStableDwell_)
    {
      mc_rtc::log::info(
          "[TerminalCaptureGate] geometry=true velocity={} stable={:.3f}/{:.3f}s dist={:.4f} angle={:.4f} centerErr={:.4f} vFiltered={:.4f} wFiltered={:.4f} zeroFeedforward=true",
          terminalVelocityReady, terminalStableTime_,
          terminalStableDwell_, dist, angle,
          captureReport.padCenteringError, filteredLinearSpeed_,
          filteredAngularSpeed_);
    }

    if(terminalStableTime_ >= terminalStableDwell_)
    {
      // The final command explicitly owns the same committed capture pose with
      // zero reference motion. Acquire therefore starts from a pose- and
      // velocity-continuous handoff rather than dropping a live feedforward.
      ctl.commandMouthTarget(goalPose_);
      mc_rtc::log::success(
          "[GuardedApproach] velocity-gated centered capture reached candidate={} dist={:.4f} angle={:.4f} centerErr={:.4f} v={:.4f} w={:.4f} stable={:.3f}s clear={:.4f}",
          ctl.selectedCandidateName(), dist, angle,
          captureReport.padCenteringError, filteredLinearSpeed_,
          filteredAngularSpeed_, terminalStableTime_,
          captureReport.minClearance);
      output("OK");
      return true;
    }
    return false;
  }
  terminalStableTime_ = 0.0;

  const double centeringError =
      captureSafe ? captureReport.padCenteringError : 0.0;
  const double combinedError = dist + 0.08 * angle + centeringError;
  if(combinedError < bestCombinedError_ - progressEps_)
  {
    bestCombinedError_ = combinedError;
    stagnantCycles_ = 0;
  }
  else { ++stagnantCycles_; }

  if(stagnantCycles_ >= progressLimit_ || iter_ >= maxIter_)
  {
    mc_rtc::log::error(
        "[GuardedApproach] continuous spatial insertion stalled dist={:.4f} angle={:.4f} centerErr={:.4f}; no physical retry",
        dist, angle, centeringError);
    ctl.commandMouthTarget(current);
    output("FAIL");
    return true;
  }

  const double speedAlpha = nearDistance_ > 1e-9
      ? clamp01Approach(dist / nearDistance_) : 1.0;
  const double nominalLinearSpeed = nearLinearSpeed_
      + speedAlpha * (farLinearSpeed_ - nearLinearSpeed_);

  const double filteredLinearSquared =
      filteredLinearSpeed_ * filteredLinearSpeed_;
  const double terminalLinearSquared =
      terminalLinearSpeedTolerance_ * terminalLinearSpeedTolerance_;
  const double requiredLinearBrakeDistance = posTol_ + terminalBrakeMargin_
      + std::max(0.0, filteredLinearSquared - terminalLinearSquared)
          / (2.0 * terminalLinearDeceleration_);
  const double filteredAngularSquared =
      filteredAngularSpeed_ * filteredAngularSpeed_;
  const double terminalAngularSquared =
      terminalAngularSpeedTolerance_ * terminalAngularSpeedTolerance_;
  const double requiredAngularBrakeAngle = oriTol_
      + std::max(0.0, filteredAngularSquared - terminalAngularSquared)
          / (2.0 * terminalAngularDeceleration_);

  if(!linearBrakingActive_ && dist <= requiredLinearBrakeDistance)
  {
    linearBrakingActive_ = true;
    dynamicBrakeDistance_ = std::max(
        lookAheadTaperDistance_, requiredLinearBrakeDistance);
    mc_rtc::log::warning(
        "[TerminalCaptureBrakeLinear] engaged dist={:.4f} required={:.4f} vFiltered={:.4f} angle={:.4f} candidateUnchanged=true noRetiming=true",
        dist, dynamicBrakeDistance_, filteredLinearSpeed_, angle);
  }
  if(angularProgressCoupled_ && !angularBrakingActive_
     && angle <= requiredAngularBrakeAngle)
  {
    angularBrakingActive_ = true;
    dynamicBrakeAngle_ = std::max(
        oriTol_ + 0.001, requiredAngularBrakeAngle);
    mc_rtc::log::warning(
        "[TerminalCaptureBrakeAngular] engaged angle={:.4f} required={:.4f} wFiltered={:.4f} dist={:.4f} translationalBrakeUnchanged=true candidateUnchanged=true noRetiming=true",
        angle, dynamicBrakeAngle_, filteredAngularSpeed_, dist);
  }
  if(linearBrakingActive_)
  {
    dynamicBrakeDistance_ = std::max(
        dynamicBrakeDistance_, requiredLinearBrakeDistance);
  }
  if(angularBrakingActive_)
  {
    dynamicBrakeAngle_ = std::max(
        dynamicBrakeAngle_, requiredAngularBrakeAngle);
  }

  const double terminalLinearBudget = std::max(
      0.0, dist - posTol_ - terminalBrakeMargin_);
  const double terminalAngularBudget = std::max(0.0, angle - oriTol_);
  const double brakingLinearSpeed = std::sqrt(std::max(
      0.0, terminalLinearSquared
           + 2.0 * terminalLinearDeceleration_ * terminalLinearBudget));
  const double brakingAngularSpeed = std::sqrt(std::max(
      0.0, terminalAngularSquared
           + 2.0 * terminalAngularDeceleration_ * terminalAngularBudget));
  const double desiredLinearSpeed = linearBrakingActive_
      ? std::min(nominalLinearSpeed, brakingLinearSpeed)
      : nominalLinearSpeed;
  const double desiredAngularSpeed = angularBrakingActive_
      ? std::min(angularSpeed_, brakingAngularSpeed)
      : angularSpeed_;

  commandedLinearSpeed_ = rampSpeedApproach(
      commandedLinearSpeed_, desiredLinearSpeed,
      linearBrakingActive_ ? terminalLinearDeceleration_
                           : linearAcceleration_,
      ctl.controlDt());
  commandedAngularSpeed_ = rampSpeedApproach(
      commandedAngularSpeed_, desiredAngularSpeed,
      angularBrakingActive_ ? terminalAngularDeceleration_
                            : angularAcceleration_,
      ctl.controlDt());

  const double measuredLinearProgress = projectedTranslationProgressApproach(
      startPose_, goalPose_, current);
  const double measuredAngularProgress = pathAngle_ > 1e-9
      ? clamp01Approach(1.0 - angle / pathAngle_) : 1.0;
  const double measuredProgress = activePathProgressApproach(
      measuredLinearProgress, measuredAngularProgress,
      linearProgressCoupled_, angularProgressCoupled_);
  measuredProgressAnchor_ = std::max(
      measuredProgressAnchor_, measuredProgress);

  const double activeLinearTaperDistance = linearBrakingActive_
      ? std::max(lookAheadTaperDistance_, dynamicBrakeDistance_)
      : lookAheadTaperDistance_;
  const double activeLinearLookAhead = taperedLookAheadApproach(
      spatialLinearLookAhead_, minimumLinearLookAhead_,
      dist, posTol_, activeLinearTaperDistance);
  const double activeAngularTaperAngle = angularBrakingActive_
      ? std::max(oriTol_ + 0.001, dynamicBrakeAngle_)
      : std::max(oriTol_ + 0.001, 0.50 * spatialAngularLookAhead_);
  const double activeAngularLookAhead = taperedLookAheadApproach(
      spatialAngularLookAhead_, minimumAngularLookAhead_,
      angle, oriTol_, activeAngularTaperAngle);
  double lookAheadProgress = activeLookAheadProgressApproach(
      allowanceProgressApproach(activeLinearLookAhead, pathDistance_),
      allowanceProgressApproach(activeAngularLookAhead, pathAngle_),
      linearProgressCoupled_, angularProgressCoupled_);
  if(!std::isfinite(lookAheadProgress)) { lookAheadProgress = 1.0; }
  lookAheadProgress = clamp01Approach(lookAheadProgress);

  const double targetProgress = std::min(
      1.0, std::max(referenceProgress_,
                    measuredProgressAnchor_ + lookAheadProgress));
  const sva::PTransformd spatialTarget = ctl.interpolatePose(
      startPose_, goalPose_, targetProgress);
  const sva::PTransformd rateLimitedReference = ctl.advancePoseReference(
      referencePose_, spatialTarget,
      commandedLinearSpeed_, commandedAngularSpeed_);
  const sva::PTransformd nextRef = ctl.boundedPoseStep(
      current, rateLimitedReference,
      maxLinearTrackingLead_, maxAngularTrackingLead_);

  sva::PTransformd safeRef;
  HandoverSafetyReport report;
  if(!ctl.filterSafeMouthCommand(current, nextRef, safeRef, report, true))
  {
    ctl.commandMouthTarget(current);
    mc_rtc::log::error(
        "[GuardedApproach] runtime safety mismatch clear={:.4f} limiting={}/{}; aborting without candidate trial-and-error",
        report.minClearance, report.sample, report.obstacle);
    output("FAIL");
    return true;
  }

  referencePose_ = safeRef;
  const double referenceLinearProgress = projectedTranslationProgressApproach(
      startPose_, goalPose_, referencePose_);
  const double referenceAngleToGoal = ctl.orientationError(
      referencePose_, goalPose_);
  const double referenceAngularProgress = pathAngle_ > 1e-9
      ? clamp01Approach(1.0 - referenceAngleToGoal / pathAngle_) : 1.0;
  const double acceptedReferenceProgress = activePathProgressApproach(
      referenceLinearProgress, referenceAngularProgress,
      linearProgressCoupled_, angularProgressCoupled_);
  referenceProgress_ = std::max(
      referenceProgress_, acceptedReferenceProgress);

  Eigen::Vector3d linearFeedforwardWorld = Eigen::Vector3d::Zero();
  Eigen::Vector3d angularFeedforwardWorld = Eigen::Vector3d::Zero();
  double terminalFeedforwardScale = 1.0;
  if(linearBrakingActive_ && dynamicBrakeDistance_ > posTol_ + 1e-9)
  {
    terminalFeedforwardScale = smoothStepApproach(
        (dist - posTol_) / (dynamicBrakeDistance_ - posTol_));
  }
  if(pathDistance_ > posTol_ && dist > posTol_)
  {
    linearFeedforwardWorld = terminalFeedforwardScale
                           * linearFeedforwardRatio_
                           * commandedLinearSpeed_
                           * linearPathDirectionWorld_;
  }
  double terminalAngularFeedforwardScale = 1.0;
  if(angularBrakingActive_ && dynamicBrakeAngle_ > oriTol_ + 1e-9)
  {
    terminalAngularFeedforwardScale = smoothStepApproach(
        (angle - oriTol_) / (dynamicBrakeAngle_ - oriTol_));
  }
  if(angularProgressCoupled_ && angle > oriTol_)
  {
    angularFeedforwardWorld = terminalAngularFeedforwardScale
                            * angularFeedforwardRatio_
                            * commandedAngularSpeed_
                            * angularPathDirectionWorld_;
  }
  ctl.commandMouthTargetWithWorldVelocity(
      referencePose_, linearFeedforwardWorld, angularFeedforwardWorld);
  if(iter_ % 60 == 1)
  {
    mc_rtc::log::info(
        "[GuardedApproach] measuredS={:.3f} anchorS={:.3f} targetS={:.3f} refS={:.3f} linearS={:.3f} angularS={:.3f} dist={:.4f} angle={:.4f} centerErr={:.4f} vCmd={:.3f} vFF={:.3f} vActual={:.3f} vFiltered={:.3f} wCmd={:.3f} wFiltered={:.3f} brakeLinear={} brakeAngular={} brakeDist={:.4f} brakeAngle={:.4f} ffScale={:.3f} angularFFScale={:.3f} lookAhead={:.4f} refLead={:.4f} clear={:.4f} limiting={}/{}",
        measuredProgress, measuredProgressAnchor_, targetProgress,
        referenceProgress_, measuredLinearProgress, measuredAngularProgress,
        dist, angle,
        centeringError, commandedLinearSpeed_,
        linearFeedforwardWorld.norm(), measuredLinearVelocity.norm(),
        filteredLinearSpeed_, commandedAngularSpeed_,
        filteredAngularSpeed_, linearBrakingActive_,
        angularBrakingActive_, dynamicBrakeDistance_, dynamicBrakeAngle_,
        terminalFeedforwardScale, terminalAngularFeedforwardScale,
        activeLinearLookAhead,
        (referencePose_.translation() - current.translation()).norm(),
        report.minClearance, report.sample, report.obstacle);
  }
  return false;
}

void HandoverInterceptionController_MovePregrasp::teardown(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ctl.finishPhaseTiming("approach");
  ctl.clearToolTaskReferenceMotion();
  mc_rtc::log::success(
      "[VelocityGatedTerminalCaptureSummary] phase=approach peakLinear={:.3f}m/s peakAngular={:.3f}rad/s feedforwardRatio={:.2f} linearBrake={} angularBrake={} brakeDistance={:.4f}m brakeAngle={:.4f}rad terminalStable={:.3f}s",
      peakMeasuredLinearSpeed_, peakMeasuredAngularSpeed_,
      linearFeedforwardRatio_, linearBrakingActive_, angularBrakingActive_,
      dynamicBrakeDistance_, dynamicBrakeAngle_, terminalStableTime_);
  mc_rtc::log::info("[GuardedApproach] teardown");
}

EXPORT_SINGLE_STATE("HandoverInterceptionController_MovePregrasp",
                    HandoverInterceptionController_MovePregrasp)
