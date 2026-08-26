#include "HandoverInterceptionController_Retreat.h"
#include "../HandoverInterceptionController.h"

#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
double clamp01Retreat(double value)
{
  return std::max(0.0, std::min(1.0, value));
}

double projectedTranslationProgressRetreat(
    const sva::PTransformd & start,
    const sva::PTransformd & goal,
    const sva::PTransformd & current)
{
  const Eigen::Vector3d path = goal.translation() - start.translation();
  const double squaredLength = path.squaredNorm();
  if(squaredLength <= 1e-12) { return 1.0; }
  return clamp01Retreat(
      (current.translation() - start.translation()).dot(path)
      / squaredLength);
}

double allowanceProgressRetreat(double allowance, double total)
{
  if(total <= 1e-9) { return std::numeric_limits<double>::infinity(); }
  return std::max(0.0, allowance) / total;
}


double taperedLookAheadRetreat(double maximum, double minimum,
                               double remaining, double tolerance,
                               double taperDistance)
{
  const double maxValue = std::max(0.0, maximum);
  const double minValue = std::min(maxValue, std::max(0.0, minimum));
  const double start = std::max(tolerance + 1e-6, taperDistance);
  if(remaining >= start) { return maxValue; }
  const double alpha = clamp01Retreat(
      (remaining - tolerance) / std::max(1e-9, start - tolerance));
  return minValue + alpha * (maxValue - minValue);
}

double rampSpeedRetreat(double current, double desired,
                        double acceleration, double dt)
{
  const double step = std::max(0.0, acceleration) * std::max(0.0, dt);
  return current + std::max(-step, std::min(step, desired - current));
}

/**
 * Combine only path dimensions that require terminal correction. Retreat
 * commonly starts already inside its orientation tolerance; treating that
 * inactive angle as zero progress would otherwise stop translation after one
 * look-ahead length.
 */
double activePathProgressRetreat(double linearProgress,
                                 double angularProgress,
                                 double pathDistance,
                                 double pathAngle,
                                 double posTol,
                                 double oriTol)
{
  const bool linearRequired = pathDistance > std::max(0.0, posTol);
  const bool angularRequired = pathAngle > std::max(0.0, oriTol);
  if(linearRequired && angularRequired)
  {
    return std::min(linearProgress, angularProgress);
  }
  if(linearRequired) { return linearProgress; }
  if(angularRequired) { return angularProgress; }
  return 1.0;
}
}

void HandoverInterceptionController_Retreat::configure(
    const mc_rtc::Configuration & config)
{
  if(config.has("linearSpeed")) { config("linearSpeed", linearSpeed_); }
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
  if(config.has("taskStiffness")) { config("taskStiffness", taskStiffness_); }
  if(config.has("taskWeight")) { config("taskWeight", taskWeight_); }
  if(config.has("progressEps")) { config("progressEps", progressEps_); }
  if(config.has("progressLimit")) { config("progressLimit", progressLimit_); }
  if(config.has("maxIter")) { config("maxIter", maxIter_); }

  spatialLinearLookAhead_ = std::max(0.001, spatialLinearLookAhead_);
  spatialAngularLookAhead_ = std::max(0.001, spatialAngularLookAhead_);
  minimumLinearLookAhead_ = std::min(
      spatialLinearLookAhead_, std::max(0.001, minimumLinearLookAhead_));
  minimumAngularLookAhead_ = std::min(
      spatialAngularLookAhead_, std::max(0.001, minimumAngularLookAhead_));
  lookAheadRemainingRatio_ = std::max(
      0.05, std::min(1.0, lookAheadRemainingRatio_));
  lookAheadTaperDistance_ = std::max(
      posTol_ + 0.001, lookAheadTaperDistance_);
  linearFeedforwardRatio_ = clamp01Retreat(linearFeedforwardRatio_);
  angularFeedforwardRatio_ = clamp01Retreat(angularFeedforwardRatio_);
  maxLinearTrackingLead_ = std::max(
      spatialLinearLookAhead_, std::max(0.001, maxLinearTrackingLead_));
  maxAngularTrackingLead_ = std::max(
      spatialAngularLookAhead_, std::max(0.001, maxAngularTrackingLead_));
  linearAcceleration_ = std::max(0.01, linearAcceleration_);
  angularAcceleration_ = std::max(0.01, angularAcceleration_);
}

void HandoverInterceptionController_Retreat::start(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ctl.setGripperClosureAuthorized(true);
  ctl.startPhaseTiming("retreat");
  iter_ = 0;
  stagnantCycles_ = 0;
  bestCombinedError_ = 1e9;
  commandedLinearSpeed_ = 0.0;
  commandedAngularSpeed_ = 0.0;
  referenceProgress_ = 0.0;
  measuredProgressAnchor_ = 0.0;
  peakMeasuredLinearSpeed_ = 0.0;
  peakMeasuredAngularSpeed_ = 0.0;
  holdClosure_ = std::max(ctl.gripperCommand(), ctl.measuredGripperClosure());

  ctl.activateToolTask();
  ctl.setToolTaskGains(taskStiffness_, taskWeight_);
  ctl.commandSelectedRetreatPosture();
  ctl.setGripperJointPriority(true);

  startPose_ = ctl.actualMouthPose();
  goalPose_ = ctl.currentMouthRetreatTarget();
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
  ctl.commandGripper(holdClosure_);

  const auto p = goalPose_.translation();
  mc_rtc::log::warning(
      "[Retreat] speed-consistent spatial carried-object retreat to [{:.3f},{:.3f},{:.3f}] vMax={:.2f}m/s lookAhead=[{:.3f}m,{:.3f}rad] feedforward=[{:.2f},{:.2f}] taperDistance={:.3f}m",
      p.x(), p.y(), p.z(), linearSpeed_,
      spatialLinearLookAhead_, spatialAngularLookAhead_,
      linearFeedforwardRatio_, angularFeedforwardRatio_,
      lookAheadTaperDistance_);
  mc_rtc::log::success(
      "[SpatialProgressContract] phase=retreat linearRequired={} angularRequired={} pathDistance={:.4f} pathAngle={:.4f} posTol={:.4f} oriTol={:.4f} activeDimensionGate=true monotonicAnchor=true tangentFeedforward=true",
      pathDistance_ > posTol_, pathAngle_ > oriTol_, pathDistance_,
      pathAngle_, posTol_, oriTol_);
}

bool HandoverInterceptionController_Retreat::run(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ++iter_;
  ctl.setGripperClosureAuthorized(true);

  if(!ctl.hasSelectedCandidate() || !ctl.objectAttached())
  {
    mc_rtc::log::error("[Retreat] selected plan or confirmed attachment unavailable");
    output("FAIL");
    return true;
  }

  ctl.commandSelectedRetreatPosture();
  ctl.setGripperJointPriority(true);
  ctl.commandGripper(holdClosure_);
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
  const double dist = (goalPose_.translation() - current.translation()).norm();
  const double angle = ctl.orientationError(current, goalPose_);

  HandoverSafetyReport currentSafety;
  const bool currentSafe = ctl.evaluateAttachedRetreatSafety(currentSafety);
  if(dist <= posTol_ && angle <= oriTol_ && currentSafe)
  {
    mc_rtc::log::success(
        "[Retreat] carried object reached certified retreat pose dist={:.4f} angle={:.4f} clear={:.4f}",
        dist, angle, currentSafety.minClearance);
    output("OK");
    return true;
  }

  const double combinedError = dist + 0.08 * angle;
  if(combinedError < bestCombinedError_ - progressEps_)
  {
    bestCombinedError_ = combinedError;
    stagnantCycles_ = 0;
  }
  else { ++stagnantCycles_; }

  if(stagnantCycles_ >= progressLimit_ || iter_ >= maxIter_)
  {
    mc_rtc::log::error(
        "[Retreat] continuous spatial carried-object trajectory stalled dist={:.4f} angle={:.4f}",
        dist, angle);
    ctl.commandMouthTarget(current);
    output("FAIL");
    return true;
  }

  const double brakingLinearSpeed = std::sqrt(std::max(
      0.0, 2.0 * linearAcceleration_ * std::max(0.0, dist - posTol_)));
  const double brakingAngularSpeed = std::sqrt(std::max(
      0.0, 2.0 * angularAcceleration_ * std::max(0.0, angle - oriTol_)));
  const double desiredLinearSpeed = std::min(
      linearSpeed_, brakingLinearSpeed);
  const double desiredAngularSpeed = std::min(
      angularSpeed_, brakingAngularSpeed);

  commandedLinearSpeed_ = rampSpeedRetreat(
      commandedLinearSpeed_, desiredLinearSpeed,
      linearAcceleration_, ctl.controlDt());
  commandedAngularSpeed_ = rampSpeedRetreat(
      commandedAngularSpeed_, desiredAngularSpeed,
      angularAcceleration_, ctl.controlDt());

  const double measuredLinearProgress = projectedTranslationProgressRetreat(
      startPose_, goalPose_, current);
  const double measuredAngularProgress = pathAngle_ > 1e-9
      ? clamp01Retreat(1.0 - angle / pathAngle_) : 1.0;
  const double measuredProgress = activePathProgressRetreat(
      measuredLinearProgress, measuredAngularProgress,
      pathDistance_, pathAngle_, posTol_, oriTol_);
  measuredProgressAnchor_ = std::max(
      measuredProgressAnchor_, measuredProgress);

  const double activeLinearLookAhead = taperedLookAheadRetreat(
      spatialLinearLookAhead_, minimumLinearLookAhead_,
      dist, posTol_, lookAheadTaperDistance_);
  const double activeAngularLookAhead = taperedLookAheadRetreat(
      spatialAngularLookAhead_, minimumAngularLookAhead_,
      angle, oriTol_, std::max(oriTol_ + 0.001,
                              0.50 * spatialAngularLookAhead_));
  double lookAheadProgress = std::min(
      allowanceProgressRetreat(activeLinearLookAhead, pathDistance_),
      allowanceProgressRetreat(activeAngularLookAhead, pathAngle_));
  if(!std::isfinite(lookAheadProgress)) { lookAheadProgress = 1.0; }
  lookAheadProgress = clamp01Retreat(lookAheadProgress);

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
  if(!ctl.filterSafeAttachedRetreatCommand(
         current, nextRef, safeRef, report))
  {
    ctl.commandMouthTarget(current);
    mc_rtc::log::error(
        "[Retreat] runtime carried-object safety mismatch clear={:.4f} limiting={}/{}",
        report.minClearance, report.sample, report.obstacle);
    output("FAIL");
    return true;
  }

  referencePose_ = safeRef;
  const double referenceLinearProgress = projectedTranslationProgressRetreat(
      startPose_, goalPose_, referencePose_);
  const double referenceAngleToGoal = ctl.orientationError(
      referencePose_, goalPose_);
  const double referenceAngularProgress = pathAngle_ > 1e-9
      ? clamp01Retreat(1.0 - referenceAngleToGoal / pathAngle_) : 1.0;
  const double acceptedReferenceProgress = activePathProgressRetreat(
      referenceLinearProgress, referenceAngularProgress,
      pathDistance_, pathAngle_, posTol_, oriTol_);
  referenceProgress_ = std::max(
      referenceProgress_, acceptedReferenceProgress);

  Eigen::Vector3d linearFeedforwardWorld = Eigen::Vector3d::Zero();
  Eigen::Vector3d angularFeedforwardWorld = Eigen::Vector3d::Zero();
  if(pathDistance_ > posTol_ && dist > posTol_)
  {
    linearFeedforwardWorld = linearFeedforwardRatio_
                           * commandedLinearSpeed_
                           * linearPathDirectionWorld_;
  }
  if(pathAngle_ > oriTol_ && angle > oriTol_)
  {
    angularFeedforwardWorld = angularFeedforwardRatio_
                            * commandedAngularSpeed_
                            * angularPathDirectionWorld_;
  }
  ctl.commandMouthTargetWithWorldVelocity(
      referencePose_, linearFeedforwardWorld, angularFeedforwardWorld);
  if(iter_ % 60 == 1)
  {
    mc_rtc::log::info(
        "[Retreat] measuredS={:.3f} anchorS={:.3f} targetS={:.3f} refS={:.3f} linearS={:.3f} angularS={:.3f} dist={:.4f} angle={:.4f} vCmd={:.3f} vFF={:.3f} vActual={:.3f} wCmd={:.3f} lookAhead={:.4f} refLead={:.4f} clear={:.4f}",
        measuredProgress, measuredProgressAnchor_, targetProgress,
        referenceProgress_, measuredLinearProgress, measuredAngularProgress,
        dist, angle, commandedLinearSpeed_,
        linearFeedforwardWorld.norm(), measuredLinearVelocity.norm(),
        commandedAngularSpeed_, activeLinearLookAhead,
        (referencePose_.translation() - current.translation()).norm(),
        report.minClearance);
  }
  return false;
}

void HandoverInterceptionController_Retreat::teardown(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ctl.finishPhaseTiming("retreat");
  ctl.clearToolTaskReferenceMotion();
  mc_rtc::log::success(
      "[SpeedConsistentSpatialServoSummary] phase=retreat peakLinear={:.3f}m/s peakAngular={:.3f}rad/s feedforwardRatio={:.2f} taperDistance={:.3f}m",
      peakMeasuredLinearSpeed_, peakMeasuredAngularSpeed_,
      linearFeedforwardRatio_, lookAheadTaperDistance_);
  mc_rtc::log::info("[Retreat] teardown");
}

EXPORT_SINGLE_STATE("HandoverInterceptionController_Retreat",
                    HandoverInterceptionController_Retreat)
