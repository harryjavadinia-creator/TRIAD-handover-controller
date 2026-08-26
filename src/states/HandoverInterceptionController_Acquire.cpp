#include "HandoverInterceptionController_Acquire.h"
#include "../HandoverInterceptionController.h"

#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>

namespace
{
double clamp01(double x)
{
  return std::max(0.0, std::min(1.0, x));
}

double clampValue(double x, double lo, double hi)
{
  return std::max(lo, std::min(hi, x));
}

double smoothStep01(double x)
{
  x = clamp01(x);
  return x * x * (3.0 - 2.0 * x);
}
}

void HandoverInterceptionController_Acquire::configure(
    const mc_rtc::Configuration & config)
{
  if(config.has("stableDwell")) { config("stableDwell", stableDwell_); }
  if(config.has("confirmationDwell"))
  {
    config("confirmationDwell", confirmationDwell_);
  }
  if(config.has("confirmationTimeout"))
  {
    config("confirmationTimeout", confirmationTimeout_);
  }
  if(config.has("confirmationLogEvery"))
  {
    config("confirmationLogEvery", confirmationLogEvery_);
  }
  if(config.has("watchdogTimeout"))
  {
    config("watchdogTimeout", watchdogTimeout_);
  }
  else if(config.has("timeout"))
  {
    config("timeout", watchdogTimeout_);
  }
  if(config.has("taskStiffness")) { config("taskStiffness", taskStiffness_); }
  if(config.has("taskWeight")) { config("taskWeight", taskWeight_); }
  if(config.has("gapProgressEps")) { config("gapProgressEps", gapProgressEps_); }
  if(config.has("gapStallLimit")) { config("gapStallLimit", gapStallLimit_); }

  if(config.has("priorityBlendDuration"))
  {
    config("priorityBlendDuration", priorityBlendDuration_);
  }
  if(config.has("lockDwell")) { config("lockDwell", lockDwell_); }
  if(config.has("lockTimeout")) { config("lockTimeout", lockTimeout_); }
  if(config.has("lockPosTolerance"))
  {
    config("lockPosTolerance", lockPosTolerance_);
  }
  if(config.has("lockOriTolerance"))
  {
    config("lockOriTolerance", lockOriTolerance_);
  }
  if(config.has("lockCenterTolerance"))
  {
    config("lockCenterTolerance", lockCenterTolerance_);
  }
  if(config.has("lockCenterRateTolerance"))
  {
    config("lockCenterRateTolerance", lockCenterRateTolerance_);
  }
  if(config.has("lockTransientPosLimit"))
  {
    config("lockTransientPosLimit", lockTransientPosLimit_);
  }
  if(config.has("lockTransientCenterLimit"))
  {
    config("lockTransientCenterLimit", lockTransientCenterLimit_);
  }

  if(config.has("centeringGain")) { config("centeringGain", centeringGain_); }
  if(config.has("centeringDamping"))
  {
    config("centeringDamping", centeringDamping_);
  }
  if(config.has("maxCenteringSpeed"))
  {
    config("maxCenteringSpeed", maxCenteringSpeed_);
  }
  if(config.has("maxCenteringAcceleration"))
  {
    config("maxCenteringAcceleration", maxCenteringAcceleration_);
  }
  if(config.has("maxCenteringOffset"))
  {
    config("maxCenteringOffset", maxCenteringOffset_);
  }
  if(config.has("centeringDeadband"))
  {
    config("centeringDeadband", centeringDeadband_);
  }
  if(config.has("closureCenterHysteresis"))
  {
    config("closureCenterHysteresis", closureCenterHysteresis_);
  }
  if(config.has("centeringBiasGain"))
  {
    config("centeringBiasGain", centeringBiasGain_);
  }
  if(config.has("centeringBiasLeak"))
  {
    config("centeringBiasLeak", centeringBiasLeak_);
  }
  if(config.has("maxCenteringBias"))
  {
    config("maxCenteringBias", maxCenteringBias_);
  }
  if(config.has("centeringRecoveryTimeout"))
  {
    config("centeringRecoveryTimeout", centeringRecoveryTimeout_);
  }
  if(config.has("centerRateFilter"))
  {
    config("centerRateFilter", centerRateFilter_);
  }
}

void HandoverInterceptionController_Acquire::start(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ctl.setGripperClosureAuthorized(true);
  ctl.startPhaseTiming("acquire");

  phase_ = Phase::PriorityBlend;
  priorityBlendElapsed_ = 0.0;
  stableTime_ = 0.0;
  confirmationStableTime_ = 0.0;
  confirmationElapsed_ = 0.0;
  holdClosure_ = 0.0;
  lockStableTime_ = 0.0;
  lockElapsed_ = 0.0;
  recoveryElapsed_ = 0.0;
  elapsed_ = 0.0;
  iter_ = 0;
  gapStallCycles_ = 0;
  watchdogWarned_ = false;
  havePreviousSignedError_ = false;
  previousSignedError_ = 0.0;
  filteredCenterRate_ = 0.0;
  centeringVelocity_ = 0.0;
  centeringBias_ = 0.0;
  centeringPaused_ = false;
  maxTransitionPosError_ = 0.0;
  maxTransitionCenterError_ = 0.0;
  maxTransitionOffset_ = 0.0;

  nominalTarget_ = ctl.currentMouthPregraspTarget();
  const sva::PTransformd current = ctl.actualMouthPose();
  const Eigen::Vector3d xW = ctl.worldRotation(nominalTarget_).col(0);
  centeringOffset_ = clampValue(
      (current.translation() - nominalTarget_.translation()).dot(xW),
      -maxCenteringOffset_, maxCenteringOffset_);
  desiredCenteringOffset_ = centeringOffset_;
  centeredTarget_ = nominalTarget_;
  centeredTarget_.translation() += xW * centeringOffset_;

  closeCommand_ = ctl.measuredGripperClosure();
  bestGap_ = ctl.liveMouthGap();

  ctl.activateToolTask();
  ctl.setToolTaskGains(taskStiffness_, taskWeight_);
  ctl.commandSelectedArmPosture();
  ctl.setGripperJointPriority(0.0);
  ctl.commandGripper(closeCommand_);
  ctl.commandMouthTarget(centeredTarget_);

  if(!ctl.gripperActuationAvailable())
  {
    mc_rtc::log::error(
        "[Acquire] cannot close: {}. Load the actuated kinova_gen3_2f85 RobotModule",
        ctl.gripperActuationStatus());
  }

  mc_rtc::log::warning(
      "[Acquire] bumpless priority blend, aperture-dependent capture tube, and in-loop grasp confirmation: blend={:.2f}s centerTube={:.4f}->{:.4f}m predictedContact={:.3f}",
      priorityBlendDuration_, ctl.acquisitionFarCenterTolerance(),
      ctl.acquisitionNearCenterTolerance(),
      ctl.selectedCandidateContactClosure());
}

bool HandoverInterceptionController_Acquire::run(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ++iter_;
  ctl.setGripperClosureAuthorized(true);
  elapsed_ += ctl.controlDt();

  if(!ctl.hasSelectedCandidate() || !ctl.gripperActuationAvailable())
  {
    mc_rtc::log::error("[Acquire] certified candidate or actuated gripper unavailable");
    output("FAIL");
    return true;
  }

  if(ctl.committedPlanValid()
     && ctl.controllerTime() > ctl.committedAcquisitionDeadline())
  {
    mc_rtc::log::error(
        "[Acquire] bounded static acquisition deadline expired before stable confirmation; no retry");
    ctl.commandGripper(ctl.measuredGripperClosure());
    output("FAIL");
    return true;
  }

  double priorityAlpha = 1.0;
  if(phase_ == Phase::PriorityBlend)
  {
    const double u = priorityBlendDuration_ > 1e-9
        ? priorityBlendElapsed_ / priorityBlendDuration_ : 1.0;
    priorityAlpha = smoothStep01(u);
  }

  // The arm posture and Cartesian task gains remain unchanged. Only gripper
  // joint stiffness/weight are blended, so this transition cannot kick the arm
  // through a shared global posture-task gain switch.
  ctl.commandSelectedArmPosture();
  ctl.setGripperJointPriority(priorityAlpha);

  const sva::PTransformd current = ctl.actualMouthPose();
  ctl.refreshGripperGeometry(false);

  HandoverSafetyReport report;
  const bool intentionalPadContactMode = phase_ == Phase::Confirming;
  if(!ctl.evaluateCurrentClosureSafety(
         report, false, intentionalPadContactMode))
  {
    mc_rtc::log::error(
        "[Acquire] hard closure geometry violation clear={:.4f} limiting={}/{} ground={:.4f} left={:.4f} right={:.4f} centerErr={:.4f}",
        report.minClearance, report.sample, report.obstacle,
        report.groundClearance, report.leftPadClearance,
        report.rightPadClearance, report.padCenteringError);
    ctl.commandGripper(ctl.measuredGripperClosure());
    output("FAIL");
    return true;
  }

  const double signedError = report.signedPadCenteringError;
  if(havePreviousSignedError_)
  {
    const double rawRate = (signedError - previousSignedError_)
                         / std::max(1e-9, ctl.controlDt());
    const double alpha = clampValue(centerRateFilter_, 0.0, 1.0);
    filteredCenterRate_ = (1.0 - alpha) * filteredCenterRate_
                        + alpha * rawRate;
  }
  else
  {
    havePreviousSignedError_ = true;
    filteredCenterRate_ = 0.0;
  }
  previousSignedError_ = signedError;

  const Eigen::Vector3d xW = ctl.worldRotation(nominalTarget_).col(0);
  const double actualCenteringOffset =
      (current.translation() - nominalTarget_.translation()).dot(xW);

  if(phase_ != Phase::PriorityBlend)
  {
    // The geometric centering target is reconstructed from measurement:
    // current physical offset + current signed handle offset. A deliberately
    // slow, bounded bias term then removes the small steady-state QP residual
    // seen in the ground configuration. This is integral compensation around
    // the measured geometry, not command-as-motion integration.
    const bool biasAtUpper = centeringBias_ >= maxCenteringBias_ - 1e-9;
    const bool biasAtLower = centeringBias_ <= -maxCenteringBias_ + 1e-9;
    const bool biasWouldWindUp =
        (biasAtUpper && signedError > 0.0)
        || (biasAtLower && signedError < 0.0);
    if(std::abs(signedError) > centeringDeadband_ && !biasWouldWindUp)
    {
      centeringBias_ += centeringBiasGain_ * signedError * ctl.controlDt();
    }
    else if(std::abs(signedError) <= centeringDeadband_)
    {
      centeringBias_ *= std::max(
          0.0, 1.0 - centeringBiasLeak_ * ctl.controlDt());
    }
    centeringBias_ = clampValue(
        centeringBias_, -maxCenteringBias_, maxCenteringBias_);

    desiredCenteringOffset_ = clampValue(
        actualCenteringOffset + signedError + centeringBias_,
        -maxCenteringOffset_, maxCenteringOffset_);

    const double offsetError = desiredCenteringOffset_ - centeringOffset_;
    double desiredCenterVelocity =
        centeringGain_ * offsetError
        - centeringDamping_ * centeringVelocity_;
    if(std::abs(signedError) <= centeringDeadband_
       && std::abs(offsetError) <= centeringDeadband_)
    {
      desiredCenterVelocity = 0.0;
    }
    desiredCenterVelocity = clampValue(
        desiredCenterVelocity, -maxCenteringSpeed_, maxCenteringSpeed_);

    const double maxDv = std::max(0.0, maxCenteringAcceleration_)
                       * ctl.controlDt();
    centeringVelocity_ += clampValue(
        desiredCenterVelocity - centeringVelocity_, -maxDv, maxDv);
    centeringVelocity_ = clampValue(
        centeringVelocity_, -maxCenteringSpeed_, maxCenteringSpeed_);
    centeringOffset_ = clampValue(
        centeringOffset_ + centeringVelocity_ * ctl.controlDt(),
        -maxCenteringOffset_, maxCenteringOffset_);
  }
  else
  {
    centeringVelocity_ = 0.0;
    centeringBias_ = 0.0;
    desiredCenteringOffset_ = centeringOffset_;
  }

  centeredTarget_ = nominalTarget_;
  centeredTarget_.translation() += xW * centeringOffset_;
  ctl.commandMouthTarget(centeredTarget_);

  const double posError =
      (centeredTarget_.translation() - current.translation()).norm();
  const double oriError = ctl.orientationError(current, centeredTarget_);

  if(phase_ == Phase::PriorityBlend || phase_ == Phase::CaptureLock)
  {
    maxTransitionPosError_ = std::max(maxTransitionPosError_, posError);
    maxTransitionCenterError_ = std::max(
        maxTransitionCenterError_, report.padCenteringError);
    maxTransitionOffset_ = std::max(
        maxTransitionOffset_, std::abs(actualCenteringOffset));

    if(posError > lockTransientPosLimit_
       || report.padCenteringError > lockTransientCenterLimit_)
    {
      mc_rtc::log::error(
          "[AcquireLock] excessive transition transient pos={:.4f} centerErr={:.4f} actualOffset={:.4f} targetOffset={:.4f}; failing before visible arm jitter",
          posError, report.padCenteringError, actualCenteringOffset,
          centeringOffset_);
      ctl.commandGripper(ctl.measuredGripperClosure());
      output("FAIL");
      return true;
    }
  }

  if(posError > 0.030 || oriError > 0.15)
  {
    mc_rtc::log::error(
        "[Acquire] capture pose lost posError={:.4f} oriError={:.4f} centerOffset={:.4f}",
        posError, oriError, centeringOffset_);
    ctl.commandGripper(ctl.measuredGripperClosure());
    output("FAIL");
    return true;
  }

  const double measuredClosure = ctl.measuredGripperClosure();
  const double minPadClear = std::min(report.leftPadClearance,
                                      report.rightPadClearance);
  const double maxPadClear = std::max(report.leftPadClearance,
                                      report.rightPadClearance);
  const double centerTube = ctl.acquisitionCenterTolerance(minPadClear);

  if(phase_ == Phase::PriorityBlend)
  {
    closeCommand_ = measuredClosure;
    ctl.commandGripper(closeCommand_);
    priorityBlendElapsed_ += ctl.controlDt();

    if(iter_ % 100 == 1)
    {
      mc_rtc::log::info(
          "[AcquireBlend] t={:.2f}s alpha={:.3f} pos={:.4f} ori={:.4f} centerSigned={:.4f} actualOffset={:.4f} targetOffset={:.4f}",
          priorityBlendElapsed_, priorityAlpha, posError, oriError,
          signedError, actualCenteringOffset, centeringOffset_);
    }

    if(priorityBlendElapsed_ >= priorityBlendDuration_)
    {
      phase_ = Phase::CaptureLock;
      lockElapsed_ = 0.0;
      lockStableTime_ = 0.0;
      havePreviousSignedError_ = false;
      filteredCenterRate_ = 0.0;

      // Re-seed the target from measured position so CaptureLock starts with
      // zero lateral command discontinuity.
      centeringOffset_ = clampValue(
          actualCenteringOffset, -maxCenteringOffset_, maxCenteringOffset_);
      desiredCenteringOffset_ = centeringOffset_;
      centeringVelocity_ = 0.0;
      centeringBias_ = 0.0;
      centeringPaused_ = false;
      centeredTarget_ = nominalTarget_;
      centeredTarget_.translation() += xW * centeringOffset_;
      ctl.commandMouthTarget(centeredTarget_);

      mc_rtc::log::success(
          "[AcquireBlend] gripper priority blended without changing arm-task gains; capture-lock target reseeded actualOffset={:.4f}",
          actualCenteringOffset);
    }
    return false;
  }

  if(phase_ == Phase::CaptureLock)
  {
    lockElapsed_ += ctl.controlDt();
    closeCommand_ = measuredClosure;
    ctl.commandGripper(closeCommand_);

    const double lockCenterTube = std::max(
        lockCenterTolerance_, centerTube);
    const bool lockStable = posError <= lockPosTolerance_
        && oriError <= lockOriTolerance_
        && std::abs(signedError) <= lockCenterTube
        && std::abs(filteredCenterRate_) <= lockCenterRateTolerance_;

    if(lockStable) { lockStableTime_ += ctl.controlDt(); }
    else { lockStableTime_ = 0.0; }

    if(iter_ % 100 == 1)
    {
      mc_rtc::log::info(
          "[AcquireLock] t={:.2f}s stable={:.3f}/{:.3f}s pos={:.4f} ori={:.4f} centerSigned={:.4f} centerRate={:.4f} tube={:.4f} actualOffset={:.4f} desiredOffset={:.4f} targetOffset={:.4f} bias={:.4f} left={:.4f} right={:.4f}",
          lockElapsed_, lockStableTime_, lockDwell_, posError, oriError,
          signedError, filteredCenterRate_, lockCenterTube,
          actualCenteringOffset, desiredCenteringOffset_, centeringOffset_,
          centeringBias_,
          report.leftPadClearance, report.rightPadClearance);
    }

    if(lockStableTime_ >= lockDwell_)
    {
      phase_ = Phase::Closing;
      stableTime_ = 0.0;
      recoveryElapsed_ = 0.0;
      bestGap_ = ctl.liveMouthGap();
      gapStallCycles_ = 0;
      mc_rtc::log::success(
          "[AcquireLock] capture admitted inside aperture-dependent tube; closure enabled centerErr={:.4f} tube={:.4f} centerRate={:.4f} maxPos={:.4f} maxCenter={:.4f} maxOffset={:.4f}",
          report.padCenteringError, lockCenterTube, filteredCenterRate_,
          maxTransitionPosError_, maxTransitionCenterError_,
          maxTransitionOffset_);
      return false;
    }

    if(lockElapsed_ >= lockTimeout_)
    {
      mc_rtc::log::error(
          "[AcquireLock] could not enter aperture-dependent capture tube before closure pos={:.4f} ori={:.4f} centerErr={:.4f} tube={:.4f} centerRate={:.4f} offset={:.4f}",
          posError, oriError, report.padCenteringError,
          lockCenterTube, filteredCenterRate_, centeringOffset_);
      output("FAIL");
      return true;
    }
    return false;
  }

  const bool bilateralContactInsideTube =
      report.bilateralPadContact
      && report.padCenteringError <= centerTube;

  if(phase_ == Phase::Confirming)
  {
    // Keep the exact Acquire controller active: same Cartesian gains, same
    // candidate posture, same corrected mouth target, same centering servo,
    // and the same gripper-joint priority. The designated bilateral contact
    // pair is now regulated around tangency instead of being frozen at a
    // lead-biased closure command. Positive mean residual closes gently; a
    // negative mean residual opens gently. This is a contact constraint
    // regulator inside the committed grasp, not a new target or a retry.
    confirmationElapsed_ += ctl.controlDt();
    const double meanPadResidual = 0.5 * (
        report.leftPadClearance + report.rightPadClearance);
    const double normalizedContactResidual = clampValue(
        meanPadResidual / std::max(1e-9, ctl.gripperContactTolerance()),
        -1.0, 1.0);
    const double contactRegulationRate = ctl.gripperMinimumCloseRate()
                                       * normalizedContactResidual;
    holdClosure_ = clampValue(
        holdClosure_ + contactRegulationRate * ctl.controlDt(),
        0.0, ctl.gripperMaxClosure());
    closeCommand_ = holdClosure_;
    ctl.commandGripper(holdClosure_);

    if(bilateralContactInsideTube)
    {
      confirmationStableTime_ += ctl.controlDt();
    }
    else
    {
      confirmationStableTime_ = 0.0;
    }

    if(confirmationLogEvery_ > 0
       && iter_ % confirmationLogEvery_ == 1)
    {
      mc_rtc::log::info(
          "[AcquireConfirm] hybridContactMode=true stable={:.3f}/{:.3f}s elapsed={:.3f}/{:.3f}s contact={} nonContactClear={:.4f} leftResidual={:.4f} rightResidual={:.4f} meanResidual={:.4f} contactRate={:+.4f}/s closure={:.3f} centerErr={:.4f} tube={:.4f} targetOffset={:.4f} bias={:.4f}",
          confirmationStableTime_, confirmationDwell_,
          confirmationElapsed_, confirmationTimeout_,
          bilateralContactInsideTube, report.minClearance,
          report.leftPadClearance, report.rightPadClearance,
          meanPadResidual, contactRegulationRate, holdClosure_,
          report.padCenteringError, centerTube, centeringOffset_,
          centeringBias_);
    }

    if(confirmationStableTime_ >= confirmationDwell_)
    {
      ctl.commitAcquiredMouthTarget(centeredTarget_);
      if(!ctl.attachObjectToMouth())
      {
        mc_rtc::log::error(
            "[AcquireConfirm] attachment refused despite in-loop contact confirmation");
        output("FAIL");
        return true;
      }

      mc_rtc::log::success(
          "[AcquireConfirm] stable grasp confirmed under unchanged Acquire controller; proceeding directly to certified retreat closure={:.3f} left={:.4f} right={:.4f} centerErr={:.4f}",
          measuredClosure, report.leftPadClearance,
          report.rightPadClearance, report.padCenteringError);
      output("OK");
      return true;
    }

    if(confirmationElapsed_ >= confirmationTimeout_)
    {
      mc_rtc::log::error(
          "[AcquireConfirm] grasp could not remain bilateral under unchanged Acquire controller elapsed={:.3f}s left={:.4f} right={:.4f} centerErr={:.4f} clear={:.4f}",
          confirmationElapsed_, report.leftPadClearance,
          report.rightPadClearance, report.padCenteringError,
          report.minClearance);
      output("FAIL");
      return true;
    }

    return false;
  }

  if(bilateralContactInsideTube) { stableTime_ += ctl.controlDt(); }
  else { stableTime_ = 0.0; }

  if(stableTime_ >= stableDwell_)
  {
    phase_ = Phase::Confirming;
    // Freeze at the measured aperture, not at a possibly lead-biased command.
    // This prevents any residual closing motion during confirmation.
    holdClosure_ = measuredClosure;
    closeCommand_ = holdClosure_;
    confirmationStableTime_ = 0.0;
    confirmationElapsed_ = 0.0;
    ctl.commandGripper(holdClosure_);
    mc_rtc::log::success(
        "[Acquire] stable bilateral pad contact acquired; designated pad/blue-handle pairs now enter active-contact confirmation while every non-contact safety pair remains hard closure={:.3f} left={:.4f} right={:.4f} centerErr={:.4f}",
        measuredClosure, report.leftPadClearance,
        report.rightPadClearance, report.padCenteringError);
    return false;
  }

  const bool oneSidedContact =
      minPadClear <= ctl.gripperContactTolerance()
      && maxPadClear > ctl.gripperContactTolerance();
  const double resumeTube = std::max(
      ctl.acquisitionNearCenterTolerance(),
      centerTube - closureCenterHysteresis_);
  if(oneSidedContact || report.padCenteringError > centerTube)
  {
    centeringPaused_ = true;
  }
  else if(centeringPaused_
          && report.padCenteringError <= resumeTube
          && std::abs(filteredCenterRate_) <= lockCenterRateTolerance_)
  {
    centeringPaused_ = false;
  }
  const bool pauseForCentering = centeringPaused_ || oneSidedContact;

  if(pauseForCentering) { recoveryElapsed_ += ctl.controlDt(); }
  else { recoveryElapsed_ = 0.0; }

  if(recoveryElapsed_ >= centeringRecoveryTimeout_)
  {
    mc_rtc::log::error(
        "[Acquire] centering recovery failed before contact centerErr={:.4f} centerRate={:.4f} offset={:.4f} left={:.4f} right={:.4f}",
        report.padCenteringError, filteredCenterRate_, centeringOffset_,
        report.leftPadClearance, report.rightPadClearance);
    ctl.commandGripper(measuredClosure);
    output("FAIL");
    return true;
  }

  double closeRate = ctl.gripperCloseRate();
  if(maxPadClear <= ctl.gripperContactTolerance())
  {
    closeRate = 0.0;
  }
  else if(pauseForCentering)
  {
    closeRate = 0.0;
  }
  else if(minPadClear < ctl.gripperSlowDistance())
  {
    const double den = std::max(
        1e-6, ctl.gripperSlowDistance() - ctl.gripperContactTolerance());
    const double alpha = clamp01(
        (minPadClear - ctl.gripperContactTolerance()) / den);
    closeRate = ctl.gripperMinimumCloseRate()
              + alpha * (ctl.gripperCloseRate()
                       - ctl.gripperMinimumCloseRate());
  }

  const bool nearContact = minPadClear < ctl.gripperSlowDistance();
  const double trackingLead = nearContact
      ? ctl.gripperNearContactCommandLead()
      : ctl.gripperCommandLead();
  const double leadLimited = measuredClosure + trackingLead;

  double plannedUpper = ctl.gripperMaxClosure();
  if(ctl.selectedCandidateContactClosure() >= 0.0)
  {
    plannedUpper = std::min(
        ctl.gripperMaxClosure(),
        ctl.selectedCandidateContactClosure()
          + ctl.gripperContactClosureGuard());
  }

  if(pauseForCentering)
  {
    closeCommand_ = measuredClosure;
  }
  else
  {
    const double proposed = std::min(
        plannedUpper,
        std::min(leadLimited, closeCommand_ + closeRate * ctl.controlDt()));
    closeCommand_ = std::max(
        closeCommand_, std::max(measuredClosure, proposed));
  }
  ctl.commandGripper(closeCommand_);

  const double gap = ctl.liveMouthGap();
  if(gap < bestGap_ - gapProgressEps_)
  {
    bestGap_ = gap;
    gapStallCycles_ = 0;
  }
  else if(pauseForCentering)
  {
    gapStallCycles_ = 0;
  }
  else if(closeCommand_ > 0.05
          && (closeRate > 0.0
              || measuredClosure >= closeCommand_ - 1e-3))
  {
    ++gapStallCycles_;
  }

  if(iter_ % 100 == 1)
  {
    mc_rtc::log::info(
        "[Acquire] t={:.2f}s command={:.3f} measured={:.3f} lead={:.3f} rate={:.3f} plannedUpper={:.3f} gap={:.4f} left={:.4f} right={:.4f} centerSigned={:.4f} centerRate={:.4f} tube={:.4f} actualOffset={:.4f} desiredOffset={:.4f} targetOffset={:.4f} bias={:.4f} pause={} recovery={:.2f}s clear={:.4f} contact={}",
        elapsed_, closeCommand_, measuredClosure,
        closeCommand_ - measuredClosure, closeRate, plannedUpper, gap,
        report.leftPadClearance, report.rightPadClearance,
        signedError, filteredCenterRate_, centerTube,
        actualCenteringOffset, desiredCenteringOffset_, centeringOffset_,
        centeringBias_, pauseForCentering, recoveryElapsed_,
        report.minClearance, report.bilateralPadContact);
  }

  const bool guardReached =
      measuredClosure >= plannedUpper - 0.002
      || closeCommand_ >= plannedUpper - 1e-6;
  if(guardReached && gapStallCycles_ >= gapStallLimit_)
  {
    mc_rtc::log::error(
        "[Acquire] planned closure guard exhausted without bilateral contact measured={:.3f} command={:.3f} left={:.4f} right={:.4f} centerErr={:.4f}",
        measuredClosure, closeCommand_, report.leftPadClearance,
        report.rightPadClearance, report.padCenteringError);
    ctl.commandGripper(measuredClosure);
    output("FAIL");
    return true;
  }

  if(elapsed_ >= watchdogTimeout_)
  {
    if(gapStallCycles_ >= gapStallLimit_)
    {
      mc_rtc::log::error(
          "[Acquire] emergency watchdog plus measured stall before contact t={:.2f}s closure={:.3f} left={:.4f} right={:.4f}",
          elapsed_, measuredClosure,
          report.leftPadClearance, report.rightPadClearance);
      ctl.commandGripper(measuredClosure);
      output("FAIL");
      return true;
    }
    if(!watchdogWarned_)
    {
      watchdogWarned_ = true;
      mc_rtc::log::warning(
          "[Acquire] watchdog time reached but safe geometric progress continues; event-driven closure remains active");
    }
  }

  return false;
}

void HandoverInterceptionController_Acquire::teardown(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ctl.finishPhaseTiming("acquire");
  mc_rtc::log::info("[Acquire] teardown");
}

EXPORT_SINGLE_STATE("HandoverInterceptionController_Acquire",
                    HandoverInterceptionController_Acquire)
