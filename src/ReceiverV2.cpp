// TRIAD V2: receding complete-action receiver control.
//
// Architecture (frozen, see research/triad_v2/PHASE_A_ADVERSARIAL_CHECK.md):
//
//   current robot state + independent object prediction
//   -> build complete-action bank (FULL_SEARCH, same V1 finite search)
//   -> select an ACTIVE PROVISIONAL plan (same V1 finite selector)
//   -> robot moves concurrently (provisional reference, closure unauthorized)
//   -> new state/prediction -> re-certify the active plan (RECERTIFY_ACTIVE)
//   -> retain while certified; replace only when infeasible or invalid
//   -> terminal standoff/capture region (TERMINAL_TRACK)
//   -> current-state gate + fresh TERMINAL_CERTIFY
//   -> COMMIT EXACTLY ONCE -> existing MovePregrasp/CaptureTransfer/Retreat.
//
// Nothing in this file changes V1 behaviour: every entry point requires
// receiverArchitecture == v2_receding, which also requires an independent giver.

#include "HandoverInterceptionController.h"
#include "BoundedEventLeadSchedule.h"
#include "FiniteEventPlanSelector.h"

#include <mc_rtc/logging.h>

#include <RBDyn/FK.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <queue>
#include <stdexcept>
#include <thread>

namespace
{
double clamp01V2(double x) { return std::max(0.0, std::min(1.0, x)); }

double readDouble(const mc_rtc::Configuration & c, const char * key, double fallback)
{
  if(!c.has(key)) { return fallback; }
  double v = fallback;
  c(key, v);
  return v;
}

int readInt(const mc_rtc::Configuration & c, const char * key, int fallback)
{
  if(!c.has(key)) { return fallback; }
  int v = fallback;
  c(key, v);
  return v;
}

bool readBool(const mc_rtc::Configuration & c, const char * key, bool fallback)
{
  if(!c.has(key)) { return fallback; }
  bool v = fallback;
  c(key, v);
  return v;
}
} // namespace

const char * HandoverInterceptionController::receiverJobTypeNameV2(ReceiverJobTypeV2 type)
{
  switch(type)
  {
    case ReceiverJobTypeV2::FullSearch: return "FULL_SEARCH";
    case ReceiverJobTypeV2::RecertifyActive: return "RECERTIFY_ACTIVE";
    case ReceiverJobTypeV2::TerminalCertify: return "TERMINAL_CERTIFY";
    case ReceiverJobTypeV2::CertifySelected: return "CERTIFY_SELECTED";
    case ReceiverJobTypeV2::ControlAwareSelect: return "CONTROL_AWARE_SELECT";
    default: return "NONE";
  }
}

const char * HandoverInterceptionController::receiverPhaseNameV2(ReceiverPhaseV2 phase)
{
  switch(phase)
  {
    case ReceiverPhaseV2::SearchHeld: return "SEARCH_HELD";
    case ReceiverPhaseV2::ProvisionalReach: return "PROVISIONAL_REACH";
    case ReceiverPhaseV2::TerminalTrack: return "TERMINAL_TRACK";
    case ReceiverPhaseV2::Committed: return "COMMITTED";
    case ReceiverPhaseV2::Failed: return "FAILED";
    case ReceiverPhaseV2::ControlAwareTrack: return "CONTROL_AWARE_TRACK";
    default: return "IDLE";
  }
}

sva::PTransformd HandoverInterceptionController::predictionPoseAtV2(
    const ObjectPredictionRecordV2 & record, double absoluteTime) const
{
  return propagatePoseConstantTwist(record.pose, absoluteTime - record.stamp,
                                    record.linearVelocity, record.angularVelocity);
}

HandoverInterceptionController::ObjectPredictionRecordV2
HandoverInterceptionController::currentObjectPredictionV2() const
{
  ObjectPredictionRecordV2 record;
  record.valid = objectMotionEstimateValid_ && objectPerceptionMeasurementValid_;
  record.generation = v2PredictionGeneration_;
  record.stamp = controllerTime_;
  record.measurementAge = objectPerceptionMeasurementAge_;
  record.pose = W_T_O_;
  record.linearVelocity = objectLinearVelocityEstimate_;
  record.angularVelocity = objectAngularVelocityEstimate_;
  // Presented-at-rest convention: below the presentation rest thresholds the
  // record carries the stationary model, not residual estimator decay (as V1
  // does for a STATIC observation).
  if(record.linearVelocity.norm() <= presentationMaximumLinearSpeed_
     && record.angularVelocity.norm() <= presentationMaximumAngularSpeed_)
  {
    record.linearVelocity.setZero();
    record.angularVelocity.setZero();
  }
  return record;
}

double HandoverInterceptionController::independentGiverSpeedForLogV2(double now) const
{
  // Logging only: never used by any V2 decision.
  if(!independentGiverScriptArmed_) { return 0.0; }
  return call_handover::independentGiverStateAt(
      independentGiverScript_, now - independentGiverStartTime_).linearVelocity.norm();
}

// =============================================================================
// Session
// =============================================================================

bool HandoverInterceptionController::beginReceiverV2(
    const mc_rtc::Configuration & stateConfig)
{
  const double now = controllerTime_;
  if(!receiverArchitectureV2() || !receiverArchitectureConfigurationValid_
     || !independentGiverTruth())
  {
    mc_rtc::log::error(
        "[V2ReceiverBegin] refused architecture={} giverTruthModel={} valid={}; V2 requires v2_receding with independent_scripted giver",
        receiverArchitecture_, giverTruthModel_, receiverArchitectureConfigurationValid_);
    return false;
  }

  // Same mouth-frame and gripper-geometry preparation as V1 SolveInterception.
  if(!prepareCaptureSelection())
  {
    mc_rtc::log::error("[V2ReceiverBegin] refused reason=mouth_gripper_geometry_preparation_failed");
    return false;
  }
  // calibrateMouthControlFrame() does not refresh the planner mirror (its
  // refresh call is unreachable in V1, left untouched); V1 refreshes it later
  // in resetGlobalTimePlanSearch(). Runtime safety checks read the mirror, so
  // refresh it here, on the control thread, before any V2 safety evaluation.
  refreshPlannerModel();
  refreshPlannerConfig();

  // Reuse the exact parameter values of the V1 states this architecture
  // replaces, read from the same configuration sections, so the comparison
  // changes the control architecture and nothing else.
  const auto & configs = config()("configs");
  const auto sol = configs("HandoverInterceptionController_SolveInterception");
  const auto hold = configs("HandoverInterceptionController_PresentationHold");
  const auto observe = configs("HandoverInterceptionController_ObserveObject");

  call_handover::BoundedEventLeadScheduleConfig schedule;
  schedule.boundedEventSearchEnabled = readBool(sol, "boundedEventSearchEnabled", schedule.boundedEventSearchEnabled);
  schedule.maximumEventHypotheses = readInt(sol, "maximumEventHypotheses", schedule.maximumEventHypotheses);
  schedule.eventSearchLeadStep = readDouble(sol, "eventSearchLeadStep", schedule.eventSearchLeadStep);
  schedule.attemptedLeadTolerance = readDouble(sol, "attemptedLeadTolerance", schedule.attemptedLeadTolerance);
  schedule.initialPresentationLead = readDouble(sol, "initialPresentationLead", schedule.initialPresentationLead);
  schedule.minimumPresentationLead = readDouble(sol, "minimumPresentationLead", schedule.minimumPresentationLead);
  schedule.maximumPresentationLead = readDouble(sol, "maximumPresentationLead", schedule.maximumPresentationLead);
  v2Leads_ = call_handover::buildBoundedEventLeadSchedule(schedule);

  v2Params_ = ReceiverV2Parameters{};
  v2Params_.planningStepsPerCycle = std::max(1, readInt(sol, "planningStepsPerCycle", v2Params_.planningStepsPerCycle));
  v2Params_.routeWorkUnitsPerCycle = std::max(1, readInt(sol, "routeWorkUnitsPerCycle", v2Params_.routeWorkUnitsPerCycle));
  v2Params_.minimumCommitRemainingTime = readDouble(sol, "minimumCommitRemainingTime", v2Params_.minimumCommitRemainingTime);
  v2Params_.minimumReachEntryLead = readDouble(sol, "minimumReachEntryLead", v2Params_.minimumReachEntryLead);
  v2Params_.maximumEventSearchWallTime = readDouble(sol, "maximumEventSearchWallTime", v2Params_.maximumEventSearchWallTime);
  v2Params_.holdTaskStiffness = readDouble(sol, "holdTaskStiffness", v2Params_.holdTaskStiffness);
  v2Params_.holdTaskWeight = readDouble(sol, "holdTaskWeight", v2Params_.holdTaskWeight);
  v2Params_.terminalLinearTrackingLead = readDouble(hold, "maxLinearTrackingLead", v2Params_.terminalLinearTrackingLead);
  v2Params_.terminalAngularTrackingLead = readDouble(hold, "maxAngularTrackingLead", v2Params_.terminalAngularTrackingLead);
  v2Params_.terminalPositionTolerance = readDouble(hold, "posTol", v2Params_.terminalPositionTolerance);
  v2Params_.terminalOrientationTolerance = readDouble(hold, "oriTol", v2Params_.terminalOrientationTolerance);
  v2Params_.terminalMaximumOpenClosure = readDouble(hold, "maximumOpenClosure", v2Params_.terminalMaximumOpenClosure);
  v2Params_.terminalStableDwell = readDouble(hold, "stableDwell", v2Params_.terminalStableDwell);
  v2Params_.terminalTaskStiffness = readDouble(hold, "taskStiffness", v2Params_.terminalTaskStiffness);
  v2Params_.terminalTaskWeight = readDouble(hold, "taskWeight", v2Params_.terminalTaskWeight);
  v2Params_.settleLinearSpeedTolerance = readDouble(observe, "settleLinearSpeedTolerance", v2Params_.settleLinearSpeedTolerance);
  v2Params_.settleAngularSpeedTolerance = readDouble(observe, "settleAngularSpeedTolerance", v2Params_.settleAngularSpeedTolerance);
  v2Params_.logEvery = std::max(1, readInt(stateConfig, "logEvery", v2Params_.logEvery));
  v2Params_.injectStaleTerminalResultOnce = readBool(stateConfig, "injectStaleTerminalResultOnce", false);
  v2Params_.injectStaleRecertifyResultOnce = readBool(stateConfig, "injectStaleRecertifyResultOnce", false);
  v2Params_.injectSupersedeFullSearchOnce = readBool(stateConfig, "injectSupersedeFullSearchOnce", false);
  v2Params_.characterizeOnly = readBool(stateConfig, "characterizeOnly", false);
  {
    const std::string mode = stateConfig.has("fullSearchPredictionUpdate")
        ? static_cast<std::string>(stateConfig("fullSearchPredictionUpdate"))
        : std::string("cancel_and_restart");
    if(mode != "select_then_certify" && mode != "cancel_and_restart")
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[V2ReceiverBegin] fullSearchPredictionUpdate must be select_then_certify or cancel_and_restart, got {}", mode);
    }
    v2SelectThenCertify_ = mode == "select_then_certify";
    v2ExactTimingPrune_ = readBool(stateConfig, "fullSearchExactTimingPrune", false);
    plannerConfig_.v2ExactTimingPrune = v2ExactTimingPrune_;
    plannerConfig_.v2PruneCommitLead = std::max(
        v2Params_.minimumCommitRemainingTime, presentationDecelerationDuration_ + 0.25);
    plannerConfig_.v2PruneEntryLead = v2Params_.minimumReachEntryLead;
    v2ControllerClockForWorker_.store(controllerTime_, std::memory_order_release);
    mc_rtc::log::info("[V2ExactTimingPrune] enabled={} commitLead={:.3f}s entryLead={:.3f}s",
                      v2ExactTimingPrune_, plannerConfig_.v2PruneCommitLead, plannerConfig_.v2PruneEntryLead);
    mc_rtc::log::info("[V2PredictionUpdateMode] fullSearchPredictionUpdate={}", mode);
  }
  v2Params_.characterizationRestDwell = readDouble(stateConfig, "characterizationRestDwell", v2Params_.characterizationRestDwell);
  // Phase 2B preview/runtime parity traces (logging only, default off).
  v2ParityTrace_ = readBool(stateConfig, "parityTrace", false);
  parityRuntimeTraceActive_ = v2ParityTrace_;
  v2ParityReachTrace_.clear();
  v2ParityReachTracePlanId_ = 0;
  v2ParityReachLoggedPlanId_ = 0;
  v2ParityLastRuntimeLog_ = -1.0;
  mc_rtc::log::info("[V2ParityTrace] enabled={} decisionEffect=none", v2ParityTrace_);
  v2CharacterizationStage_ = 0;
  if(!loadControlAwareConfigV2(stateConfig))
  {
    return false;
  }

  v2Active_ = true;
  v2EstimationActive_ = true;
  v2Phase_ = ReceiverPhaseV2::SearchHeld;
  v2StateGeneration_ = 1;
  v2PlanIdCounter_ = 0;
  provisionalReceiverPlan_ = ProvisionalReceiverPlanV2{};
  v2Pending_ = PendingJobV2{};
  v2LatestTerminalCertificateValid_ = false;
  v2CommitLatched_ = false;
  v2CommitCount_ = 0;
  v2ReselectionLocked_ = false;
  v2Counters_ = ReceiverV2Counters{};
  v2GateStableSince_ = -1.0;
  v2SearchBudgetStart_ = now;
  v2ObjectQuasiStaticSince_ = -1.0;
  v2PhaseEntryTime_ = now;
  v2HoldPose_ = actualMouthPose();
  v2ReferencePose_ = v2HoldPose_;
  v2PreviousMouthPose_ = v2HoldPose_;
  v2PreviousMouthTime_ = now;
  v2HoldArmPosture_ = currentArmPosture();
  v2ReplacementReason_ = "initial";
  v2StaleTerminalInjected_ = false;
  v2StaleRecertifyInjected_ = false;
  v2SupersedeInjected_ = false;
  v2FirstMotionTime_ = -1.0;

  setGripperClosureAuthorized(false);
  activateToolTask();
  setToolTaskGains(v2Params_.holdTaskStiffness, v2Params_.holdTaskWeight);
  commandMouthTarget(v2HoldPose_);
  commandReadyArmPosture(v2HoldArmPosture_);
  setGripperJointPriority(false);
  commandGripper(0.0);

  std::string leads;
  for(double l : v2Leads_) { leads += (leads.empty() ? "" : ",") + std::to_string(l).substr(0, 4); }
  mc_rtc::log::success(
      "[V2ReceiverBegin] architecture=v2_receding giverTruthModel={} predictor=constant_twist_no_stop_assumption eventLeads={} leads=[{}] graspSamples={} routeGenerators={} commitBoundary=terminal_capture closureAuthorizedBeforeCommit=false t={:.6f}",
      giverTruthModel_, v2Leads_.size(), leads, 2 * std::max(4, candidateCount_),
      1 + (transitPlanningEnabled_ ? transitRouteDirections_ * static_cast<int>(transitRouteApexOffsets_.size()) : 0),
      now);
  return true;
}

void HandoverInterceptionController::endReceiverV2()
{
  shutdownPlannerWorker();
  if(v2ControlAware_)
  {
    mc_rtc::log::success(
        "[TriadLiteSummary] selectionsSubmitted={} selectionsProcessed={} graspSwitches={} abortsToHold={} admitsDeferred={} frozen={} selectedGraspId={}",
        v2CaSelectionsSubmitted_, v2CaSelectionsProcessed_, v2CaSwitches_, v2CaAborts_, v2CaAdmitsDeferred_,
        v2CaSelector_.frozen, v2CaSelector_.incumbentId);
  }
  v2Active_ = false;
  if(!v2CommitLatched_) { v2EstimationActive_ = false; }
  mc_rtc::log::success(
      "[V2ReceiverSummary] phase={} commitCount={} adoptions={} replacements={} retained={} updated={} staleRejected={} cancelRequested={} cancelled={} fullSearches={} recertifications={} terminalCertifications={} generationsWhileRobotMoving={} generationsWhileObjectMoving={} searchFailures={} minRuntimeClearance={:.4f} reselectionLocked={}",
      receiverPhaseNameV2(v2Phase_), v2CommitCount_, v2Counters_.adoptions,
      v2Counters_.replacements, v2Counters_.retained, v2Counters_.updated,
      v2Counters_.staleRejected, v2Counters_.cancelRequested, v2Counters_.cancelled,
      v2Counters_.fullSearchSubmitted,
      v2Counters_.recertifySubmitted, v2Counters_.terminalSubmitted,
      v2Counters_.generationsWhileRobotMoving, v2Counters_.generationsWhileObjectMoving,
      v2Counters_.searchFailures, v2Counters_.minimumRuntimeClearance, v2ReselectionLocked_);
}

// =============================================================================
// Control-thread supervisor
// =============================================================================

HandoverInterceptionController::ReceiverStepStatusV2
HandoverInterceptionController::stepReceiverV2()
{
  const double now = controllerTime_;
  if(!v2Active_) { return ReceiverStepStatusV2::Failed; }
  v2ControllerClockForWorker_.store(now, std::memory_order_release);

  // A provisional plan never authorizes closure.
  setGripperClosureAuthorized(false);

  const sva::PTransformd mouth = actualMouthPose();
  const double dt = std::max(1e-9, now - v2PreviousMouthTime_);
  if(v2PreviousMouthTime_ >= 0.0 && now > v2PreviousMouthTime_)
  {
    v2MouthLinearSpeed_ = (mouth.translation() - v2PreviousMouthPose_.translation()).norm() / dt;
    v2MouthAngularSpeed_ = orientationError(mouth, v2PreviousMouthPose_) / dt;
  }
  v2PreviousMouthPose_ = mouth;
  v2PreviousMouthTime_ = now;

  if(v2Pending_.active && !v2Pending_.cancelRequested && !v2Params_.characterizeOnly)
  {
    checkSupersessionV2(now);
  }
  if(v2Pending_.active)
  {
    const auto job = plannerJobState();
    if(job == PlannerJobState::Ready || job == PlannerJobState::Failed)
    {
      processReceiverJobResultV2(now);
    }
  }
  if(v2Phase_ == ReceiverPhaseV2::Failed) { return ReceiverStepStatusV2::Failed; }

  const bool workerIdle = !v2Pending_.active && plannerJobState() != PlannerJobState::Running;

  switch(v2Phase_)
  {
    case ReceiverPhaseV2::SearchHeld:
    {
      activateToolTask();
      setToolTaskGains(v2Params_.holdTaskStiffness, v2Params_.holdTaskWeight);
      commandMouthTarget(v2HoldPose_);
      commandReadyArmPosture(v2HoldArmPosture_);
      setGripperJointPriority(false);
      commandGripper(0.0);
      HandoverSafetyReport report;
      if(!evaluateCurrentPoseSafety(report, false))
      {
        mc_rtc::log::error("[V2Failure] reason=unsafe_while_held clear={:.4f} limiting={}/{}",
                           report.minClearance, report.sample, report.obstacle);
        v2Phase_ = ReceiverPhaseV2::Failed;
        return ReceiverStepStatusV2::Failed;
      }
      // No-plan bound. maximumEventSearchWallTime is a per-search compute
      // budget in V1 and is not a handover-opportunity bound. Before commitment
      // the receiver keeps searching from a held, safe state while the giver may
      // still present; it fails closed without a plan only when the observed
      // presentation has been quasi-static for longer than V1's post-stop
      // acquisition window (presentationAcquisitionWindow).
      const bool objectQuasiStatic =
          objectLinearVelocityEstimate_.norm() <= presentationMaximumLinearSpeed_
          && objectAngularVelocityEstimate_.norm() <= presentationMaximumAngularSpeed_;
      if(!objectQuasiStatic) { v2ObjectQuasiStaticSince_ = -1.0; }
      else if(v2ObjectQuasiStaticSince_ < 0.0) { v2ObjectQuasiStaticSince_ = now; }
      if(v2Params_.characterizeOnly)
      {
        const bool settledArm = v2MouthLinearSpeed_ <= v2Params_.settleLinearSpeedTolerance
            && v2MouthAngularSpeed_ <= v2Params_.settleAngularSpeedTolerance;
        if(workerIdle && settledArm && v2CharacterizationStage_ == 0)
        {
          v2CharacterizationStage_ = 1;
          submitReceiverFullSearchV2(now);
        }
        else if(workerIdle && settledArm && v2CharacterizationStage_ == 2
                && v2ObjectQuasiStaticSince_ >= 0.0
                && now - v2ObjectQuasiStaticSince_ >= v2Params_.characterizationRestDwell)
        {
          v2CharacterizationStage_ = 3;
          submitReceiverFullSearchV2(now);
        }
        else if(workerIdle && v2CharacterizationStage_ >= 4)
        {
          mc_rtc::log::warning(
              "[V2CharacterizationComplete] searches=2 adopted=0 t={:.6f} outcome=FAIL_BY_DESIGN", now);
          v2Phase_ = ReceiverPhaseV2::Failed;
          return ReceiverStepStatusV2::Failed;
        }
        break;
      }
      if(v2ObjectQuasiStaticSince_ >= 0.0
         && now - v2ObjectQuasiStaticSince_ > presentationAcquisitionWindow_)
      {
        mc_rtc::log::error(
            "[V2Failure] reason=no_certified_provisional_plan_within_presentation_window window={:.3f}s quasiStaticSince={:.6f} searchFailures={} fullSearches={}",
            presentationAcquisitionWindow_, v2ObjectQuasiStaticSince_,
            v2Counters_.searchFailures, v2Counters_.fullSearchSubmitted);
        v2Phase_ = ReceiverPhaseV2::Failed;
        return ReceiverStepStatusV2::Failed;
      }
      // The full bank is certified from a held arm: every rollout seeds from
      // the frozen configuration at rest and a start mouth pose that the arm
      // must still occupy at adoption.
      const bool settled = v2MouthLinearSpeed_ <= v2Params_.settleLinearSpeedTolerance
          && v2MouthAngularSpeed_ <= v2Params_.settleAngularSpeedTolerance;
      if(workerIdle && settled)
      {
        if(v2ControlAware_) { submitReceiverCertificationV2(ReceiverJobTypeV2::ControlAwareSelect, now); }
        else { submitReceiverFullSearchV2(now); }
      }
      break;
    }
    case ReceiverPhaseV2::ProvisionalReach:
    {
      if(!executeProvisionalReachV2(now))
      {
        v2Phase_ = ReceiverPhaseV2::Failed;
        return ReceiverStepStatusV2::Failed;
      }
      if(now >= provisionalReceiverPlan_.plan.standoffTime)
      {
        ++v2StateGeneration_;
        v2Phase_ = ReceiverPhaseV2::TerminalTrack;
        v2PhaseEntryTime_ = now;
        v2GateStableSince_ = -1.0;
        v2LatestTerminalCertificateValid_ = false;
        mc_rtc::log::success(
            "[V2Phase] phase=TERMINAL_TRACK planId={} stateGeneration={} t={:.6f} tau={:.6f}",
            provisionalReceiverPlan_.planId, v2StateGeneration_, now,
            provisionalReceiverPlan_.plan.presentationTime);
      }
      else if(workerIdle)
      {
        submitReceiverCertificationV2(ReceiverJobTypeV2::RecertifyActive, now);
      }
      break;
    }
    case ReceiverPhaseV2::TerminalTrack:
    {
      bool gate = false;
      if(!executeTerminalTrackV2(now, gate))
      {
        v2Phase_ = ReceiverPhaseV2::Failed;
        return ReceiverStepStatusV2::Failed;
      }
      if(now > provisionalReceiverPlan_.plan.presentationTime + presentationAcquisitionWindow_)
      {
        mc_rtc::log::error(
            "[V2Failure] reason=terminal_presentation_window_expired planId={} window={:.3f}s",
            provisionalReceiverPlan_.planId, presentationAcquisitionWindow_);
        v2Phase_ = ReceiverPhaseV2::Failed;
        return ReceiverStepStatusV2::Failed;
      }
      if(gate)
      {
        if(v2GateStableSince_ < 0.0) { v2GateStableSince_ = now; }
      }
      else
      {
        v2GateStableSince_ = -1.0;
      }
      const bool gateStable = gate && now - v2GateStableSince_ + 1e-12 >= v2Params_.terminalStableDwell;
      const auto & cert = v2LatestTerminalCertificate_;
      const bool certificateUsable = v2LatestTerminalCertificateValid_ && gateStable
          && cert.stateGeneration == v2StateGeneration_
          && cert.planId == provisionalReceiverPlan_.planId
          && cert.snapshotTime >= v2GateStableSince_ - 1e-12;
      if(certificateUsable)
      {
        if(commitProvisionalReceiverPlanV2(cert, now))
        {
          return ReceiverStepStatusV2::Committed;
        }
        if(v2Phase_ == ReceiverPhaseV2::Failed) { return ReceiverStepStatusV2::Failed; }
        v2LatestTerminalCertificateValid_ = false;
      }
      if(workerIdle && gate)
      {
        submitReceiverCertificationV2(ReceiverJobTypeV2::TerminalCertify, now);
      }
      else if(workerIdle && now - v2PhaseEntryTime_ > 0.0)
      {
        // Keep the active plan certified while waiting for presentation.
        submitReceiverCertificationV2(ReceiverJobTypeV2::TerminalCertify, now);
      }
      break;
    }
    case ReceiverPhaseV2::ControlAwareTrack:
    {
      const int status = stepControlAwareTrackV2(now, workerIdle);
      if(status < 0)
      {
        v2Phase_ = ReceiverPhaseV2::Failed;
        return ReceiverStepStatusV2::Failed;
      }
      if(status > 0) { return ReceiverStepStatusV2::Committed; }
      break;
    }
    default:
      break;
  }

  logReceiverMotionV2(now, false);
  return v2Phase_ == ReceiverPhaseV2::Failed ? ReceiverStepStatusV2::Failed
                                              : ReceiverStepStatusV2::Running;
}

void HandoverInterceptionController::logReceiverMotionV2(double now, bool force)
{
  if(!force && v2LastMotionLogTime_ >= 0.0 && now < v2LastMotionLogTime_ + 0.05 - 1e-9) { return; }
  v2LastMotionLogTime_ = now;
  const double objectEstimateSpeed = objectLinearVelocityEstimate_.norm();
  if(v2FirstMotionTime_ < 0.0 && v2MouthLinearSpeed_ > v2Params_.settleLinearSpeedTolerance
     && independentGiverSpeedForLogV2(now) > presentationMaximumLinearSpeed_)
  {
    v2FirstMotionTime_ = now;
  }
  mc_rtc::log::info(
      "[V2Motion] t={:.6f} phase={} planId={} stateGeneration={} mouthSpeed={:.5f} objectTruthSpeed={:.5f} objectEstimateSpeed={:.5f} robotMoving={} objectMoving={} pendingJob={} pendingGeneration={} closureAuthorized={}",
      now, receiverPhaseNameV2(v2Phase_), provisionalReceiverPlan_.planId, v2StateGeneration_,
      v2MouthLinearSpeed_, independentGiverSpeedForLogV2(now), objectEstimateSpeed,
      v2MouthLinearSpeed_ > v2Params_.settleLinearSpeedTolerance,
      independentGiverSpeedForLogV2(now) > presentationMaximumLinearSpeed_,
      v2Pending_.active ? receiverJobTypeNameV2(v2Pending_.type) : "none",
      v2Pending_.planningGeneration, gripperClosureAuthorized());
}

// =============================================================================
// Job submission (control thread; latest-only, one job in flight)
// =============================================================================

bool HandoverInterceptionController::submitReceiverFullSearchV2(double now)
{
  if(v2ReselectionLocked_)
  {
    mc_rtc::log::error("[V2PostCommitReselectionRefused] job=FULL_SEARCH t={:.6f}", now);
    return false;
  }
  if(v2Pending_.active || plannerJobState() == PlannerJobState::Running) { return false; }
  const ObjectPredictionRecordV2 prediction = currentObjectPredictionV2();
  if(!prediction.valid) { return false; }

  resetGlobalTimePlanSearch(now, true);
  FrozenEventBank bank;
  bank.searchEpoch = now;
  bank.configuredHypotheses = v2Leads_.size();
  // The shared search checks the hypothesis budget before it detects the end
  // of the schedule, so the budget must exceed the lead count for the search
  // to complete (V1: maximumEventHypotheses 15 for 14 leads). leads + 1 equals
  // the previous value (15) for the default 14-lead bank.
  bank.maximumHypotheses = static_cast<int>(v2Leads_.size()) + 1;
  bank.maximumSearchWallTime = v2Params_.maximumEventSearchWallTime;
  bank.minimumSafeCommitLead = std::max(v2Params_.minimumCommitRemainingTime,
                                        presentationDecelerationDuration_ + 0.25);
  if(v2Params_.characterizeOnly)
  {
    // Characterization evaluates geometry at every lead; timing admission is
    // applied afterwards by the analysis with the same selector constants.
    bank.minimumSafeCommitLead = 0.0;
  }
  bank.source = "v2_receding_fixed_schedule";
  for(const double lead : v2Leads_)
  {
    bank.leads.push_back(lead);
    bank.presentationPoses.push_back(predictionPoseAtV2(prediction, now + lead));
  }
  submitFiniteTriadSearch(bank, now, v2Params_.planningStepsPerCycle,
                          v2Params_.routeWorkUnitsPerCycle);

  v2Pending_ = PendingJobV2{};
  v2Pending_.active = true;
  v2Pending_.type = ReceiverJobTypeV2::FullSearch;
  v2Pending_.planningGeneration = plannerRequestGeneration();
  v2Pending_.stateGeneration = v2StateGeneration_;
  v2Pending_.planId = provisionalReceiverPlan_.planId;
  v2Pending_.submitTime = now;
  v2Pending_.prediction = prediction;
  if(!previewMouthPose(planningSnapshot_.frozenRobotState, v2Pending_.snapshotMouthPose))
  {
    v2Pending_.snapshotMouthPose = actualMouthPose();
  }
  v2Pending_.bankPoses = bank.presentationPoses;
  for(const double lead : bank.leads) { v2Pending_.bankEventTimes.push_back(now + lead); }
  ++v2Counters_.fullSearchSubmitted;
  const bool robotMoving = v2MouthLinearSpeed_ > v2Params_.settleLinearSpeedTolerance;
  const bool objectMoving = prediction.linearVelocity.norm() > presentationMaximumLinearSpeed_;
  if(robotMoving) { ++v2Counters_.generationsWhileRobotMoving; }
  if(objectMoving) { ++v2Counters_.generationsWhileObjectMoving; }
  mc_rtc::log::success(
      "[V2PlanningJobSubmit] type=FULL_SEARCH planningGeneration={} stateGeneration={} predictionGeneration={} activePlanId={} t={:.6f} mouthSpeed={:.5f} objectEstimateSpeed={:.5f} objectTruthSpeed={:.5f} robotMoving={} objectMoving={} eventLeads={} snapshot=frozen_robot_state",
      v2Pending_.planningGeneration, v2StateGeneration_, prediction.generation,
      provisionalReceiverPlan_.planId, now, v2MouthLinearSpeed_,
      prediction.linearVelocity.norm(), independentGiverSpeedForLogV2(now),
      robotMoving, objectMoving, v2Leads_.size());
  return true;
}

bool HandoverInterceptionController::submitReceiverCertificationV2(
    ReceiverJobTypeV2 type, double now, const CaptureCandidate * candidate, const InterceptionPlan * plan)
{
  if(v2ReselectionLocked_)
  {
    mc_rtc::log::error("[V2PostCommitReselectionRefused] job={} t={:.6f}",
                       receiverJobTypeNameV2(type), now);
    return false;
  }
  if(v2Pending_.active || plannerJobState() == PlannerJobState::Running) { return false; }
  if(type != ReceiverJobTypeV2::CertifySelected && type != ReceiverJobTypeV2::ControlAwareSelect
     && !provisionalReceiverPlan_.valid)
  {
    return false;
  }
  if(type == ReceiverJobTypeV2::CertifySelected && (candidate == nullptr || plan == nullptr)) { return false; }
  const ObjectPredictionRecordV2 prediction = currentObjectPredictionV2();
  if(!prediction.valid) { return false; }

  // Immutable planning snapshot of every live input the worker needs.
  shutdownPlannerWorker();
  refreshPlannerModel();
  refreshPlannerConfig();
  planningSnapshot_.searchEpoch = now;
  planningSnapshot_.frozenRobotState = robot().mbc();
  planningSnapshot_.frozenRobotStateValid = true;
  planningSnapshot_.mouthToBaseTransformValid = frozenMouthToBaseTransform(
      planningSnapshot_.frozenRobotState, planningSnapshot_.mouthToBaseTransform);
  if(!planningSnapshot_.mouthToBaseTransformValid)
  {
    planningSnapshot_.mouthToBaseTransform = liveMouthToBaseTransform();
    planningSnapshot_.mouthToBaseTransformValid = true;
  }

  ReceiverJobRequestV2 request;
  request.type = type;
  request.stateGeneration = v2StateGeneration_;
  request.planId = provisionalReceiverPlan_.planId;
  request.snapshotTime = now;
  request.prediction = prediction;
  request.candidate = candidate != nullptr ? *candidate : provisionalReceiverPlan_.candidate;
  request.plan = plan != nullptr ? *plan : provisionalReceiverPlan_.plan;
  if(!previewMouthPose(planningSnapshot_.frozenRobotState, request.snapshotMouthPose))
  {
    request.snapshotMouthPose = actualMouthPose();
  }
  request.terminalObjectPose = predictionPoseAtV2(prediction, now);
  request.routeWorkUnits = v2Params_.routeWorkUnitsPerCycle;

  const std::uint64_t generation =
      plannerRequestGeneration_.fetch_add(1, std::memory_order_acq_rel) + 1;
  request.planningGeneration = generation;
  v2Request_ = request;
  plannerCancel_.store(false, std::memory_order_relaxed);
  plannerFailureReason_.clear();
  plannerWorkerStartWall_ = std::chrono::steady_clock::now();
  plannerJobState_.store(static_cast<int>(PlannerJobState::Running), std::memory_order_release);
  plannerThread_ = std::thread([this, generation]() { runReceiverWorkerJobV2(generation); });

  v2Pending_ = PendingJobV2{};
  v2Pending_.active = true;
  v2Pending_.type = type;
  v2Pending_.planningGeneration = generation;
  v2Pending_.stateGeneration = v2StateGeneration_;
  v2Pending_.planId = provisionalReceiverPlan_.planId;
  v2Pending_.submitTime = now;
  v2Pending_.prediction = prediction;
  v2Pending_.snapshotMouthPose = request.snapshotMouthPose;
  if(type == ReceiverJobTypeV2::RecertifyActive || type == ReceiverJobTypeV2::CertifySelected)
  {
    v2Pending_.bankEventTimes.push_back(request.plan.presentationTime);
    v2Pending_.bankPoses.push_back(predictionPoseAtV2(prediction, request.plan.presentationTime));
  }
  else
  {
    v2Pending_.bankEventTimes.push_back(now);
    v2Pending_.bankPoses.push_back(request.terminalObjectPose);
  }
  if(type == ReceiverJobTypeV2::CertifySelected) { ++v2SelectedCertifications_; }
  else if(type == ReceiverJobTypeV2::ControlAwareSelect) { ++v2CaSelectionsSubmitted_; }
  else if(type == ReceiverJobTypeV2::RecertifyActive) { ++v2Counters_.recertifySubmitted; }
  else { ++v2Counters_.terminalSubmitted; }
  const bool robotMoving = v2MouthLinearSpeed_ > v2Params_.settleLinearSpeedTolerance;
  const bool objectMoving = prediction.linearVelocity.norm() > presentationMaximumLinearSpeed_;
  if(robotMoving) { ++v2Counters_.generationsWhileRobotMoving; }
  if(objectMoving) { ++v2Counters_.generationsWhileObjectMoving; }
  const Eigen::Vector3d ps = request.snapshotMouthPose.translation();
  mc_rtc::log::info(
      "[V2PlanningJobSubmit] type={} planningGeneration={} stateGeneration={} predictionGeneration={} activePlanId={} t={:.6f} mouthSpeed={:.5f} objectEstimateSpeed={:.5f} objectTruthSpeed={:.5f} robotMoving={} objectMoving={} snapshotMouth=[{:.4f},{:.4f},{:.4f}] snapshot=frozen_robot_state",
      receiverJobTypeNameV2(type), generation, v2StateGeneration_, prediction.generation,
      provisionalReceiverPlan_.planId, now, v2MouthLinearSpeed_,
      prediction.linearVelocity.norm(), independentGiverSpeedForLogV2(now),
      robotMoving, objectMoving, ps.x(), ps.y(), ps.z());
  return true;
}

// =============================================================================
// Worker (reads only v2Request_, planningSnapshot_, plannerConfig_, plannerContext_)
// =============================================================================

void HandoverInterceptionController::runReceiverWorkerJobV2(std::uint64_t generation)
{
  setPlannerWorkerThreadFlag(true);
  const auto start = std::chrono::steady_clock::now();
  const ReceiverJobRequestV2 & request = v2Request_;
  resetStageProfileV2(generation, receiverJobTypeNameV2(request.type));
  ReceiverJobResultV2 result;
  result.type = request.type;
  result.planningGeneration = request.planningGeneration;
  result.stateGeneration = request.stateGeneration;
  result.planId = request.planId;
  result.snapshotTime = request.snapshotTime;
  result.prediction = request.prediction;
  result.snapshotMouthPose = request.snapshotMouthPose;
  try
  {
    if(!planningSnapshot_.frozenRobotStateValid)
    {
      throw std::runtime_error("receiver worker requires a frozen robot state");
    }
    refreshPreviewKinematicCache();
    if(request.type == ReceiverJobTypeV2::RecertifyActive
       || request.type == ReceiverJobTypeV2::CertifySelected)
    {
      runRecertifyActiveRolloutV2(result);
    }
    else if(request.type == ReceiverJobTypeV2::TerminalCertify)
    {
      runTerminalCertificationV2(result);
    }
    else if(request.type == ReceiverJobTypeV2::ControlAwareSelect)
    {
      runControlAwareSelectionV2(result);
    }
    else
    {
      result.reason = "unsupported_job_type";
    }
  }
  catch(const std::exception & e)
  {
    result.success = false;
    result.reason = std::string("worker_exception/") + e.what();
  }
  result.wallDuration = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();
  v2Result_ = result;
  plannerWorkerWallDuration_ = result.wallDuration;
  plannerContext_.certJobWall = result.wallDuration;
  plannerResultGeneration_.store(generation, std::memory_order_release);
  plannerJobState_.store(static_cast<int>(PlannerJobState::Ready), std::memory_order_release);
  setPlannerWorkerThreadFlag(false);
}

HandoverInterceptionController::RouteStepOutcome
HandoverInterceptionController::runRouteStepToCompletionV2()
{
  RouteStepOutcome outcome = RouteStepOutcome::Running;
  while(outcome == RouteStepOutcome::Running)
  {
    if(plannerCancel_.load(std::memory_order_relaxed))
    {
      return routeStepFail("v2/cancelled");
    }
    outcome = stepPredictiveRouteCandidate(std::max(1, v2Request_.routeWorkUnits));
  }
  return outcome;
}

void HandoverInterceptionController::runRecertifyActiveRolloutV2(ReceiverJobResultV2 & result)
{
  const ReceiverJobRequestV2 & request = v2Request_;
  CaptureCandidate candidate = request.candidate;
  InterceptionPlan plan = request.plan;
  const double tSnap = request.snapshotTime;

  // Newest independent prediction at the plan's (unchanged) interception time.
  const sva::PTransformd presentation = predictionPoseAtV2(request.prediction, plan.presentationTime);
  plan.objectAtPresentation = presentation;
  plan.objectAtContact = presentation;
  plan.objectLinearVelocity = request.prediction.linearVelocity;
  plan.objectAngularVelocity = request.prediction.angularVelocity;
  candidate.W_T_M_standoff = compose(presentation, plan.O_T_M_standoff);
  candidate.W_T_M_transit = candidate.W_T_M_standoff;
  candidate.W_T_M_pre = compose(presentation, plan.O_T_M_capture);
  candidate.W_T_M_retreat = compose(presentation, plan.O_T_M_retreat);
  result.plan = plan;
  result.objectPose = presentation;
  result.candidate = candidate;

  std::string why;
  if(!validateInterceptionPlan(plan, &why, false))
  {
    result.reason = "v2_recertify/invalid_plan/" + why;
    return;
  }

  plannerContext_.W_T_O = presentation;
  plannerContext_.W_T_H = compose(presentation, O_T_H_);
  plannerContext_.planningM_T_O = sva::PTransformd::Identity();
  plannerContext_.plannerWorldActive = true;
  plannerContext_.planningStartMouthPose = plan.mouthAtReachStart;
  plannerContext_.planningStartMouthPoseValid = true;

  plannerContext_.routeStepCandidate = candidate;
  plannerContext_.routeStepPlan = plan;
  plannerContext_.routeStepPresentationAnchor = presentation;
  plannerContext_.routeStepSavedObject = plannerContext_.W_T_O;
  plannerContext_.routeStepSavedHandle = plannerContext_.W_T_H;
  plannerContext_.routeStepSavedPlanningAttachment = plannerContext_.planningM_T_O;
  plannerContext_.routeStepMbc = planningSnapshot_.frozenRobotState;
  for(auto & a : plannerContext_.routeStepMbc.alpha) { std::fill(a.begin(), a.end(), 0.0); }
  for(auto & aD : plannerContext_.routeStepMbc.alphaD) { std::fill(aD.begin(), aD.end(), 0.0); }
  setPreviewGripperClosure(plannerContext_.routeStepMbc, 0.0);

  plannerContext_.routeStepReachResult = PreviewResult();
  plannerContext_.routeStepReachResult.minClearance = std::numeric_limits<double>::infinity();
  plannerContext_.routeStepReachIteration = 0;
  const int steps = std::max(1, static_cast<int>(std::ceil(plan.reachDuration / plannerConfig_.previewDt)));
  // CERTIFY_SELECTED certifies a not-yet-adopted action from the held arm, so
  // like the FULL_SEARCH it starts at reach index 0 from the planned start pose.
  const bool fromStart = request.type == ReceiverJobTypeV2::CertifySelected;
  const int resumeIndex = fromStart ? 0 : std::max(0, std::min(steps, static_cast<int>(
      std::floor((tSnap - plan.reachStartTime) / plannerConfig_.previewDt))));
  plannerContext_.routeStepReachIndex = resumeIndex;
  plannerContext_.routeStepReachSteps = steps;
  // Resume the same reference from where the moving arm actually is.
  plannerContext_.routeStepCommandReference = (fromStart || tSnap < plan.reachStartTime)
      ? plan.mouthAtReachStart : request.snapshotMouthPose;
  plannerContext_.routeStepClearanceScale = 1.0;
  plannerContext_.routeStepTransitPostureSaved = false;
  plannerContext_.routeStepPhase = RouteStepPhase::Reach;

  plannerContext_.routeStepReachPositionError = std::numeric_limits<double>::quiet_NaN();
  plannerContext_.routeStepReachOrientationError = std::numeric_limits<double>::quiet_NaN();
  plannerContext_.parityTrace.clear();
  plannerContext_.parityTraceActive = v2ParityTrace_;
  const RouteStepOutcome outcome = runRouteStepToCompletionV2();
  result.parityFromReachStart = v2ParityTrace_ && resumeIndex == 0;
  plannerContext_.parityTraceActive = false;
  result.parityTrace.swap(plannerContext_.parityTrace);
  result.candidate = plannerContext_.routeStepCandidate;
  result.success = outcome == RouteStepOutcome::Feasible
      && result.candidate.completeCostAuditValid;
  result.reason = result.success ? "certified"
      : (outcome != RouteStepOutcome::Feasible ? result.candidate.failureReason
                                              : std::string("cost_invalid/") + result.candidate.terminalTimingAuditReason);
  logCertStageV2("recert", result.candidate, outcome == RouteStepOutcome::Feasible,
                 routeDeepestStageV2(outcome == RouteStepOutcome::Feasible),
                 std::numeric_limits<double>::quiet_NaN(), plan.reachDuration, result.reason);
  mc_rtc::log::info(
      "[V2CertificationDetail] type={} planId={} planningGeneration={} candidate={} route={} resumeIndex={}/{} success={} reason={} stageReached={} reachEndPositionError={:.5f} reachEndOrientationError={:.5f} snapshotMouthDeviationFromPlanStart={:.5f}",
      receiverJobTypeNameV2(request.type), request.planId, request.planningGeneration, candidate.name, candidate.transitRouteName,
      resumeIndex, steps, result.success, result.reason,
      outcome == RouteStepOutcome::Feasible ? "complete_through_carried_retreat" : "rejected",
      plannerContext_.routeStepReachPositionError, plannerContext_.routeStepReachOrientationError,
      (request.snapshotMouthPose.translation() - plan.mouthAtReachStart.translation()).norm());
}

void HandoverInterceptionController::runTerminalCertificationV2(ReceiverJobResultV2 & result)
{
  const ReceiverJobRequestV2 & request = v2Request_;
  const sva::PTransformd objectPose = request.terminalObjectPose;
  CaptureCandidate candidate = request.candidate;
  InterceptionPlan plan = request.plan;
  candidate.W_T_M_standoff = compose(objectPose, plan.O_T_M_standoff);
  candidate.W_T_M_transit = candidate.W_T_M_standoff;
  candidate.W_T_M_pre = compose(objectPose, plan.O_T_M_capture);
  candidate.W_T_M_retreat = compose(objectPose, plan.O_T_M_retreat);
  plan.objectAtPresentation = objectPose;
  plan.objectAtContact = objectPose;
  result.objectPose = objectPose;
  result.plan = plan;
  result.candidate = candidate;

  plannerContext_.W_T_O = objectPose;
  plannerContext_.W_T_H = compose(objectPose, O_T_H_);
  plannerContext_.planningM_T_O = sva::PTransformd::Identity();
  plannerContext_.plannerWorldActive = true;

  rbd::MultiBodyConfig mbc = planningSnapshot_.frozenRobotState;
  for(auto & a : mbc.alpha) { std::fill(a.begin(), a.end(), 0.0); }
  for(auto & aD : mbc.alphaD) { std::fill(aD.begin(), aD.end(), 0.0); }
  setPreviewGripperClosure(mbc, 0.0);

  // Remaining terminal feasibility from the current state, with the object at
  // its current estimate: standoff, then the unchanged V1 terminal chain
  // (corridor insertion, capture dwell, closure/contact sweep, carried retreat,
  // terminal timing audit).
  PreviewResult reach;
  reach.minClearance = std::numeric_limits<double>::infinity();
  bool standoffReached = false;
  {
    TRIAD_V2_STAGE_TIMER(StageTerminalStandoff);
    standoffReached = previewReachSegment(mbc, candidate.W_T_M_standoff, false, false, reach, true);
  }
  if(!standoffReached)
  {
    candidate.predictiveReachClearance = reach.minClearance;
    logCertStageV2("terminal", candidate, false, "NONE", std::numeric_limits<double>::quiet_NaN(),
                   std::numeric_limits<double>::quiet_NaN(), "v2_terminal/standoff/" + reach.reason);
    result.reason = "v2_terminal/standoff/" + reach.reason;
    mc_rtc::log::info(
        "[V2CertificationDetail] type=TERMINAL_CERTIFY planId={} planningGeneration={} candidate={} success=false reason={} stageReached=standoff",
        request.planId, request.planningGeneration, candidate.name, result.reason);
    return;
  }
  candidate.predictiveReachClearance = reach.minClearance;

  plannerContext_.routeStepCandidate = candidate;
  plannerContext_.routeStepPlan = plan;
  plannerContext_.routeStepPresentationAnchor = objectPose;
  plannerContext_.routeStepSavedObject = plannerContext_.W_T_O;
  plannerContext_.routeStepSavedHandle = plannerContext_.W_T_H;
  plannerContext_.routeStepSavedPlanningAttachment = plannerContext_.planningM_T_O;
  plannerContext_.routeStepMbc = mbc;
  plannerContext_.routeStepTimingAuditStartMbc = mbc;
  plannerContext_.routeStepReachResult = reach;
  plannerContext_.routeStepTerminalResult = PreviewResult();
  plannerContext_.routeStepTerminalResult.minClearance = std::numeric_limits<double>::infinity();
  plannerContext_.routeStepPhaseStart = 0.0;
  plannerContext_.routeStepSegmentIteration = 0;
  plannerContext_.routeStepPhase = RouteStepPhase::Approach;

  plannerContext_.parityTrace.clear();
  plannerContext_.parityTraceActive = v2ParityTrace_;
  const RouteStepOutcome outcome = runRouteStepToCompletionV2();
  plannerContext_.parityTraceActive = false;
  result.parityTrace.swap(plannerContext_.parityTrace);
  result.candidate = plannerContext_.routeStepCandidate;
  result.success = outcome == RouteStepOutcome::Feasible
      && result.candidate.completeCostAuditValid;
  result.reason = result.success ? "certified"
      : (outcome != RouteStepOutcome::Feasible ? result.candidate.failureReason
                                              : std::string("cost_invalid/") + result.candidate.terminalTimingAuditReason);
  logCertStageV2("terminal", result.candidate, outcome == RouteStepOutcome::Feasible,
                 routeDeepestStageV2(outcome == RouteStepOutcome::Feasible),
                 std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
                 result.reason);
  mc_rtc::log::info(
      "[V2CertificationDetail] type=TERMINAL_CERTIFY planId={} planningGeneration={} candidate={} success={} reason={} stageReached={}",
      request.planId, request.planningGeneration, candidate.name, result.success, result.reason,
      outcome == RouteStepOutcome::Feasible ? "complete_through_carried_retreat" : "rejected");
}

// =============================================================================
// Result handling (control thread)
// =============================================================================

void HandoverInterceptionController::processReceiverJobResultV2(double now)
{
  const PendingJobV2 pending = v2Pending_;
  const bool jobFailed = plannerJobState() == PlannerJobState::Failed;
  v2Pending_.active = false;
  logJobProfileV2(pending, now);
  if(pending.type == ReceiverJobTypeV2::CertifySelected) { v2SelectedCertificationPending_ = false; }

  if(pending.cancelRequested)
  {
    // A superseded generation never reaches plan adoption, retention,
    // certification or commitment, whether or not the worker finished first.
    ++v2Counters_.cancelled;
    plannerCancel_.store(false, std::memory_order_relaxed);
    mc_rtc::log::warning(
        "[V2JobCancelled] type={} planningGeneration={} reason={} cancelLatency={:.6f}s workerStoppedByCancel={} effect=none canCommit=false canReplacePlan=false t={:.6f}",
        receiverJobTypeNameV2(pending.type), pending.planningGeneration, pending.cancelReason,
        now - pending.cancelRequestTime,
        pending.type == ReceiverJobTypeV2::FullSearch
            ? (jobFailed && plannerFailureReason_ == "cancelled")
            : (v2Result_.reason.find("v2/cancelled") != std::string::npos),
        now);
    logSnapshotAuditV2(pending, now, "cancelled");
    return;
  }

  bool injected = false;
  if(pending.type == ReceiverJobTypeV2::TerminalCertify
     && v2Params_.injectStaleTerminalResultOnce && !v2StaleTerminalInjected_)
  {
    // Fault injection: simulate a state change while the job was in flight.
    v2StaleTerminalInjected_ = true;
    ++v2StateGeneration_;
    injected = true;
  }
  if(pending.type == ReceiverJobTypeV2::RecertifyActive
     && v2Params_.injectStaleRecertifyResultOnce && !v2StaleRecertifyInjected_)
  {
    v2StaleRecertifyInjected_ = true;
    ++v2StateGeneration_;
    injected = true;
  }

  const std::uint64_t resultGeneration = plannerResultGeneration();
  const bool generationMismatch = !jobFailed && resultGeneration != pending.planningGeneration;
  const bool stateMismatch = pending.stateGeneration != v2StateGeneration_;
  const bool planMismatch = pending.type != ReceiverJobTypeV2::FullSearch
      && pending.planId != provisionalReceiverPlan_.planId;
  if(generationMismatch || stateMismatch || planMismatch)
  {
    ++v2Counters_.staleRejected;
    logSnapshotAuditV2(pending, now, "stale");
    mc_rtc::log::warning(
        "[V2StaleResultRejected] type={} planningGeneration={} resultGeneration={} jobStateGeneration={} currentStateGeneration={} jobPlanId={} activePlanId={} generationMismatch={} stateMismatch={} planMismatch={} injected={} effect=none canCommit=false canReplacePlan=false t={:.6f}",
        receiverJobTypeNameV2(pending.type), pending.planningGeneration, resultGeneration,
        pending.stateGeneration, v2StateGeneration_, pending.planId,
        provisionalReceiverPlan_.planId, generationMismatch, stateMismatch, planMismatch,
        injected, now);
    return;
  }

  logSnapshotAuditV2(pending, now, "accepted");
  if(pending.type == ReceiverJobTypeV2::ControlAwareSelect)
  {
    handleControlAwareSelectionV2(pending, now);
    return;
  }
  if(pending.type == ReceiverJobTypeV2::FullSearch)
  {
    const auto & set = plannerResult();
    mc_rtc::log::success(
        "[V2PlanningJobResult] type=FULL_SEARCH planningGeneration={} stateGeneration={} failed={} reason={} records={} workerWall={:.6f}s latency={:.6f}s t={:.6f}",
        pending.planningGeneration, pending.stateGeneration, jobFailed,
        jobFailed ? plannerFailureReason() : std::string("complete"),
        jobFailed ? 0 : set.alternatives.size(), plannerWorkerWallDuration(),
        now - pending.submitTime, now);
    if(!jobFailed && v2SelectThenCertify_ && !v2Params_.characterizeOnly)
    {
      filterHypothesisFreshnessV2(pending, now);
      v2ExcludedRecords_.assign(plannerResult_.alternatives.size(), 0);
      v2SelectedSearchGeneration_ = pending.planningGeneration;
    }
    if(v2Params_.characterizeOnly)
    {
      if(!jobFailed) { logCharacterizationSearchV2(pending, now); }
      ++v2CharacterizationStage_;
      return;
    }
    if(jobFailed || !adoptFullSearchResultV2(now))
    {
      ++v2Counters_.searchFailures;
    }
    return;
  }

  const ReceiverJobResultV2 result = v2Result_;
  mc_rtc::log::success(
      "[V2PlanningJobResult] type={} planningGeneration={} stateGeneration={} planId={} success={} reason={} workerWall={:.6f}s latency={:.6f}s t={:.6f}",
      receiverJobTypeNameV2(pending.type), pending.planningGeneration,
      pending.stateGeneration, pending.planId, result.success, result.reason,
      result.wallDuration, now - pending.submitTime, now);

  if(pending.type == ReceiverJobTypeV2::CertifySelected)
  {
    handleSelectedCertificationV2(pending, now);
    return;
  }

  if(pending.type == ReceiverJobTypeV2::RecertifyActive)
  {
    if(v2ParityTrace_ && !result.parityFromReachStart && !result.success)
    {
      // Logging only: the rejected resumed rollout next to the last accepted one.
      mc_rtc::log::info("[V2ParityResumeFailure] planId={} failedGeneration={} lastAcceptedResumeGeneration={} failedSamples={} acceptedSamples={} reason={} t={:.6f}",
                        pending.planId, pending.planningGeneration, v2ParityLastResumeGeneration_,
                        result.parityTrace.size(), v2ParityLastResumeTrace_.size(), result.reason, now);
      logParityTraceV2("V2ParityResumeRejected", pending.planId, result.parityTrace);
      logParityTraceV2("V2ParityResumeAccepted", pending.planId, v2ParityLastResumeTrace_);
    }
    if(v2ParityTrace_ && !result.parityFromReachStart && result.success)
    {
      v2ParityLastResumeTrace_ = result.parityTrace;
      v2ParityLastResumeGeneration_ = pending.planningGeneration;
    }
    if(!result.success)
    {
      invalidateProvisionalPlanV2("recertification_infeasible/" + result.reason, now);
      return;
    }
    const auto & oldPresentation = provisionalReceiverPlan_.plan.objectAtPresentation;
    const double drift = (result.plan.objectAtPresentation.translation()
                          - oldPresentation.translation()).norm();
    const double rotationDrift = orientationError(result.plan.objectAtPresentation, oldPresentation);
    // Retain the same (tau, g, r); only its prediction-dependent world targets
    // follow the newest certified prediction.
    provisionalReceiverPlan_.plan.objectAtPresentation = result.plan.objectAtPresentation;
    provisionalReceiverPlan_.plan.objectAtContact = result.plan.objectAtContact;
    provisionalReceiverPlan_.plan.objectLinearVelocity = result.plan.objectLinearVelocity;
    provisionalReceiverPlan_.plan.objectAngularVelocity = result.plan.objectAngularVelocity;
    const auto name = provisionalReceiverPlan_.candidate.name;
    const auto route = provisionalReceiverPlan_.candidate.transitRouteName;
    provisionalReceiverPlan_.candidate = result.candidate;
    provisionalReceiverPlan_.candidate.name = name;
    provisionalReceiverPlan_.candidate.transitRouteName = route;
    ++provisionalReceiverPlan_.certifications;
    ++v2Counters_.retained;
    if(v2ParityTrace_ && result.parityFromReachStart && now < provisionalReceiverPlan_.plan.reachStartTime)
    {
      v2ParityReachTrace_ = result.parityTrace;
      v2ParityReachTracePlanId_ = provisionalReceiverPlan_.planId;
      v2ParityReachTraceGeneration_ = pending.planningGeneration;
    }
    if(drift > 0.0 || rotationDrift > 0.0) { ++v2Counters_.updated; }
    ++v2StateGeneration_;
    mc_rtc::log::success(
        "[V2ProvisionalRetain] planId={} planningGeneration={} newStateGeneration={} candidate={} route={} tau={:.6f} certifications={} presentationUpdate={:.6f}m rotationUpdate={:.6f}rad reachClear={:.4f} retreatClear={:.4f} rule=retain_while_certified t={:.6f}",
        provisionalReceiverPlan_.planId, pending.planningGeneration, v2StateGeneration_,
        name, route, provisionalReceiverPlan_.plan.presentationTime,
        provisionalReceiverPlan_.certifications, drift, rotationDrift,
        result.candidate.predictiveReachClearance, result.candidate.predictiveRetreatClearance, now);
    return;
  }

  // TERMINAL_CERTIFY
  if(!result.success)
  {
    invalidateProvisionalPlanV2("terminal_certification_infeasible/" + result.reason, now);
    return;
  }
  v2LatestTerminalCertificate_ = result;
  v2LatestTerminalCertificateValid_ = true;
  ++provisionalReceiverPlan_.certifications;
  mc_rtc::log::success(
      "[V2TerminalCertificate] planId={} planningGeneration={} stateGeneration={} snapshotTime={:.6f} candidate={} retreatClear={:.4f} t={:.6f}",
      pending.planId, pending.planningGeneration, pending.stateGeneration,
      result.snapshotTime, result.candidate.name,
      result.candidate.predictiveRetreatClearance, now);
}

bool HandoverInterceptionController::adoptFullSearchResultV2(double now)
{
  const FrozenPlanSet & set = plannerResult();
  std::vector<call_handover::FiniteEventPlanRecord> records;
  records.reserve(set.alternatives.size());
  const bool useExclusions = v2SelectThenCertify_
      && v2ExcludedRecords_.size() == set.alternatives.size();
  for(std::size_t i = 0; i < set.alternatives.size(); ++i)
  {
    if(useExclusions && v2ExcludedRecords_[i]) { continue; }
    const auto & alternative = set.alternatives[i];
    call_handover::FiniteEventPlanRecord record;
    record.sourceIndex = i;
    record.hypothesisIndex = alternative.hypothesisIndex;
    record.costValid = alternative.candidate.completeCostAuditValid;
    record.motionCost = alternative.candidate.completeCostAudit;
    record.globalCost = alternative.globalObjectiveCost;
    record.eventLead = alternative.eventLeadFromSearchEpoch;
    record.eventPresentationTime = alternative.eventPresentationTime;
    record.predictedPresentationDuration = alternative.candidate.predictedPresentationTime;
    record.predictedExecutionDuration = alternative.candidate.auditEstimatedTime;
    record.clearance = alternative.candidate.predictiveReachClearance;
    record.candidateName = alternative.candidate.name;
    record.routeName = alternative.candidate.transitRouteName;
    records.push_back(record);
  }
  const double minimumSafeCommitLead = std::max(
      v2Params_.minimumCommitRemainingTime, presentationDecelerationDuration_ + 0.25);
  const auto selectionStart = std::chrono::steady_clock::now();
  const auto selection = call_handover::selectFiniteEventPlan(
      records, now, v2Params_.minimumReachEntryLead, minimumSafeCommitLead,
      decisionCostTieTolerance_);
  const double selectionWall = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - selectionStart).count();
  mc_rtc::log::success(
      "[V2FullSearchSelection] success={} reason={} completePlans={} costValidPlans={} timingAdmissiblePlans={} selectionWall={:.9f}s selectedHypothesis={} selectedCandidate={} selectedRoute={} t={:.6f} selector=selectFiniteEventPlan(unchanged)",
      selection.success, selection.reason, selection.completePlanCount,
      selection.costValidCount, selection.timingAdmissibleCount, selectionWall,
      selection.success && selection.selectedRecord < records.size() ? records[selection.selectedRecord].hypothesisIndex : 0,
      selection.success && selection.selectedRecord < records.size() ? records[selection.selectedRecord].candidateName : std::string("none"),
      selection.success && selection.selectedRecord < records.size() ? records[selection.selectedRecord].routeName : std::string("none"),
      now);
  if(!selection.success || selection.selectedRecord >= records.size())
  {
    return false;
  }
  // The selector indexes the record vector; map back to the plan set (records
  // skip excluded alternatives under select-then-certify).
  const std::size_t chosen = records[selection.selectedRecord].sourceIndex;
  const auto & alternative = set.alternatives[chosen];
  // Start-state premise: the certified reach begins at the snapshot mouth pose.
  const sva::PTransformd mouth = actualMouthPose();
  const double startError = (mouth.translation() - alternative.planningStartMouthPose.translation()).norm();
  const double startAngle = orientationError(mouth, alternative.planningStartMouthPose);
  if(startError > predictiveReachPolicy_.positionTolerance
     || startAngle > predictiveReachPolicy_.orientationTolerance)
  {
    ++v2Counters_.staleRejected;
    mc_rtc::log::warning(
        "[V2StaleResultRejected] type=FULL_SEARCH reason=robot_left_certified_start startError={:.5f} startAngle={:.5f} effect=none canCommit=false canReplacePlan=false t={:.6f}",
        startError, startAngle, now);
    return false;
  }

  const CaptureCandidate & c = alternative.candidate;
  const double retreatDuration = std::max(
      0.0, c.estimatedTime - c.predictedContactTime - timingBilateralDwell_ - timingConfirmationDwell_);
  InterceptionPlan plan = makeInterceptionPlan(
      c, alternative.W_T_O_presentation, alternative.eventPresentationTime,
      c.predictedReachTime, c.predictedApproachTime, c.predictedAcquireTime, retreatDuration);
  plan.conditionalPresentationV2 = true;
  plan.decelerationDuration = 0.0;
  plan.decelerationStartTime = plan.presentationTime;
  plan.objectLinearVelocity = plannerConfig_.objectLinearVelocity;
  plan.objectAngularVelocity = plannerConfig_.objectAngularVelocity;
  plan.mouthAtReachStart = alternative.planningStartMouthPose;
  std::string why;
  if(!validateInterceptionPlan(plan, &why, true))
  {
    mc_rtc::log::error("[V2FullSearchSelection] adoption refused reason=invalid_plan/{}", why);
    return false;
  }

  // Freshness of the selected record against the newest prediction. A record
  // whose target moved beyond the commit-freshness tube is not unsafe: that one
  // action is re-certified at the newest prediction (same rollout as
  // RECERTIFY_ACTIVE) before it can be adopted. Other records are untouched.
  const ObjectPredictionRecordV2 latest = currentObjectPredictionV2();
  const sva::PTransformd predicted = predictionPoseAtV2(latest, alternative.eventPresentationTime);
  const double deviation = (predicted.translation() - alternative.W_T_O_presentation.translation()).norm();
  const double rotation = orientationError(predicted, alternative.W_T_O_presentation);
  const bool fresh = deviation <= predictiveReachPolicy_.maximumObjectTranslationDeviation
      && rotation <= predictiveReachPolicy_.maximumObjectRotationDeviation;
  if(!fresh && v2SelectThenCertify_)
  {
    v2SelectedRecord_ = chosen;
    if(!submitReceiverCertificationV2(ReceiverJobTypeV2::CertifySelected, now, &c, &plan))
    {
      return false;
    }
    v2SelectedCertificationPending_ = true;
    mc_rtc::log::info(
        "[V2SelectedTargetMoved] searchGeneration={} record={} hypothesis={} eventTime={:.6f} candidate={} route={} deviation={:.6f}m rotation={:.6f}rad tolerance={:.3f}m/{:.3f}rad action=certify_selected certificateGeneration={} t={:.6f}",
        v2SelectedSearchGeneration_, chosen, alternative.hypothesisIndex,
        alternative.eventPresentationTime, c.name, c.transitRouteName, deviation, rotation,
        predictiveReachPolicy_.maximumObjectTranslationDeviation,
        predictiveReachPolicy_.maximumObjectRotationDeviation, plannerRequestGeneration(), now);
    return true;
  }
  adoptProvisionalPlanV2(alternative, c, plan, now, "search", plannerResultGeneration(), 0);
  return true;
}

void HandoverInterceptionController::adoptProvisionalPlanV2(
    const GlobalEventPlanAlternative & alternative, const CaptureCandidate & c,
    const InterceptionPlan & plan, double now, const char * source, std::uint64_t sourceGeneration,
    std::uint64_t certificateGeneration)
{
  const sva::PTransformd mouth = actualMouthPose();
  {
    const ObjectPredictionRecordV2 latest = currentObjectPredictionV2();
    const sva::PTransformd predicted = predictionPoseAtV2(latest, plan.presentationTime);
    const double deviation = (predicted.translation() - plan.objectAtPresentation.translation()).norm();
    const double rotation = orientationError(predicted, plan.objectAtPresentation);
    mc_rtc::log::info(
        "[V2AdoptFreshness] planningGeneration={} hypothesis={} eventTime={:.6f} source={} certificateGeneration={} deviation={:.6f}m rotation={:.6f}rad tolerance={:.3f}m/{:.3f}rad fresh={} t={:.6f}",
        sourceGeneration, alternative.hypothesisIndex, plan.presentationTime, source,
        certificateGeneration, deviation, rotation,
        predictiveReachPolicy_.maximumObjectTranslationDeviation,
        predictiveReachPolicy_.maximumObjectRotationDeviation,
        deviation <= predictiveReachPolicy_.maximumObjectTranslationDeviation
            && rotation <= predictiveReachPolicy_.maximumObjectRotationDeviation, now);
  }
  const std::uint64_t previousPlanId = provisionalReceiverPlan_.planId;
  const bool replacement = previousPlanId != 0;
  ProvisionalReceiverPlanV2 next;
  next.valid = true;
  next.planId = ++v2PlanIdCounter_;
  next.sourcePlanningGeneration = sourceGeneration;
  next.adoptedStateGeneration = ++v2StateGeneration_;
  next.candidate = c;
  next.plan = plan;
  next.hypothesisIndex = alternative.hypothesisIndex;
  next.eventLead = alternative.eventLeadFromSearchEpoch;
  next.globalCost = alternative.globalObjectiveCost;
  next.adoptedTime = now;
  next.certifications = 1;
  next.holdPosture = currentArmPosture();
  provisionalReceiverPlan_ = next;
  ++v2Counters_.adoptions;
  if(replacement) { ++v2Counters_.replacements; }

  v2Phase_ = ReceiverPhaseV2::ProvisionalReach;
  v2PhaseEntryTime_ = now;
  v2ReferencePose_ = mouth;
  v2ClearanceScale_ = 1.0;
  v2HoldArmPosture_ = next.holdPosture;
  v2LatestTerminalCertificateValid_ = false;
  v2GateStableSince_ = -1.0;

  const Eigen::Vector3d po = plan.objectAtPresentation.translation();
  mc_rtc::log::success(
      "[V2ProvisionalAdopt] planId={} previousPlanId={} kind={} reason={} sourcePlanningGeneration={} adoptionSource={} adoptionCertificateGeneration={} stateGeneration={} hypothesis={} eventLead={:.3f}s tau={:.6f} candidate={} route={} globalJ={:.9f} reachStart={:.6f} standoffTime={:.6f} predictedPresentation=[{:.4f},{:.4f},{:.4f}] closureAuthorized=false committed=false t={:.6f}",
      next.planId, previousPlanId, replacement ? "REPLACEMENT" : "INITIAL",
      replacement ? v2ReplacementReason_ : std::string("initial"),
      next.sourcePlanningGeneration, source, certificateGeneration, v2StateGeneration_, next.hypothesisIndex,
      next.eventLead, plan.presentationTime, c.name, c.transitRouteName,
      next.globalCost, plan.reachStartTime, plan.standoffTime, po.x(), po.y(), po.z(), now);
}


void HandoverInterceptionController::invalidateProvisionalPlanV2(
    const std::string & reason, double now)
{
  mc_rtc::log::warning(
      "[V2ProvisionalInvalidated] planId={} phase={} reason={} stateGeneration={} action=hold_and_full_search t={:.6f}",
      provisionalReceiverPlan_.planId, receiverPhaseNameV2(v2Phase_), reason,
      v2StateGeneration_ + 1, now);
  v2ReplacementReason_ = reason;
  provisionalReceiverPlan_.valid = false;
  ++v2StateGeneration_;
  v2Phase_ = ReceiverPhaseV2::SearchHeld;
  v2PhaseEntryTime_ = now;
  v2HoldPose_ = actualMouthPose();
  v2HoldArmPosture_ = currentArmPosture();
  v2SearchBudgetStart_ = now;
  v2LatestTerminalCertificateValid_ = false;
  v2GateStableSince_ = -1.0;
  if(v2ControlAware_ && !v2CaSelector_.frozen)
  {
    ++v2CaAborts_;
    mc_rtc::log::warning(
        "[TriadLiteEvent] type=abort_to_hold graspId={} reason={} t={:.6f}",
        v2CaSelector_.incumbentId, reason, now);
    v2CaSelector_.incumbentId = -1;
    v2CaSelector_.challengerId = -1;
  }
}

// =============================================================================
// Execution of the provisional plan (local feedback identical to V1 reach)
// =============================================================================

bool HandoverInterceptionController::executeProvisionalReachV2(double now)
{
  const auto & policy = predictiveReachPolicy_;
  const auto & active = provisionalReceiverPlan_;
  const auto & plan = active.plan;
  activateToolTask();
  setToolTaskGains(policy.taskStiffness, policy.taskWeight);
  setGripperJointPriority(false);
  commandGripper(0.0);

  const sva::PTransformd current = actualMouthPose();
  HandoverSafetyReport currentReport;
  if(!evaluateCurrentPoseSafety(currentReport, false))
  {
    commandMouthTarget(current);
    mc_rtc::log::error(
        "[V2Failure] reason=provisional_reach_unsafe clear={:.4f} limiting={}/{} planId={}",
        currentReport.minClearance, currentReport.sample, currentReport.obstacle, active.planId);
    return false;
  }
  v2Counters_.minimumRuntimeClearance = std::min(
      v2Counters_.minimumRuntimeClearance, currentReport.minClearance);
  if(currentReport.minClearance < policy.minimumRuntimeClearance)
  {
    commandMouthTarget(current);
    mc_rtc::log::error(
        "[V2Failure] reason=provisional_reach_clearance_reserve clear={:.4f} minimum={:.4f} planId={}",
        currentReport.minClearance, policy.minimumRuntimeClearance, active.planId);
    return false;
  }

  if(now < plan.reachStartTime)
  {
    commandMouthTarget(plan.mouthAtReachStart);
    commandReadyArmPosture(v2HoldArmPosture_);
    v2ReferencePose_ = plan.mouthAtReachStart;
    return true;
  }

  if(v2ParityTrace_ && v2ParityReachLoggedPlanId_ != active.planId)
  {
    v2ParityReachLoggedPlanId_ = active.planId;
    const bool available = v2ParityReachTracePlanId_ == active.planId && !v2ParityReachTrace_.empty();
    const Eigen::Vector3d sp = compose(plan.objectAtPresentation, plan.O_T_M_standoff).translation();
    const Eigen::Quaterniond sq(worldRotation(compose(plan.objectAtPresentation, plan.O_T_M_standoff)));
    mc_rtc::log::info(
        "[V2ParityReachStart] planId={} available={} traceGeneration={} samples={} reachStart={:.6f} standoffTime={:.6f} reachDuration={:.6f} candidate={} route={} standoff=[{:.6f},{:.6f},{:.6f}] standoffQ=[{:.6f},{:.6f},{:.6f},{:.6f}] predictedReachClear={:.5f} predictedRetreatClear={:.5f} predictedMinClear={:.5f} t={:.6f}",
        active.planId, available, v2ParityReachTraceGeneration_, available ? v2ParityReachTrace_.size() : 0,
        plan.reachStartTime, plan.standoffTime, plan.reachDuration, active.candidate.name,
        active.candidate.transitRouteName, sp.x(), sp.y(), sp.z(), sq.w(), sq.x(), sq.y(), sq.z(),
        active.candidate.predictiveReachClearance, active.candidate.predictiveRetreatClearance,
        active.candidate.minClearance, now);
    if(available) { logParityTraceV2("V2ParityPreviewReach", active.planId, v2ParityReachTrace_); }
  }

  const double referenceTime = std::min(now, plan.standoffTime);
  auto reference = interceptionReferenceAt(plan, referenceTime, controlDt_);
  if(now >= plan.standoffTime)
  {
    reference.mouthLinearVelocityWorld.setZero();
    reference.mouthAngularVelocityWorld.setZero();
  }
  {
    const double progress = clamp01V2(reference.phaseProgress);
    const auto & transit = active.candidate.plannedTransitArmPosture.empty()
        ? active.candidate.plannedStandoffArmPosture : active.candidate.plannedTransitArmPosture;
    if(progress <= 0.5)
    {
      commandArmPosture(interpolateArmPosture(v2HoldArmPosture_, transit, 2.0 * progress));
    }
    else
    {
      commandArmPosture(interpolateArmPosture(
          transit, active.candidate.plannedStandoffArmPosture, 2.0 * progress - 1.0));
    }
  }

  double rawClearanceScale = 1.0;
  if(currentReport.minClearance < policy.clearanceSlowdownStart)
  {
    const double denominator = std::max(1e-6, policy.clearanceSlowdownStart - policy.clearanceHardMargin);
    const double u = std::min(1.0, std::max(0.0,
        (currentReport.minClearance - policy.clearanceHardMargin) / denominator));
    const double smooth = u * u * (3.0 - 2.0 * u);
    rawClearanceScale = policy.minimumVelocityScale + (1.0 - policy.minimumVelocityScale) * smooth;
  }
  const double scaleRate = rawClearanceScale < v2ClearanceScale_
      ? policy.clearanceScaleDropRate : policy.clearanceScaleRiseRate;
  const double maximumScaleChange = scaleRate * controlDt_;
  v2ClearanceScale_ += std::max(-maximumScaleChange,
                                std::min(maximumScaleChange, rawClearanceScale - v2ClearanceScale_));
  v2ClearanceScale_ = std::min(1.0, std::max(policy.minimumVelocityScale, v2ClearanceScale_));

  const double linearSpeedLimit = policy.nearLinearSpeed
      + v2ClearanceScale_ * (policy.farLinearSpeed - policy.nearLinearSpeed);
  const double angularSpeedLimit = policy.nearAngularSpeed
      + v2ClearanceScale_ * (policy.farAngularSpeed - policy.nearAngularSpeed);
  const double linearLeadLimit = policy.nearLinearTrackingLead
      + v2ClearanceScale_ * (policy.maxLinearTrackingLead - policy.nearLinearTrackingLead);
  const double angularLeadLimit = policy.nearAngularTrackingLead
      + v2ClearanceScale_ * (policy.maxAngularTrackingLead - policy.nearAngularTrackingLead);

  const sva::PTransformd rateLimitedReference = advancePoseReference(
      v2ReferencePose_, reference.mouthPose, linearSpeedLimit, angularSpeedLimit);
  const sva::PTransformd nextReference = boundedPoseStep(
      current, rateLimitedReference, linearLeadLimit, angularLeadLimit);
  sva::PTransformd safeReference;
  HandoverSafetyReport report;
  if(!filterSafeMouthCommand(current, nextReference, safeReference, report, false))
  {
    commandMouthTarget(current);
    mc_rtc::log::error(
        "[V2Failure] reason=provisional_reach_safety_filter clear={:.4f} limiting={}/{} planId={}",
        report.minClearance, report.sample, report.obstacle, active.planId);
    return false;
  }
  const sva::PTransformd previousCommand = v2ReferencePose_;
  v2ReferencePose_ = safeReference;
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Vector3d w = Eigen::Vector3d::Zero();
  worldPoseTwist(previousCommand, v2ReferencePose_, controlDt_, v, w);
  if(v.norm() > linearSpeedLimit && v.norm() > 1e-12) { v *= linearSpeedLimit / v.norm(); }
  if(w.norm() > angularSpeedLimit && w.norm() > 1e-12) { w *= angularSpeedLimit / w.norm(); }
  commandMouthTargetWithWorldVelocity(v2ReferencePose_, v, w);
  return true;
}

bool HandoverInterceptionController::executeTerminalTrackV2(double now, bool & gateSatisfied)
{
  gateSatisfied = false;
  const auto & active = provisionalReceiverPlan_;
  activateToolTask();
  setToolTaskGains(v2Params_.terminalTaskStiffness, v2Params_.terminalTaskWeight);
  if(!active.candidate.plannedStandoffArmPosture.empty())
  {
    commandArmPosture(active.candidate.plannedStandoffArmPosture);
  }
  setGripperJointPriority(false);
  commandGripper(0.0);

  const sva::PTransformd current = actualMouthPose();
  // Object-relative standoff of the immutable grasp at the CURRENT estimate.
  const sva::PTransformd target = compose(W_T_O_, active.plan.O_T_M_standoff);
  HandoverSafetyReport currentReport;
  if(!evaluateCurrentPoseSafety(currentReport, false))
  {
    commandMouthTarget(current);
    mc_rtc::log::error(
        "[V2Failure] reason=terminal_track_unsafe clear={:.4f} limiting={}/{} planId={}",
        currentReport.minClearance, currentReport.sample, currentReport.obstacle, active.planId);
    return false;
  }
  v2Counters_.minimumRuntimeClearance = std::min(
      v2Counters_.minimumRuntimeClearance, currentReport.minClearance);
  const sva::PTransformd next = boundedPoseStep(
      current, target, v2Params_.terminalLinearTrackingLead, v2Params_.terminalAngularTrackingLead);
  sva::PTransformd safe;
  HandoverSafetyReport report;
  if(!filterSafeMouthCommand(current, next, safe, report, false))
  {
    commandMouthTarget(current);
    mc_rtc::log::error(
        "[V2Failure] reason=terminal_track_safety_filter clear={:.4f} limiting={}/{} planId={}",
        report.minClearance, report.sample, report.obstacle, active.planId);
    return false;
  }
  commandMouthTargetWithWorldVelocity(safe, objectLinearVelocityEstimate_,
                                      objectAngularVelocityEstimate_);

  const double dist = (target.translation() - current.translation()).norm();
  const double angle = orientationError(current, target);
  const bool objectStopped = objectLinearVelocityEstimate_.norm() <= presentationMaximumLinearSpeed_
      && objectAngularVelocityEstimate_.norm() <= presentationMaximumAngularSpeed_;
  const bool gripperOpen = measuredGripperClosure() <= v2Params_.terminalMaximumOpenClosure;
  const bool fresh = objectPerceptionMeasurementValid_ && objectMotionEstimateValid_
      && objectPerceptionMeasurementAge_ <= perceptionLatencyBufferDuration_;
  gateSatisfied = now >= active.plan.presentationTime
      && dist <= v2Params_.terminalPositionTolerance
      && angle <= v2Params_.terminalOrientationTolerance
      && objectStopped && gripperOpen && fresh && report.safe;
  return true;
}

// =============================================================================
// Terminal commitment: exactly once
// =============================================================================

bool HandoverInterceptionController::commitProvisionalReceiverPlanV2(
    const ReceiverJobResultV2 & certificate, double now)
{
  if(v2CommitLatched_ || v2ReselectionLocked_ || v2CommitCount_ > 0)
  {
    mc_rtc::log::error(
        "[V2TerminalCommit] committed=false reason=commit_already_latched commitCount={} t={:.6f}",
        v2CommitCount_, now);
    return false;
  }
  const auto & active = provisionalReceiverPlan_;
  if(!active.valid || !certificate.success
     || certificate.type != ReceiverJobTypeV2::TerminalCertify
     || certificate.stateGeneration != v2StateGeneration_
     || certificate.planId != active.planId)
  {
    mc_rtc::log::warning(
        "[V2TerminalCommit] committed=false reason=certificate_not_current certStateGeneration={} currentStateGeneration={} certPlanId={} activePlanId={} t={:.6f}",
        certificate.stateGeneration, v2StateGeneration_, certificate.planId, active.planId, now);
    return false;
  }

  // Final current-state check against the certificate snapshot.
  const sva::PTransformd mouth = actualMouthPose();
  const sva::PTransformd objectNow = W_T_O_;
  const double objectDrift = (certificate.objectPose.translation() - objectNow.translation()).norm();
  const double objectRotationDrift = orientationError(certificate.objectPose, objectNow);
  const double mouthDrift = (certificate.snapshotMouthPose.translation() - mouth.translation()).norm();
  const double mouthRotationDrift = orientationError(certificate.snapshotMouthPose, mouth);
  HandoverSafetyReport corridorReport;
  const sva::PTransformd captureNow = compose(objectNow, active.plan.O_T_M_capture);
  const bool corridorOk = graspCorridorSafe(captureNow, corridorReport);
  HandoverSafetyReport clearanceReport;
  const bool clearanceOk = evaluateCurrentPoseSafety(clearanceReport, false);
  if(objectDrift > predictiveReachPolicy_.maximumObjectTranslationDeviation
     || objectRotationDrift > predictiveReachPolicy_.maximumObjectRotationDeviation
     || mouthDrift > v2Params_.terminalPositionTolerance
     || mouthRotationDrift > v2Params_.terminalOrientationTolerance
     || !corridorOk || !clearanceOk)
  {
    mc_rtc::log::warning(
        "[V2TerminalGate] commit deferred objectDrift={:.5f}/{:.5f} objectRotationDrift={:.5f}/{:.5f} mouthDrift={:.5f}/{:.5f} mouthRotationDrift={:.5f}/{:.5f} corridor={} clearance={} t={:.6f}",
        objectDrift, predictiveReachPolicy_.maximumObjectTranslationDeviation,
        objectRotationDrift, predictiveReachPolicy_.maximumObjectRotationDeviation,
        mouthDrift, v2Params_.terminalPositionTolerance, mouthRotationDrift,
        v2Params_.terminalOrientationTolerance, corridorOk, clearanceOk, now);
    return false;
  }

  CaptureCandidate c = certificate.candidate;
  c.W_T_M_standoff = compose(objectNow, active.plan.O_T_M_standoff);
  c.W_T_M_transit = c.W_T_M_standoff;
  c.W_T_M_pre = captureNow;
  c.W_T_M_retreat = compose(objectNow, active.plan.O_T_M_retreat);
  const double retreatDuration = std::max(
      0.0, c.estimatedTime - c.predictedContactTime - timingBilateralDwell_ - timingConfirmationDwell_);
  InterceptionPlan plan = makeInterceptionPlan(
      c, objectNow, now, 2.0 * previewDt_, c.predictedApproachTime,
      c.predictedAcquireTime, retreatDuration);
  plan.conditionalPresentationV2 = true;
  plan.decelerationDuration = 0.0;
  plan.decelerationStartTime = plan.presentationTime;
  plan.objectLinearVelocity = objectLinearVelocityEstimate_;
  plan.objectAngularVelocity = objectAngularVelocityEstimate_;
  plan.mouthAtReachStart = mouth;
  plan.O_T_M_standoff = active.plan.O_T_M_standoff;
  plan.O_T_M_capture = active.plan.O_T_M_capture;
  plan.O_T_M_retreat = active.plan.O_T_M_retreat;
  std::string why;
  if(!validateInterceptionPlan(plan, &why, true))
  {
    mc_rtc::log::error("[V2TerminalCommit] committed=false reason=invalid_committed_plan/{}", why);
    v2Phase_ = ReceiverPhaseV2::Failed;
    return false;
  }

  // Promote provisional -> committed. This is the only V2 write of the
  // committed plan and the selected-candidate fields.
  candidateSelected_ = true;
  selectedCandidateName_ = active.candidate.name;
  selectedCandidateClearance_ = c.minClearance;
  selectedCandidateScore_ = c.score;
  selectedCandidatePredictedTime_ = c.estimatedTime;
  selectedCandidatePredictedPresentationTime_ = c.predictedPresentationTime;
  selectedCandidatePredictedContactTime_ = c.predictedContactTime;
  selectedCandidatePredictedReachTime_ = c.predictedReachTime;
  selectedCandidatePredictedApproachTime_ = c.predictedApproachTime;
  selectedCandidatePredictedAcquireTime_ = c.predictedAcquireTime;
  selectedCandidatePredictedEffort_ = c.predictedEffort;
  selectedCandidateContactClosure_ = c.contactClosure;
  selectedTransitArmPosture_ = c.plannedTransitArmPosture;
  selectedStandoffArmPosture_ = c.plannedStandoffArmPosture;
  selectedArmPosture_ = c.plannedArmPosture;
  selectedRetreatArmPosture_ = c.plannedRetreatArmPosture;
  W_T_M_transit_ = c.W_T_M_transit;
  W_T_M_standoff_ = c.W_T_M_standoff;
  W_T_M_pre_ = c.W_T_M_pre;
  W_T_M_acquired_ = c.W_T_M_pre;
  W_T_M_retreat_ = c.W_T_M_retreat;
  O_T_M_transit_ = active.plan.O_T_M_standoff;
  O_T_M_standoff_ = active.plan.O_T_M_standoff;
  O_T_M_pre_ = active.plan.O_T_M_capture;
  O_T_M_retreat_ = active.plan.O_T_M_retreat;

  committedInterceptionPlan_ = plan;
  interceptionCommitted_ = true;
  committedContactTime_ = plan.contactTime;
  committedTimingResidual_ = 0.0;
  W_T_O_committedContact_ = plan.objectAtContact;
  committedObjectLinearVelocity_ = plan.objectLinearVelocity;
  committedObjectAngularVelocity_ = plan.objectAngularVelocity;
  if(!lockStaticPresentationToCurrentObject())
  {
    mc_rtc::log::error("[V2TerminalCommit] committed=false reason=terminal_anchor_lock_failed");
    v2Phase_ = ReceiverPhaseV2::Failed;
    return false;
  }

  if(v2ParityTrace_)
  {
    const double predictedRetreat = std::max(0.0, c.estimatedTime - c.predictedContactTime
                                                  - timingBilateralDwell_ - timingConfirmationDwell_);
    mc_rtc::log::info(
        "[V2ParityCommitPreview] planId={} certificatePlanningGeneration={} predictedApproach={:.6f} auditRan={} auditSuccess={} auditDuration={:.6f} legacyApproach={:.6f} predictedAcquire={:.6f} predictedContact={:.6f} predictedRetreat={:.6f} bilateralDwell={:.6f} confirmationDwell={:.6f} estimated={:.6f} contactClosure={:.6f} reachClear={:.5f} retreatClear={:.5f} minClear={:.5f} capture=[{:.6f},{:.6f},{:.6f}] retreat=[{:.6f},{:.6f},{:.6f}] samples={} t={:.6f}",
        active.planId, certificate.planningGeneration, c.predictedApproachTime, c.terminalTimingAuditRan,
        c.terminalTimingAuditSuccess, c.terminalTimingAuditDuration, c.legacyPredictedApproachTime,
        c.predictedAcquireTime, c.predictedContactTime, predictedRetreat, timingBilateralDwell_,
        timingConfirmationDwell_, c.estimatedTime, c.contactClosure, c.predictiveReachClearance,
        c.predictiveRetreatClearance, c.minClearance, c.W_T_M_pre.translation().x(),
        c.W_T_M_pre.translation().y(), c.W_T_M_pre.translation().z(), c.W_T_M_retreat.translation().x(),
        c.W_T_M_retreat.translation().y(), c.W_T_M_retreat.translation().z(),
        certificate.parityTrace.size(), now);
    logParityTraceV2("V2ParityPreviewTerminal", active.planId, certificate.parityTrace);
  }

  v2CommitLatched_ = true;
  v2ReselectionLocked_ = true;
  ++v2CommitCount_;
  ++v2StateGeneration_;
  v2Phase_ = ReceiverPhaseV2::Committed;
  mc_rtc::log::success(
      "[V2TerminalCommit] committed=true commitCount={} planId={} certificatePlanningGeneration={} stateGeneration={} candidate={} route={} tau={:.6f} commitTime={:.6f} tauMinusCommit={:+.6f} objectDrift={:.6f} mouthDrift={:.6f} objectSpeed={:.6f} adoptions={} replacements={} retained={} staleRejected={} closureAuthorizedBeforeCommit=false globalReselectionLocked=true",
      v2CommitCount_, active.planId, certificate.planningGeneration, v2StateGeneration_,
      active.candidate.name, active.candidate.transitRouteName, active.plan.presentationTime,
      now, active.plan.presentationTime - now, objectDrift, mouthDrift,
      objectLinearVelocityEstimate_.norm(), v2Counters_.adoptions, v2Counters_.replacements,
      v2Counters_.retained, v2Counters_.staleRejected);
  mc_rtc::log::success(
      "[GiverTruthIndependence] commit planId={} simulatedTruthPlanCreated=false model={}",
      active.planId, giverTruthModel_);
  logReceiverMotionV2(now, true);
  return true;
}

// =============================================================================
// Supersession cancellation and snapshot audit (control thread)
// =============================================================================

void HandoverInterceptionController::checkSupersessionV2(double now)
{
  PendingJobV2 & pending = v2Pending_;
  if(pending.type == ReceiverJobTypeV2::FullSearch
     && v2Params_.injectSupersedeFullSearchOnce && !v2SupersedeInjected_
     && now - pending.submitTime > 0.5)
  {
    v2SupersedeInjected_ = true;
    ++v2StateGeneration_;
    mc_rtc::log::warning(
        "[V2FaultInjection] kind=supersede_full_search planningGeneration={} newStateGeneration={} t={:.6f}",
        pending.planningGeneration, v2StateGeneration_, now);
  }

  const auto & policy = predictiveReachPolicy_;
  std::string reason;
  double maxTranslation = 0.0;
  double maxRotation = 0.0;
  const sva::PTransformd mouth = actualMouthPose();
  const double robotDrift = (mouth.translation() - pending.snapshotMouthPose.translation()).norm();
  const double robotRotationDrift = orientationError(mouth, pending.snapshotMouthPose);
  if(pending.stateGeneration != v2StateGeneration_)
  {
    reason = "state_generation_advanced";
  }
  else if(pending.type == ReceiverJobTypeV2::FullSearch && v2SelectThenCertify_)
  {
    // The robot snapshot is what the whole job depends on: leaving it makes
    // every record unsafe. A prediction change only moves targets; records are
    // classified against the tube at receipt and a selected record whose
    // target moved is re-certified alone (handleSelectedCertificationV2).
    if(robotDrift > policy.positionTolerance || robotRotationDrift > policy.orientationTolerance)
    {
      reason = "robot_left_snapshot_start";
    }
    else
    {
      // The job stays useful while at least one hypothesis that can still be
      // admitted (event time - now >= the selector's minimum safe commit lead)
      // keeps its target inside the tube. When none does, no unaffected work
      // remains: the job is obsolete and a new epoch is searched.
      const ObjectPredictionRecordV2 prediction = currentObjectPredictionV2();
      if(prediction.valid)
      {
        const double minimumSafeCommitLead = std::max(
            v2Params_.minimumCommitRemainingTime, presentationDecelerationDuration_ + 0.25);
        bool anyAdmissible = false;
        bool anyUnaffectedAdmissible = false;
        for(std::size_t i = 0; i < pending.bankPoses.size() && i < pending.bankEventTimes.size(); ++i)
        {
          if(pending.bankEventTimes[i] - now + 1e-12 < minimumSafeCommitLead) { continue; }
          anyAdmissible = true;
          const sva::PTransformd latest = predictionPoseAtV2(prediction, pending.bankEventTimes[i]);
          const double d = (latest.translation() - pending.bankPoses[i].translation()).norm();
          const double r = orientationError(latest, pending.bankPoses[i]);
          maxTranslation = std::max(maxTranslation, d);
          maxRotation = std::max(maxRotation, r);
          if(d <= policy.maximumObjectTranslationDeviation && r <= policy.maximumObjectRotationDeviation)
          {
            anyUnaffectedAdmissible = true;
            break;
          }
        }
        if(!anyUnaffectedAdmissible)
        {
          reason = anyAdmissible ? "no_unaffected_admissible_hypothesis" : "no_admissible_hypothesis";
        }
      }
    }
  }
  else if(pending.type == ReceiverJobTypeV2::FullSearch)
  {
    // The bank certified these poses at these instants. If the newest
    // independent prediction disagrees at any instant by more than the
    // commit-freshness tube, the generation is obsolete.
    const ObjectPredictionRecordV2 prediction = currentObjectPredictionV2();
    if(prediction.valid)
    {
      for(std::size_t i = 0; i < pending.bankPoses.size() && i < pending.bankEventTimes.size(); ++i)
      {
        const sva::PTransformd latest = predictionPoseAtV2(prediction, pending.bankEventTimes[i]);
        maxTranslation = std::max(maxTranslation,
            (latest.translation() - pending.bankPoses[i].translation()).norm());
        maxRotation = std::max(maxRotation, orientationError(latest, pending.bankPoses[i]));
      }
      if(maxTranslation > policy.maximumObjectTranslationDeviation
         || maxRotation > policy.maximumObjectRotationDeviation)
      {
        reason = "prediction_superseded";
      }
    }
    if(reason.empty()
       && (robotDrift > policy.positionTolerance || robotRotationDrift > policy.orientationTolerance))
    {
      reason = "robot_left_snapshot_start";
    }
  }
  if(reason.empty()) { return; }

  pending.cancelRequested = true;
  pending.cancelReason = reason;
  pending.cancelRequestTime = now;
  plannerCancel_.store(true, std::memory_order_relaxed);
  ++v2Counters_.cancelRequested;
  mc_rtc::log::warning(
      "[V2JobCancelRequested] type={} planningGeneration={} jobStateGeneration={} currentStateGeneration={} reason={} maxPredictionDeviation={:.6f}m/{:.6f}rad tolerance={:.3f}m/{:.3f}rad robotDrift={:.6f}m/{:.6f}rad snapshotAge={:.6f}s nonBlocking=true t={:.6f}",
      receiverJobTypeNameV2(pending.type), pending.planningGeneration, pending.stateGeneration,
      v2StateGeneration_, reason, maxTranslation, maxRotation,
      policy.maximumObjectTranslationDeviation, policy.maximumObjectRotationDeviation,
      robotDrift, robotRotationDrift, now - pending.submitTime, now);
}

void HandoverInterceptionController::logSnapshotAuditV2(
    const PendingJobV2 & pending, double now, const char * outcome)
{
  const sva::PTransformd mouth = actualMouthPose();
  const double objectDisplacement =
      (W_T_O_.translation() - pending.prediction.pose.translation()).norm();
  const double predictionError =
      (W_T_O_.translation() - predictionPoseAtV2(pending.prediction, now).translation()).norm();
  mc_rtc::log::info(
      "[V2SnapshotAudit] type={} planningGeneration={} outcome={} snapshotAge={:.6f}s robotDrift={:.6f}m robotRotationDrift={:.6f}rad objectDisplacementSinceSnapshot={:.6f}m snapshotPredictionError={:.6f}m objectEstimateSpeed={:.6f} t={:.6f}",
      receiverJobTypeNameV2(pending.type), pending.planningGeneration, outcome,
      now - pending.submitTime,
      (mouth.translation() - pending.snapshotMouthPose.translation()).norm(),
      orientationError(mouth, pending.snapshotMouthPose), objectDisplacement,
      predictionError, objectLinearVelocityEstimate_.norm(), now);
}

// =============================================================================
// Characterization instrumentation (never read by any decision)
// =============================================================================

void HandoverInterceptionController::resetStageProfileV2(
    std::uint64_t generation, const char * jobType) const
{
  plannerContext_.stageWall.fill(0.0);
  plannerContext_.stageCount.fill(0);
  plannerContext_.certJobGeneration = generation;
  plannerContext_.certJobStart = std::chrono::steady_clock::now();
  plannerContext_.certJobType = jobType;
  plannerContext_.certJobWall = 0.0;
  plannerContext_.certStaticRecords = 0;
  plannerContext_.certRouteRecords = 0;
  plannerContext_.certMemoReuses = 0;
  plannerContext_.certTimingPruned = 0;
  plannerContext_.certStaticReachClearance = std::numeric_limits<double>::quiet_NaN();
  plannerContext_.certRouteReachDuration = std::numeric_limits<double>::quiet_NaN();
  plannerContext_.routeStepFailedPhase = RouteStepPhase::Idle;
}

// Deepest successfully certified stage of the copied-state static screen
// (reach standoff -> corridor insertion -> closure/contact -> carried retreat),
// from the phase in which it stopped.
const char * HandoverInterceptionController::staticDeepestStageV2(bool feasible) const
{
  if(feasible) { return "CARRIED_RETREAT"; }
  switch(plannerContext_.planningPhase)
  {
    case PlanningPhase::ReachStandoff: return "NONE";
    case PlanningPhase::ReachCapture: return "REACH";
    case PlanningPhase::Closure: return "INSERTION";
    case PlanningPhase::Retreat: return "CLOSURE_CONTACT";
  }
  return "NONE";
}

// Same for a route rollout (transit reach -> insertion + capture dwell ->
// closure/contact -> carried retreat -> finalize), from the phase recorded by
// routeStepFail(). Transfer readiness has no candidate-dependent test in the
// certifier: it is implied by bilateral contact, so it is not a separate stage.
const char * HandoverInterceptionController::routeDeepestStageV2(bool feasible) const
{
  if(feasible) { return "CARRIED_RETREAT"; }
  switch(plannerContext_.routeStepFailedPhase)
  {
    case RouteStepPhase::Idle:
    case RouteStepPhase::Reach: return "NONE";
    case RouteStepPhase::Approach:
    case RouteStepPhase::Dwell: return "REACH";
    case RouteStepPhase::Closure: return "INSERTION";
    case RouteStepPhase::Retreat: return "CLOSURE_CONTACT";
    case RouteStepPhase::Finalize: return "CARRIED_RETREAT";
  }
  return "NONE";
}

void HandoverInterceptionController::logCertStageV2(
    const char * path, const CaptureCandidate & candidate, bool feasible,
    const char * deepest, double staticReach, double routeReachDuration,
    const std::string & reason) const
{
  const bool search = plannerContext_.certJobType == "FULL_SEARCH";
  const bool isStatic = std::string(path) == "static";
  const bool isMemo = std::string(path) == "memo";
  if(isStatic) { ++plannerContext_.certStaticRecords; }
  else if(!isMemo) { ++plannerContext_.certRouteRecords; }
  // Replay fields: time and work since job start, and the exact global
  // objective the selector will use for this record.
  const double jobWall = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - plannerContext_.certJobStart).count();
  const double lead = search ? finiteSearch_.currentLead : std::numeric_limits<double>::quiet_NaN();
  const double globalJ = (!isStatic && feasible)
      ? call_handover::extendMotionCostToSearchEpoch(
            candidate.completeCostAudit, plannerConfig_.decisionTimeWeight,
            plannerConfig_.decisionTimeReference, lead, candidate.predictedPresentationTime)
      : std::numeric_limits<double>::quiet_NaN();
  std::string why = reason;
  for(auto & ch : why) { if(ch == ' ') { ch = '_'; } }
  if(search && std::string(path) == "route" && feasible
     && candidate.predictedPresentationTime + 1e-12 < staticReach)
  {
    // Premise of the exact timing prune: route presentation duration is never
    // shorter than the static reach time it was derived from.
    mc_rtc::log::error(
        "[V2TimingPrunePremiseViolation] planningGeneration={} hypothesis={} candidate={} route={} presentationDuration={:.12g} staticReachTime={:.12g}",
        plannerContext_.certJobGeneration, finiteSearch_.evaluatedHypotheses, candidate.name,
        candidate.transitRouteName, candidate.predictedPresentationTime, staticReach);
  }
  // A1 logging only: terminal timing audit outcome and cost-input finiteness of
  // every retreat-certified route record, plus a PROXY objective computed on a
  // copy with the certifier's own pre-audit execution-time estimate. Nothing
  // below writes to the candidate, the plan set or any decision input.
  std::string a1Fields =
      " auditRan=- auditSuccess=- auditReason=- auditDuration=nan legacyExecutionDuration=nan costInputsFinite=- proxyGlobalJPreAuditTime=nan";
  if(!isStatic && feasible)
  {
    const bool inputsFinite = std::isfinite(candidate.predictedEffort)
        && std::isfinite(candidate.transitPathLength)
        && std::isfinite(candidate.predictiveReachClearance)
        && std::isfinite(candidate.predictiveRetreatClearance)
        && std::isfinite(candidate.minimumJointMarginRatio)
        && std::isfinite(candidate.minimumConditionIndex)
        && std::isfinite(candidate.maximumJointVelocityUtilization)
        && std::isfinite(candidate.terminalVelocityUtilization);
    CaptureCandidate proxy = candidate;
    proxy.terminalTimingAuditSuccess = true;
    proxy.auditEstimatedTime = candidate.legacyEstimatedTime;
    computeCompletePlanAuditCost(proxy);
    const double proxyJ = proxy.completeCostAuditValid
        ? call_handover::extendMotionCostToSearchEpoch(
              proxy.completeCostAudit, plannerConfig_.decisionTimeWeight,
              plannerConfig_.decisionTimeReference, lead, candidate.predictedPresentationTime)
        : std::numeric_limits<double>::quiet_NaN();
    std::string auditReason = candidate.terminalTimingAuditReason.empty() ? std::string("none") : candidate.terminalTimingAuditReason;
    for(auto & ch : auditReason) { if(ch == ' ') { ch = '_'; } }
    a1Fields = fmt::format(
        " auditRan={} auditSuccess={} auditReason={} auditDuration={:.6f} legacyExecutionDuration={:.12g} costInputsFinite={} proxyGlobalJPreAuditTime={:.12g}",
        candidate.terminalTimingAuditRan, candidate.terminalTimingAuditSuccess, auditReason,
        candidate.terminalTimingAuditDuration, candidate.legacyEstimatedTime, inputsFinite, proxyJ);
  }
  mc_rtc::log::info(
      "[CertStage] job={} planningGeneration={} hypothesis={} lead={:.3f} eventTime={:.9f} path={} grasp={}/{} candidate={} route={} feasible={} deepest={} costValid={} staticReachTime={:.6f} routeReachDuration={:.6f} reachClear={:.9f} retreatClear={:.9f} reason={} jobWall={:.6f} workUnits={} motionJ={:.12g} globalJ={:.12g} presentationDuration={:.12g} executionDuration={:.12g} prof={}{}",
      plannerContext_.certJobType, plannerContext_.certJobGeneration,
      search ? finiteSearch_.evaluatedHypotheses : 0,
      search ? finiteSearch_.currentLead : std::numeric_limits<double>::quiet_NaN(),
      search ? finiteSearch_.currentPresentationTime : v2Request_.plan.presentationTime,
      path, (search && !isMemo) ? plannerContext_.planningCandidateIndex : -1,
      search ? plannerContext_.planningCandidateCount : 0,
      candidate.name, isStatic ? std::string("-") : candidate.transitRouteName,
      feasible, deepest, isStatic ? false : candidate.completeCostAuditValid,
      staticReach, routeReachDuration,
      isStatic ? plannerContext_.certStaticReachClearance : candidate.predictiveReachClearance,
      candidate.predictiveRetreatClearance, why.empty() ? std::string("none") : why,
      jobWall, workUnitsV2(), candidate.completeCostAudit, globalJ,
      candidate.predictedPresentationTime, candidate.auditEstimatedTime, stageSnapshotV2(), a1Fields);
}

void HandoverInterceptionController::logMemoRecordsV2(const std::vector<CaptureCandidate> & completePlans) const
{
  if(!stageProfilingActiveV2()) { return; }
  for(const auto & candidate : completePlans)
  {
    logCertStageV2("memo", candidate, true, "CARRIED_RETREAT",
                   std::numeric_limits<double>::quiet_NaN(),
                   std::numeric_limits<double>::quiet_NaN(), "memo_reuse");
  }
}

std::string HandoverInterceptionController::stageSnapshotV2() const
{
  std::string out;
  for(int b = 0; b < PlannerContext::StageBucketCount; ++b)
  {
    if(b) { out += ','; }
    out += fmt::format("{}:{:.6f}", plannerContext_.stageCount[b], plannerContext_.stageWall[b]);
  }
  return out;
}

void HandoverInterceptionController::logJobProfileV2(const PendingJobV2 & pending, double now) const
{
  static const char * names[PlannerContext::StageBucketCount] = {
      "staticReachStandoff", "staticReachCapture", "staticClosure", "staticRetreat",
      "routeSetup", "routeReach", "routeApproach", "routeDwell", "routeClosure", "routeRetreat",
      "routeFinalize", "hypothesisSetup", "terminalStandoff",
      "nestedIkStep", "nestedSweptQuery", "nestedConfigurationSafety", "nestedClosureSafety"};
  std::string buckets;
  double partition = 0.0;
  for(int b = 0; b < PlannerContext::StageBucketCount; ++b)
  {
    if(b < PlannerContext::StageNestedIkStep) { partition += plannerContext_.stageWall[b]; }
    buckets += fmt::format(" {}={:.6f}s/{}", names[b], plannerContext_.stageWall[b],
                           plannerContext_.stageCount[b]);
  }
  mc_rtc::log::info(
      "[V2JobProfile] type={} planningGeneration={} profileGeneration={} cancelRequested={} jobWall={:.6f}s partitionedWall={:.6f}s hypotheses={} memoReuses={} staticRecords={} routeRecords={} timingPrunedGraspTau={} workUnits={} latency={:.6f}s{} t={:.6f}",
      receiverJobTypeNameV2(pending.type), pending.planningGeneration,
      plannerContext_.certJobGeneration, pending.cancelRequested, plannerContext_.certJobWall,
      partition,
      pending.type == ReceiverJobTypeV2::FullSearch ? finiteSearch_.evaluatedHypotheses : 0,
      plannerContext_.certMemoReuses, plannerContext_.certStaticRecords,
      plannerContext_.certRouteRecords, plannerContext_.certTimingPruned, workUnitsV2(),
      now - pending.submitTime, buckets, now);
}

// Characterization: every complete (tau, g, r) record of one search, and the
// unchanged selector's choice evaluated both at the search epoch and at receipt.
void HandoverInterceptionController::logCharacterizationSearchV2(const PendingJobV2 & pending, double now)
{
  const FrozenPlanSet & set = plannerResult();
  std::vector<call_handover::FiniteEventPlanRecord> records;
  for(std::size_t i = 0; i < set.alternatives.size(); ++i)
  {
    const auto & a = set.alternatives[i];
    call_handover::FiniteEventPlanRecord record;
    record.sourceIndex = i;
    record.hypothesisIndex = a.hypothesisIndex;
    record.costValid = a.candidate.completeCostAuditValid;
    record.motionCost = a.candidate.completeCostAudit;
    record.globalCost = a.globalObjectiveCost;
    record.eventLead = a.eventLeadFromSearchEpoch;
    record.eventPresentationTime = a.eventPresentationTime;
    record.predictedPresentationDuration = a.candidate.predictedPresentationTime;
    record.predictedExecutionDuration = a.candidate.auditEstimatedTime;
    record.clearance = a.candidate.predictiveReachClearance;
    record.candidateName = a.candidate.name;
    record.routeName = a.candidate.transitRouteName;
    records.push_back(record);
    const Eigen::Vector3d p = a.W_T_O_presentation.translation();
    mc_rtc::log::info(
        "[V2CompleteRecord] planningGeneration={} hypothesis={} lead={:.3f} eventTime={:.6f} candidate={} route={} costValid={} motionJ={:.9f} globalJ={:.9f} presentationDuration={:.6f} executionDuration={:.6f} reachClear={:.5f} retreatClear={:.5f} presentation=[{:.5f},{:.5f},{:.5f}]",
        pending.planningGeneration, a.hypothesisIndex, a.eventLeadFromSearchEpoch,
        a.eventPresentationTime, a.candidate.name, a.candidate.transitRouteName,
        a.candidate.completeCostAuditValid, a.candidate.completeCostAudit, a.globalObjectiveCost,
        a.candidate.predictedPresentationTime, a.candidate.auditEstimatedTime,
        a.candidate.predictiveReachClearance, a.candidate.predictiveRetreatClearance,
        p.x(), p.y(), p.z());
  }
  const double minimumSafeCommitLead = std::max(
      v2Params_.minimumCommitRemainingTime, presentationDecelerationDuration_ + 0.25);
  for(const auto & when : {std::make_pair("epoch", pending.submitTime), std::make_pair("receipt", now)})
  {
    const auto selection = call_handover::selectFiniteEventPlan(
        records, when.second, v2Params_.minimumReachEntryLead, minimumSafeCommitLead,
        decisionCostTieTolerance_);
    const bool ok = selection.success && selection.selectedRecord < set.alternatives.size();
    mc_rtc::log::success(
        "[V2CharacterizationSelection] planningGeneration={} evaluatedAt={} t={:.6f} success={} reason={} completePlans={} costValidPlans={} timingAdmissiblePlans={} hypothesis={} lead={:.3f} candidate={} route={} globalJ={:.9f} minimumSafeCommitLead={:.3f} minimumReachEntryLead={:.3f}",
        pending.planningGeneration, when.first, when.second, selection.success, selection.reason,
        selection.completePlanCount, selection.costValidCount, selection.timingAdmissibleCount,
        ok ? set.alternatives[selection.selectedRecord].hypothesisIndex : 0,
        ok ? set.alternatives[selection.selectedRecord].eventLeadFromSearchEpoch : 0.0,
        ok ? set.alternatives[selection.selectedRecord].candidate.name : std::string("none"),
        ok ? set.alternatives[selection.selectedRecord].candidate.transitRouteName : std::string("none"),
        ok ? set.alternatives[selection.selectedRecord].globalObjectiveCost : 0.0,
        minimumSafeCommitLead, v2Params_.minimumReachEntryLead);
  }
}

// =============================================================================
// FULL_SEARCH prediction updates: select, then certify only the selected action
// =============================================================================
//
// Classification when the independent prediction changes while a FULL_SEARCH
// runs (tolerances are the existing ones):
//  - unsafe/stale job: the held arm left the snapshot or the receiver state
//    generation advanced -> the job is cancelled (unchanged);
//  - obsolete job: no hypothesis that can still be admitted keeps its target
//    inside the tube -> nothing unaffected remains; cancelled, new epoch;
//  - record whose target moved: its certified presentation pose is outside the
//    commit-freshness tube of the prediction at its event instant -> not
//    adoptable as is; if the unchanged selector picks it, that single action is
//    certified at the newest prediction (the RECERTIFY_ACTIVE rollout, from the
//    same held start and reach index 0) before adoption; a failed certificate
//    excludes that record and the selector runs again on the rest;
//  - record within the tube: adoptable exactly as before.
// No search work is discarded because a target moved.

long long HandoverInterceptionController::workUnitsV2() const
{
  long long units = 0;
  for(int b = PlannerContext::StageStaticReachStandoff; b <= PlannerContext::StageHypothesisSetup; ++b)
  {
    units += plannerContext_.stageCount[b];
  }
  return units;
}

void HandoverInterceptionController::filterHypothesisFreshnessV2(const PendingJobV2 & pending, double now)
{
  // Classification only. Records whose target moved beyond the tube stay in
  // the set; if the selector picks one, only that action is re-certified.
  const ObjectPredictionRecordV2 prediction = currentObjectPredictionV2();
  if(!prediction.valid) { return; }
  const auto & policy = predictiveReachPolicy_;
  std::size_t moved = 0;
  double maxDeviation = 0.0;
  for(const auto & a : plannerResult_.alternatives)
  {
    const sva::PTransformd p = predictionPoseAtV2(prediction, a.eventPresentationTime);
    const double d = (p.translation() - a.W_T_O_presentation.translation()).norm();
    const double rot = orientationError(p, a.W_T_O_presentation);
    maxDeviation = std::max(maxDeviation, d);
    if(d > policy.maximumObjectTranslationDeviation || rot > policy.maximumObjectRotationDeviation) { ++moved; }
  }
  mc_rtc::log::info(
      "[V2HypothesisFreshnessAtReceipt] planningGeneration={} records={} withinTube={} targetMoved={} maxDeviation={:.6f}m tolerance={:.3f}m t={:.6f}",
      pending.planningGeneration, plannerResult_.alternatives.size(),
      plannerResult_.alternatives.size() - moved, moved, maxDeviation,
      policy.maximumObjectTranslationDeviation, now);
}

void HandoverInterceptionController::handleSelectedCertificationV2(const PendingJobV2 & pending, double now)
{
  const ReceiverJobResultV2 result = v2Result_;
  if(v2SelectedRecord_ >= plannerResult_.alternatives.size()
     || v2ExcludedRecords_.size() != plannerResult_.alternatives.size())
  {
    mc_rtc::log::warning(
        "[V2SelectedCertification] certificateGeneration={} success=false reason=search_result_replaced effect=none t={:.6f}",
        pending.planningGeneration, now);
    return;
  }
  const GlobalEventPlanAlternative alternative = plannerResult_.alternatives[v2SelectedRecord_];
  InterceptionPlan plan = result.plan;
  const bool ok = result.success;
  mc_rtc::log::info(
      "[V2SelectedCertification] certificateGeneration={} searchGeneration={} record={} hypothesis={} candidate={} route={} success={} reason={} t={:.6f}",
      pending.planningGeneration, v2SelectedSearchGeneration_, v2SelectedRecord_,
      alternative.hypothesisIndex, alternative.candidate.name, alternative.candidate.transitRouteName,
      ok, result.success ? std::string("certified") : result.reason, now);
  if(ok)
  {
    // The target may have moved again while the certificate was computed.
    const ObjectPredictionRecordV2 latest = currentObjectPredictionV2();
    const sva::PTransformd predicted = predictionPoseAtV2(latest, plan.presentationTime);
    const bool freshNow =
        (predicted.translation() - plan.objectAtPresentation.translation()).norm()
            <= predictiveReachPolicy_.maximumObjectTranslationDeviation
        && orientationError(predicted, plan.objectAtPresentation)
            <= predictiveReachPolicy_.maximumObjectRotationDeviation;
    if(!freshNow)
    {
      mc_rtc::log::info(
          "[V2SelectedCertification] certificateGeneration={} record={} outcome=target_moved_during_certification action=certify_selected_again t={:.6f}",
          pending.planningGeneration, v2SelectedRecord_, now);
      if(!adoptFullSearchResultV2(now)) { ++v2Counters_.searchFailures; }
      return;
    }
    CaptureCandidate candidate = result.candidate;
    candidate.name = alternative.candidate.name;
    candidate.transitRouteName = alternative.candidate.transitRouteName;
    adoptProvisionalPlanV2(alternative, candidate, plan, now, "certified",
                           v2SelectedSearchGeneration_, pending.planningGeneration);
    return;
  }
  ++v2SelectedCertificationFailures_;
  v2ExcludedRecords_[v2SelectedRecord_] = 1;
  if(!adoptFullSearchResultV2(now))
  {
    ++v2Counters_.searchFailures;
  }
}

// =============================================================================
// Exact timing prune of route certification (FULL_SEARCH, logging + decision)
// =============================================================================
//
// The selector admits a record at decision time t_d iff
//   (A) tau - t_d + eps >= L_commit      and
//   (B) T_pres + L_entry <= tau - t_d + eps,          eps = 1e-12,
// with T_pres the record's presentation duration. For a route of (tau, g):
//   T_pres = max(2 dt, max(2 dt, T_s) * max(1, stretch)) >= T_s,
// where T_s = timingArmScale * static reach-to-standoff duration of g at tau
// (beginPredictiveRouteCandidate, makeInterceptionPlan, finalize). The route
// record can only be selected at a time t_d >= t_k, the controller clock
// published before this check (monotone). Hence if
//   tau - t_k + eps < L_commit   or   T_s + L_entry > tau - t_k + eps,
// then for every route of (tau, g) and every t_d >= t_k, (A) or (B) fails.
// Memoized hypotheses reuse this hypothesis's routes for later events with a
// bitwise identical presentation pose, so tau is taken as the latest such event.
bool HandoverInterceptionController::exactTimingPruneRoutesV2(const CaptureCandidate & staticCandidate) const
{
  if(!stageProfilingActiveV2() || !plannerConfig_.v2ExactTimingPrune
     || plannerContext_.certJobType != "FULL_SEARCH")
  {
    return false;
  }
  const auto & bank = finiteSearch_.bank;
  if(finiteSearch_.cursor == 0 || finiteSearch_.cursor > bank.leads.size()
     || bank.presentationPoses.size() != bank.leads.size())
  {
    return false;
  }
  const std::size_t h = finiteSearch_.cursor - 1;
  const sva::PTransformd & pose = bank.presentationPoses[h];
  double latestTau = bank.searchEpoch + bank.leads[h];
  for(std::size_t i = h + 1; i < bank.leads.size(); ++i)
  {
    if(bank.presentationPoses[i].translation() == pose.translation()
       && bank.presentationPoses[i].rotation() == pose.rotation())
    {
      latestTau = std::max(latestTau, bank.searchEpoch + bank.leads[i]);
    }
  }
  const double tk = v2ControllerClockForWorker_.load(std::memory_order_acquire);
  const double staticReachTime = staticCandidate.predictedPresentationTime;
  const double remaining = latestTau - tk;
  const bool commitFails = remaining + 1e-12 < plannerConfig_.v2PruneCommitLead;
  const bool entryFails = staticReachTime + plannerConfig_.v2PruneEntryLead > remaining + 1e-12;
  if(!(commitFails || entryFails)) { return false; }
  ++plannerContext_.certTimingPruned;
  mc_rtc::log::info(
      "[V2TimingPrune] planningGeneration={} hypothesis={} grasp={}/{} candidate={} tau={:.9f} latestTauSamePose={:.9f} clock={:.6f} staticReachTime={:.9f} remaining={:.9f} commitLead={:.3f} entryLead={:.3f} commitFails={} entryFails={} jobWall={:.6f} workUnits={}",
      plannerContext_.certJobGeneration, finiteSearch_.evaluatedHypotheses,
      plannerContext_.planningCandidateIndex, plannerContext_.planningCandidateCount,
      staticCandidate.name, bank.searchEpoch + bank.leads[h], latestTau, tk, staticReachTime, remaining,
      plannerConfig_.v2PruneCommitLead, plannerConfig_.v2PruneEntryLead, commitFails, entryFails,
      std::chrono::duration<double>(std::chrono::steady_clock::now() - plannerContext_.certJobStart).count(),
      workUnitsV2());
  return true;
}

// =============================================================================
// Phase 2B preview/runtime parity instrumentation (logging only)
// =============================================================================

void HandoverInterceptionController::logParityTraceV2(
    const char * tag, std::uint64_t planId, const std::vector<ParitySampleV2> & trace) const
{
  for(std::size_t i = 0; i < trace.size(); ++i)
  {
    const auto & s = trace[i];
    mc_rtc::log::info(
        "[{}] planId={} i={} phase={} t={:.6f} p=[{:.6f},{:.6f},{:.6f}] q=[{:.6f},{:.6f},{:.6f},{:.6f}] clear={:.5f} ref=[{:.6f},{:.6f},{:.6f}] rate=[{:.6f},{:.6f},{:.6f}] cmd=[{:.6f},{:.6f},{:.6f}] scale={:.4f}",
        tag, planId, i, s.phase, s.t, s.p.x(), s.p.y(), s.p.z(), s.q.w(), s.q.x(), s.q.y(), s.q.z(),
        s.clearance, s.reference.x(), s.reference.y(), s.reference.z(), s.rateLimited.x(), s.rateLimited.y(),
        s.rateLimited.z(), s.command.x(), s.command.y(), s.command.z(), s.clearanceScale);
    if(!s.jointQ.empty())
    {
      std::string q, pt;
      for(std::size_t k = 0; k < s.jointQ.size(); ++k)
      {
        q += (k ? "," : "") + std::to_string(s.jointQ[k]);
        pt += (k ? "," : "") + std::to_string(k < s.postureTarget.size() ? s.postureTarget[k] : 0.0);
      }
      mc_rtc::log::info("[{}Joints] planId={} i={} jointMarginRatio={:.4f} q=[{}] postureTarget=[{}]", tag, planId, i,
                        s.jointMarginRatio, q, pt);
    }
  }
}

void HandoverInterceptionController::logParityRuntimeV2()
{
  if(v2ParityLastRuntimeLog_ >= 0.0 && controllerTime_ < v2ParityLastRuntimeLog_ + 0.01 - 1e-9) { return; }
  v2ParityLastRuntimeLog_ = controllerTime_;
  const sva::PTransformd mouth = actualMouthPose();
  const Eigen::Quaterniond mq(worldRotation(mouth));
  const Eigen::Quaterniond oq(worldRotation(W_T_O_));
  HandoverSafetyReport report;
  double clear = std::numeric_limits<double>::quiet_NaN();
  const char * clearSource = "pose";
  if(objectAttached_)
  {
    evaluateAttachedRetreatSafety(report);
    clearSource = "attached_retreat";
  }
  else
  {
    evaluateCurrentPoseSafety(report, false);
  }
  clear = report.minClearance;
  mc_rtc::log::info(
      "[V2ParityRuntime] t={:.6f} state={} v2phase={} planId={} mouth=[{:.6f},{:.6f},{:.6f}] mouthQ=[{:.6f},{:.6f},{:.6f},{:.6f}] object=[{:.6f},{:.6f},{:.6f}] objectQ=[{:.6f},{:.6f},{:.6f},{:.6f}] truth=[{:.6f},{:.6f},{:.6f}] closure={:.5f} attached={} clear={:.5f} clearSource={} v2ref=[{:.6f},{:.6f},{:.6f}] clearanceScale={:.4f}",
      controllerTime_, executor_.state(), receiverPhaseNameV2(v2Phase_), provisionalReceiverPlan_.planId,
      mouth.translation().x(), mouth.translation().y(), mouth.translation().z(), mq.w(), mq.x(), mq.y(), mq.z(),
      W_T_O_.translation().x(), W_T_O_.translation().y(), W_T_O_.translation().z(), oq.w(), oq.x(), oq.y(), oq.z(),
      W_T_O_truth_.translation().x(), W_T_O_truth_.translation().y(), W_T_O_truth_.translation().z(),
      measuredGripperClosure(), objectAttached_, clear, clearSource, v2ReferencePose_.translation().x(),
      v2ReferencePose_.translation().y(), v2ReferencePose_.translation().z(), v2ClearanceScale_);
}

// =============================================================================
// TRIAD-lite: control-aware supervisory grasp selection
// (ReceiverV2 supervisorMode: control_aware; default bank_search is unchanged)
// =============================================================================
//
// Decision variable: a receiving grasp g = (sign, phi) about the handle axis.
// Layers, evaluated from the frozen current robot state and the current object
// estimate with the controller's own preview kinematics:
//   geometry  - insertion corridor to the capture pose, bilateral pad contact
//               without penetration, no interference with the modelled object;
//   robot     - convergence, joint limits, ground clearance, carried retreat,
//               and no limited joint inside the QP security distance;
//   clearance - min(reach, carried-retreat) non-contact clearance >= floor;
//   authority - directional realizability of the declared acquisition twist
//               within the QP joint-velocity box (ControlAwareGraspSupervisor.h).
// Selection is the transparent cascade in selectGraspLexicographic with
// hysteresis. The event time is not searched: acquisition entry is the first
// time the measured gate holds for the stable dwell together with a fresh
// terminal certificate; the grasp is frozen at that single commit.

namespace
{
std::string caVec3(const Eigen::Vector3d & v)
{
  return fmt::format("[{:.5f},{:.5f},{:.5f}]", v.x(), v.y(), v.z());
}

std::string caDamper(const std::vector<int> & d)
{
  std::string s;
  for(std::size_t i = 0; i < d.size(); ++i) { s += (i ? "," : "") + std::to_string(d[i]); }
  return "[" + s + "]";
}

std::string caAuthority(const std::vector<call_handover::AuthorityEvaluation> & evals)
{
  std::string s;
  for(const auto & e : evals)
  {
    s += fmt::format("{}{}:res={:.6f}:kappa={:.4f}", s.empty() ? "" : ";", e.label, e.residual, e.reserve);
  }
  return "[" + s + "]";
}
} // namespace

bool HandoverInterceptionController::loadControlAwareConfigV2(const mc_rtc::Configuration & stateConfig)
{
  const std::string mode = stateConfig.has("supervisorMode")
      ? static_cast<std::string>(stateConfig("supervisorMode")) : std::string("bank_search");
  if(mode != "bank_search" && mode != "control_aware")
  {
    mc_rtc::log::error("[TriadLiteConfig] supervisorMode must be bank_search or control_aware, got {}", mode);
    return false;
  }
  v2ControlAware_ = mode == "control_aware";
  v2CaSelector_ = call_handover::GraspSelectorState{};
  v2CaSelectionsSubmitted_ = 0;
  v2CaSelectionsProcessed_ = 0;
  v2CaSwitches_ = 0;
  v2CaAborts_ = 0;
  v2CaAdmitsDeferred_ = 0;
  if(!v2ControlAware_)
  {
    mc_rtc::log::info("[TriadLiteConfig] supervisorMode=bank_search controlAware=false");
    return true;
  }
  if(v2Params_.characterizeOnly)
  {
    mc_rtc::log::error("[TriadLiteConfig] supervisorMode=control_aware is incompatible with characterizeOnly");
    return false;
  }

  // The authority test must use exactly the bounds the QP was built with:
  // the kinematics constraint entry of this controller configuration, with the
  // mc_rtc loader default velocityPercent = 0.5 when the key is absent.
  bool found = false;
  if(config().has("constraints"))
  {
    const auto constraints = config()("constraints");
    for(std::size_t i = 0; i < constraints.size(); ++i)
    {
      const auto c = constraints[i];
      if(!c.has("type") || static_cast<std::string>(c("type")) != "kinematics") { continue; }
      if(!c.has("damper"))
      {
        mc_rtc::log::error("[TriadLiteConfig] control_aware requires the kinematics constraint to declare its damper");
        return false;
      }
      const std::vector<double> damper = c("damper");
      if(damper.size() != 3)
      {
        mc_rtc::log::error("[TriadLiteConfig] kinematics damper must have 3 entries, got {}", damper.size());
        return false;
      }
      v2QpLimits_.interPercent = damper[0];
      v2QpLimits_.securityPercent = damper[1];
      v2QpLimits_.damperOffset = damper[2];
      v2QpLimits_.velocityPercent = c("velocityPercent", 0.5);
      found = true;
      break;
    }
  }
  if(!found)
  {
    mc_rtc::log::error("[TriadLiteConfig] control_aware requires a kinematics constraint in the controller configuration");
    return false;
  }

  ControlAwareParametersV2 p;
  const auto configs = config()("configs");
  if(configs.has("HandoverInterceptionController_MovePregrasp"))
  {
    p.insertionSpeed = readDouble(configs("HandoverInterceptionController_MovePregrasp"), "farLinearSpeed", p.insertionSpeed);
  }
  if(config().has("decisionCost"))
  {
    p.angularCharacteristicLength = readDouble(config()("decisionCost"), "characteristicLength", p.angularCharacteristicLength);
  }
  if(stateConfig.has("controlAware"))
  {
    const auto ca = stateConfig("controlAware");
    p.graspsPerSign = std::max(4, readInt(ca, "graspsPerSign", p.graspsPerSign));
    p.clearanceFloor = readDouble(ca, "clearanceFloor", p.clearanceFloor);
    p.kappaMin = readDouble(ca, "kappaMin", p.kappaMin);
    p.kappaMaximum = readDouble(ca, "kappaMaximum", p.kappaMaximum);
    p.residualTolerance = readDouble(ca, "residualTolerance", p.residualTolerance);
    p.angularCharacteristicLength = readDouble(ca, "angularCharacteristicLength", p.angularCharacteristicLength);
    if(ca.has("insertionSpeed") && readDouble(ca, "insertionSpeed", -1.0) >= 0.0)
    {
      p.insertionSpeed = readDouble(ca, "insertionSpeed", p.insertionSpeed);
    }
    p.disturbanceSpeed = readDouble(ca, "disturbanceSpeed", p.disturbanceSpeed);
    p.switchDwell = readDouble(ca, "switchDwell", p.switchDwell);
    p.requireObjectStopped = readBool(ca, "requireObjectStopped", p.requireObjectStopped);
    p.admissionRequiresCurrentAuthority = readBool(ca, "admissionRequiresCurrentAuthority", p.admissionRequiresCurrentAuthority);
    p.reselectWhileTracking = readBool(ca, "reselectWhileTracking", p.reselectWhileTracking);
    p.trustIncumbentReevaluation = readBool(ca, "trustIncumbentReevaluation", p.trustIncumbentReevaluation);
    if(ca.has("graspFamily")) { p.graspFamily = static_cast<std::string>(ca("graspFamily")); }
    if(ca.has("receivingFamily"))
    {
      const auto rf = ca("receivingFamily");
      p.receivingSpec.thetaSamples = std::max(1, readInt(rf, "thetaSamples", p.receivingSpec.thetaSamples));
      p.receivingSpec.axialSamples = std::max(1, readInt(rf, "axialSamples", p.receivingSpec.axialSamples));
      p.receivingAxialMax = readDouble(rf, "axialMax", p.receivingAxialMax);
      p.fingerHalfWidth = readDouble(rf, "fingerHalfWidth", p.fingerHalfWidth);
      p.axialMargin = readDouble(rf, "axialMargin", p.axialMargin);
    }
    if(ca.has("reachabilityFunnel"))
    {
      const auto rf = ca("reachabilityFunnel");
      p.funnelEnabled = readBool(rf, "enabled", p.funnelEnabled);
      p.funnelShortlistSize = readInt(rf, "shortlistSize", p.funnelShortlistSize);
      p.funnelPruneTolerance = readDouble(rf, "pruneTolerance", p.funnelPruneTolerance);
      p.funnelThetaBinWidth = readDouble(rf, "thetaBinWidthDeg", p.funnelThetaBinWidth * 180.0 / M_PI) * M_PI / 180.0;
      p.funnelCharacterizeAll = readBool(rf, "characterizeAll", p.funnelCharacterizeAll);
    }
    if(ca.has("interception"))
    {
      const auto ic = ca("interception");
      if(ic.has("mode")) { p.interceptionMode = static_cast<std::string>(ic("mode")); }
      p.interceptionHorizon = readDouble(ic, "horizon", p.interceptionHorizon);
      p.interceptionMinimumStep = readDouble(ic, "minimumStep", p.interceptionMinimumStep);
      p.interceptionComputationLatency = readDouble(ic, "computationLatency", p.interceptionComputationLatency);
      p.interceptionTimingSkip = readBool(ic, "timingSkip", p.interceptionTimingSkip);
      p.interceptionMaximumExactEvaluations = readInt(ic, "maximumExactEvaluations", p.interceptionMaximumExactEvaluations);
      p.interceptionMaximumRollouts = readInt(ic, "maximumRollouts", p.interceptionMaximumRollouts);
      p.interceptionLogAttempts = readBool(ic, "logAttempts", p.interceptionLogAttempts);
      p.authorityStride = std::max(1, readInt(ic, "authorityStride", p.authorityStride));
      p.authorityFilter = readBool(ic, "authorityFilter", p.authorityFilter);
    }
    p.selection.clearanceTieBand = readDouble(ca, "clearanceTieBand", p.selection.clearanceTieBand);
    p.selection.reserveTieBand = readDouble(ca, "reserveTieBand", p.selection.reserveTieBand);
    p.selection.reserveSaturation = readDouble(ca, "reserveSaturation", p.selection.reserveSaturation);
  }
  // Interface values inherited from the states that own them (not tuned here).
  p.interceptionEntryLead = v2Params_.minimumReachEntryLead;
  p.interceptionEpsPosition = plannerConfig_.predictiveReachPolicy.maximumObjectTranslationDeviation;
  p.interceptionEpsRotation = plannerConfig_.predictiveReachPolicy.maximumObjectRotationDeviation;
  if(configs.has("HandoverInterceptionController_MovePregrasp"))
  {
    const auto mp = configs("HandoverInterceptionController_MovePregrasp");
    p.interceptionTerminalLinearSpeed = readDouble(mp, "terminalLinearSpeedTolerance", p.interceptionTerminalLinearSpeed);
    p.interceptionTerminalAngularSpeed = readDouble(mp, "terminalAngularSpeedTolerance", p.interceptionTerminalAngularSpeed);
  }
  if(p.interceptionMode != "disabled" && p.interceptionMode != "characterize" && p.interceptionMode != "characterize_hold")
  {
    mc_rtc::log::error("[TriadLiteConfig] interception.mode must be disabled, characterize or characterize_hold, got {}",
                       p.interceptionMode);
    return false;
  }
  if(p.interceptionMode != "disabled"
     && (p.graspFamily != "receiving" || !(p.interceptionHorizon > 0.0) || !(p.interceptionMinimumStep > 0.0)
         || p.interceptionComputationLatency < 0.0 || p.interceptionMaximumExactEvaluations < 1
         || p.interceptionMaximumRollouts < 1))
  {
    mc_rtc::log::error("[TriadLiteConfig] interception requires graspFamily=receiving and positive horizon/step/budgets");
    return false;
  }
  if(p.graspFamily != "legacy_ring" && p.graspFamily != "receiving")
  {
    mc_rtc::log::error("[TriadLiteConfig] graspFamily must be legacy_ring or receiving, got {}", p.graspFamily);
    return false;
  }
  if(!(p.kappaMin > 0.0 && p.kappaMaximum > p.kappaMin && p.residualTolerance > 0.0 && p.switchDwell >= 0.0
       && p.angularCharacteristicLength > 0.0 && p.insertionSpeed >= 0.0 && p.disturbanceSpeed >= 0.0))
  {
    mc_rtc::log::error("[TriadLiteConfig] invalid controlAware parameters");
    return false;
  }
  v2CaParams_ = p;
  mc_rtc::log::info(
      "[TriadLiteConfig] graspFamily={} thetaSamples={} axialSamples={} axialMax={:.4f} fingerHalfWidth={:.4f} axialMargin={:.4f} funnel=[enabled:{},K:{},pruneTolerance:{:.4f},thetaBinDeg:{:.3f},characterizeAll:{}] reachabilityMap=[urdfSha256:{},samples:{},step:{:.3f}]",
      p.graspFamily, p.receivingSpec.thetaSamples, p.receivingSpec.axialSamples, p.receivingAxialMax, p.fingerHalfWidth,
      p.axialMargin, p.funnelEnabled, p.funnelShortlistSize, p.funnelPruneTolerance, p.funnelThetaBinWidth * 180.0 / M_PI,
      p.funnelCharacterizeAll, call_handover::gen3_wrist_reachability::kUrdfSha256,
      call_handover::gen3_wrist_reachability::kSamples, call_handover::gen3_wrist_reachability::kStep);
  mc_rtc::log::info(
      "[TriadLiteConfig] interception=[mode:{},horizon:{:.3f},minimumStep:{:.3f},computationLatency:{:.3f},entryLead:{:.3f},epsPosition:{:.4f},epsRotation:{:.4f},terminalLinearSpeed:{:.3f},terminalAngularSpeed:{:.3f},timingSkip:{},maximumExactEvaluations:{},maximumRollouts:{},logAttempts:{}]",
      p.interceptionMode, p.interceptionHorizon, p.interceptionMinimumStep, p.interceptionComputationLatency,
      p.interceptionEntryLead, p.interceptionEpsPosition, p.interceptionEpsRotation, p.interceptionTerminalLinearSpeed,
      p.interceptionTerminalAngularSpeed, p.interceptionTimingSkip, p.interceptionMaximumExactEvaluations,
      p.interceptionMaximumRollouts, p.interceptionLogAttempts);
  mc_rtc::log::info("[TriadLiteConfig] interceptionAuthority=[demands:path_command+synchronization+insertion,stride:{},filter:{},kappaMin:{:.3f},insertionSpeed:{:.3f}]",
                    p.authorityStride, p.authorityFilter, p.kappaMin, p.insertionSpeed);
  mc_rtc::log::success(
      "[TriadLiteConfig] supervisorMode=control_aware graspsPerSign={} clearanceFloor={:.4f} kappaMin={:.3f} kappaMaximum={:.3f} residualTolerance={:.2e} angularLength={:.3f} insertionSpeed={:.3f} disturbanceSpeed={:.3f} switchDwell={:.3f} requireObjectStopped={} admissionRequiresCurrentAuthority={} reselectWhileTracking={} trustIncumbentReevaluation={} tieBands=[clearance:{:.4f},reserve:{:.3f}] reserveSaturation={:.3f} qpLimits=[velocityPercent:{:.3f},inter:{:.3f},security:{:.3f},damperOffset:{:.3f}] authorityLevel=velocity accelerationBounds=module_default_infinite xi=damperOffset_lower_bound",
      p.graspsPerSign, p.clearanceFloor, p.kappaMin, p.kappaMaximum, p.residualTolerance,
      p.angularCharacteristicLength, p.insertionSpeed, p.disturbanceSpeed, p.switchDwell,
      p.requireObjectStopped, p.admissionRequiresCurrentAuthority, p.reselectWhileTracking, p.trustIncumbentReevaluation,
      p.selection.clearanceTieBand, p.selection.reserveTieBand, p.selection.reserveSaturation,
      v2QpLimits_.velocityPercent, v2QpLimits_.interPercent, v2QpLimits_.securityPercent, v2QpLimits_.damperOffset);
  return true;
}

std::string HandoverInterceptionController::classifyControlAwareRejectionV2(const std::string & reason)
{
  auto has = [&reason](const char * s) { return reason.find(s) != std::string::npos; };
  if(has("no_convergence") || has("kinematics_unavailable")) { return "ik"; }
  if(has("joint_limit")) { return "joint_limits"; }
  if(has("ground_plane")) { return "collision"; }
  return "geometry";
}

std::vector<call_handover::AuthorityDemand> HandoverInterceptionController::controlAwareDemandsV2(
    const Eigen::Vector3d & objectLinearVelocity, const Eigen::Vector3d & objectAngularVelocity,
    const Eigen::Vector3d & objectPosition, const sva::PTransformd & W_T_M_goal,
    const sva::PTransformd & W_T_B_eval) const
{
  // Tool-body (gen3_robotiq_85_base_link) twist, world frame, [angular; linear]:
  // follow the rigid object motion at the body origin, and additionally insert
  // along -y_M of the grasp at the acquisition insertion speed.
  const Eigen::Vector3d follow = objectLinearVelocity
      + objectAngularVelocity.cross(W_T_B_eval.translation() - objectPosition);
  const Eigen::Vector3d insert = -v2CaParams_.insertionSpeed * worldRotation(W_T_M_goal).col(1);
  std::vector<call_handover::AuthorityDemand> demands;
  call_handover::AuthorityDemand d;
  d.label = "follow";
  d.twist.head<3>() = objectAngularVelocity;
  d.twist.tail<3>() = follow;
  demands.push_back(d);
  d.label = "follow_insert";
  // Same expression as call_handover::followInsertTwist (unit-tested against
  // the Phase D insertion demand at rest).
  d.twist = call_handover::followInsertTwist(objectLinearVelocity, objectAngularVelocity, W_T_B_eval.translation(),
                                             objectPosition, worldRotation(W_T_M_goal).col(1), v2CaParams_.insertionSpeed);
  demands.push_back(d);
  if(v2CaParams_.disturbanceSpeed > 0.0)
  {
    static const char * names[6] = {"dist+x", "dist-x", "dist+y", "dist-y", "dist+z", "dist-z"};
    for(int i = 0; i < 6; ++i)
    {
      Eigen::Vector3d e = Eigen::Vector3d::Zero();
      e[i / 2] = (i % 2 == 0 ? 1.0 : -1.0) * v2CaParams_.disturbanceSpeed;
      call_handover::AuthorityDemand dd;
      dd.label = std::string("follow_insert_") + names[i];
      dd.twist.head<3>() = objectAngularVelocity;
      dd.twist.tail<3>() = follow + insert + e;
      demands.push_back(dd);
    }
  }
  return demands;
}

std::vector<call_handover::AuthorityDemand> HandoverInterceptionController::insertionDemandV2(
    const sva::PTransformd & W_T_M_goal) const
{
  // MovePregrasp moves the mouth from the standoff (capture + d_s y_M) to the
  // capture pose at most at farLinearSpeed; the tool body shares the mouth's
  // translation velocity for a pure translation.
  call_handover::AuthorityDemand d;
  d.label = "insert";
  d.twist = call_handover::insertionTwist(worldRotation(W_T_M_goal).col(1), v2CaParams_.insertionSpeed);
  return {d};
}

std::vector<call_handover::AuthorityEvaluation> HandoverInterceptionController::controlAwareAuthorityAtPreviewV2(
    const rbd::MultiBodyConfig & mbc, const std::vector<call_handover::AuthorityDemand> & demands,
    std::vector<int> & damper, bool & insideSecurity) const
{
  const auto & mb = plannerModel();
  if(!plannerContext_.previewKinematicCacheValid || !plannerContext_.previewToolJacobian)
  {
    refreshPreviewKinematicCache();
  }
  rbd::Jacobian & jac = *plannerContext_.previewToolJacobian;
  const Eigen::MatrixXd Jc = jac.jacobian(mb, mbc);
  Eigen::MatrixXd Jfull = Eigen::MatrixXd::Zero(6, mb.nrDof());
  jac.fullJacobian(mb, Jc, Jfull);

  std::vector<int> cols;
  std::vector<double> q, qMin, qMax, vMin, vMax;
  const auto & ql = plannerConfig_.jointPositionLower;
  const auto & qu = plannerConfig_.jointPositionUpper;
  for(int j = 0; j < mb.nrJoints(); ++j)
  {
    if(plannerContext_.previewGripperJoint[static_cast<std::size_t>(j)] != 0u) { continue; }
    if(mb.joint(j).dof() != 1) { continue; }
    const int d = mb.jointPosInDof(j);
    cols.push_back(d);
    q.push_back(mbc.q[static_cast<std::size_t>(j)][0]);
    qMin.push_back(static_cast<std::size_t>(j) < ql.size() && !ql[static_cast<std::size_t>(j)].empty()
                       ? ql[static_cast<std::size_t>(j)][0] : -std::numeric_limits<double>::infinity());
    qMax.push_back(static_cast<std::size_t>(j) < qu.size() && !qu[static_cast<std::size_t>(j)].empty()
                       ? qu[static_cast<std::size_t>(j)][0] : std::numeric_limits<double>::infinity());
    vMin.push_back(plannerContext_.previewJointVelocityLower[d]);
    vMax.push_back(plannerContext_.previewJointVelocityUpper[d]);
  }
  const Eigen::Index n = static_cast<Eigen::Index>(cols.size());
  Eigen::MatrixXd J(6, n);
  for(Eigen::Index c = 0; c < n; ++c) { J.col(c) = Jfull.col(cols[static_cast<std::size_t>(c)]); }
  const auto toVec = [](const std::vector<double> & v)
  { return Eigen::Map<const Eigen::VectorXd>(v.data(), static_cast<Eigen::Index>(v.size())).eval(); };
  const auto box = call_handover::qpJointVelocityBox(toVec(q), toVec(qMin), toVec(qMax), toVec(vMin), toVec(vMax), v2QpLimits_);
  damper = box.damper;
  insideSecurity = box.insideSecurity;
  std::vector<call_handover::AuthorityEvaluation> out;
  for(const auto & d : demands)
  {
    out.push_back(call_handover::evaluateAuthorityDemand(J, box, d, v2CaParams_.angularCharacteristicLength,
                                                         v2CaParams_.residualTolerance, v2CaParams_.kappaMaximum));
  }
  return out;
}

std::vector<call_handover::AuthorityEvaluation> HandoverInterceptionController::controlAwareAuthorityAtRuntimeV2(
    const std::vector<call_handover::AuthorityDemand> & demands, bool & insideSecurity)
{
  const auto & mb = robot().mb();
  const auto & mbc = robot().mbc();
  if(!v2CaRuntimeJacobian_) { v2CaRuntimeJacobian_ = std::make_unique<rbd::Jacobian>(mb, toolFrame_); }
  const Eigen::MatrixXd Jc = v2CaRuntimeJacobian_->jacobian(mb, mbc);
  Eigen::MatrixXd Jfull = Eigen::MatrixXd::Zero(6, mb.nrDof());
  v2CaRuntimeJacobian_->fullJacobian(mb, Jc, Jfull);
  std::vector<int> cols;
  std::vector<double> q, qMin, qMax, vMin, vMax;
  for(int j = 0; j < mb.nrJoints(); ++j)
  {
    if(mb.joint(j).dof() != 1) { continue; }
    if(mb.joint(j).name().rfind("gen3_robotiq_85_", 0) == 0) { continue; }
    const std::size_t sj = static_cast<std::size_t>(j);
    cols.push_back(mb.jointPosInDof(j));
    q.push_back(mbc.q[sj][0]);
    qMin.push_back(robot().ql()[sj].empty() ? -std::numeric_limits<double>::infinity() : robot().ql()[sj][0]);
    qMax.push_back(robot().qu()[sj].empty() ? std::numeric_limits<double>::infinity() : robot().qu()[sj][0]);
    vMin.push_back(robot().vl()[sj].empty() ? -std::numeric_limits<double>::infinity() : robot().vl()[sj][0]);
    vMax.push_back(robot().vu()[sj].empty() ? std::numeric_limits<double>::infinity() : robot().vu()[sj][0]);
  }
  const Eigen::Index n = static_cast<Eigen::Index>(cols.size());
  Eigen::MatrixXd J(6, n);
  for(Eigen::Index c = 0; c < n; ++c) { J.col(c) = Jfull.col(cols[static_cast<std::size_t>(c)]); }
  const auto toVec = [](const std::vector<double> & v)
  { return Eigen::Map<const Eigen::VectorXd>(v.data(), static_cast<Eigen::Index>(v.size())).eval(); };
  const auto box = call_handover::qpJointVelocityBox(toVec(q), toVec(qMin), toVec(qMax), toVec(vMin), toVec(vMax), v2QpLimits_);
  insideSecurity = box.insideSecurity;
  // Gate use: realizability of kappaMin * demand (one bounded least squares per
  // demand, no bisection) keeps the control-thread cost bounded.
  std::vector<call_handover::AuthorityEvaluation> out;
  Eigen::Matrix<double, 6, 1> w;
  const double L = v2CaParams_.angularCharacteristicLength;
  w << L, L, L, 1.0, 1.0, 1.0;
  const Eigen::MatrixXd A = w.asDiagonal() * J;
  for(const auto & d : demands)
  {
    call_handover::AuthorityEvaluation e;
    e.label = d.label;
    if(box.consistent)
    {
      const Eigen::VectorXd y = w.asDiagonal() * (v2CaParams_.kappaMin * d.twist);
      e.residual = call_handover::boxConstrainedLeastSquares(A, y, box.lower, box.upper).residual;
      e.realizable = e.residual <= v2CaParams_.residualTolerance;
      e.reserve = e.realizable ? v2CaParams_.kappaMin : 0.0;
    }
    out.push_back(e);
  }
  return out;
}

void HandoverInterceptionController::evaluateControlAwareExactLayersV2(
    ControlAwareCandidateEvalV2 & eval, CaptureCandidate c, const sva::PTransformd & objectPose,
    const sva::PTransformd & startMouthPose, const Eigen::Vector3d & objectLinearVelocity,
    const Eigen::Vector3d & objectAngularVelocity)
{
  const double inf = std::numeric_limits<double>::infinity();
  eval.objectPose = objectPose;
  plannerContext_.W_T_O = objectPose;
  plannerContext_.W_T_H = compose(objectPose, O_T_H_);
  plannerContext_.planningM_T_O = sva::PTransformd::Identity();
  c.rotation = orientationError(startMouthPose, c.W_T_M_standoff);
  c.transitPathLength = (c.W_T_M_standoff.translation() - startMouthPose.translation()).norm();
  eval.record.reachDistance = c.transitPathLength + v2CaParams_.angularCharacteristicLength * c.rotation;
  auto reject = [&eval, &c](const std::string & layer, const std::string & why)
  {
    eval.record.rejectionLayer = layer;
    eval.record.rejectionReason = why;
    eval.candidate = c;
  };

  rbd::MultiBodyConfig mbc = planningSnapshot_.frozenRobotState;
  for(auto & a : mbc.alpha) { std::fill(a.begin(), a.end(), 0.0); }
  for(auto & aD : mbc.alphaD) { std::fill(aD.begin(), aD.end(), 0.0); }
  setPreviewGripperClosure(mbc, 0.0);

  PreviewResult reach;
  reach.minClearance = inf;
  if(!previewReachSegment(mbc, c.W_T_M_standoff, false, false, reach, true))
  {
    reject(classifyControlAwareRejectionV2(reach.reason), "standoff/" + reach.reason);
    return;
  }
  eval.reachStandoffDuration = reach.duration;
  const rbd::MultiBodyConfig standoffMbc = mbc;
  c.plannedStandoffArmPosture = armPostureFromMbc(mbc);
  c.plannedTransitArmPosture = c.plannedStandoffArmPosture;

  PreviewResult capture;
  capture.minClearance = inf;
  if(!previewReachSegment(mbc, c.W_T_M_pre, true, false, capture, true))
  {
    reject(classifyControlAwareRejectionV2(capture.reason), "capture/" + capture.reason);
    return;
  }
  eval.reachCaptureDuration = capture.duration;
  const rbd::MultiBodyConfig captureMbc = mbc;

  PreviewResult closure;
  closure.minClearance = inf;
  if(!previewClosureSweep(mbc, closure))
  {
    reject(classifyControlAwareRejectionV2(closure.reason), "closure/" + closure.reason);
    return;
  }
  eval.closureDuration = closure.duration;
  c.contactClosure = closure.contactClosure;
  c.plannedArmPosture = armPostureFromMbc(mbc);

  sva::PTransformd captureMouth;
  if(!previewMouthPose(mbc, captureMouth))
  {
    reject("ik", "retreat/preview_kinematics_unavailable");
    return;
  }
  plannerContext_.planningM_T_O = relativePose(captureMouth, plannerContext_.W_T_O);
  PreviewResult retreat;
  retreat.minClearance = inf;
  const bool retreatOk = previewReachSegment(mbc, c.W_T_M_retreat, false, true, retreat, true);
  plannerContext_.planningM_T_O = sva::PTransformd::Identity();
  if(!retreatOk)
  {
    reject(classifyControlAwareRejectionV2(retreat.reason), "retreat/" + retreat.reason);
    return;
  }
  eval.retreatDuration = retreat.duration;
  c.plannedRetreatArmPosture = armPostureFromMbc(mbc);
  eval.record.geometryFeasible = true;

  // Authority at the standoff (start of insertion) and capture (end of
  // insertion) configurations; the security-distance screen comes from the
  // same box construction.
  sva::PTransformd W_T_B_s;
  sva::PTransformd W_T_B_c;
  if(!previewBasePose(standoffMbc, W_T_B_s) || !previewBasePose(captureMbc, W_T_B_c))
  {
    reject("ik", "authority/preview_kinematics_unavailable");
    return;
  }
  bool securityS = false;
  bool securityC = false;
  // Interception mode: insertion is the only demand at the standoff/capture
  // configurations because acquisition starts with the object at rest (sec. 3.7,
  // interface limitation); following and synchronization are evaluated on the
  // rollout. Otherwise the TRIAD-lite follow(+insert) demands are unchanged.
  const bool phaseDemands = v2CaParams_.interceptionMode != "disabled";
  eval.standoffAuthority = controlAwareAuthorityAtPreviewV2(
      standoffMbc, phaseDemands ? insertionDemandV2(c.W_T_M_pre)
                                : controlAwareDemandsV2(objectLinearVelocity, objectAngularVelocity, objectPose.translation(), c.W_T_M_pre, W_T_B_s),
      eval.standoffDamper, securityS);
  eval.captureAuthority = controlAwareAuthorityAtPreviewV2(
      captureMbc, phaseDemands ? insertionDemandV2(c.W_T_M_pre)
                               : controlAwareDemandsV2(objectLinearVelocity, objectAngularVelocity, objectPose.translation(), c.W_T_M_pre, W_T_B_c),
      eval.captureDamper, securityC);
  double reserve = inf;
  double residual = 0.0;
  for(const auto * evals : {&eval.standoffAuthority, &eval.captureAuthority})
  {
    for(const auto & e : *evals)
    {
      reserve = std::min(reserve, e.reserve);
      residual = std::max(residual, e.residual);
    }
  }
  eval.record.reserve = std::isfinite(reserve) ? reserve : 0.0;
  eval.record.residual = residual;

  // Complete-action fields consumed by adoption, TERMINAL_CERTIFY and commit.
  c.previewFeasible = true;
  c.failureReason = "feasible_control_aware";
  c.predictiveReachClearance = reach.minClearance;
  c.predictiveRetreatClearance = retreat.minClearance;
  c.minClearance = std::min(reach.minClearance, std::min(capture.minClearance, retreat.minClearance));
  c.predictedEffort = reach.effort + capture.effort + retreat.effort;
  c.predictedReachTime = plannerConfig_.timingArmScale * eval.reachStandoffDuration;
  c.predictedPresentationTime = c.predictedReachTime;
  c.predictedApproachTime = plannerConfig_.timingTerminalCaptureDwell
      + plannerConfig_.timingArmScale * eval.reachCaptureDuration;
  c.predictedAcquireTime = plannerConfig_.timingPriorityBlend + plannerConfig_.timingCaptureLock
      + c.contactClosure / std::max(1e-6, plannerConfig_.timingEffectiveGripperRate);
  c.predictedContactTime = c.predictedPresentationTime + c.predictedApproachTime + c.predictedAcquireTime;
  c.estimatedTime = c.predictedContactTime + plannerConfig_.timingBilateralDwell
      + plannerConfig_.timingConfirmationDwell + plannerConfig_.timingArmScale * eval.retreatDuration;

  if(securityS || securityC)
  {
    reject("security_distance", securityS ? "standoff_configuration" : "capture_configuration");
    return;
  }
  eval.record.robotFeasible = true;
  eval.record.clearance = std::min(reach.minClearance, retreat.minClearance);
  eval.record.clearanceFeasible = eval.record.clearance >= v2CaParams_.clearanceFloor;
  if(!eval.record.clearanceFeasible)
  {
    reject("clearance_floor", fmt::format("clearance={:.4f}<floor={:.4f}", eval.record.clearance, v2CaParams_.clearanceFloor));
    return;
  }
  eval.record.authorityFeasible = eval.record.reserve >= v2CaParams_.kappaMin;
  if(!eval.record.authorityFeasible)
  {
    reject("authority", fmt::format("kappa={:.4f}<kappaMin={:.3f}", eval.record.reserve, v2CaParams_.kappaMin));
    return;
  }
  eval.record.admissible = true;
  eval.record.rejectionLayer = "none";
  eval.record.rejectionReason = "admissible";
  eval.candidate = c;
}

void HandoverInterceptionController::runControlAwareSelectionV2(ReceiverJobResultV2 & result)
{
  const ReceiverJobRequestV2 & request = v2Request_;
  const sva::PTransformd objectPose = request.terminalObjectPose;
  result.objectPose = objectPose;
  plannerContext_.W_T_O = objectPose;
  plannerContext_.W_T_H = compose(objectPose, O_T_H_);
  plannerContext_.planningM_T_O = sva::PTransformd::Identity();
  plannerContext_.plannerWorldActive = true;
  plannerContext_.planningStartMouthPose = request.snapshotMouthPose;
  plannerContext_.planningStartMouthPoseValid = true;

  const Eigen::Vector3d pH = plannerContext_.W_T_H.translation();
  const Eigen::Vector3d zH = plannerHandleAxis();
  // Same robot-relative outward reference as the V2 static screen.
  Eigen::Vector3d outward = request.snapshotMouthPose.translation() - pH;
  outward -= zH * zH.dot(outward);
  if(outward.norm() < 1e-6) { outward = plannerConfig_.worldUp - zH * zH.dot(plannerConfig_.worldUp); }
  if(outward.norm() < 1e-6) { outward = Eigen::Vector3d::UnitY() - zH * zH.y(); }
  if(outward.norm() < 1e-6) { outward = Eigen::Vector3d::UnitX() - zH * zH.x(); }

  const auto frontEndStart = std::chrono::steady_clock::now();
  const std::vector<ControlAwareHypothesisV2> hypotheses =
      controlAwareHypothesesV2(request, objectPose, zH, outward, result.controlAwareFunnel);
  result.controlAwareFunnel.frontEndWall =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - frontEndStart).count();
  const auto exactStart = std::chrono::steady_clock::now();
  const Eigen::Vector3d vObj = request.prediction.linearVelocity;
  const Eigen::Vector3d wObj = request.prediction.angularVelocity;
  const double inf = std::numeric_limits<double>::infinity();
  result.controlAwareCandidates.reserve(hypotheses.size());

  for(const auto & hyp : hypotheses)
  {
    if(!hyp.evaluate) { continue; }
    if(plannerCancel_.load(std::memory_order_relaxed))
    {
      result.success = false;
      result.reason = "v2/cancelled";
      return;
    }
    ++result.controlAwareFunnel.evaluated;
    const call_handover::GraspParameters & g = hyp.grasp;
    ControlAwareCandidateEvalV2 eval;
    eval.objectPose = objectPose;
    eval.record.grasp = g;
    eval.family = hyp.family;
    eval.theta = hyp.theta;
    eval.axialOffset = hyp.axialOffset;
    eval.reachabilityScore = hyp.reachabilityScore;
    eval.shortlisted = hyp.shortlisted;
    CaptureCandidate c = hyp.candidate;
    eval.record.name = c.name;
    if(std::isnan(result.controlAwareFrameConsistency) && std::abs(hyp.axialOffset) < 1e-12)
    {
      // Guard against divergence between the pure grasp-frame definitions and
      // the controller's buildCandidate (single geometric source of V2).
      const CaptureCandidate ref = buildCandidate(g.phi, static_cast<double>(g.sign), request.snapshotMouthPose, outward);
      result.controlAwareFrameConsistency = (worldRotation(ref.W_T_M_pre) - worldRotation(c.W_T_M_pre)).norm()
          + (ref.W_T_M_pre.translation() - c.W_T_M_pre.translation()).norm()
          + (call_handover::graspFrameRotation(zH, outward, g) - worldRotation(c.W_T_M_pre)).norm();
    }
    evaluateControlAwareExactLayersV2(eval, c, objectPose, request.snapshotMouthPose, vObj, wObj);
    result.controlAwareCandidates.push_back(eval);
  }
  result.controlAwareFunnel.exactWall = std::chrono::duration<double>(std::chrono::steady_clock::now() - exactStart).count();
  result.success = true;
  result.reason = "evaluated";

  if(v2CaParams_.interceptionMode != "disabled")
  {
    // Phase C (characterization; execution unchanged): earliest feasible
    // encounter per grasp on the predicted object trajectory.
    const auto frontStart = std::chrono::steady_clock::now();
    auto & st = result.interception;
    const double Lcalc = v2CaParams_.interceptionComputationLatency;
    call_handover::InterceptionScheduleSpec spec;
    spec.lowerBound = 0.0;
    spec.latencyAnchor = Lcalc + v2CaParams_.interceptionEntryLead;
    // Largest distance of any standoff/retreat mouth centre from the object
    // origin bounds the grasp-point speed |v + w x r|.
    const double rMax = O_T_H_.translation().norm() + plannerConfig_.handleHalfLength
        + std::max(plannerConfig_.candidateStandoffDistance, plannerConfig_.candidateRetreatDistance);
    st.pointSpeed = request.prediction.linearVelocity.norm() + request.prediction.angularVelocity.norm() * rMax;
    spec.linearSpeed = st.pointSpeed;
    spec.angularSpeed = request.prediction.angularVelocity.norm();
    spec.restLinearSpeed = 0.0;   // the prediction record already carries the at-rest model
    spec.restAngularSpeed = 0.0;
    spec.epsPosition = v2CaParams_.interceptionEpsPosition;
    spec.epsRotation = v2CaParams_.interceptionEpsRotation;
    spec.minimumStep = v2CaParams_.interceptionMinimumStep;
    // The rollout integrates at most previewMaxIterationsPerSegment steps.
    spec.horizon = std::min(v2CaParams_.interceptionHorizon,
                            Lcalc + (plannerConfig_.previewMaxIterationsPerSegment - 1) * plannerConfig_.previewDt);
    double step = 0.0;
    std::vector<std::pair<double, sva::PTransformd>> events;
    for(double tau : call_handover::interceptionEventSchedule(spec, &step))
    {
      events.emplace_back(tau, predictionPoseAtV2(request.prediction, request.snapshotTime + tau));
    }
    st.step = step;
    st.horizon = spec.horizon;
    st.anchor = spec.latencyAnchor;
    // Encounters closer than one resolution step are indistinguishable at the
    // certification tolerance: eps_p / |v_G| moving, eps_p / v_far at rest.
    st.tieBand = std::isfinite(step) ? step
        : v2CaParams_.interceptionEpsPosition / std::max(1e-6, plannerConfig_.predictiveReachPolicy.farLinearSpeed);
    ControlAwareFunnelStatsV2 interceptionFunnel;
    const std::vector<ControlAwareHypothesisV2> interceptionHypotheses =
        controlAwareHypothesesV2(request, objectPose, zH, outward, interceptionFunnel, &events);
    st.frontEndWall = std::chrono::duration<double>(std::chrono::steady_clock::now() - frontStart).count();
    st.funnel = interceptionFunnel;
    solveInterceptionV2(result, interceptionHypotheses, events, objectPose);
    plannerContext_.W_T_O = objectPose;
    plannerContext_.W_T_H = compose(objectPose, O_T_H_);
    plannerContext_.planningM_T_O = sva::PTransformd::Identity();
  }
}

std::vector<HandoverInterceptionController::ControlAwareHypothesisV2>
HandoverInterceptionController::controlAwareHypothesesV2(const ReceiverJobRequestV2 & request,
                                                        const sva::PTransformd & objectPose,
                                                        const Eigen::Vector3d & handleAxis,
                                                        const Eigen::Vector3d & outward,
                                                        ControlAwareFunnelStatsV2 & stats,
                                                        const std::vector<std::pair<double, sva::PTransformd>> * events) const
{
  std::vector<ControlAwareHypothesisV2> out;
  if(v2CaParams_.graspFamily != "receiving")
  {
    for(const auto & g : call_handover::generateGraspFamily(v2CaParams_.graspsPerSign))
    {
      ControlAwareHypothesisV2 h;
      h.grasp = g;
      h.candidate = buildCandidate(g.phi, static_cast<double>(g.sign), request.snapshotMouthPose, outward);
      out.push_back(h);
    }
    stats.generated = stats.mechanical = stats.reachable = stats.shortlisted = static_cast<int>(out.size());
    return out;
  }

  // Mechanically derived receiving family (ReceivingGraspFamily.h).
  call_handover::HandleGeometry handle;
  handle.center = compose(objectPose, O_T_H_).translation();
  handle.axis = handleAxis;
  handle.radius = plannerConfig_.handleRadius;
  handle.halfLength = plannerConfig_.handleHalfLength;
  call_handover::GripperInterface gi;
  gi.captureDepth = plannerConfig_.captureDepth;
  gi.standoffDistance = plannerConfig_.candidateStandoffDistance;
  gi.retreatDistance = plannerConfig_.candidateRetreatDistance;
  gi.fingerHalfWidth = v2CaParams_.fingerHalfWidth;
  gi.axialMargin = v2CaParams_.axialMargin;
  gi.corridorAxialTolerance = plannerConfig_.corridorAxialTolerance;
  const double axialLimit = call_handover::receivingAxialLimit(handle, gi);
  call_handover::ReceivingGraspFamilySpec spec = v2CaParams_.receivingSpec;
  spec.axialMax = v2CaParams_.receivingAxialMax < 0.0 ? axialLimit : std::min(v2CaParams_.receivingAxialMax, axialLimit);

  // Robot base (root body) pose of the frozen state: the reachability map is
  // expressed in the base frame.
  const sva::PTransformd & W_T_root = planningSnapshot_.frozenRobotState.bodyPosW[0];
  const Eigen::Matrix3d R_W_root = worldRotation(W_T_root);
  const Eigen::Vector3d p_W_root = W_T_root.translation();
  auto score = [&](const sva::PTransformd & W_T_M)
  {
    const sva::PTransformd W_T_B = basePoseFromMouthPoseWith(W_T_M, planningSnapshot_.mouthToBaseTransform);
    const Eigen::Vector3d pw = call_handover::gen3WristPoint(worldRotation(W_T_B), W_T_B.translation());
    return call_handover::gen3WristReachabilitySdf(R_W_root.transpose() * (pw - p_W_root));
  };

  std::vector<call_handover::FunnelCandidate> funnel;
  for(const auto & rg : call_handover::generateReceivingGraspFamily(spec))
  {
    ++stats.generated;
    const auto poses = call_handover::receivingGraspPoses(handle, gi, outward, rg);
    ControlAwareHypothesisV2 h;
    h.family = "receiving";
    h.grasp.id = rg.id;
    h.grasp.sign = rg.sign;
    h.grasp.phi = call_handover::legacyPhiFromTheta(rg.sign, rg.theta);
    h.theta = rg.theta;
    h.axialOffset = rg.s;
    CaptureCandidate & c = h.candidate;
    c.W_T_M_pre = fromWorldPose(poses.rotation, poses.capture);
    c.W_T_M_standoff = fromWorldPose(poses.rotation, poses.standoff);
    c.W_T_M_transit = c.W_T_M_standoff;
    c.W_T_M_retreat = fromWorldPose(poses.rotation, poses.retreat);
    c.verticalComponent = clamp01V2(poses.rotation.col(1).dot(plannerConfig_.worldUp));
    c.name = fmt::format("rg_{}_s{:+03d}_t{:03d}", rg.sign >= 0 ? "P" : "N",
                         static_cast<int>(std::lround(rg.s * 1000.0)),
                         static_cast<int>(std::lround(rg.theta * 180.0 / M_PI)));
    const auto screen = call_handover::receivingMechanicalScreen(rg, poses, axialLimit, plannerConfig_.groundZ);
    h.mechanicalPassed = screen.passed;
    h.mechanicalReason = screen.reason;
    h.evaluate = false;
    h.shortlisted = false;
    if(events != nullptr && screen.axialOk)
    {
      // Interception: the grasp is object-fixed; the ground condition, the
      // pursuit bound and the surrogate are necessary conditions per event.
      // The first event passing all three lower-bounds tau*_g.
      const sva::PTransformd O_T_S = relativePose(objectPose, c.W_T_M_standoff);
      const sva::PTransformd O_T_C = relativePose(objectPose, c.W_T_M_pre);
      const sva::PTransformd O_T_R = relativePose(objectPose, c.W_T_M_retreat);
      h.surrogateEvent = -1;
      h.tauSurrogate = std::numeric_limits<double>::infinity();
      h.reachabilityScore = -std::numeric_limits<double>::infinity();
      bool groundAtSomeEvent = false;
      for(std::size_t j = 0; j < events->size(); ++j)
      {
        const auto & ev = (*events)[j];
        const sva::PTransformd W_T_S = compose(ev.second, O_T_S);
        const sva::PTransformd W_T_C = compose(ev.second, O_T_C);
        if(std::min({W_T_S.translation().z(), W_T_C.translation().z(), compose(ev.second, O_T_R).translation().z()})
           < plannerConfig_.groundZ)
        {
          continue;
        }
        groundAtSomeEvent = true;
        // At rest (single event) the encounter time is free (rest collapse), so
        // the pursuit bound does not constrain the pose; timing decides tau.
        if(events->size() > 1 && !interceptionPursuitPossibleV2(request.snapshotMouthPose, W_T_S, ev.first)) { continue; }
        const double sc = std::min(score(W_T_S), score(W_T_C));
        h.reachabilityScore = std::max(h.reachabilityScore, sc);
        if(sc >= -v2CaParams_.funnelPruneTolerance)
        {
          h.surrogateEvent = static_cast<int>(j);
          h.tauSurrogate = ev.first;
          h.reachabilityScore = sc;
          break;
        }
      }
      h.mechanicalPassed = groundAtSomeEvent;
      h.mechanicalReason = groundAtSomeEvent ? "passed" : "mouth_below_ground_all_events";
    }
    if(h.mechanicalPassed)
    {
      ++stats.mechanical;
      if(events == nullptr) { h.reachabilityScore = std::min(score(c.W_T_M_standoff), score(c.W_T_M_pre)); }
      if(h.reachabilityScore >= -v2CaParams_.funnelPruneTolerance) { ++stats.reachable; }
      call_handover::FunnelCandidate f;
      f.index = static_cast<int>(out.size());
      f.id = rg.id;
      f.sign = rg.sign;
      f.axialIndex = rg.axialIndex;
      f.theta = rg.theta;
      f.score = h.reachabilityScore;
      f.eventIndex = std::max(0, h.surrogateEvent);
      funnel.push_back(f);
    }
    out.push_back(h);
  }
  const std::vector<int> selected = v2CaParams_.funnelEnabled
      ? call_handover::reachabilityShortlist(funnel, v2CaParams_.funnelShortlistSize,
                                             v2CaParams_.funnelPruneTolerance, v2CaParams_.funnelThetaBinWidth)
      : call_handover::reachabilityShortlist(funnel, 0, std::numeric_limits<double>::infinity(), 0.0);
  for(int idx : selected)
  {
    out[static_cast<std::size_t>(idx)].shortlisted = true;
    out[static_cast<std::size_t>(idx)].evaluate = true;
  }
  stats.shortlisted = static_cast<int>(selected.size());
  if(v2CaParams_.funnelCharacterizeAll)
  {
    // Characterization: evaluate every mechanically valid hypothesis exactly;
    // only shortlisted records take part in selection.
    for(auto & h : out) { h.evaluate = h.mechanicalPassed; }
  }
  return out;
}

void HandoverInterceptionController::handleControlAwareSelectionV2(const PendingJobV2 & pending, double now)
{
  const ReceiverJobResultV2 result = v2Result_;
  ++v2CaSelectionsProcessed_;
  if(plannerJobState() == PlannerJobState::Failed || !result.success)
  {
    mc_rtc::log::warning("[TriadLiteSelection] planningGeneration={} success=false reason={} t={:.6f}",
                         pending.planningGeneration, result.reason, now);
    return;
  }
  std::vector<call_handover::GraspCandidateRecord> records;
  records.reserve(result.controlAwareCandidates.size());
  std::map<std::string, int> layers;
  for(const auto & e : result.controlAwareCandidates)
  {
    if(e.shortlisted) { records.push_back(e.record); }
    ++layers[e.record.rejectionLayer];
    const sva::PTransformd O_T_G = relativePose(e.objectPose, e.candidate.W_T_M_pre);
    const Eigen::Quaterniond qG(worldRotation(O_T_G));
    mc_rtc::log::info(
        "[TriadLiteCandidate] planningGeneration={} id={} name={} family={} thetaDeg={:.3f} s={:.4f} reachability={:.4f} shortlisted={} sign={:+d} phiDeg={:.3f} O_T_G_p={} O_T_G_q=[{:.6f},{:.6f},{:.6f},{:.6f}] geometryFeasible={} robotFeasible={} clearance={:.5f} clearanceFeasible={} reserve={:.5f} residual={:.6f} authorityFeasible={} admissible={} rejectionLayer={} rejectionReason={} reachDistance={:.5f} standoffAuthority={} captureAuthority={} standoffDamper={} captureDamper={}",
        pending.planningGeneration, e.record.grasp.id, e.record.name, e.family, e.theta * 180.0 / M_PI, e.axialOffset,
        e.reachabilityScore, e.shortlisted, e.record.grasp.sign,
        e.record.grasp.phi * 180.0 / M_PI, caVec3(O_T_G.translation()), qG.w(), qG.x(), qG.y(), qG.z(),
        e.record.geometryFeasible, e.record.robotFeasible, e.record.clearance, e.record.clearanceFeasible,
        e.record.reserve, e.record.residual, e.record.authorityFeasible, e.record.admissible,
        e.record.rejectionLayer, e.record.rejectionReason, e.record.reachDistance,
        caAuthority(e.standoffAuthority), caAuthority(e.captureAuthority), caDamper(e.standoffDamper),
        caDamper(e.captureDamper));
  }
  const auto & fs = result.controlAwareFunnel;
  mc_rtc::log::info(
      "[TriadLiteFunnel] planningGeneration={} family={} G0={} Gmech={} GR={} GK={} evaluated={} frontEndMs={:.3f} exactMs={:.3f} characterizeAll={} t={:.6f}",
      pending.planningGeneration, v2CaParams_.graspFamily, fs.generated, fs.mechanical, fs.reachable, fs.shortlisted,
      fs.evaluated, 1e3 * fs.frontEndWall, 1e3 * fs.exactWall, v2CaParams_.funnelCharacterizeAll, now);
  logInterceptionResultV2(pending, result, now);
  if(v2CaParams_.interceptionMode == "characterize_hold")
  {
    // Solver characterization from the held arm: nothing is adopted, so every
    // job starts from rest (the rollout start state is exact) and jobs continue
    // through the whole giver motion until the presentation window expires.
    return;
  }
  const auto outcome = call_handover::selectGraspLexicographic(records, v2CaParams_.selection);
  const int previousId = v2CaSelector_.incumbentId;
  // trustIncumbentReevaluation (default true): a re-evaluation of the executing
  // incumbent from the moving arm state may remove it. Distrusting it was tested
  // in simulation and failed 3/4 runs on the runtime clearance reserve
  // (research/triad_lite/TRIAD_CONTROL_AWARE_IMPLEMENTATION.md).
  const bool trustIncumbent = v2Phase_ != ReceiverPhaseV2::ControlAwareTrack || v2CaParams_.trustIncumbentReevaluation;
  const auto decision = call_handover::updateGraspSelector(v2CaSelector_, records, outcome, now, v2CaParams_.switchDwell,
                                                           trustIncumbent);
  std::string layerSummary;
  for(const auto & kv : layers) { layerSummary += fmt::format("{}{}:{}", layerSummary.empty() ? "" : ",", kv.first, kv.second); }
  mc_rtc::log::success(
      "[TriadLiteSelection] planningGeneration={} evaluated={} layers=[{}] bestId={} bestClearance={:.5f} bestReserve={:.5f} decision={} selectedId={} previousId={} phase={} frameConsistency={:.3e} selectionRule=admissible>clearance>reserve>reach t={:.6f}",
      pending.planningGeneration, records.size(), layerSummary,
      outcome.bestIndex >= 0 ? records[static_cast<std::size_t>(outcome.bestIndex)].grasp.id : -1,
      outcome.bestClearance, outcome.bestReserve, decision.event, decision.selectedId, previousId,
      receiverPhaseNameV2(v2Phase_), result.controlAwareFrameConsistency, now);

  if(decision.event == "abort_to_hold")
  {
    if(provisionalReceiverPlan_.valid)
    {
      // invalidateProvisionalPlanV2 logs the abort and holds the current pose.
      v2CaSelector_.incumbentId = previousId;
      invalidateProvisionalPlanV2("control_aware/no_admissible_grasp", now);
    }
    return;
  }
  if(!decision.changed || decision.selectedId < 0) { return; }
  for(const auto & e : result.controlAwareCandidates)
  {
    if(e.record.grasp.id != decision.selectedId) { continue; }
    const std::string event = decision.event == "initial" ? "initial_select" : "grasp_switch/" + decision.event;
    if(!adoptControlAwareCandidateV2(e, now, event))
    {
      v2CaSelector_.incumbentId = previousId;
    }
    return;
  }
}

bool HandoverInterceptionController::adoptControlAwareCandidateV2(const ControlAwareCandidateEvalV2 & eval,
                                                                  double now, const std::string & event)
{
  const CaptureCandidate & c = eval.candidate;
  const ObjectPredictionRecordV2 prediction = currentObjectPredictionV2();
  InterceptionPlan plan = makeInterceptionPlan(
      c, eval.objectPose, now, timingArmScale_ * eval.reachStandoffDuration,
      timingTerminalCaptureDwell_ + timingArmScale_ * eval.reachCaptureDuration,
      timingPriorityBlend_ + timingCaptureLock_ + c.contactClosure / std::max(1e-6, timingEffectiveGripperRate_),
      timingArmScale_ * eval.retreatDuration);
  // Not an event hypothesis: presented "now" at the evaluated object pose, no
  // modelled stop; the tracking law follows the live estimate.
  plan.conditionalPresentationV2 = true;
  plan.decelerationDuration = 0.0;
  plan.decelerationStartTime = plan.presentationTime;
  plan.objectLinearVelocity = prediction.linearVelocity;
  plan.objectAngularVelocity = prediction.angularVelocity;
  plan.mouthAtReachStart = actualMouthPose();
  std::string why;
  if(!validateInterceptionPlan(plan, &why, false))
  {
    mc_rtc::log::error("[TriadLiteEvent] type=adoption_refused graspId={} reason=invalid_plan/{} t={:.6f}",
                       eval.record.grasp.id, why, now);
    return false;
  }
  const bool replacement = provisionalReceiverPlan_.valid;
  const std::uint64_t previousPlanId = provisionalReceiverPlan_.planId;
  ProvisionalReceiverPlanV2 next;
  next.valid = true;
  next.planId = ++v2PlanIdCounter_;
  next.sourcePlanningGeneration = plannerResultGeneration();
  next.adoptedStateGeneration = ++v2StateGeneration_;
  next.candidate = c;
  next.plan = plan;
  next.adoptedTime = now;
  next.certifications = 1;
  next.holdPosture = currentArmPosture();
  provisionalReceiverPlan_ = next;
  ++v2Counters_.adoptions;
  if(replacement)
  {
    ++v2Counters_.replacements;
    ++v2CaSwitches_;
  }
  v2Phase_ = ReceiverPhaseV2::ControlAwareTrack;
  v2PhaseEntryTime_ = now;
  v2ReferencePose_ = actualMouthPose();
  v2ClearanceScale_ = 1.0;
  v2LatestTerminalCertificateValid_ = false;
  v2GateStableSince_ = -1.0;
  const sva::PTransformd O_T_G = relativePose(eval.objectPose, c.W_T_M_pre);
  mc_rtc::log::success(
      "[TriadLiteEvent] type={} planId={} previousPlanId={} graspId={} name={} sign={:+d} phiDeg={:.3f} O_T_G_p={} clearance={:.5f} reserve={:.5f} residual={:.6f} stateGeneration={} t={:.6f}",
      event, next.planId, previousPlanId, eval.record.grasp.id, eval.record.name, eval.record.grasp.sign,
      eval.record.grasp.phi * 180.0 / M_PI, caVec3(O_T_G.translation()), eval.record.clearance, eval.record.reserve,
      eval.record.residual, v2StateGeneration_, now);
  return true;
}

int HandoverInterceptionController::stepControlAwareTrackV2(double now, bool workerIdle)
{
  const auto & policy = predictiveReachPolicy_;
  const auto & active = provisionalReceiverPlan_;
  activateToolTask();
  setToolTaskGains(policy.taskStiffness, policy.taskWeight);
  setGripperJointPriority(false);
  commandGripper(0.0);
  if(!active.candidate.plannedStandoffArmPosture.empty())
  {
    commandArmPosture(active.candidate.plannedStandoffArmPosture);
  }

  const sva::PTransformd current = actualMouthPose();
  HandoverSafetyReport currentReport;
  if(!evaluateCurrentPoseSafety(currentReport, false))
  {
    commandMouthTarget(current);
    mc_rtc::log::error("[V2Failure] reason=control_aware_track_unsafe clear={:.4f} limiting={}/{} planId={}",
                       currentReport.minClearance, currentReport.sample, currentReport.obstacle, active.planId);
    return -1;
  }
  v2Counters_.minimumRuntimeClearance = std::min(v2Counters_.minimumRuntimeClearance, currentReport.minClearance);
  if(currentReport.minClearance < policy.minimumRuntimeClearance)
  {
    commandMouthTarget(current);
    mc_rtc::log::error("[V2Failure] reason=control_aware_track_clearance_reserve clear={:.4f} minimum={:.4f} planId={}",
                       currentReport.minClearance, policy.minimumRuntimeClearance, active.planId);
    return -1;
  }

  // Tracking law: rate-limited reference toward the object-relative standoff
  // at the LIVE object estimate, same clearance governor, speed and lead limits
  // as the V2 provisional reach, then the geometric safety filter.
  const sva::PTransformd target = compose(W_T_O_, active.plan.O_T_M_standoff);
  double rawClearanceScale = 1.0;
  if(currentReport.minClearance < policy.clearanceSlowdownStart)
  {
    const double denominator = std::max(1e-6, policy.clearanceSlowdownStart - policy.clearanceHardMargin);
    const double u = std::min(1.0, std::max(0.0, (currentReport.minClearance - policy.clearanceHardMargin) / denominator));
    rawClearanceScale = policy.minimumVelocityScale + (1.0 - policy.minimumVelocityScale) * u * u * (3.0 - 2.0 * u);
  }
  const double scaleRate = rawClearanceScale < v2ClearanceScale_ ? policy.clearanceScaleDropRate : policy.clearanceScaleRiseRate;
  const double maximumScaleChange = scaleRate * controlDt_;
  v2ClearanceScale_ += std::max(-maximumScaleChange, std::min(maximumScaleChange, rawClearanceScale - v2ClearanceScale_));
  v2ClearanceScale_ = std::min(1.0, std::max(policy.minimumVelocityScale, v2ClearanceScale_));
  const double linearSpeedLimit = policy.nearLinearSpeed + v2ClearanceScale_ * (policy.farLinearSpeed - policy.nearLinearSpeed);
  const double angularSpeedLimit = policy.nearAngularSpeed + v2ClearanceScale_ * (policy.farAngularSpeed - policy.nearAngularSpeed);
  const double linearLeadLimit = policy.nearLinearTrackingLead
      + v2ClearanceScale_ * (policy.maxLinearTrackingLead - policy.nearLinearTrackingLead);
  const double angularLeadLimit = policy.nearAngularTrackingLead
      + v2ClearanceScale_ * (policy.maxAngularTrackingLead - policy.nearAngularTrackingLead);
  const sva::PTransformd rateLimited = advancePoseReference(v2ReferencePose_, target, linearSpeedLimit, angularSpeedLimit);
  const sva::PTransformd next = boundedPoseStep(current, rateLimited, linearLeadLimit, angularLeadLimit);
  sva::PTransformd safe;
  HandoverSafetyReport report;
  if(!filterSafeMouthCommand(current, next, safe, report, false))
  {
    commandMouthTarget(current);
    mc_rtc::log::error("[V2Failure] reason=control_aware_track_safety_filter clear={:.4f} limiting={}/{} planId={}",
                       report.minClearance, report.sample, report.obstacle, active.planId);
    return -1;
  }
  const sva::PTransformd previousCommand = v2ReferencePose_;
  v2ReferencePose_ = safe;
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Vector3d w = Eigen::Vector3d::Zero();
  worldPoseTwist(previousCommand, v2ReferencePose_, controlDt_, v, w);
  if(v.norm() > linearSpeedLimit && v.norm() > 1e-12) { v *= linearSpeedLimit / v.norm(); }
  if(w.norm() > angularSpeedLimit && w.norm() > 1e-12) { w *= angularSpeedLimit / w.norm(); }
  commandMouthTargetWithWorldVelocity(v2ReferencePose_, v, w);

  // Measured acquisition-entry gate (tau is this event, not a searched lead).
  const ObjectPredictionRecordV2 prediction = currentObjectPredictionV2();
  const bool objectQuasiStatic = objectLinearVelocityEstimate_.norm() <= presentationMaximumLinearSpeed_
      && objectAngularVelocityEstimate_.norm() <= presentationMaximumAngularSpeed_;
  if(!objectQuasiStatic) { v2ObjectQuasiStaticSince_ = -1.0; }
  else if(v2ObjectQuasiStaticSince_ < 0.0) { v2ObjectQuasiStaticSince_ = now; }
  const double dist = (target.translation() - current.translation()).norm();
  const double angle = orientationError(current, target);
  const bool gripperOpen = measuredGripperClosure() <= v2Params_.terminalMaximumOpenClosure;
  const bool fresh = objectPerceptionMeasurementValid_ && objectMotionEstimateValid_
      && objectPerceptionMeasurementAge_ <= perceptionLatencyBufferDuration_;
  const bool geometricGate = dist <= v2Params_.terminalPositionTolerance
      && angle <= v2Params_.terminalOrientationTolerance && gripperOpen && fresh && report.safe
      && (!v2CaParams_.requireObjectStopped || objectQuasiStatic);
  bool authorityGate = !v2CaParams_.admissionRequiresCurrentAuthority;
  double gateResidual = std::numeric_limits<double>::quiet_NaN();
  if(geometricGate && v2CaParams_.admissionRequiresCurrentAuthority)
  {
    bool insideSecurity = false;
    const sva::PTransformd captureNow = compose(W_T_O_, active.plan.O_T_M_capture);
    const auto evals = controlAwareAuthorityAtRuntimeV2(
        controlAwareDemandsV2(prediction.linearVelocity, prediction.angularVelocity, W_T_O_.translation(), captureNow,
                              actualBasePose()),
        insideSecurity);
    authorityGate = !insideSecurity;
    gateResidual = 0.0;
    for(const auto & e : evals)
    {
      authorityGate = authorityGate && e.realizable;
      gateResidual = std::max(gateResidual, e.residual);
    }
  }
  const bool gate = geometricGate && authorityGate;
  if(gate)
  {
    if(v2GateStableSince_ < 0.0) { v2GateStableSince_ = now; }
  }
  else
  {
    v2GateStableSince_ = -1.0;
  }
  const bool gateStable = gate && now - v2GateStableSince_ + 1e-12 >= v2Params_.terminalStableDwell;
  if(v2LastMotionLogTime_ < 0.0 || now >= v2LastMotionLogTime_ + 0.05 - 1e-9)
  {
    mc_rtc::log::info(
        "[TriadLiteTrack] planId={} graspId={} dist={:.5f} angle={:.5f} objectQuasiStatic={} gripperOpen={} fresh={} safe={} geometricGate={} authorityGate={} gateResidual={:.6f} gateStable={} clearanceScale={:.3f} t={:.6f}",
        active.planId, v2CaSelector_.incumbentId, dist, angle, objectQuasiStatic, gripperOpen, fresh, report.safe,
        geometricGate, authorityGate, gateResidual, gateStable, v2ClearanceScale_, now);
  }

  const auto & cert = v2LatestTerminalCertificate_;
  const bool certificateUsable = v2LatestTerminalCertificateValid_ && gateStable
      && cert.stateGeneration == v2StateGeneration_ && cert.planId == active.planId
      && cert.snapshotTime >= v2GateStableSince_ - 1e-12;
  if(certificateUsable)
  {
    mc_rtc::log::success(
        "[TriadLiteEvent] type=acquisition_admit planId={} graspId={} dist={:.5f} angle={:.5f} gateResidual={:.6f} certificateGeneration={} t={:.6f}",
        active.planId, v2CaSelector_.incumbentId, dist, angle, gateResidual, cert.planningGeneration, now);
    if(commitProvisionalReceiverPlanV2(cert, now))
    {
      v2CaSelector_.frozen = true;
      mc_rtc::log::success("[TriadLiteEvent] type=grasp_freeze planId={} graspId={} executionAuthority=MovePregrasp t={:.6f}",
                           active.planId, v2CaSelector_.incumbentId, now);
      return 1;
    }
    if(v2Phase_ == ReceiverPhaseV2::Failed) { return -1; }
    ++v2CaAdmitsDeferred_;
    mc_rtc::log::warning("[TriadLiteEvent] type=admit_deferred planId={} graspId={} reason=commit_current_state_check t={:.6f}",
                         active.planId, v2CaSelector_.incumbentId, now);
    v2LatestTerminalCertificateValid_ = false;
  }

  if(v2ObjectQuasiStaticSince_ >= 0.0 && now - v2ObjectQuasiStaticSince_ > presentationAcquisitionWindow_)
  {
    mc_rtc::log::error(
        "[V2Failure] reason=control_aware_no_admission_within_presentation_window window={:.3f}s quasiStaticSince={:.6f} planId={}",
        presentationAcquisitionWindow_, v2ObjectQuasiStaticSince_, active.planId);
    return -1;
  }

  if(workerIdle)
  {
    if(gate) { submitReceiverCertificationV2(ReceiverJobTypeV2::TerminalCertify, now); }
    else if(v2CaParams_.reselectWhileTracking) { submitReceiverCertificationV2(ReceiverJobTypeV2::ControlAwareSelect, now); }
  }
  return 0;
}

// =============================================================================
// TRIAD-lite Phase C: predictive interception solver (worker thread)
// research/triad_lite/TRIAD_PREDICTIVE_INTERCEPTION_AUDIT.md sec. 3.3-3.5
// =============================================================================

sva::PTransformd HandoverInterceptionController::controlAwareGraspPoseAtV2(
    const ObjectPredictionRecordV2 & prediction, double absoluteTime, const sva::PTransformd & O_T_M) const
{
  return compose(predictionPoseAtV2(prediction, absoluteTime), O_T_M);
}

bool HandoverInterceptionController::interceptionPursuitPossibleV2(const sva::PTransformd & startMouth,
                                                                   const sva::PTransformd & W_T_M, double tau) const
{
  // The executed reference is rate limited at the far speeds of the shared
  // reach policy and the measured pose may lead it by at most the tracking
  // lead, so no pose change larger than speed * motion time + lead is possible.
  const auto & policy = plannerConfig_.predictiveReachPolicy;
  const double motionTime = std::max(0.0, tau - v2CaParams_.interceptionComputationLatency);
  const double d = (W_T_M.translation() - startMouth.translation()).norm();
  const double a = orientationError(startMouth, W_T_M);
  return d <= policy.farLinearSpeed * motionTime + policy.maxLinearTrackingLead + 1e-9
      && a <= policy.farAngularSpeed * motionTime + policy.maxAngularTrackingLead + 1e-9;
}

void HandoverInterceptionController::rolloutInterceptionV2(
    const ObjectPredictionRecordV2 & prediction, double tStart, double tRendezvous,
    const sva::PTransformd & O_T_M_standoff, const std::map<std::string, std::vector<double>> & postureTarget,
    InterceptionRolloutV2 & out)
{
  const PredictiveReachPolicy & policy = plannerConfig_.predictiveReachPolicy;
  const double dt = plannerConfig_.previewDt;
  out = InterceptionRolloutV2{};
  const double duration = tRendezvous - tStart;
  if(!(duration >= 2.0 * dt))
  {
    out.reason = "duration_below_two_steps";
    return;
  }
  const sva::PTransformd savedObject = plannerContext_.W_T_O;
  const sva::PTransformd savedHandle = plannerContext_.W_T_H;
  const sva::PTransformd savedAttachment = plannerContext_.planningM_T_O;
  auto restore = [&]()
  {
    plannerContext_.W_T_O = savedObject;
    plannerContext_.W_T_H = savedHandle;
    plannerContext_.planningM_T_O = savedAttachment;
  };
  auto fail = [&](const std::string & why)
  {
    out.reason = why;
    restore();
  };
  plannerContext_.planningM_T_O = sva::PTransformd::Identity();

  // Frozen decision state (robot assumed held during L_calc; exact for a
  // first plan from rest, approximate while the arm moves).
  rbd::MultiBodyConfig mbc = planningSnapshot_.frozenRobotState;
  for(auto & a : mbc.alpha) { std::fill(a.begin(), a.end(), 0.0); }
  for(auto & aD : mbc.alphaD) { std::fill(aD.begin(), aD.end(), 0.0); }
  setPreviewGripperClosure(mbc, 0.0);
  sva::PTransformd startMouth;
  if(!previewMouthPose(mbc, startMouth)) { return fail("preview_kinematics_unavailable"); }

  // Velocity-matched Hermite rendezvous reference (PredictiveInterception.h).
  const Eigen::Vector3d omega = prediction.angularVelocity;
  auto graspState = [&](double t, Eigen::Vector3d & p, Eigen::Vector3d & v, Eigen::Matrix3d & R)
  {
    const sva::PTransformd W_T_O = predictionPoseAtV2(prediction, t);
    const sva::PTransformd G = compose(W_T_O, O_T_M_standoff);
    p = G.translation();
    R = worldRotation(G);
    v = prediction.linearVelocity + omega.cross(p - W_T_O.translation());
  };
  Eigen::Vector3d pG0, vG0;
  Eigen::Matrix3d RG0;
  graspState(tStart, pG0, vG0, RG0);
  const Eigen::Vector3d p0 = startMouth.translation();
  const Eigen::Matrix3d R0 = worldRotation(startMouth);
  const Eigen::Vector3d v0 = Eigen::Vector3d::Zero();
  auto referenceAt = [&](double t)
  {
    Eigen::Vector3d pG, vG;
    Eigen::Matrix3d RG;
    graspState(t, pG, vG, RG);
    return call_handover::rendezvousReference(t, tStart, duration, p0, v0, R0, pG0, vG0, RG0, pG, vG, RG, omega);
  };

  // Same per-step law as the V2 predictive reach rollout
  // (stepPredictiveRouteCandidate, RouteStepPhase::Reach): clearance governor,
  // rate cap, measured-pose lead tube, swept safety, whole-arm preview IK step.
  PreviewResult reach;
  reach.minClearance = std::numeric_limits<double>::infinity();
  int iteration = 0;
  sva::PTransformd commandReference = startMouth;
  double clearanceScale = 1.0;
  const int steps = std::max(1, static_cast<int>(std::ceil(duration / dt)));
  sva::PTransformd previousMouth = startMouth;
  sva::PTransformd currentMouth = startMouth;
  for(int k = 0; k < steps; ++k)
  {
    const double t = std::min(tRendezvous, tStart + k * dt);
    const double tNext = std::min(tRendezvous, t + dt);
    const auto ref = referenceAt(tNext);
    const sva::PTransformd refPose = fromWorldPose(ref.rotation, ref.position);
    routeStepSetVirtualObject(predictionPoseAtV2(prediction, tNext));
    if(!previewMouthPose(mbc, currentMouth)) { return fail("preview_kinematics_unavailable"); }
    HandoverSafetyReport currentReport;
    if(!previewConfigurationSafe(mbc, currentMouth, false, currentReport))
    {
      out.minClearance = std::min(reach.minClearance, currentReport.minClearance);
      return fail("reach_current/" + currentReport.sample + "/" + currentReport.obstacle);
    }
    reach.minClearance = std::min(reach.minClearance, currentReport.minClearance);
    if(currentReport.minClearance < policy.minimumRuntimeClearance)
    {
      out.minClearance = reach.minClearance;
      return fail("runtime_clearance_reserve");
    }
    double rawClearanceScale = 1.0;
    if(currentReport.minClearance < policy.clearanceSlowdownStart)
    {
      const double denominator = std::max(1e-6, policy.clearanceSlowdownStart - policy.clearanceHardMargin);
      const double u = std::min(1.0, std::max(0.0, (currentReport.minClearance - policy.clearanceHardMargin) / denominator));
      rawClearanceScale = policy.minimumVelocityScale + (1.0 - policy.minimumVelocityScale) * u * u * (3.0 - 2.0 * u);
    }
    const double scaleRate = rawClearanceScale < clearanceScale ? policy.clearanceScaleDropRate : policy.clearanceScaleRiseRate;
    const double maximumScaleChange = scaleRate * dt;
    clearanceScale += std::max(-maximumScaleChange, std::min(maximumScaleChange, rawClearanceScale - clearanceScale));
    clearanceScale = std::min(1.0, std::max(policy.minimumVelocityScale, clearanceScale));
    const double linearSpeedLimit = policy.nearLinearSpeed + clearanceScale * (policy.farLinearSpeed - policy.nearLinearSpeed);
    const double angularSpeedLimit = policy.nearAngularSpeed + clearanceScale * (policy.farAngularSpeed - policy.nearAngularSpeed);
    const double linearLeadLimit = policy.nearLinearTrackingLead
        + clearanceScale * (policy.maxLinearTrackingLead - policy.nearLinearTrackingLead);
    const double angularLeadLimit = policy.nearAngularTrackingLead
        + clearanceScale * (policy.maxAngularTrackingLead - policy.nearAngularTrackingLead);
    const sva::PTransformd rateLimited = boundedPoseStep(commandReference, refPose, linearSpeedLimit * dt, angularSpeedLimit * dt);
    const sva::PTransformd nextCommand = boundedPoseStep(currentMouth, rateLimited, linearLeadLimit, angularLeadLimit);
    HandoverSafetyReport sweptReport;
    if(!sweptGripperPoseSafeWith(currentMouth, nextCommand, planningSnapshot_.mouthToBaseTransform, plannerContext_.W_T_O,
                                 plannerContext_.W_T_H, sweptReport, false))
    {
      out.minClearance = std::min(reach.minClearance, sweptReport.minClearance);
      return fail("reach_command/" + sweptReport.sample + "/" + sweptReport.obstacle);
    }
    reach.minClearance = std::min(reach.minClearance, sweptReport.minClearance);
    Eigen::Vector3d ffLinear = Eigen::Vector3d::Zero();
    Eigen::Vector3d ffAngular = Eigen::Vector3d::Zero();
    sva::PTransformd baseNow;
    sva::PTransformd baseNext;
    if(!previewBasePoseFromMouthPose(commandReference, mbc, baseNow)
       || !previewBasePoseFromMouthPose(nextCommand, mbc, baseNext))
    {
      return fail("preview_kinematics_unavailable");
    }
    worldPoseTwist(baseNow, baseNext, dt, ffLinear, ffAngular);
    if(k % v2CaParams_.authorityStride == 0)
    {
      // Phase D demand: the tool-body twist this step commands (the same
      // quantity commandMouthTargetWithWorldVelocity sends to the QP task),
      // tested at the configuration that must realise it.
      call_handover::AuthorityDemand d;
      d.label = "path";
      d.twist.head<3>() = ffAngular;
      d.twist.tail<3>() = ffLinear;
      std::vector<int> damper;
      bool inside = false;
      const auto e = controlAwareAuthorityAtPreviewV2(mbc, {d}, damper, inside);
      out.insideSecurity = out.insideSecurity || inside;
      if(!(e.front().reserve >= out.pathReserve)) { out.pathLimitingTime = t - tStart; }
      out.pathReserve = std::isnan(out.pathReserve) ? e.front().reserve : std::min(out.pathReserve, e.front().reserve);
      out.pathResidual = std::max(out.pathResidual, e.front().residual);
      out.pathPeakLinearSpeed = std::max(out.pathPeakLinearSpeed, ffLinear.norm());
      ++out.pathSamples;
    }
    const PreviewStepStatus status = previewReachStep(mbc, nextCommand, false, false, iteration, reach, ffLinear, ffAngular,
                                                      false, postureTarget.empty() ? nullptr : &postureTarget, true);
    if(status == PreviewStepStatus::Failed)
    {
      out.minClearance = reach.minClearance;
      return fail("reach/" + reach.reason);
    }
    commandReference = nextCommand;
    previousMouth = currentMouth;
    ++out.steps;
  }
  out.duration = out.steps * dt;
  out.minClearance = reach.minClearance;
  routeStepSetVirtualObject(predictionPoseAtV2(prediction, tRendezvous));
  sva::PTransformd finalMouth;
  if(!previewMouthPose(mbc, finalMouth)) { return fail("preview_kinematics_unavailable"); }
  Eigen::Vector3d pG, vG;
  Eigen::Matrix3d RG;
  graspState(tRendezvous, pG, vG, RG);
  out.finalPositionError = (finalMouth.translation() - pG).norm();
  out.finalOrientationError = orientationError(finalMouth, fromWorldPose(RG, pG));
  Eigen::Vector3d vM = Eigen::Vector3d::Zero();
  Eigen::Vector3d wM = Eigen::Vector3d::Zero();
  worldPoseTwist(currentMouth, finalMouth, dt, vM, wM);
  out.finalRelativeLinearSpeed = (vM - vG).norm();
  out.finalRelativeAngularSpeed = (wM - omega).norm();
  out.rendezvousArmPosture = armPostureFromMbc(mbc);
  {
    // Generic capability at the rendezvous (B2 tie-break): condition index of
    // the length-scaled tool Jacobian, as previewReachStep's decision metric.
    const auto & mb = plannerModel();
    if(!plannerContext_.previewKinematicCacheValid || !plannerContext_.previewToolJacobian) { refreshPreviewKinematicCache(); }
    rbd::Jacobian & jac = *plannerContext_.previewToolJacobian;
    const Eigen::MatrixXd Jc = jac.jacobian(mb, mbc);
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, mb.nrDof());
    jac.fullJacobian(mb, Jc, J);
    J.topRows(3) *= plannerConfig_.decisionCharacteristicLength;
    const Eigen::Matrix<double, 6, 6> gram = J * J.transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig(gram);
    out.conditionIndexAtRendezvous = eig.info() == Eigen::Success
        ? std::sqrt(std::max(0.0, eig.eigenvalues().minCoeff()) / std::max(1e-12, eig.eigenvalues().maxCoeff()))
        : 0.0;
  }
  {
    // Synchronization demand at the rendezvous configuration: the rigid object
    // twist at the tool-body origin (TerminalTrack / control-aware tracking
    // feedforward), [w_O; v_O + w_O x (p_B - p_O)].
    sva::PTransformd W_T_B;
    if(previewBasePose(mbc, W_T_B))
    {
      const sva::PTransformd W_T_O_R = predictionPoseAtV2(prediction, tRendezvous);
      call_handover::AuthorityDemand d;
      d.label = "synchronize";
      d.twist = call_handover::synchronizationTwist(prediction.linearVelocity, omega, W_T_B.translation(),
                                                    W_T_O_R.translation());
      std::vector<int> damper;
      bool inside = false;
      const auto e = controlAwareAuthorityAtPreviewV2(mbc, {d}, damper, inside);
      out.insideSecurity = out.insideSecurity || inside;
      out.syncReserve = e.front().reserve;
      out.syncResidual = e.front().residual;
    }
  }
  restore();
  if(!std::isfinite(out.minClearance) || out.minClearance < plannerConfig_.transitMinimumPredictedClearance)
  {
    out.reason = "robust_transit_clearance_reserve";
    return;
  }
  if(out.finalPositionError > policy.positionTolerance || out.finalOrientationError > policy.orientationTolerance)
  {
    out.reason = "rendezvous_tracking";
    return;
  }
  if(out.finalRelativeLinearSpeed > v2CaParams_.interceptionTerminalLinearSpeed
     || out.finalRelativeAngularSpeed > v2CaParams_.interceptionTerminalAngularSpeed)
  {
    out.reason = "terminal_relative_motion";
    return;
  }
  out.feasible = true;
  out.reason = "feasible";
}

void HandoverInterceptionController::solveInterceptionV2(
    ReceiverJobResultV2 & result, const std::vector<ControlAwareHypothesisV2> & hypotheses,
    const std::vector<std::pair<double, sva::PTransformd>> & events, const sva::PTransformd & snapshotObjectPose)
{
  const ReceiverJobRequestV2 & request = v2Request_;
  const ObjectPredictionRecordV2 & prediction = request.prediction;
  auto & st = result.interception;
  const auto sweepStart = std::chrono::steady_clock::now();
  st.active = true;
  st.events = static_cast<int>(events.size());
  if(events.empty()) { return; }
  const double inf = std::numeric_limits<double>::infinity();
  const double Lcalc = v2CaParams_.interceptionComputationLatency;
  const double Lentry = v2CaParams_.interceptionEntryLead;
  const double armScale = plannerConfig_.timingArmScale;
  // Timing skip: under the straight-line motion-time model T(tau) =
  // armScale * |p_G(tau) - p_0| / v, T changes at most at rate
  // r = armScale * pointSpeed / v, so events closer than deficit / (1 + r)
  // to a timing-infeasible event are timing-infeasible as well.
  const double skipRate = armScale * st.pointSpeed / std::max(1e-6, plannerConfig_.predictiveReachPolicy.farLinearSpeed);

  struct Cursor
  {
    double tau;
    int id;
    int hyp;
    int event;
  };
  auto later = [](const Cursor & a, const Cursor & b) { return a.tau > b.tau || (a.tau == b.tau && a.id > b.id); };
  std::priority_queue<Cursor, std::vector<Cursor>, decltype(later)> queue(later);
  std::vector<int> candidateOf(hypotheses.size(), -1);
  for(std::size_t i = 0; i < hypotheses.size(); ++i)
  {
    const auto & h = hypotheses[i];
    if(!h.evaluate) { continue; }
    InterceptionCandidateV2 ic;
    ic.hypothesisIndex = static_cast<int>(i);
    ic.graspId = h.grasp.id;
    ic.tauSurrogate = h.tauSurrogate;
    ic.surrogateEvent = h.surrogateEvent;
    candidateOf[i] = static_cast<int>(result.interceptionCandidates.size());
    if(h.surrogateEvent < 0)
    {
      ic.status = "infeasible_within_horizon";
      result.interceptionCandidates.push_back(ic);
      continue;
    }
    ic.status = "infeasible_within_horizon";
    result.interceptionCandidates.push_back(ic);
    queue.push(Cursor{events[static_cast<std::size_t>(h.surrogateEvent)].first, h.grasp.id, static_cast<int>(i), h.surrogateEvent});
  }
  auto attempt = [&](const Cursor & cur, const char * stage, const std::string & why, double surrogate, double required,
                     double wall)
  {
    ++result.interceptionCandidates[static_cast<std::size_t>(candidateOf[static_cast<std::size_t>(cur.hyp)])].attempts;
    if(!v2CaParams_.interceptionLogAttempts) { return; }
    InterceptionAttemptV2 a;
    a.graspId = cur.id;
    a.tau = cur.tau;
    a.stage = stage;
    a.reason = why;
    a.surrogate = surrogate;
    a.requiredTime = required;
    a.wall = wall;
    result.interceptionAttempts.push_back(a);
  };
  auto pushNext = [&](const Cursor & cur, int nextEvent)
  {
    if(nextEvent >= static_cast<int>(events.size())) { return; }
    queue.push(Cursor{events[static_cast<std::size_t>(nextEvent)].first, cur.id, cur.hyp, nextEvent});
  };

  const sva::PTransformd & W_T_root = planningSnapshot_.frozenRobotState.bodyPosW[0];
  const Eigen::Matrix3d R_W_root = worldRotation(W_T_root);
  const Eigen::Vector3d p_W_root = W_T_root.translation();
  auto surrogateScore = [&](const sva::PTransformd & W_T_M)
  {
    const sva::PTransformd W_T_B = basePoseFromMouthPoseWith(W_T_M, planningSnapshot_.mouthToBaseTransform);
    const Eigen::Vector3d pw = call_handover::gen3WristPoint(worldRotation(W_T_B), W_T_B.translation());
    return call_handover::gen3WristReachabilitySdf(R_W_root.transpose() * (pw - p_W_root));
  };

  st.tauBest = inf;
  while(!queue.empty())
  {
    const Cursor cur = queue.top();
    if(cur.tau > st.tauBest + st.tieBand + 1e-9) { break; }
    queue.pop();
    if(plannerCancel_.load(std::memory_order_relaxed))
    {
      result.success = false;
      result.reason = "v2/cancelled";
      return;
    }
    const auto & h = hypotheses[static_cast<std::size_t>(cur.hyp)];
    auto & ic = result.interceptionCandidates[static_cast<std::size_t>(candidateOf[static_cast<std::size_t>(cur.hyp)])];
    const sva::PTransformd & W_T_O_tau = events[static_cast<std::size_t>(cur.event)].second;
    CaptureCandidate c = h.candidate;
    c.W_T_M_standoff = compose(W_T_O_tau, relativePose(snapshotObjectPose, h.candidate.W_T_M_standoff));
    c.W_T_M_pre = compose(W_T_O_tau, relativePose(snapshotObjectPose, h.candidate.W_T_M_pre));
    c.W_T_M_retreat = compose(W_T_O_tau, relativePose(snapshotObjectPose, h.candidate.W_T_M_retreat));
    c.W_T_M_transit = c.W_T_M_standoff;

    // Necessary conditions first (cheap).
    if(std::min({c.W_T_M_standoff.translation().z(), c.W_T_M_pre.translation().z(), c.W_T_M_retreat.translation().z()})
       < plannerConfig_.groundZ)
    {
      attempt(cur, "mechanical", "mouth_below_ground", std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(), 0.0);
      pushNext(cur, cur.event + 1);
      continue;
    }
    if(std::isfinite(st.step) && !interceptionPursuitPossibleV2(request.snapshotMouthPose, c.W_T_M_standoff, cur.tau))
    {
      attempt(cur, "pursuit", "pose_change_exceeds_rate_bound", std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(), 0.0);
      pushNext(cur, cur.event + 1);
      continue;
    }
    const double sc = std::min(surrogateScore(c.W_T_M_standoff), surrogateScore(c.W_T_M_pre));
    if(sc < -v2CaParams_.funnelPruneTolerance)
    {
      attempt(cur, "surrogate", "wrist_outside_reachable_set", sc, std::numeric_limits<double>::quiet_NaN(), 0.0);
      pushNext(cur, cur.event + 1);
      continue;
    }
    if(st.exactEvaluations >= v2CaParams_.interceptionMaximumExactEvaluations)
    {
      st.budgetExhausted = true;
      ic.status = "budget";
      attempt(cur, "budget", "exact_evaluations", sc, std::numeric_limits<double>::quiet_NaN(), 0.0);
      continue;
    }

    // Exact controller layers at T_G(g, tau) with the object held at the event pose.
    const auto exactStart = std::chrono::steady_clock::now();
    ControlAwareCandidateEvalV2 eval;
    eval.record.grasp = h.grasp;
    eval.record.name = h.candidate.name;
    eval.family = h.family;
    eval.theta = h.theta;
    eval.axialOffset = h.axialOffset;
    eval.reachabilityScore = sc;
    eval.shortlisted = h.shortlisted;
    evaluateControlAwareExactLayersV2(eval, c, W_T_O_tau, request.snapshotMouthPose, prediction.linearVelocity,
                                      prediction.angularVelocity);
    const double exactWall = std::chrono::duration<double>(std::chrono::steady_clock::now() - exactStart).count();
    st.exactWall += exactWall;
    ++st.exactEvaluations;
    ++ic.exactEvaluations;
    // Controller authority is not part of F_I (sec. 3.7): only robot
    // feasibility and the clearance floor decide interception here.
    if(!eval.record.robotFeasible || !eval.record.clearanceFeasible)
    {
      attempt(cur, "exact", eval.record.rejectionLayer + "/" + eval.record.rejectionReason, sc,
              std::numeric_limits<double>::quiet_NaN(), exactWall);
      pushNext(cur, cur.event + 1);
      continue;
    }
    const double required = armScale * eval.reachStandoffDuration + Lcalc + Lentry;
    Cursor at = cur;
    if(!std::isfinite(st.step) && at.tau < required)
    {
      // Object at rest: the meeting pose does not depend on tau, so the
      // earliest encounter of this grasp is its travel time (rest collapse).
      at.tau = required;
      if(at.tau > st.tauBest + st.tieBand + 1e-9)
      {
        attempt(at, "dominated", "rest_travel_time_after_best", sc, required, exactWall);
        ic.status = "dominated";
        ic.tauSurrogate = at.tau;
        continue;
      }
    }
    if(at.tau + 1e-9 < required)
    {
      attempt(cur, "timing", "travel_time_exceeds_arrival_time", sc, required, exactWall);
      int next = cur.event + 1;
      if(v2CaParams_.interceptionTimingSkip)
      {
        const double skipTo = cur.tau + (required - cur.tau) / (1.0 + skipRate);
        while(next < static_cast<int>(events.size()) && events[static_cast<std::size_t>(next)].first + 1e-12 < skipTo) { ++next; }
      }
      pushNext(cur, next);
      continue;
    }
    if(st.rollouts >= v2CaParams_.interceptionMaximumRollouts)
    {
      st.budgetExhausted = true;
      ic.status = "budget";
      attempt(at, "budget", "rollouts", sc, required, exactWall);
      continue;
    }
    const auto rolloutStart = std::chrono::steady_clock::now();
    InterceptionRolloutV2 rollout;
    rolloutInterceptionV2(prediction, request.snapshotTime + Lcalc, request.snapshotTime + at.tau,
                          relativePose(W_T_O_tau, c.W_T_M_standoff), eval.candidate.plannedStandoffArmPosture, rollout);
    const double rolloutWall = std::chrono::duration<double>(std::chrono::steady_clock::now() - rolloutStart).count();
    st.rolloutWall += rolloutWall;
    ++st.rollouts;
    ++ic.rollouts;
    // restore the exact-layer world of this event for any later reads
    plannerContext_.W_T_O = W_T_O_tau;
    plannerContext_.W_T_H = compose(W_T_O_tau, O_T_H_);
    if(!rollout.feasible)
    {
      attempt(at, "rollout", rollout.reason, sc, required, exactWall + rolloutWall);
      pushNext(cur, cur.event + 1);
      continue;
    }
    // Phase D: kappa(g, tau) = min over path, synchronization and insertion
    // (insertion = eval.record.reserve under the phase demands).
    double kappa = eval.record.reserve;
    std::string limiting = "insertion";
    if(rollout.pathSamples > 0 && rollout.pathReserve < kappa)
    {
      kappa = rollout.pathReserve;
      limiting = "path";
    }
    if(std::isfinite(rollout.syncReserve) && rollout.syncReserve < kappa)
    {
      kappa = rollout.syncReserve;
      limiting = "synchronization";
    }
    if(std::isnan(rollout.syncReserve) || rollout.insideSecurity)
    {
      kappa = 0.0;
      limiting = rollout.insideSecurity ? "security_distance" : "synchronization_unavailable";
    }
    if(!std::isfinite(ic.tauStarInterception)) { ic.tauStarInterception = at.tau; }
    if(v2CaParams_.authorityFilter && kappa < v2CaParams_.kappaMin)
    {
      ++ic.authorityRejectedEvents;
      attempt(at, "authority", fmt::format("{}/kappa={:.3f}<kappaMin={:.3f}", limiting, kappa, v2CaParams_.kappaMin), sc,
              required, exactWall + rolloutWall);
      pushNext(cur, cur.event + 1);
      continue;
    }
    ic.kappa = kappa;
    ic.kappaLimitingPhase = limiting;
    eval.record.reserve = kappa;
    eval.record.authorityFeasible = kappa >= v2CaParams_.kappaMin;
    attempt(at, "feasible", "feasible", sc, required, exactWall + rolloutWall);
    ic.status = "feasible";
    ic.tauStar = at.tau;
    ic.eval = eval;
    ic.rollout = rollout;
    ic.objectPoseAtRendezvous = W_T_O_tau;
    st.tauBest = std::min(st.tauBest, at.tau);
  }
  while(!queue.empty())
  {
    const Cursor cur = queue.top();
    queue.pop();
    auto & ic = result.interceptionCandidates[static_cast<std::size_t>(candidateOf[static_cast<std::size_t>(cur.hyp)])];
    if(ic.status != "feasible")
    {
      ic.status = "dominated";
      ic.tauSurrogate = cur.tau;  // proven lower bound on this grasp's encounter
    }
  }
  st.sweepWall = std::chrono::duration<double>(std::chrono::steady_clock::now() - sweepStart).count();
}

void HandoverInterceptionController::logInterceptionResultV2(const PendingJobV2 & pending,
                                                             const ReceiverJobResultV2 & result, double now) const
{
  const auto & st = result.interception;
  if(!st.active) { return; }
  for(const auto & a : result.interceptionAttempts)
  {
    mc_rtc::log::info(
        "[TriadLiteInterceptAttempt] planningGeneration={} graspId={} tau={:.4f} stage={} reason={} surrogate={:.4f} requiredTime={:.4f} wallMs={:.3f}",
        pending.planningGeneration, a.graspId, a.tau, a.stage, a.reason, a.surrogate, a.requiredTime, 1e3 * a.wall);
  }
  std::vector<call_handover::InterceptionRecord> records;
  for(const auto & ic : result.interceptionCandidates)
  {
    const auto & e = ic.eval;
    const sva::PTransformd T_G = e.candidate.W_T_M_standoff;
    const Eigen::Quaterniond qG(worldRotation(T_G));
    mc_rtc::log::info(
        "[TriadLiteInterception] planningGeneration={} graspId={} status={} tauStar={:.4f} tauSurrogate={:.4f} surrogateEvent={} attempts={} exact={} rollouts={} T_G_p={} T_G_q=[{:.6f},{:.6f},{:.6f},{:.6f}] robotFeasible={} clearance={:.5f} reserve={:.5f} residual={:.6f} rejectionLayer={} rolloutReason={} rolloutClearance={:.5f} finalPositionError={:.5f} finalOrientationError={:.5f} relativeLinearSpeed={:.5f} relativeAngularSpeed={:.5f} conditionIndex={:.5f} reachStandoffDuration={:.4f} kappa={:.4f} kappaLimiting={} pathReserve={:.4f} pathSamples={} pathLimitingTime={:.3f} pathPeakLinearSpeed={:.4f} syncReserve={:.4f} insertionReserve={:.4f} tauStarInterception={:.4f} authorityRejectedEvents={}",
        pending.planningGeneration, ic.graspId, ic.status, ic.tauStar, ic.tauSurrogate, ic.surrogateEvent, ic.attempts,
        ic.exactEvaluations, ic.rollouts, caVec3(T_G.translation()), qG.w(), qG.x(), qG.y(), qG.z(),
        e.record.robotFeasible, e.record.clearance, e.record.reserve, e.record.residual, e.record.rejectionLayer,
        ic.rollout.reason, ic.rollout.minClearance, ic.rollout.finalPositionError, ic.rollout.finalOrientationError,
        ic.rollout.finalRelativeLinearSpeed, ic.rollout.finalRelativeAngularSpeed, ic.rollout.conditionIndexAtRendezvous,
        e.reachStandoffDuration, ic.kappa, ic.kappaLimitingPhase, ic.rollout.pathReserve, ic.rollout.pathSamples,
        ic.rollout.pathLimitingTime, ic.rollout.pathPeakLinearSpeed, ic.rollout.syncReserve,
        std::min([&e]() { double r = std::numeric_limits<double>::infinity(); for(const auto & a : e.standoffAuthority) { r = std::min(r, a.reserve); } return r; }(),
                 [&e]() { double r = std::numeric_limits<double>::infinity(); for(const auto & a : e.captureAuthority) { r = std::min(r, a.reserve); } return r; }()),
        ic.tauStarInterception, ic.authorityRejectedEvents);
    call_handover::InterceptionRecord r;
    r.base = e.record;
    r.base.grasp.id = ic.graspId;
    r.base.admissible = ic.status == "feasible";
    r.tau = ic.tauStar;
    r.capability = ic.rollout.conditionIndexAtRendezvous;
    records.push_back(r);
  }
  call_handover::InterceptionSelectionTolerances tol;
  tol.tauTieBand = st.tieBand;
  tol.clearanceTieBand = v2CaParams_.selection.clearanceTieBand;
  tol.reserveTieBand = v2CaParams_.selection.reserveTieBand;
  tol.reserveSaturation = v2CaParams_.selection.reserveSaturation;
  const auto b1 = call_handover::selectEarliestInterception(records, tol, call_handover::InterceptionTieBreak::Clearance);
  const auto b2 = call_handover::selectEarliestInterception(records, tol, call_handover::InterceptionTieBreak::Capability);
  const auto full = call_handover::selectEarliestInterception(records, tol, call_handover::InterceptionTieBreak::AuthorityReserve);
  auto idOf = [&records](const call_handover::GraspSelectionOutcome & o)
  { return o.bestIndex >= 0 ? records[static_cast<std::size_t>(o.bestIndex)].base.grasp.id : -1; };
  mc_rtc::log::success(
      "[TriadLiteInterceptionJob] planningGeneration={} events={} step={:.4f} tieBand={:.4f} horizon={:.3f} anchor={:.3f} pointSpeed={:.4f} shortlisted={} feasible={} tauBest={:.4f} exactEvaluations={} rollouts={} budgetExhausted={} sweepMs={:.2f} exactMs={:.2f} rolloutMs={:.2f} frontEndMs={:.3f} selectedB1={} selectedB2={} selectedFull={} execution=unchanged t={:.6f}",
      pending.planningGeneration, st.events, st.step, st.tieBand, st.horizon, st.anchor, st.pointSpeed,
      result.interceptionCandidates.size(),
      std::count_if(result.interceptionCandidates.begin(), result.interceptionCandidates.end(),
                    [](const InterceptionCandidateV2 & c) { return c.status == "feasible"; }),
      st.tauBest, st.exactEvaluations, st.rollouts, st.budgetExhausted, 1e3 * st.sweepWall, 1e3 * st.exactWall,
      1e3 * st.rolloutWall, 1e3 * st.frontEndWall, idOf(b1), idOf(b2), idOf(full), now);
  mc_rtc::log::info("[TriadLiteInterceptionFunnel] planningGeneration={} G0={} Gmech={} GR={} GK={} t={:.6f}",
                    pending.planningGeneration, st.funnel.generated, st.funnel.mechanical, st.funnel.reachable,
                    st.funnel.shortlisted, now);
}
