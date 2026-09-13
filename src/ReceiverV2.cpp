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
  v2Active_ = false;
  if(!v2CommitLatched_) { v2EstimationActive_ = false; }
  mc_rtc::log::success(
      "[V2ReceiverSummary] phase={} commitCount={} adoptions={} replacements={} retained={} updated={} staleRejected={} fullSearches={} recertifications={} terminalCertifications={} generationsWhileRobotMoving={} generationsWhileObjectMoving={} searchFailures={} minRuntimeClearance={:.4f} reselectionLocked={}",
      receiverPhaseNameV2(v2Phase_), v2CommitCount_, v2Counters_.adoptions,
      v2Counters_.replacements, v2Counters_.retained, v2Counters_.updated,
      v2Counters_.staleRejected, v2Counters_.fullSearchSubmitted,
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
      if(workerIdle && settled) { submitReceiverFullSearchV2(now); }
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
  bank.maximumHypotheses = static_cast<int>(std::max<std::size_t>(v2Leads_.size(), 15));
  bank.maximumSearchWallTime = v2Params_.maximumEventSearchWallTime;
  bank.minimumSafeCommitLead = std::max(v2Params_.minimumCommitRemainingTime,
                                        presentationDecelerationDuration_ + 0.25);
  bank.source = "v2_receding_fixed_schedule";
  for(const double lead : v2Leads_)
  {
    bank.leads.push_back(lead);
    bank.presentationPoses.push_back(predictionPoseAtV2(prediction, now + lead));
  }
  submitFiniteTriadSearch(bank, now, v2Params_.planningStepsPerCycle,
                          v2Params_.routeWorkUnitsPerCycle);

  v2Pending_.active = true;
  v2Pending_.type = ReceiverJobTypeV2::FullSearch;
  v2Pending_.planningGeneration = plannerRequestGeneration();
  v2Pending_.stateGeneration = v2StateGeneration_;
  v2Pending_.planId = provisionalReceiverPlan_.planId;
  v2Pending_.submitTime = now;
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
    ReceiverJobTypeV2 type, double now)
{
  if(v2ReselectionLocked_)
  {
    mc_rtc::log::error("[V2PostCommitReselectionRefused] job={} t={:.6f}",
                       receiverJobTypeNameV2(type), now);
    return false;
  }
  if(v2Pending_.active || plannerJobState() == PlannerJobState::Running) { return false; }
  if(!provisionalReceiverPlan_.valid) { return false; }
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
  request.candidate = provisionalReceiverPlan_.candidate;
  request.plan = provisionalReceiverPlan_.plan;
  if(!previewMouthPose(planningSnapshot_.frozenRobotState, request.snapshotMouthPose))
  {
    request.snapshotMouthPose = actualMouthPose();
  }
  request.terminalObjectPose = predictionPoseAtV2(prediction, now);

  const std::uint64_t generation =
      plannerRequestGeneration_.fetch_add(1, std::memory_order_acq_rel) + 1;
  request.planningGeneration = generation;
  v2Request_ = request;
  plannerCancel_.store(false, std::memory_order_relaxed);
  plannerFailureReason_.clear();
  plannerWorkerStartWall_ = std::chrono::steady_clock::now();
  plannerJobState_.store(static_cast<int>(PlannerJobState::Running), std::memory_order_release);
  plannerThread_ = std::thread([this, generation]() { runReceiverWorkerJobV2(generation); });

  v2Pending_.active = true;
  v2Pending_.type = type;
  v2Pending_.planningGeneration = generation;
  v2Pending_.stateGeneration = v2StateGeneration_;
  v2Pending_.planId = provisionalReceiverPlan_.planId;
  v2Pending_.submitTime = now;
  if(type == ReceiverJobTypeV2::RecertifyActive) { ++v2Counters_.recertifySubmitted; }
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
    if(request.type == ReceiverJobTypeV2::RecertifyActive)
    {
      runRecertifyActiveRolloutV2(result);
    }
    else if(request.type == ReceiverJobTypeV2::TerminalCertify)
    {
      runTerminalCertificationV2(result);
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
    outcome = stepPredictiveRouteCandidate(4096);
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
  const int resumeIndex = std::max(0, std::min(steps, static_cast<int>(
      std::floor((tSnap - plan.reachStartTime) / plannerConfig_.previewDt))));
  plannerContext_.routeStepReachIndex = resumeIndex;
  plannerContext_.routeStepReachSteps = steps;
  // Resume the same reference from where the moving arm actually is.
  plannerContext_.routeStepCommandReference = tSnap < plan.reachStartTime
      ? plan.mouthAtReachStart : request.snapshotMouthPose;
  plannerContext_.routeStepClearanceScale = 1.0;
  plannerContext_.routeStepTransitPostureSaved = false;
  plannerContext_.routeStepPhase = RouteStepPhase::Reach;

  const RouteStepOutcome outcome = runRouteStepToCompletionV2();
  result.candidate = plannerContext_.routeStepCandidate;
  result.success = outcome == RouteStepOutcome::Feasible
      && result.candidate.completeCostAuditValid;
  result.reason = result.success ? "certified"
      : (outcome != RouteStepOutcome::Feasible ? result.candidate.failureReason
                                              : std::string("cost_invalid/") + result.candidate.terminalTimingAuditReason);
  mc_rtc::log::info(
      "[V2CertificationDetail] type=RECERTIFY_ACTIVE planId={} planningGeneration={} candidate={} route={} resumeIndex={}/{} success={} reason={} stageReached={}",
      request.planId, request.planningGeneration, candidate.name, candidate.transitRouteName,
      resumeIndex, steps, result.success, result.reason,
      outcome == RouteStepOutcome::Feasible ? "complete_through_carried_retreat" : "rejected");
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
  if(!previewReachSegment(mbc, candidate.W_T_M_standoff, false, false, reach, true))
  {
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

  const RouteStepOutcome outcome = runRouteStepToCompletionV2();
  result.candidate = plannerContext_.routeStepCandidate;
  result.success = outcome == RouteStepOutcome::Feasible
      && result.candidate.completeCostAuditValid;
  result.reason = result.success ? "certified"
      : (outcome != RouteStepOutcome::Feasible ? result.candidate.failureReason
                                              : std::string("cost_invalid/") + result.candidate.terminalTimingAuditReason);
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
    mc_rtc::log::warning(
        "[V2StaleResultRejected] type={} planningGeneration={} resultGeneration={} jobStateGeneration={} currentStateGeneration={} jobPlanId={} activePlanId={} generationMismatch={} stateMismatch={} planMismatch={} injected={} effect=none canCommit=false canReplacePlan=false t={:.6f}",
        receiverJobTypeNameV2(pending.type), pending.planningGeneration, resultGeneration,
        pending.stateGeneration, v2StateGeneration_, pending.planId,
        provisionalReceiverPlan_.planId, generationMismatch, stateMismatch, planMismatch,
        injected, now);
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

  if(pending.type == ReceiverJobTypeV2::RecertifyActive)
  {
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
  for(std::size_t i = 0; i < set.alternatives.size(); ++i)
  {
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
  const auto selection = call_handover::selectFiniteEventPlan(
      records, now, v2Params_.minimumReachEntryLead, minimumSafeCommitLead,
      decisionCostTieTolerance_);
  mc_rtc::log::success(
      "[V2FullSearchSelection] success={} reason={} completePlans={} costValidPlans={} timingAdmissiblePlans={} t={:.6f} selector=selectFiniteEventPlan(unchanged)",
      selection.success, selection.reason, selection.completePlanCount,
      selection.costValidCount, selection.timingAdmissibleCount, now);
  if(!selection.success || selection.selectedRecord >= set.alternatives.size())
  {
    return false;
  }
  const auto & alternative = set.alternatives[selection.selectedRecord];

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

  const std::uint64_t previousPlanId = provisionalReceiverPlan_.planId;
  const bool replacement = previousPlanId != 0;
  ProvisionalReceiverPlanV2 next;
  next.valid = true;
  next.planId = ++v2PlanIdCounter_;
  next.sourcePlanningGeneration = plannerResultGeneration();
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
      "[V2ProvisionalAdopt] planId={} previousPlanId={} kind={} reason={} sourcePlanningGeneration={} stateGeneration={} hypothesis={} eventLead={:.3f}s tau={:.6f} candidate={} route={} globalJ={:.9f} reachStart={:.6f} standoffTime={:.6f} predictedPresentation=[{:.4f},{:.4f},{:.4f}] closureAuthorized=false committed=false t={:.6f}",
      next.planId, previousPlanId, replacement ? "REPLACEMENT" : "INITIAL",
      replacement ? v2ReplacementReason_ : std::string("initial"),
      next.sourcePlanningGeneration, v2StateGeneration_, next.hypothesisIndex,
      next.eventLead, plan.presentationTime, c.name, c.transitRouteName,
      next.globalCost, plan.reachStartTime, plan.standoffTime, po.x(), po.y(), po.z(), now);
  return true;
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
