#include "HandoverInterceptionController_SolveInterception.h"
#include "../HandoverInterceptionController.h"

#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>

void HandoverInterceptionController_SolveInterception::configure(
    const mc_rtc::Configuration & config)
{
  if(config.has("planningStepsPerCycle"))
  {
    config("planningStepsPerCycle", planningStepsPerCycle_);
  }
  if(config.has("maximumFixedPointIterations"))
  {
    config("maximumFixedPointIterations", maximumFixedPointIterations_);
  }
  if(config.has("boundedEventSearchEnabled"))
  {
    config("boundedEventSearchEnabled", boundedEventSearchEnabled_);
  }
  if(config.has("maximumEventHypotheses"))
  {
    config("maximumEventHypotheses", maximumEventHypotheses_);
  }
  if(config.has("eventSearchLeadStep"))
  {
    config("eventSearchLeadStep", eventSearchLeadStep_);
  }
  if(config.has("maximumEventSearchWallTime"))
  {
    config("maximumEventSearchWallTime", maximumEventSearchWallTime_);
  }
  if(config.has("attemptedLeadTolerance"))
  {
    config("attemptedLeadTolerance", attemptedLeadTolerance_);
  }
  if(config.has("initialPresentationLead"))
  {
    config("initialPresentationLead", initialPresentationLead_);
  }
  else if(config.has("initialContactLead"))
  {
    config("initialContactLead", initialPresentationLead_);
  }
  if(config.has("minimumPresentationLead"))
  {
    config("minimumPresentationLead", minimumPresentationLead_);
  }
  else if(config.has("minimumContactLead"))
  {
    config("minimumContactLead", minimumPresentationLead_);
  }
  if(config.has("maximumPresentationLead"))
  {
    config("maximumPresentationLead", maximumPresentationLead_);
  }
  else if(config.has("maximumContactLead"))
  {
    config("maximumContactLead", maximumPresentationLead_);
  }
  if(config.has("timingTolerance"))
  {
    config("timingTolerance", timingTolerance_);
  }
  if(config.has("fixedPointRelaxation"))
  {
    config("fixedPointRelaxation", fixedPointRelaxation_);
  }
  if(config.has("maximumLeadStep"))
  {
    config("maximumLeadStep", maximumLeadStep_);
  }
  if(config.has("secantDenominatorTolerance"))
  {
    config("secantDenominatorTolerance", secantDenominatorTolerance_);
  }
  if(config.has("minimumCommitRemainingTime"))
  {
    config("minimumCommitRemainingTime", minimumCommitRemainingTime_);
  }
  if(config.has("minimumReachEntryLead"))
  {
    config("minimumReachEntryLead", minimumReachEntryLead_);
  }
  if(config.has("maximumRobotTranslation"))
  {
    config("maximumRobotTranslation", maximumRobotTranslation_);
  }
  if(config.has("maximumRobotRotation"))
  {
    config("maximumRobotRotation", maximumRobotRotation_);
  }
  if(config.has("holdTaskStiffness"))
  {
    config("holdTaskStiffness", holdTaskStiffness_);
  }
  if(config.has("holdTaskWeight"))
  {
    config("holdTaskWeight", holdTaskWeight_);
  }
  if(config.has("logEvery")) { config("logEvery", logEvery_); }

  planningStepsPerCycle_ = std::max(1, planningStepsPerCycle_);
  maximumFixedPointIterations_ = std::max(1, maximumFixedPointIterations_);
  maximumEventHypotheses_ = std::max(1, maximumEventHypotheses_);
  eventSearchLeadStep_ = std::max(0.05, eventSearchLeadStep_);
  maximumEventSearchWallTime_ = std::max(0.50, maximumEventSearchWallTime_);
  attemptedLeadTolerance_ = std::max(1e-4, attemptedLeadTolerance_);
  minimumPresentationLead_ = std::max(0.1, minimumPresentationLead_);
  maximumPresentationLead_ = std::max(
      minimumPresentationLead_, maximumPresentationLead_);
  initialPresentationLead_ = std::max(
      minimumPresentationLead_,
      std::min(maximumPresentationLead_, initialPresentationLead_));
  timingTolerance_ = std::max(0.01, timingTolerance_);
  fixedPointRelaxation_ = std::max(0.1, std::min(1.0, fixedPointRelaxation_));
  maximumLeadStep_ = std::max(0.05, maximumLeadStep_);
  secantDenominatorTolerance_ = std::max(
      1e-4, secantDenominatorTolerance_);
  minimumCommitRemainingTime_ = std::max(0.0, minimumCommitRemainingTime_);
  minimumReachEntryLead_ = std::max(0.0, minimumReachEntryLead_);
  maximumRobotTranslation_ = std::max(0.0, maximumRobotTranslation_);
  maximumRobotRotation_ = std::max(0.0, maximumRobotRotation_);
  holdTaskStiffness_ = std::max(0.0, holdTaskStiffness_);
  holdTaskWeight_ = std::max(0.0, holdTaskWeight_);
  logEvery_ = std::max<uint64_t>(1, logEvery_);
}

void HandoverInterceptionController_SolveInterception::buildBoundedEventLeadSchedule()
{
  boundedEventLeads_.clear();
  eventSearchCursor_ = 0;

  auto addLead = [this](double lead)
  {
    lead = std::max(minimumPresentationLead_,
                    std::min(maximumPresentationLead_, lead));
    for(const double existing : boundedEventLeads_)
    {
      if(std::abs(existing - lead) <= attemptedLeadTolerance_) { return; }
    }
    if(static_cast<int>(boundedEventLeads_.size()) < maximumEventHypotheses_)
    {
      boundedEventLeads_.push_back(lead);
    }
  };

  addLead(initialPresentationLead_);
  if(!boundedEventSearchEnabled_ || maximumEventHypotheses_ <= 1) { return; }

  // Deterministic center-out exploration. Earlier and later events are treated
  // symmetrically; no world direction or scenario label enters this schedule.
  const int endpointReserve = maximumEventHypotheses_ >= 5 ? 2 : 0;
  const int interiorLimit = maximumEventHypotheses_ - endpointReserve;
  for(int ring = 1;
      static_cast<int>(boundedEventLeads_.size()) < interiorLimit;
      ++ring)
  {
    const double lower = initialPresentationLead_
                       - static_cast<double>(ring) * eventSearchLeadStep_;
    const double upper = initialPresentationLead_
                       + static_cast<double>(ring) * eventSearchLeadStep_;
    addLead(lower);
    if(static_cast<int>(boundedEventLeads_.size()) >= interiorLimit) { break; }
    addLead(upper);

    if(lower <= minimumPresentationLead_
       && upper >= maximumPresentationLead_)
    {
      break;
    }
  }

  if(endpointReserve > 0)
  {
    addLead(minimumPresentationLead_);
    addLead(maximumPresentationLead_);
  }

  // Global optimization freezes one finite hypothesis set at a common epoch.
  // Chronological ordering makes the evaluated set and its logs easy to
  // reproduce; it does not alter the final exhaustive argmin.
  if(globalTimePlanMode_)
  {
    std::sort(boundedEventLeads_.begin(), boundedEventLeads_.end());
  }
}

bool HandoverInterceptionController_SolveInterception::leadAlreadyAttempted(
    double lead) const
{
  for(const double attempted : attemptedEventLeads_)
  {
    if(std::abs(attempted - lead) <= attemptedLeadTolerance_) { return true; }
  }
  return false;
}

bool HandoverInterceptionController_SolveInterception::nextBoundedEventLead(
    double & lead)
{
  while(eventSearchCursor_ < boundedEventLeads_.size())
  {
    const double candidate = boundedEventLeads_[eventSearchCursor_++];
    if(leadAlreadyAttempted(candidate)) { continue; }
    lead = candidate;
    return true;
  }
  return false;
}

bool HandoverInterceptionController_SolveInterception::eventSearchBudgetAvailable(
    double now) const
{
  const bool hypothesisBudget = eventHypothesisCount_ < maximumEventHypotheses_;
  const bool timeBudget = now - eventSearchStartTime_
                        < maximumEventSearchWallTime_;
  return hypothesisBudget && timeBudget;
}

bool HandoverInterceptionController_SolveInterception::scheduleNextHypothesis(
    double lead, const std::string & source)
{
  lead = std::max(minimumPresentationLead_,
                  std::min(maximumPresentationLead_, lead));
  if(leadAlreadyAttempted(lead)) { return false; }
  guessLead_ = lead;
  currentHypothesisSource_ = source;
  phase_ = Phase::StartIteration;
  return true;
}

void HandoverInterceptionController_SolveInterception::start(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  iter_ = 0;
  ready_ = false;
  staticMode_ = ctl.staticObjectModeSelected();
  globalTimePlanMode_ = !staticMode_
      && ctl.globalTimePlanSelectionEnabled();
  phase_ = Phase::StartIteration;
  fixedPointIteration_ = 0;
  eventHypothesisCount_ = 0;
  feasibleHypothesisCount_ = 0;
  geometryFailureCount_ = 0;
  eventSearchCursor_ = 0;
  guessLead_ = initialPresentationLead_;
  havePreviousResidual_ = false;
  previousGuessLead_ = guessLead_;
  previousResidual_ = 0.0;
  maxRobotTranslationObserved_ = 0.0;
  maxRobotRotationObserved_ = 0.0;
  attemptedEventLeads_.clear();
  boundedEventPresentationPoses_.clear();
  currentHypothesisSource_ = "initial";

  ctl.setGripperClosureAuthorized(false);
  holdBasePose_ = ctl.actualBasePose();
  holdArmPosture_ = ctl.currentArmPosture();
  ctl.activateToolTask();
  ctl.setToolTaskGains(holdTaskStiffness_, holdTaskWeight_);
  ctl.commandBaseTarget(holdBasePose_);
  ctl.commandReadyArmPosture(holdArmPosture_);
  ctl.setGripperJointPriority(false);
  ctl.commandGripper(0.0);
  ctl.clearPlanningObjectSnapshot();
  ctl.startPhaseTiming("interception_planning");
  eventSearchStartTime_ = ctl.controllerTime();
  ctl.resetGlobalTimePlanSearch(eventSearchStartTime_, globalTimePlanMode_);

  if(!ctl.objectMotionEstimateValid())
  {
    mc_rtc::log::error(
        "[InterceptionSolve] object observation estimate is invalid; refusing complete action planning");
    return;
  }
  if(ctl.observedObjectMode()
     == HandoverInterceptionController::ObservedObjectMode::Unclassified)
  {
    mc_rtc::log::error(
        "[InterceptionSolve] observation mode is unclassified; no plan can be committed");
    return;
  }
  if(!ctl.prepareCaptureSelection())
  {
    mc_rtc::log::error(
        "[InterceptionSolve] mouth/gripper geometry preparation failed");
    return;
  }

  ready_ = true;
  const Eigen::Vector3d p = ctl.objectPose().translation();
  const Eigen::Vector3d v = ctl.objectLinearVelocityEstimate();
  const Eigen::Vector3d w = ctl.objectAngularVelocityEstimate();
  if(staticMode_)
  {
    planningObjectPresentationPose_ = ctl.objectPose();
    mc_rtc::log::warning(
        "[StaticPresentationSolve] robot stationary, object stationary. Evaluating complete current-pose grasp-route-acquire-transfer-retreat actions at p=[{:.3f},{:.3f},{:.3f}] v=[{:.4f},{:.4f},{:.4f}] w=[{:.4f},{:.4f},{:.4f}] oneEvent=true noRuntimeRetry=true",
        p.x(), p.y(), p.z(), v.x(), v.y(), v.z(),
        w.x(), w.y(), w.z());
    mc_rtc::log::success(
        "[UnifiedPresentationArchitecture] mode=STATIC completePlanPerEvent=true currentPoseEvent=true robotStationary=true sameCandidateBank=true sameExecutionChain=true oneCommit=true");
  }
  else
  {
    buildBoundedEventLeadSchedule();
    if(globalTimePlanMode_ && !boundedEventLeads_.empty())
    {
      for(const double lead : boundedEventLeads_)
      {
        boundedEventPresentationPoses_.emplace(
            lead, ctl.predictPresentationPose(lead));
      }
      guessLead_ = boundedEventLeads_.front();
      eventSearchCursor_ = 1;
      currentHypothesisSource_ = "global_fixed_schedule";
    }
    if(globalTimePlanMode_)
    {
      mc_rtc::log::warning(
          "[PresentationSolve] robot stationary, object approaching. Exhaustively evaluating one fixed bounded set of presentation-time, grasp and route alternatives from p=[{:.3f},{:.3f},{:.3f}] v=[{:.4f},{:.4f},{:.4f}] w=[{:.4f},{:.4f},{:.4f}] deceleration={:.3f}s firstChronologicalLead={:.3f}s minimumReachEntryLead={:.3f}s noEarlyCommit=true",
          p.x(), p.y(), p.z(), v.x(), v.y(), v.z(),
          w.x(), w.y(), w.z(), ctl.presentationDecelerationDuration(),
          guessLead_, minimumReachEntryLead_);
    }
    else
    {
      mc_rtc::log::warning(
          "[PresentationSolve] robot stationary, object approaching. Solving one candidate-specific presentation-time fixed point from p=[{:.3f},{:.3f},{:.3f}] v=[{:.4f},{:.4f},{:.4f}] w=[{:.4f},{:.4f},{:.4f}] deceleration={:.3f}s initialLead={:.3f}s tolerance={:.3f}s minimumReachEntryLead={:.3f}s",
          p.x(), p.y(), p.z(), v.x(), v.y(), v.z(),
          w.x(), w.y(), w.z(), ctl.presentationDecelerationDuration(),
          guessLead_, timingTolerance_, minimumReachEntryLead_);
    }
    mc_rtc::log::success(
        "[BoundedCompleteEventSearch] enabled={} hypotheses={} leadRange=[{:.3f},{:.3f}] step={:.3f}s wallTime={:.3f}s timingRefinements={} robotStationary=true completePlanPerEvent=true noRuntimeRetry=true",
        boundedEventSearchEnabled_, maximumEventHypotheses_,
        minimumPresentationLead_, maximumPresentationLead_,
        eventSearchLeadStep_, maximumEventSearchWallTime_,
        globalTimePlanMode_ ? 0 : maximumFixedPointIterations_);
    mc_rtc::log::success(
        "[UnifiedPresentationArchitecture] mode=MOVING completePlanPerEvent=true boundedFutureEventSearch=true robotStationary=true sameCandidateBank=true sameExecutionChain=true oneCommit=true");
    if(globalTimePlanMode_)
    {
      mc_rtc::log::success(
          "[GlobalTimePlanSearchConfiguration] enabled=true fixedSearchEpoch={:.3f}s fixedAbsoluteEvents=true predictionModelFrozen=true configuredHypotheses={} leadRange=[{:.3f},{:.3f}] objective=min_time_grasp_route timeTerm=search_to_completion scheduleMustComplete=true finalTimingReadmission=true robotStationary=true",
          eventSearchStartTime_, boundedEventLeads_.size(),
          minimumPresentationLead_, maximumPresentationLead_);
    }
  }
}

bool HandoverInterceptionController_SolveInterception::run(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ++iter_;
  ctl.setGripperClosureAuthorized(false);

  ctl.activateToolTask();
  ctl.setToolTaskGains(holdTaskStiffness_, holdTaskWeight_);
  ctl.commandBaseTarget(holdBasePose_);
  ctl.commandReadyArmPosture(holdArmPosture_);
  ctl.setGripperJointPriority(false);
  ctl.commandGripper(0.0);

  const sva::PTransformd currentBase = ctl.actualBasePose();
  const double robotTranslation =
      (currentBase.translation() - holdBasePose_.translation()).norm();
  const double robotRotation = ctl.orientationError(currentBase, holdBasePose_);
  maxRobotTranslationObserved_ = std::max(
      maxRobotTranslationObserved_, robotTranslation);
  maxRobotRotationObserved_ = std::max(
      maxRobotRotationObserved_, robotRotation);

  if(robotTranslation > maximumRobotTranslation_
     || robotRotation > maximumRobotRotation_)
  {
    ctl.clearPlanningObjectSnapshot();
    ctl.endObjectObservation(true);
    ctl.finishPhaseTiming("interception_planning");
    mc_rtc::log::error(
        "[InterceptionSolve] physical robot left stationary tube translation={:.4f} rotation={:.4f}; no plan committed",
        robotTranslation, robotRotation);
    output("FAIL");
    return true;
  }

  if(!ready_)
  {
    ctl.endObjectObservation(true);
    ctl.finishPhaseTiming("interception_planning");
    output("FAIL");
    return true;
  }

  if(staticMode_)
  {
    const auto & policy = ctl.predictiveReachPolicy();
    const sva::PTransformd liveObject = ctl.objectPose();
    const double objectTranslation = (liveObject.translation()
        - planningObjectPresentationPose_.translation()).norm();
    const double objectRotation = ctl.orientationError(
        liveObject, planningObjectPresentationPose_);
    if(objectTranslation > policy.maximumObjectTranslationDeviation
       || objectRotation > policy.maximumObjectRotationDeviation)
    {
      ctl.clearPlanningObjectSnapshot();
      ctl.endObjectObservation(true);
      ctl.finishPhaseTiming("interception_planning");
      mc_rtc::log::error(
          "[StaticPresentationSolve] object moved outside the committed stationary planning tube before selection translation={:.4f}/{:.4f} rotation={:.4f}/{:.4f}; no plan committed",
          objectTranslation, policy.maximumObjectTranslationDeviation,
          objectRotation, policy.maximumObjectRotationDeviation);
      output("FAIL");
      return true;
    }

    if(phase_ == Phase::StartIteration)
    {
      if(ctl.controllerTime() - eventSearchStartTime_
         >= maximumEventSearchWallTime_)
      {
        ctl.endObjectObservation(true);
        ctl.finishPhaseTiming("interception_planning");
        mc_rtc::log::error(
            "[StaticPresentationSearchSummary] committed=false reason=planning_budget_exhausted elapsed={:.3f}s robotMoved=[{:.4f},{:.4f}]",
            ctl.controllerTime() - eventSearchStartTime_,
            maxRobotTranslationObserved_, maxRobotRotationObserved_);
        output("FAIL");
        return true;
      }

      iterationStartTime_ = ctl.controllerTime();
      eventHypothesisCount_ = 1;
      currentHypothesisSource_ = "measured_current_pose";
      ctl.setPlanningObjectSnapshot(planningObjectPresentationPose_);
      ctl.applyPlanningObjectSnapshot();
      const auto status = ctl.beginCapturePlanning(false);
      ctl.clearPlanningObjectSnapshot();

      const Eigen::Vector3d po = planningObjectPresentationPose_.translation();
      mc_rtc::log::warning(
          "[StaticPresentationHypothesis] index=1/1 source=measured_current_pose object=[{:.3f},{:.3f},{:.3f}] robotStationary=true completePlan=true",
          po.x(), po.y(), po.z());
      if(status == HandoverInterceptionController::CapturePlanningStatus::Failure)
      {
        ctl.endObjectObservation(true);
        ctl.finishPhaseTiming("interception_planning");
        mc_rtc::log::error(
            "[StaticPresentationSolve] capture planner could not start; no candidate committed");
        output("FAIL");
        return true;
      }
      phase_ = Phase::Planning;
      return false;
    }

    ctl.setPlanningObjectSnapshot(planningObjectPresentationPose_);
    ctl.applyPlanningObjectSnapshot();
    const auto status = ctl.stepCapturePlanning(planningStepsPerCycle_);
    ctl.clearPlanningObjectSnapshot();

    if(status == HandoverInterceptionController::CapturePlanningStatus::Running)
    {
      if(iter_ % logEvery_ == 1)
      {
        mc_rtc::log::info(
            "[StaticPresentationSolve] planning current-pose event elapsed={:.3f}s objectDrift=[{:.4f},{:.4f}] robotMoved={:.4f}",
            ctl.controllerTime() - iterationStartTime_,
            objectTranslation, objectRotation, robotTranslation);
      }
      return false;
    }

    if(status == HandoverInterceptionController::CapturePlanningStatus::Failure
       || !ctl.planningBestCandidateAvailable())
    {
      ++geometryFailureCount_;
      ctl.endObjectObservation(true);
      ctl.finishPhaseTiming("interception_planning");
      mc_rtc::log::error(
          "[StaticPresentationSearchSummary] committed=false reason=no_complete_current_pose_action hypotheses=1 feasible=0 geometryFailures=1 elapsed={:.3f}s robotMoved=[{:.4f},{:.4f}] safeRejection=true",
          ctl.controllerTime() - eventSearchStartTime_,
          maxRobotTranslationObserved_, maxRobotRotationObserved_);
      output("FAIL");
      return true;
    }

    if(!ctl.selectPlanningBestForCommit())
    {
      ctl.endObjectObservation(true);
      ctl.finishPhaseTiming("interception_planning");
      mc_rtc::log::error(
          "[StaticPresentationSearchSummary] committed=false reason=plan_selection_failed detail={} no fallback permitted",
          ctl.planningSelectionReason());
      output("FAIL");
      return true;
    }

    ++feasibleHypothesisCount_;
    const double now = ctl.controllerTime();
    const double predictedReachDuration =
        ctl.planningBestPredictedPresentationTime();
    const double committedPresentationTime = now
        + predictedReachDuration + minimumReachEntryLead_;
    const double residual = -minimumReachEntryLead_;
    if(!ctl.commitPlanningBestAsInterception(
           committedPresentationTime, residual,
           planningObjectPresentationPose_))
    {
      ctl.endObjectObservation(true);
      ctl.finishPhaseTiming("interception_planning");
      output("FAIL");
      return true;
    }

    ctl.endObjectObservation(true);
    ctl.finishPhaseTiming("interception_planning");
    const Eigen::Vector3d po = planningObjectPresentationPose_.translation();
    mc_rtc::log::success(
        "[StaticPresentationCommit] COMMITTED candidate={} route={} object=[{:.3f},{:.3f},{:.3f}] predictedReach={:.3f}s launchReserve={:.3f}s presentationTime={:.3f}s oneCommit=true noRetiming=true noReplanning=true",
        ctl.planningBestCandidateName(),
        ctl.committedInterceptionPlan().transitRouteName,
        po.x(), po.y(), po.z(), predictedReachDuration,
        minimumReachEntryLead_, committedPresentationTime);
    mc_rtc::log::success(
        "[StaticPresentationSearchSummary] committed=true hypotheses=1 feasible=1 geometryFailures=0 elapsed={:.3f}s robotMoved=[{:.4f},{:.4f}] currentPoseEvent=true oneCommit=true",
        ctl.controllerTime() - eventSearchStartTime_,
        maxRobotTranslationObserved_, maxRobotRotationObserved_);
    output("OK");
    return true;
  }

  if(globalTimePlanMode_ && phase_ == Phase::SelectGlobal)
  {
    const double now = ctl.controllerTime();
    const double minimumSafeCommitLead = std::max(
        minimumCommitRemainingTime_,
        ctl.presentationDecelerationDuration() + 0.25);
    const bool scheduleComplete =
        attemptedEventLeads_.size() == boundedEventLeads_.size()
        && eventSearchCursor_ >= boundedEventLeads_.size();
    if(!ctl.selectGlobalTimePlanForCommit(
           now, minimumReachEntryLead_, minimumSafeCommitLead,
           attemptedEventLeads_.size(), boundedEventLeads_.size(),
           scheduleComplete))
    {
      ctl.endObjectObservation(true);
      ctl.finishPhaseTiming("interception_planning");
      mc_rtc::log::error(
          "[GlobalTimePlanSearchSummary] committed=false reason={} scheduleComplete={} evaluatedHypotheses={} configuredHypotheses={} feasibleHypotheses={} geometryFailures={} elapsed={:.3f}s robotMoved=[{:.4f},{:.4f}] no fallback permitted",
          ctl.planningSelectionReason(), scheduleComplete,
          attemptedEventLeads_.size(), boundedEventLeads_.size(),
          feasibleHypothesisCount_, geometryFailureCount_,
          now - eventSearchStartTime_, maxRobotTranslationObserved_,
          maxRobotRotationObserved_);
      output("FAIL");
      return true;
    }

    if(!ctl.commitGlobalTimePlanSelection())
    {
      ctl.endObjectObservation(true);
      ctl.finishPhaseTiming("interception_planning");
      mc_rtc::log::error(
          "[GlobalTimePlanSearchSummary] committed=false reason=global_commit_validation_failed detail={} no fallback permitted",
          ctl.planningSelectionReason());
      output("FAIL");
      return true;
    }

    ctl.finishPhaseTiming("interception_planning");
    mc_rtc::log::success(
        "[GlobalTimePlanSearchSummary] committed=true scheduleComplete=true evaluatedHypotheses={} configuredHypotheses={} feasibleHypotheses={} geometryFailures={} selectedLead={:.3f}s selectedMotionJ={:.9f} selectedGlobalJ={:.9f} scheduleWait={:+.3f}s elapsed={:.3f}s robotMoved=[{:.4f},{:.4f}] oneCommit=true globalArgmin=true",
        attemptedEventLeads_.size(), boundedEventLeads_.size(),
        feasibleHypothesisCount_, geometryFailureCount_,
        ctl.selectedGlobalEventLead(), ctl.selectedGlobalMotionCost(),
        ctl.selectedGlobalObjectiveCost(),
        ctl.selectedGlobalScheduleWait(),
        ctl.controllerTime() - eventSearchStartTime_,
        maxRobotTranslationObserved_, maxRobotRotationObserved_);
    output("OK");
    return true;
  }

  if(phase_ == Phase::StartIteration)
  {
    if(!eventSearchBudgetAvailable(ctl.controllerTime()))
    {
      ctl.endObjectObservation(true);
      ctl.finishPhaseTiming("interception_planning");
      mc_rtc::log::error(
          "[BoundedEventSearchSummary] committed=false reason=budget_exhausted hypotheses={} feasible={} geometryFailures={} elapsed={:.3f}s robotMoved=[{:.4f},{:.4f}]",
          eventHypothesisCount_, feasibleHypothesisCount_,
          geometryFailureCount_, ctl.controllerTime() - eventSearchStartTime_,
          maxRobotTranslationObserved_, maxRobotRotationObserved_);
      output("FAIL");
      return true;
    }

    iterationStartTime_ = ctl.controllerTime();
    hypothesizedPresentationTime_ = globalTimePlanMode_
        ? eventSearchStartTime_ + guessLead_
        : iterationStartTime_ + guessLead_;
    const double predictionHorizon = hypothesizedPresentationTime_
                                   - iterationStartTime_;
    attemptedEventLeads_.push_back(guessLead_);
    ++eventHypothesisCount_;

    if(globalTimePlanMode_)
    {
      const double minimumSafeCommitLead = std::max(
          minimumCommitRemainingTime_,
          ctl.presentationDecelerationDuration() + 0.25);
      if(predictionHorizon + 1e-12 < minimumSafeCommitLead)
      {
        mc_rtc::log::warning(
            "[GlobalEventTimingReject] hypothesis={} eventLead={:.3f}s remaining={:.3f}s minimumSafeCommitLead={:.3f}s reason=expired_before_geometry completePlanEvaluationSkipped=true hardTimingConstraint=true",
            eventHypothesisCount_, guessLead_, predictionHorizon,
            minimumSafeCommitLead);
        double nextLead = 0.0;
        if(nextBoundedEventLead(nextLead))
        {
          scheduleNextHypothesis(nextLead, "global_fixed_schedule");
        }
        else
        {
          phase_ = Phase::SelectGlobal;
        }
        return false;
      }
    }

    if(globalTimePlanMode_)
    {
      const auto frozen = boundedEventPresentationPoses_.find(guessLead_);
      if(frozen == boundedEventPresentationPoses_.end())
      {
        ctl.endObjectObservation(true);
        ctl.finishPhaseTiming("interception_planning");
        mc_rtc::log::error(
            "[GlobalTimePlanSearchSummary] committed=false reason=missing_frozen_event_prediction lead={:.9f}s no fallback permitted",
            guessLead_);
        output("FAIL");
        return true;
      }
      planningObjectPresentationPose_ = frozen->second;
    }
    else
    {
      planningObjectPresentationPose_ =
          ctl.predictPresentationPose(predictionHorizon);
    }

    ctl.setPlanningObjectSnapshot(planningObjectPresentationPose_);
    ctl.applyPlanningObjectSnapshot();
    const auto status = ctl.beginCapturePlanning(false);
    ctl.clearPlanningObjectSnapshot();

    const Eigen::Vector3d po = planningObjectPresentationPose_.translation();
    mc_rtc::log::warning(
        "[PresentationSolve] iteration={}/{} hypothesisLead={:.3f}s presentationTime={:.3f}s decelerationStart={:.3f}s presentationObject=[{:.3f},{:.3f},{:.3f}]",
        eventHypothesisCount_, maximumEventHypotheses_, guessLead_,
        hypothesizedPresentationTime_,
        hypothesizedPresentationTime_
          - ctl.presentationDecelerationDuration(),
        po.x(), po.y(), po.z());
    mc_rtc::log::info(
        "[BoundedEventHypothesis] index={}/{} source={} lead={:.3f}s absolutePresentationTime={:.3f}s remaining={:.3f}s elapsed={:.3f}s attempted={} fixedSearchEpoch={} robotStationary=true",
        eventHypothesisCount_, globalTimePlanMode_
            ? boundedEventLeads_.size()
            : static_cast<std::size_t>(maximumEventHypotheses_),
        currentHypothesisSource_, guessLead_,
        hypothesizedPresentationTime_, predictionHorizon,
        iterationStartTime_ - eventSearchStartTime_,
        attemptedEventLeads_.size(), globalTimePlanMode_);

    if(status == HandoverInterceptionController::CapturePlanningStatus::Failure)
    {
      // Failure at beginCapturePlanning indicates a structural preparation
      // problem, not event-dependent infeasibility. Do not mask it by scanning.
      ctl.endObjectObservation(true);
      ctl.finishPhaseTiming("interception_planning");
      mc_rtc::log::error(
          "[PresentationSolve] capture planner could not start; no candidate committed");
      output("FAIL");
      return true;
    }
    phase_ = Phase::Planning;
    return false;
  }

  ctl.setPlanningObjectSnapshot(planningObjectPresentationPose_);
  ctl.applyPlanningObjectSnapshot();
  const auto status = ctl.stepCapturePlanning(planningStepsPerCycle_);
  ctl.clearPlanningObjectSnapshot();

  if(status == HandoverInterceptionController::CapturePlanningStatus::Running)
  {
    if(iter_ % logEvery_ == 1)
    {
      const Eigen::Vector3d p = ctl.objectPose().translation();
      mc_rtc::log::info(
          "[PresentationSolve] planning iteration={} liveObject=[{:.3f},{:.3f},{:.3f}] timeToPresentation={:.3f}s robotMoved={:.4f}",
          eventHypothesisCount_, p.x(), p.y(), p.z(),
          hypothesizedPresentationTime_ - ctl.controllerTime(), robotTranslation);
    }
    return false;
  }

  if(status == HandoverInterceptionController::CapturePlanningStatus::Failure
     || !ctl.planningBestCandidateAvailable())
  {
    ++geometryFailureCount_;
    mc_rtc::log::warning(
        "[BoundedEventGeometryReject] hypothesis={} lead={:.3f}s source={} completePlan=false geometryFailures={} robotStationary=true; searching another future event",
        eventHypothesisCount_, guessLead_, currentHypothesisSource_,
        geometryFailureCount_);

    if(globalTimePlanMode_)
    {
      double nextLead = 0.0;
      if(nextBoundedEventLead(nextLead))
      {
        scheduleNextHypothesis(nextLead, "global_fixed_schedule");
      }
      else
      {
        phase_ = Phase::SelectGlobal;
      }
      return false;
    }

    double nextLead = 0.0;
    if(boundedEventSearchEnabled_
       && eventSearchBudgetAvailable(ctl.controllerTime())
       && nextBoundedEventLead(nextLead)
       && scheduleNextHypothesis(nextLead, "bounded_geometry_scan"))
    {
      // A geometry discontinuity invalidates a secant pair. The next feasible
      // event will seed timing refinement afresh.
      havePreviousResidual_ = false;
      fixedPointIteration_ = 0;
      mc_rtc::log::info(
          "[BoundedEventSearchContinue] reason=no_complete_plan nextLead={:.3f}s remainingHypotheses={} elapsed={:.3f}s noPhysicalMotion=true",
          guessLead_, maximumEventHypotheses_ - eventHypothesisCount_,
          ctl.controllerTime() - eventSearchStartTime_);
      return false;
    }

    ctl.endObjectObservation(true);
    ctl.finishPhaseTiming("interception_planning");
    mc_rtc::log::error(
        "[BoundedEventSearchSummary] committed=false reason=no_complete_event hypotheses={} feasible={} geometryFailures={} elapsed={:.3f}s leadRange=[{:.3f},{:.3f}] robotMoved=[{:.4f},{:.4f}]",
        eventHypothesisCount_, feasibleHypothesisCount_,
        geometryFailureCount_, ctl.controllerTime() - eventSearchStartTime_,
        minimumPresentationLead_, maximumPresentationLead_,
        maxRobotTranslationObserved_, maxRobotRotationObserved_);
    output("FAIL");
    return true;
  }

  if(globalTimePlanMode_)
  {
    ++feasibleHypothesisCount_;
    if(!ctl.captureCurrentEventPlanAlternatives(
           static_cast<std::size_t>(eventHypothesisCount_), guessLead_,
           hypothesizedPresentationTime_, planningObjectPresentationPose_))
    {
      ctl.endObjectObservation(true);
      ctl.finishPhaseTiming("interception_planning");
      mc_rtc::log::error(
          "[GlobalTimePlanSearchSummary] committed=false reason=event_alternative_capture_failed detail={} hypothesis={} no fallback permitted",
          ctl.planningSelectionReason(), eventHypothesisCount_);
      output("FAIL");
      return true;
    }

    mc_rtc::log::success(
        "[GlobalEventEvaluation] hypothesis={} eventLead={:.3f}s presentationTime={:.3f}s completePlan=true remainingAfterEvaluation={:.3f}s deferredCommit=true robotStationary=true",
        eventHypothesisCount_, guessLead_, hypothesizedPresentationTime_,
        hypothesizedPresentationTime_ - ctl.controllerTime());
    double nextLead = 0.0;
    if(nextBoundedEventLead(nextLead))
    {
      scheduleNextHypothesis(nextLead, "global_fixed_schedule");
    }
    else
    {
      phase_ = Phase::SelectGlobal;
    }
    return false;
  }

  const double now = ctl.controllerTime();
  const double planningDuration = now - iterationStartTime_;
  const double remainingToHypothesis = hypothesizedPresentationTime_ - now;
  const double minimumSafeCommitLead = std::max(
      minimumCommitRemainingTime_,
      ctl.presentationDecelerationDuration() + 0.25);
  if(!ctl.selectPlanningBestForCommit(
         remainingToHypothesis, minimumReachEntryLead_,
         minimumSafeCommitLead))
  {
    ctl.clearPlanningObjectSnapshot();
    ctl.endObjectObservation(true);
    ctl.finishPhaseTiming("interception_planning");
    mc_rtc::log::error(
        "[BoundedEventSearchSummary] committed=false reason=plan_selection_failed detail={} hypotheses={} no fallback permitted",
        ctl.planningSelectionReason(), eventHypothesisCount_);
    output("FAIL");
    return true;
  }

  ++feasibleHypothesisCount_;
  const double predictedPresentationDuration =
      ctl.planningBestPredictedPresentationTime();
  const double residual =
      predictedPresentationDuration - remainingToHypothesis;
  // A negative residual means the FSM can enter ExecuteCommittedReach before
  // the immutable reachStart and wait. A positive residual means reachStart is
  // already in the past. Commit only with an explicit pre-launch reserve.
  const double reachEntryLead = -residual;
  const bool reachEntrySafe = reachEntryLead >= minimumReachEntryLead_;
  const bool timingMatched = std::abs(residual) <= timingTolerance_;
  const double launchGuardError = residual + minimumReachEntryLead_;

  mc_rtc::log::success(
      "[PresentationSolve] iteration={} candidate={} planning={:.3f}s hypothesisLead={:.3f}s remainingToPresentation={:.3f}s predictedPresentation={:.3f}s predictedContact={:.3f}s residual={:+.3f}s reachEntryLead={:+.3f}s requiredEntryLead={:.3f}s entrySafe={} timingMatched={} predictedExecution={:.3f}s clear={:.4f}",
      eventHypothesisCount_, ctl.planningBestCandidateName(),
      planningDuration, guessLead_, remainingToHypothesis,
      predictedPresentationDuration,
      ctl.planningBestPredictedContactTime(), residual, reachEntryLead,
      minimumReachEntryLead_, reachEntrySafe, timingMatched,
      ctl.planningBestPredictedExecutionTime(), ctl.planningBestClearance());

  // Exact synchronization does not require a zero residual. A negative
  // residual is executable scheduling slack: ExecuteCommittedReach enters
  // early, waits without motion, and launches at the immutable reachStart.
  if(reachEntrySafe
     && remainingToHypothesis >= minimumSafeCommitLead)
  {
    if(!ctl.commitPlanningBestAsInterception(
           hypothesizedPresentationTime_, residual,
           planningObjectPresentationPose_))
    {
      ctl.endObjectObservation(true);
      ctl.finishPhaseTiming("interception_planning");
      output("FAIL");
      return true;
    }

    ctl.finishPhaseTiming("interception_planning");
    mc_rtc::log::success(
        "[PresentationSolve] future-safe complete event admitted after {} hypothesis/hypotheses; timingMatched={} scheduledWait={:.3f}s. One candidate and one presentation event committed; robot has not moved",
        eventHypothesisCount_, timingMatched, reachEntryLead);
    mc_rtc::log::success(
        "[BoundedEventSearchSummary] committed=true hypotheses={} feasible={} geometryFailures={} timingRefinements={} selectedLead={:.3f}s residual={:+.3f}s elapsed={:.3f}s robotMoved=[{:.4f},{:.4f}] oneCommit=true",
        eventHypothesisCount_, feasibleHypothesisCount_,
        geometryFailureCount_, fixedPointIteration_, guessLead_, residual,
        ctl.controllerTime() - eventSearchStartTime_,
        maxRobotTranslationObserved_, maxRobotRotationObserved_);
    mc_rtc::log::success(
        "[PresentationSolve] stationary gate maxTranslation={:.4f} maxRotation={:.4f}",
        maxRobotTranslationObserved_, maxRobotRotationObserved_);
    output("OK");
    return true;
  }

  bool scheduled = false;
  if(fixedPointIteration_ < maximumFixedPointIterations_
     && eventSearchBudgetAvailable(now))
  {
    ++fixedPointIteration_;

    double updateResidual = reachEntrySafe ? residual : launchGuardError;
    std::string updateMethod = "relaxed_fixed_point";

    // A geometrically feasible event can still expire while the controller is
    // thinking. In that case, explicitly move the next hypothesis later rather
    // than applying a residual that could shorten an already-too-soon event.
    if(remainingToHypothesis < minimumSafeCommitLead)
    {
      updateResidual = minimumSafeCommitLead - remainingToHypothesis
                     + minimumReachEntryLead_;
      updateMethod = "commit_lead_guard";
    }

    double proposedLead = guessLead_ + fixedPointRelaxation_ * updateResidual;
    if(havePreviousResidual_ && updateMethod != "commit_lead_guard")
    {
      const double denominator = updateResidual - previousResidual_;
      if(std::abs(denominator) >= secantDenominatorTolerance_
         && std::abs(guessLead_ - previousGuessLead_) > 1e-6)
      {
        proposedLead = guessLead_
            - updateResidual * (guessLead_ - previousGuessLead_)
              / denominator;
        updateMethod = "damped_secant";
      }
    }

    double leadStep = proposedLead - guessLead_;
    leadStep = std::max(-maximumLeadStep_,
                        std::min(maximumLeadStep_, leadStep));
    const double nextGuess = std::max(
        minimumPresentationLead_,
        std::min(maximumPresentationLead_, guessLead_ + leadStep));

    mc_rtc::log::info(
        "[PresentationSolveUpdate] method={} oldLead={:.3f}s residual={:+.3f}s updateResidual={:+.3f}s proposedLead={:.3f}s appliedLead={:.3f}s step={:+.3f}s",
        updateMethod, guessLead_, residual, updateResidual, proposedLead,
        nextGuess, nextGuess - guessLead_);

    previousGuessLead_ = guessLead_;
    previousResidual_ = updateResidual;
    havePreviousResidual_ = true;
    scheduled = scheduleNextHypothesis(nextGuess, "timing_refinement");
  }

  if(!scheduled && boundedEventSearchEnabled_
     && eventSearchBudgetAvailable(ctl.controllerTime()))
  {
    double nextLead = 0.0;
    if(nextBoundedEventLead(nextLead))
    {
      scheduled = scheduleNextHypothesis(nextLead, "bounded_timing_fallback");
      if(scheduled)
      {
        havePreviousResidual_ = false;
        fixedPointIteration_ = 0;
        mc_rtc::log::info(
            "[BoundedEventSearchContinue] reason=timing_or_duplicate nextLead={:.3f}s remainingHypotheses={} elapsed={:.3f}s noPhysicalMotion=true",
            guessLead_, maximumEventHypotheses_ - eventHypothesisCount_,
            ctl.controllerTime() - eventSearchStartTime_);
      }
    }
  }

  if(scheduled) { return false; }

  ctl.endObjectObservation(true);
  ctl.finishPhaseTiming("interception_planning");
  mc_rtc::log::error(
      "[BoundedEventSearchSummary] committed=false reason=no_future_safe_complete_event hypotheses={} feasible={} geometryFailures={} timingRefinements={} lastResidual={:+.3f}s elapsed={:.3f}s robotMoved=[{:.4f},{:.4f}]",
      eventHypothesisCount_, feasibleHypothesisCount_, geometryFailureCount_,
      fixedPointIteration_, residual, ctl.controllerTime() - eventSearchStartTime_,
      maxRobotTranslationObserved_, maxRobotRotationObserved_);
  output("FAIL");
  return true;
}

void HandoverInterceptionController_SolveInterception::teardown(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ctl.clearPlanningObjectSnapshot();
  mc_rtc::log::info(
      "[PresentationSolve] teardown mode={} committed={}",
      ctl.observedObjectModeName(), ctl.interceptionCommitted());
}

EXPORT_SINGLE_STATE("HandoverInterceptionController_SolveInterception",
                    HandoverInterceptionController_SolveInterception)
