#pragma once

#include <mc_control/fsm/State.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

struct HandoverInterceptionController_SolveInterception
: mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  void buildBoundedEventLeadSchedule();
  bool leadAlreadyAttempted(double lead) const;
  bool nextBoundedEventLead(double & lead);
  bool eventSearchBudgetAvailable(double now) const;
  bool scheduleNextHypothesis(double lead, const std::string & source);

  enum class Phase
  {
    StartIteration,
    Planning
  };

  int planningStepsPerCycle_ = 12;
  int maximumFixedPointIterations_ = 4;
  bool boundedEventSearchEnabled_ = true;
  int maximumEventHypotheses_ = 15;
  double eventSearchLeadStep_ = 0.45;
  double maximumEventSearchWallTime_ = 5.0;
  double attemptedLeadTolerance_ = 0.02;
  double initialPresentationLead_ = 6.0;
  double minimumPresentationLead_ = 2.5;
  double maximumPresentationLead_ = 10.0;
  double timingTolerance_ = 0.25;
  double fixedPointRelaxation_ = 1.0;
  double maximumLeadStep_ = 1.0;
  double secantDenominatorTolerance_ = 0.05;
  double minimumCommitRemainingTime_ = 2.0;
  // The reach state must be entered before the immutable reachStart.
  // This is a pre-commit scheduling reserve, not runtime retiming.
  double minimumReachEntryLead_ = 0.05;
  double maximumRobotTranslation_ = 0.003;
  double maximumRobotRotation_ = 0.015;
  double holdTaskStiffness_ = 18.0;
  double holdTaskWeight_ = 4200.0;
  uint64_t logEvery_ = 250;

  Phase phase_ = Phase::StartIteration;
  bool ready_ = false;
  bool staticMode_ = false;
  int fixedPointIteration_ = 0;
  int eventHypothesisCount_ = 0;
  int feasibleHypothesisCount_ = 0;
  int geometryFailureCount_ = 0;
  size_t eventSearchCursor_ = 0;
  double eventSearchStartTime_ = 0.0;
  std::string currentHypothesisSource_ = "initial";
  std::vector<double> boundedEventLeads_;
  std::vector<double> attemptedEventLeads_;
  double guessLead_ = 0.0;
  bool havePreviousResidual_ = false;
  double previousGuessLead_ = 0.0;
  double previousResidual_ = 0.0;
  double iterationStartTime_ = 0.0;
  double hypothesizedPresentationTime_ = 0.0;
  double maxRobotTranslationObserved_ = 0.0;
  double maxRobotRotationObserved_ = 0.0;
  uint64_t iter_ = 0;

  sva::PTransformd holdBasePose_ = sva::PTransformd::Identity();
  sva::PTransformd planningObjectPresentationPose_ =
      sva::PTransformd::Identity();
  std::map<std::string, std::vector<double>> holdArmPosture_;
};
