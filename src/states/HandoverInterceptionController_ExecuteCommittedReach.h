#pragma once

#include <mc_control/fsm/State.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

struct HandoverInterceptionController_ExecuteCommittedReach
: mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  double maxLinearTrackingLead_ = 0.055;
  double maxAngularTrackingLead_ = 0.25;
  double nearLinearTrackingLead_ = 0.015;
  double nearAngularTrackingLead_ = 0.08;
  double clearanceSlowdownStart_ = 0.070;
  double clearanceHardMargin_ = 0.012;
  double minimumRuntimeClearance_ = 0.008;
  double minimumVelocityScale_ = 0.20;
  double clearanceScaleDropRate_ = 6.0;
  double clearanceScaleRiseRate_ = 1.5;
  double farLinearSpeed_ = 0.30;
  double nearLinearSpeed_ = 0.10;
  double farAngularSpeed_ = 1.20;
  double nearAngularSpeed_ = 0.45;
  double posTol_ = 0.012;
  double oriTol_ = 0.050;
  double taskStiffness_ = 48.0;
  double taskWeight_ = 5600.0;
  double launchTimingTolerance_ = 0.010;
  double scheduleLatenessTolerance_ = 0.25;
  double minimumScheduledDuration_ = 0.25;
  double maximumObjectTranslationDeviation_ = 0.015;
  double maximumObjectRotationDeviation_ = 0.12;
  uint64_t logEvery_ = 80;

  bool ready_ = false;
  bool motionStarted_ = false;
  bool phaseTimingStarted_ = false;
  bool decelerationLogged_ = false;
  uint64_t iter_ = 0;
  double stateEntryTime_ = 0.0;
  double reachStartTime_ = 0.0;
  double startTime_ = 0.0;
  double reachEndTime_ = 0.0;
  sva::PTransformd startPose_ = sva::PTransformd::Identity();
  sva::PTransformd goalPose_ = sva::PTransformd::Identity();
  sva::PTransformd referencePose_ = sva::PTransformd::Identity();
  sva::PTransformd previousMouthPose_ = sva::PTransformd::Identity();
  std::map<std::string, std::vector<double>> holdArmPosture_;
  double previousSampleTime_ = 0.0;
  bool havePreviousSample_ = false;
  double clearanceScale_ = 1.0;
  double minimumObservedClearance_ = 1e9;
};
