#pragma once

#include <mc_control/fsm/State.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include <cstdint>

struct HandoverInterceptionController_PresentationHold
: mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  double maxLinearTrackingLead_ = 0.030;
  double maxAngularTrackingLead_ = 0.10;
  double posTol_ = 0.012;
  double oriTol_ = 0.050;
  double maximumOpenClosure_ = 0.050;
  double stableDwell_ = 0.15;
  double reserveMargin_ = 0.10;
  double taskStiffness_ = 40.0;
  double taskWeight_ = 5400.0;
  double maximumObjectTranslationDeviation_ = 0.015;
  double maximumObjectRotationDeviation_ = 0.12;
  uint64_t logEvery_ = 100;

  bool ready_ = false;
  uint64_t iter_ = 0;
  double stableTime_ = 0.0;
  double latestReleaseTime_ = 0.0;
  sva::PTransformd holdPose_ = sva::PTransformd::Identity();
};
