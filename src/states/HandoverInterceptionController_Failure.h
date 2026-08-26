#pragma once

#include <mc_control/fsm/State.h>
#include <SpaceVecAlg/SpaceVecAlg>

struct HandoverInterceptionController_Failure : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration &) override {}
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  sva::PTransformd holdPose_ = sva::PTransformd::Identity();
  double holdClosure_ = 0.0;
};
