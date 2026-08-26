#pragma once

#include <mc_control/fsm/State.h>

#include <cstdint>

struct HandoverInterceptionController_Completed : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  uint64_t safetyCheckEvery_ = 50;
  uint64_t iter_ = 0;
  double holdClosure_ = 0.0;
};
