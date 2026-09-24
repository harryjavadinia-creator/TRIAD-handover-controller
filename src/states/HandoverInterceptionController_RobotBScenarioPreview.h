#pragma once

#include <mc_control/fsm/State.h>

#include <cstdint>

/**
 * Robot-B-only scenario preview.
 *
 * Robot A remains under its posture task. Robot B prepositions its tool to the
 * selected scenario start expressed in the common Robot-A/world frame,
 * executes the world start-to-end trajectory once and holds the endpoint.
 */
struct HandoverInterceptionController_RobotBScenarioPreview
: mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  bool presentationRequested_ = false;
  bool failureLogged_ = false;
  std::uint64_t iter_ = 0;
};
