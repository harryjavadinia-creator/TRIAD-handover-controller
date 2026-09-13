#pragma once

#include <mc_control/fsm/State.h>

/**
 * TRIAD V2 receding complete-action receiver supervisor.
 *
 * Replaces V1's SolveInterception -> ExecuteCommittedReach -> PresentationHold
 * sequence only when receiverArchitecture == v2_receding. It drives the active
 * provisional plan while the object moves and exits with OK exactly once, when
 * the terminal commitment has been made; the existing MovePregrasp,
 * CaptureTransfer and Retreat states then execute unchanged.
 */
struct HandoverInterceptionController_ReceiverV2 : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  mc_rtc::Configuration config_;
  bool ready_ = false;
};
