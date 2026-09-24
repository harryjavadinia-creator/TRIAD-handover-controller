#include "HandoverInterceptionController_RobotBScenarioPreview.h"
#include "../HandoverInterceptionController.h"

#include <mc_rtc/logging.h>

#include <stdexcept>

void HandoverInterceptionController_RobotBScenarioPreview::start(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);

  presentationRequested_ = false;
  failureLogged_ = false;
  iter_ = 0;

  // Robot A does not intercept in this preview.
  ctl.detachObject();
  ctl.invalidateSelectedCandidate();
  ctl.deactivateToolTask();
  ctl.setGripperClosureAuthorized(false);
  ctl.setGripperJointPriority(false);
  ctl.commandGripper(0.0);

  if(!ctl.dualGiverEnabled())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[RobotBScenarioPreview] dualHandover.enabled must be true");
  }

  mc_rtc::log::success(
      "[RobotBScenarioPreview START] Robot A holds posture. Robot B alone "
      "prepositions to the selected Robot-A/world start, executes once and holds.");
}

bool HandoverInterceptionController_RobotBScenarioPreview::run(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ++iter_;

  ctl.setGripperClosureAuthorized(false);
  ctl.commandGripper(0.0);

  if(ctl.dualGiverFailed())
  {
    if(!failureLogged_)
    {
      failureLogged_ = true;
      mc_rtc::log::error(
          "[RobotBScenarioPreview FAILURE] Robot B entered safe hold. "
          "See the preceding DualGiver failure message.");
    }
    return false;
  }

  if(!presentationRequested_ && ctl.dualGiverReady())
  {
    presentationRequested_ = ctl.startDualGiverPresentation();
    if(presentationRequested_)
    {
      mc_rtc::log::success(
          "[RobotBScenarioPreview EXECUTE] Robot B start gate passed; "
          "world-frame scenario execution requested.");
    }
  }

  if(!presentationRequested_ && iter_ % 1000 == 1)
  {
    mc_rtc::log::info(
        "[RobotBScenarioPreview WAIT] Robot B is prepositioning to the exact "
        "world-frame scenario start.");
  }

  return false;
}

void HandoverInterceptionController_RobotBScenarioPreview::teardown(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ctl.commandDualGiverSafeHold();
  mc_rtc::log::info(
      "[RobotBScenarioPreview] teardown: Robot B safe hold requested");
}

EXPORT_SINGLE_STATE(
    "HandoverInterceptionController_RobotBScenarioPreview",
    HandoverInterceptionController_RobotBScenarioPreview)
