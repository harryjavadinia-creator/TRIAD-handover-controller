#include "HandoverInterceptionController_Failure.h"
#include "../HandoverInterceptionController.h"

#include <mc_rtc/logging.h>

void HandoverInterceptionController_Failure::start(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ctl.setGripperClosureAuthorized(true);
  ctl.activateToolTask();
  holdPose_ = ctl.actualMouthPose();
  holdClosure_ = ctl.gripperCommand();
  ctl.commandMouthTarget(holdPose_);
  ctl.commandGripper(holdClosure_);

  mc_rtc::log::error(
      "[Failure] fail-safe hold entered. No candidate retry loop and no unsafe fallback motion");
}

bool HandoverInterceptionController_Failure::run(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ctl.setGripperClosureAuthorized(true);
  ctl.commandMouthTarget(holdPose_);
  ctl.commandGripper(holdClosure_);
  return false;
}

void HandoverInterceptionController_Failure::teardown(
    mc_control::fsm::Controller &)
{
  mc_rtc::log::info("[Failure] teardown");
}

EXPORT_SINGLE_STATE("HandoverInterceptionController_Failure", HandoverInterceptionController_Failure)
