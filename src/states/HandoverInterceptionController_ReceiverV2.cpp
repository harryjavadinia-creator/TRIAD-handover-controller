#include "HandoverInterceptionController_ReceiverV2.h"
#include "../HandoverInterceptionController.h"

#include <mc_rtc/logging.h>

void HandoverInterceptionController_ReceiverV2::configure(
    const mc_rtc::Configuration & config)
{
  config_.load(config);
}

void HandoverInterceptionController_ReceiverV2::start(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ready_ = ctl.beginReceiverV2(config_);
}

bool HandoverInterceptionController_ReceiverV2::run(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  if(!ready_)
  {
    output("FAIL");
    return true;
  }
  switch(ctl.stepReceiverV2())
  {
    case HandoverInterceptionController::ReceiverStepStatusV2::Committed:
      output("OK");
      return true;
    case HandoverInterceptionController::ReceiverStepStatusV2::Failed:
      output("FAIL");
      return true;
    default:
      return false;
  }
}

void HandoverInterceptionController_ReceiverV2::teardown(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ctl.endReceiverV2();
}

EXPORT_SINGLE_STATE("HandoverInterceptionController_ReceiverV2",
                    HandoverInterceptionController_ReceiverV2)
