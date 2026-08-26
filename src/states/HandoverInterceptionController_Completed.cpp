#include "HandoverInterceptionController_Completed.h"
#include "../HandoverInterceptionController.h"

#include <mc_rtc/logging.h>

#include <algorithm>

void HandoverInterceptionController_Completed::configure(
    const mc_rtc::Configuration & config)
{
  if(config.has("safetyCheckEvery")) { config("safetyCheckEvery", safetyCheckEvery_); }
}

void HandoverInterceptionController_Completed::start(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  iter_ = 0;
  ctl.setGripperClosureAuthorized(true);
  holdClosure_ = std::max(ctl.gripperCommand(), ctl.measuredGripperClosure());
  ctl.activateToolTask();
  ctl.setToolTaskGains(18.0, 4200.0);
  ctl.commandSelectedRetreatPosture();
  ctl.setGripperJointPriority(true);
  ctl.commandMouthTarget(ctl.currentMouthRetreatTarget());
  ctl.commandGripper(holdClosure_);
  ctl.logPhaseTimingSummary();
  mc_rtc::log::success(
      "[Completed] full plan-once handover completed: grasp confirmed and carried-object retreat finished mode={}",
      ctl.observedObjectModeName());
}

bool HandoverInterceptionController_Completed::run(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ++iter_;
  ctl.setGripperClosureAuthorized(true);
  ctl.commandSelectedRetreatPosture();
  ctl.setGripperJointPriority(true);
  ctl.commandMouthTarget(ctl.currentMouthRetreatTarget());
  ctl.commandGripper(holdClosure_);

  if(!ctl.objectAttached())
  {
    mc_rtc::log::error("[Completed] carried object attachment was lost");
    output("FAIL");
    return true;
  }

  if(safetyCheckEvery_ > 0 && iter_ % safetyCheckEvery_ == 1)
  {
    HandoverSafetyReport report;
    if(!ctl.evaluateAttachedRetreatSafety(report))
    {
      mc_rtc::log::error(
          "[Completed] final hold safety lost clear={:.4f} limiting={}/{}",
          report.minClearance, report.sample, report.obstacle);
      output("FAIL");
      return true;
    }
  }
  return false;
}

void HandoverInterceptionController_Completed::teardown(
    mc_control::fsm::Controller &)
{
  mc_rtc::log::info("[Completed] teardown");
}

EXPORT_SINGLE_STATE("HandoverInterceptionController_Completed",
                    HandoverInterceptionController_Completed)
