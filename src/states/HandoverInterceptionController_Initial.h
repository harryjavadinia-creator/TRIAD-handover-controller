#pragma once

#include <mc_control/fsm/State.h>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

struct HandoverInterceptionController;

struct HandoverInterceptionController_Initial : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  bool homePostureReached(const HandoverInterceptionController & ctl) const;
  void advanceReference(HandoverInterceptionController & ctl);
  bool runHardwareGripperCommissioning(HandoverInterceptionController & ctl);
  double commissioningArmDrift(const HandoverInterceptionController & ctl) const;

private:
  double jointTol_ = 0.035;
  double jointVelTol_ = 0.03;
  double stableDwell_ = 0.25;
  double maxJointSpeed_ = 0.55;
  double maxJointAcceleration_ = 1.20;
  double stableTime_ = 0.0;
  uint64_t iter_ = 0;
  uint64_t maxIter_ = 50000;
  std::map<std::string, std::vector<double>> qRef_;
  std::map<std::string, double> qdotRef_;

  enum class CommissioningPhase
  {
    OpenSettle,
    CloseToTarget,
    HoldTarget,
    ReturnOpen,
    CompleteHold,
    FaultHold
  };

  bool hardwareCommissioningEnabled_ = false;
  double commissioningTargetPercent_ = 5.0;
  double commissioningOpenTolerancePercent_ = 1.5;
  double commissioningTargetTolerancePercent_ = 1.5;
  double commissioningVelocityTolerancePercent_ = 2.0;
  double commissioningOpenDwell_ = 0.50;
  double commissioningTargetDwell_ = 0.50;
  double commissioningHoldDuration_ = 1.00;
  double commissioningArmDriftLimit_ = 0.020;
  bool commissioningReturnOpen_ = true;
  CommissioningPhase commissioningPhase_ = CommissioningPhase::OpenSettle;
  double commissioningPhaseTime_ = 0.0;
  double commissioningStableTime_ = 0.0;
  double commissioningMaxArmDrift_ = 0.0;
  bool commissioningCompletionLogged_ = false;
  bool commissioningFaultLogged_ = false;
};
