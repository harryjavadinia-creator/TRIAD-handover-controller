#pragma once

#include <mc_control/fsm/State.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include <cstdint>

struct HandoverInterceptionController_CaptureTransfer : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  enum class Phase
  {
    PriorityBlend,
    CaptureLock,
    Closing,
    Confirming,
    LoadTransfer
  };

  double stableDwell_ = 0.12;
  // Grasp verification is performed inside the same Acquire closed loop.
  // No controller/gain/target switch is allowed after first contact.
  double confirmationDwell_ = 0.25;
  double confirmationTimeout_ = 2.0;
  uint64_t confirmationLogEvery_ = 20;
  double watchdogTimeout_ = 20.0;
  double taskStiffness_ = 30.0;
  double taskWeight_ = 4800.0;
  double gapProgressEps_ = 2e-5;
  uint64_t gapStallLimit_ = 1800;

  // Bumpless state transition. Only gripper-joint gains are blended; the
  // Cartesian and arm-posture tasks remain unchanged from terminal approach.
  double priorityBlendDuration_ = 0.40;

  // Capture-lock phase: closure starts only after pose and bilateral centering
  // remain settled under the final gripper-joint priority.
  double lockDwell_ = 0.25;
  double lockTimeout_ = 4.0;
  double lockPosTolerance_ = 0.004;
  double lockOriTolerance_ = 0.030;
  double lockCenterTolerance_ = 0.00035;
  double lockCenterRateTolerance_ = 0.003;
  double lockTransientPosLimit_ = 0.010;
  double lockTransientCenterLimit_ = 0.003;

  // Closed-loop centering and closure gate. The desired absolute offset is
  // reconstructed from the measured mouth displacement plus pad asymmetry,
  // rather than integrating command error as though command equalled motion.
  double centeringGain_ = 6.0;
  double centeringDamping_ = 1.2;
  double maxCenteringSpeed_ = 0.012;
  double maxCenteringAcceleration_ = 0.06;
  double maxCenteringOffset_ = 0.008;
  double centeringDeadband_ = 0.00012;
  // The shared controller-level acquisition tube provides the aperture-
  // dependent gate. This hysteresis prevents chatter at its boundary.
  double closureCenterHysteresis_ = 0.00012;
  double centeringRecoveryTimeout_ = 5.0;
  double centerRateFilter_ = 0.15;

  // Slow bounded bias adaptation removes steady-state Cartesian/QP residual
  // without returning to V3A.7 command integration and oscillation.
  double centeringBiasGain_ = 0.35;
  double centeringBiasLeak_ = 0.08;
  double maxCenteringBias_ = 0.003;

  Phase phase_ = Phase::PriorityBlend;
  double priorityBlendElapsed_ = 0.0;
  double stableTime_ = 0.0;
  double confirmationStableTime_ = 0.0;
  double confirmationElapsed_ = 0.0;
  double holdClosure_ = 0.0;
  double lockStableTime_ = 0.0;
  double lockElapsed_ = 0.0;
  double recoveryElapsed_ = 0.0;
  double elapsed_ = 0.0;
  double closeCommand_ = 0.0;
  double bestGap_ = 1e9;
  uint64_t gapStallCycles_ = 0;
  uint64_t iter_ = 0;
  bool watchdogWarned_ = false;
  bool havePreviousSignedError_ = false;
  double previousSignedError_ = 0.0;
  double filteredCenterRate_ = 0.0;
  double desiredCenteringOffset_ = 0.0;
  double centeringOffset_ = 0.0;
  double centeringVelocity_ = 0.0;
  double centeringBias_ = 0.0;
  bool centeringPaused_ = false;
  double maxTransitionPosError_ = 0.0;
  double maxTransitionCenterError_ = 0.0;
  double maxTransitionOffset_ = 0.0;

  // Force-aware load takeover remains inside this same controller. No target,
  // grasp, route or gain reset is allowed at the grasp-to-transfer boundary.
  double loadTransferElapsed_ = 0.0;
  double releaseStableTime_ = 0.0;
  double admittanceOffset_ = 0.0;
  double admittanceVelocity_ = 0.0;
  bool acquireTimingFinished_ = false;
  sva::PTransformd transferTarget_ = sva::PTransformd::Identity();

  sva::PTransformd nominalTarget_ = sva::PTransformd::Identity();
  sva::PTransformd centeredTarget_ = sva::PTransformd::Identity();
};
