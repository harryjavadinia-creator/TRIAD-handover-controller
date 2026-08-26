#pragma once

#include <mc_control/fsm/State.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

/**
 * Unified static/moving object observation gate.
 *
 * The robot first settles under the exact controller that will hold it during
 * observation. The controlled quantity and the safety gate use the same
 * physical gripper-base frame. The Robotiq aperture is treated independently:
 * its settled open value is measured once, then only drift from that measured
 * baseline is constrained. This avoids model-dependent normalized-open offsets
 * and avoids using the pad-midpoint mouth frame as an arm-stationarity proxy.
 *
 * After the observation window, the final estimated object twist classifies
 * the presentation as static or moving. A static presentation is frozen at its
 * measured current pose for one complete current-pose planning event. A moving
 * presentation remains under the estimated motion model and proceeds to the
 * bounded predictive future-event solver. The physical robot remains at the
 * observation hold throughout this classification stage.
 */
struct HandoverInterceptionController_ObserveObject : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  enum class Phase
  {
    Settling,
    Observing
  };

  // Motion-classification thresholds. A valid observation below the
  // displacement gate is accepted as a stationary presentation only when both
  // estimated linear and angular speeds are also small.
  double minimumObservedDisplacement_ = 0.015;
  double stationaryLinearSpeedTolerance_ = 0.010;
  double stationaryAngularSpeedTolerance_ = 0.050;
  double maximumRobotTranslation_ = 0.003;
  double maximumRobotRotation_ = 0.015;

  double settleDwell_ = 0.30;
  double settleTimeout_ = 3.00;
  double settleLinearSpeedTolerance_ = 0.0025;
  double settleAngularSpeedTolerance_ = 0.015;

  // The Robotiq model reports a small non-zero normalized closure even when
  // commanded fully open. Therefore open-ness is a sanity envelope, while
  // observation stability is defined relative to a measured baseline.
  double maximumOpenClosure_ = 0.050;
  double gripperClosureRateTolerance_ = 0.030;
  double gripperClosureDriftTolerance_ = 0.002;

  double holdTaskStiffness_ = 18.0;
  double holdTaskWeight_ = 4200.0;

  uint64_t logEvery_ = 100;
  uint64_t maxIter_ = 10000;
  uint64_t iter_ = 0;
  bool completed_ = false;
  Phase phase_ = Phase::Settling;
  double settleElapsed_ = 0.0;
  double settleStableTime_ = 0.0;
  double maxRobotTranslationObserved_ = 0.0;
  double maxRobotRotationObserved_ = 0.0;
  double maxMouthTranslationObserved_ = 0.0;
  double maxGripperClosureDriftObserved_ = 0.0;
  double previousClosure_ = 0.0;
  double openClosureReference_ = 0.0;
  sva::PTransformd holdBasePose_ = sva::PTransformd::Identity();
  sva::PTransformd previousBasePose_ = sva::PTransformd::Identity();
  sva::PTransformd holdMouthPose_ = sva::PTransformd::Identity();
  std::map<std::string, std::vector<double>> holdArmPosture_;
};
