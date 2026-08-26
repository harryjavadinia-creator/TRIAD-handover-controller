#pragma once

#include <mc_control/fsm/State.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include <cstdint>

struct HandoverInterceptionController_MovePregrasp : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  double farLinearSpeed_ = 0.24;
  double nearLinearSpeed_ = 0.070;
  double nearDistance_ = 0.025;
  double angularSpeed_ = 1.00;
  double linearAcceleration_ = 0.80;
  double angularAcceleration_ = 3.00;
  double spatialLinearLookAhead_ = 0.018;
  double spatialAngularLookAhead_ = 0.070;
  double minimumLinearLookAhead_ = 0.004;
  double minimumAngularLookAhead_ = 0.015;
  double lookAheadRemainingRatio_ = 0.55; // retained for backward-compatible parsing
  double lookAheadTaperDistance_ = 0.012;
  double linearFeedforwardRatio_ = 0.70;
  double angularFeedforwardRatio_ = 0.70;
  // V5.2.5 terminal capture admission. The fast spatial servo is retained,
  // but the final segment is dynamically braked and must be pose/velocity
  // stable before Acquire may own closure.
  double terminalLinearDeceleration_ = 2.00;
  double terminalAngularDeceleration_ = 4.50;
  double terminalBrakeMargin_ = 0.008;
  double terminalLinearSpeedTolerance_ = 0.040;
  double terminalAngularSpeedTolerance_ = 0.080;
  double terminalStableDwell_ = 0.080;
  double terminalVelocityFilter_ = 0.20;
  double maxLinearTrackingLead_ = 0.028;
  double maxAngularTrackingLead_ = 0.12;
  double posTol_ = 0.003;
  double oriTol_ = 0.025;
  double centeringTol_ = 0.00045;
  double maximumOpenClosure_ = 0.050;
  double taskStiffness_ = 30.0;
  double taskWeight_ = 4800.0;
  double progressEps_ = 5e-5;
  uint64_t progressLimit_ = 1500;
  uint64_t maxIter_ = 18000;
  uint64_t stagnantCycles_ = 0;
  uint64_t iter_ = 0;
  double bestCombinedError_ = 1e9;
  double pathDistance_ = 0.0;
  double pathAngle_ = 0.0;
  double commandedLinearSpeed_ = 0.0;
  double commandedAngularSpeed_ = 0.0;
  double referenceProgress_ = 0.0;
  double measuredProgressAnchor_ = 0.0;
  double peakMeasuredLinearSpeed_ = 0.0;
  double peakMeasuredAngularSpeed_ = 0.0;
  double filteredLinearSpeed_ = 0.0;
  double filteredAngularSpeed_ = 0.0;
  double dynamicBrakeDistance_ = 0.0;
  double dynamicBrakeAngle_ = 0.0;
  double terminalStableTime_ = 0.0;
  bool linearBrakingActive_ = false;
  bool angularBrakingActive_ = false;
  bool linearProgressCoupled_ = false;
  bool angularCorrectionRequired_ = false;
  bool angularProgressCoupled_ = false;
  bool haveVelocityEstimate_ = false;
  Eigen::Vector3d linearPathDirectionWorld_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angularPathDirectionWorld_ = Eigen::Vector3d::Zero();
  sva::PTransformd previousActualPose_ = sva::PTransformd::Identity();
  sva::PTransformd startPose_ = sva::PTransformd::Identity();
  sva::PTransformd goalPose_ = sva::PTransformd::Identity();
  sva::PTransformd referencePose_ = sva::PTransformd::Identity();
};
