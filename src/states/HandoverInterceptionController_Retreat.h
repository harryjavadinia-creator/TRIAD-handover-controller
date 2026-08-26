#pragma once

#include <mc_control/fsm/State.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include <cstdint>

struct HandoverInterceptionController_Retreat : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  double linearSpeed_ = 0.30;
  double angularSpeed_ = 0.80;
  double linearAcceleration_ = 0.80;
  double angularAcceleration_ = 2.00;
  double spatialLinearLookAhead_ = 0.050;
  double spatialAngularLookAhead_ = 0.14;
  double minimumLinearLookAhead_ = 0.012;
  double minimumAngularLookAhead_ = 0.050;
  double lookAheadRemainingRatio_ = 0.55; // retained for backward-compatible parsing
  double lookAheadTaperDistance_ = 0.030;
  double linearFeedforwardRatio_ = 0.80;
  double angularFeedforwardRatio_ = 0.80;
  double maxLinearTrackingLead_ = 0.065;
  double maxAngularTrackingLead_ = 0.20;
  double posTol_ = 0.012;
  double oriTol_ = 0.060;
  double taskStiffness_ = 36.0;
  double taskWeight_ = 5200.0;
  double progressEps_ = 8e-5;
  uint64_t progressLimit_ = 1500;
  uint64_t maxIter_ = 14000;
  uint64_t stagnantCycles_ = 0;
  uint64_t iter_ = 0;
  double bestCombinedError_ = 1e9;
  double holdClosure_ = 0.0;
  double pathDistance_ = 0.0;
  double pathAngle_ = 0.0;
  double commandedLinearSpeed_ = 0.0;
  double commandedAngularSpeed_ = 0.0;
  double referenceProgress_ = 0.0;
  double measuredProgressAnchor_ = 0.0;
  double peakMeasuredLinearSpeed_ = 0.0;
  double peakMeasuredAngularSpeed_ = 0.0;
  Eigen::Vector3d linearPathDirectionWorld_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angularPathDirectionWorld_ = Eigen::Vector3d::Zero();
  sva::PTransformd previousActualPose_ = sva::PTransformd::Identity();
  sva::PTransformd startPose_ = sva::PTransformd::Identity();
  sva::PTransformd goalPose_ = sva::PTransformd::Identity();
  sva::PTransformd referencePose_ = sva::PTransformd::Identity();
};
