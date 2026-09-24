#pragma once

#include <mc_control/fsm/State.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <string>

struct CALLRobotBFaceToFaceMover_Run : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  enum class Phase
  {
    DisabledHold,
    Prepositioning,
    PrepositionSettling,
    Ready,
    Executing,
    Settling,
    Holding,
    Failed
  };

  void enterFailure(class CALLRobotBFaceToFaceMover & ctl,
                    const std::string & reason);
  void updateMeasuredMotion(class CALLRobotBFaceToFaceMover & ctl);
  sva::PTransformd interpolatedPrepositionPose(double s) const;

private:
  Phase phase_ = Phase::DisabledHold;

  sva::PTransformd initialHoldPose_ = sva::PTransformd::Identity();
  sva::PTransformd prepositionStartPose_ = sva::PTransformd::Identity();
  sva::PTransformd startPose_ = sva::PTransformd::Identity();
  sva::PTransformd targetPose_ = sva::PTransformd::Identity();
  sva::PTransformd holdPose_ = sva::PTransformd::Identity();

  Eigen::Quaterniond prepositionStartQuaternion_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond prepositionTargetQuaternion_ = Eigen::Quaterniond::Identity();

  Eigen::Vector3d previousActualPosition_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d previousActualRotation_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d filteredActualVelocity_ = Eigen::Vector3d::Zero();
  double filteredActualAngularSpeed_ = 0.0;
  bool havePreviousActualPose_ = false;

  double prepositionElapsed_ = 0.0;
  double prepositionDuration_ = 0.0;
  double prepositionSettleElapsed_ = 0.0;
  double prepositionStableTime_ = 0.0;
  double elapsed_ = 0.0;
  double settleElapsed_ = 0.0;
  double terminalStableTime_ = 0.0;
  double maximumObservedTrackingError_ = 0.0;
  double maximumPrepositionPositionError_ = 0.0;
  uint64_t iter_ = 0;
  uint64_t logEvery_ = 200;
};
