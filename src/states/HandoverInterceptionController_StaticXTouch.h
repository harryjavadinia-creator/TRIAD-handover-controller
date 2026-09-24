#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/TransformTask.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <Eigen/Core>

#include <cstdint>
#include <memory>
#include <string>

struct HandoverInterceptionController;

/**
 * Deliberately simple two-arm static X rendezvous.
 *
 * Robot B prepositions once and then holds. The bottle/object tip is represented
 * by one fixed world-vector offset from Robot B's tool frame. Robot A keeps its
 * gripper open, moves its mouth once toward that tip and stops at a configured
 * small gap. There is no observation, candidate planning, grasp, transfer,
 * retreat, retry or runtime reselection in this mode.
 */
struct HandoverInterceptionController_StaticXTouch : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  enum class Phase
  {
    MoveGiver,
    SettleGiver,
    MoveReceiver,
    SettleReceiver,
    Hold,
    Failed
  };

  static Eigen::Vector3d readVector3(const mc_rtc::Configuration & cfg,
                                     const std::string & key,
                                     const Eigen::Vector3d & fallback);
  static double smoothStep(double u);
  static double smoothStepVelocity(double u);
  void commandGiver(const sva::PTransformd & target,
                    const Eigen::Vector3d & velocityWorld = Eigen::Vector3d::Zero());
  void fail(HandoverInterceptionController & ctl, const std::string & reason);
  void updateMeasuredSpeeds(HandoverInterceptionController & ctl);

private:
  std::string giverRobot_ = "kinova";
  std::string giverToolFrame_ = "tool_frame";
  Eigen::Vector3d objectTipOffsetWorld_ = Eigen::Vector3d(-0.27, 0.0, 0.0);
  double contactGap_ = 0.030;
  double giverDuration_ = 5.0;
  double receiverDuration_ = 5.0;
  double settleDwell_ = 0.30;
  double settleTimeout_ = 4.0;
  double positionTolerance_ = 0.008;
  double velocityTolerance_ = 0.015;
  double maximumGiverTravel_ = 0.45;
  double maximumReceiverTravel_ = 0.45;
  double giverStiffness_ = 36.0;
  double giverWeight_ = 5600.0;
  double receiverStiffness_ = 36.0;
  double receiverWeight_ = 5600.0;
  int logEvery_ = 250;

  Phase phase_ = Phase::Failed;
  std::shared_ptr<mc_tasks::TransformTask> giverTask_;
  bool giverTaskActive_ = false;
  double phaseTime_ = 0.0;
  double stableTime_ = 0.0;
  uint64_t iter_ = 0;

  sva::PTransformd giverStart_ = sva::PTransformd::Identity();
  sva::PTransformd giverTarget_ = sva::PTransformd::Identity();
  sva::PTransformd receiverStartMouth_ = sva::PTransformd::Identity();
  sva::PTransformd receiverTargetMouth_ = sva::PTransformd::Identity();
  sva::PTransformd holdGiver_ = sva::PTransformd::Identity();
  sva::PTransformd holdReceiverMouth_ = sva::PTransformd::Identity();
  Eigen::Vector3d touchPointWorld_ = Eigen::Vector3d::Zero();

  bool havePrevious_ = false;
  Eigen::Vector3d previousGiverPosition_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d previousReceiverPosition_ = Eigen::Vector3d::Zero();
  double giverSpeed_ = 0.0;
  double receiverSpeed_ = 0.0;
};
