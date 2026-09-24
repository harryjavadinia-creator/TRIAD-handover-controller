#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/TransformTask.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <Eigen/Core>

#include <cstdint>
#include <memory>
#include <string>

/**
 * Robot-B-only trajectory player with deterministic scenario pre-positioning.
 *
 * Robot A remains fully unchanged and there is no inter-robot communication.
 * Each Robot-B scenario owns a predefined absolute tool start position in
 * Robot-B's base frame. At controller start, Robot B moves from its current
 * position to that predefined start while preserving the tool orientation that
 * existed when the controller started. It verifies measured pose/velocity
 * stability, waits READY for one manual trigger, and then executes the mirrored
 * relative Robot-A object trajectory.
 */
struct CALLRobotBFaceToFaceMover : public mc_control::fsm::Controller
{
public:
  CALLRobotBFaceToFaceMover(mc_rbdyn::RobotModulePtr rm,
                           double dt,
                           const mc_rtc::Configuration & config);

  bool run() override;
  void reset(const mc_control::ControllerResetData & reset_data) override;

  void activateToolTask();
  void clearToolTaskReferenceMotion();
  void commandToolPose(const sva::PTransformd & target);
  void commandToolPoseWithWorldMotion(const sva::PTransformd & target,
                                      const Eigen::Vector3d & linearVelocityWorld,
                                      const Eigen::Vector3d & linearAccelerationWorld);
  sva::PTransformd actualToolPose() const;

  double controlDt() const { return controlDt_; }
  const std::string & toolFrame() const { return toolFrame_; }
  const std::string & scenarioName() const { return scenarioName_; }
  const std::string & triggerFile() const { return triggerFile_; }
  bool motionEnabled() const { return motionEnabled_; }

  const Eigen::Vector3d & robotAVelocity() const { return robotAVelocity_; }
  Eigen::Vector3d robotBVelocity() const;
  double constantVelocityDuration() const { return constantVelocityDuration_; }
  double decelerationDuration() const { return decelerationDuration_; }
  double maximumTravel() const { return maximumTravel_; }
  double maximumLinearSpeed() const { return maximumLinearSpeed_; }
  double maximumTrackingError() const { return maximumTrackingError_; }
  double trackingErrorGrace() const { return trackingErrorGrace_; }
  const Eigen::Vector3d & workspaceMin() const { return workspaceMin_; }
  const Eigen::Vector3d & workspaceMax() const { return workspaceMax_; }

  bool startPoseConfigured() const { return startPoseConfigured_; }
  const sva::PTransformd & scenarioStartPose() const { return scenarioStartPose_; }
  double prepositionMaximumLinearSpeed() const { return prepositionMaximumLinearSpeed_; }
  double prepositionMaximumAngularSpeed() const { return prepositionMaximumAngularSpeed_; }
  double prepositionMinimumDuration() const { return prepositionMinimumDuration_; }
  double prepositionTimeout() const { return prepositionTimeout_; }
  double prepositionPositionTolerance() const { return prepositionPositionTolerance_; }
  double prepositionOrientationTolerance() const { return prepositionOrientationTolerance_; }
  double prepositionLinearVelocityTolerance() const { return prepositionLinearVelocityTolerance_; }
  double prepositionAngularVelocityTolerance() const { return prepositionAngularVelocityTolerance_; }
  double prepositionStableDuration() const { return prepositionStableDuration_; }

  double terminalPositionTolerance() const { return terminalPositionTolerance_; }
  double terminalVelocityTolerance() const { return terminalVelocityTolerance_; }
  double terminalStableDuration() const { return terminalStableDuration_; }
  double terminalTimeout() const { return terminalTimeout_; }
  double velocityFilterAlpha() const { return velocityFilterAlpha_; }

  bool triggerPresent() const;
  void clearTrigger() const;
  bool targetInsideWorkspace(const Eigen::Vector3d & p) const;
  bool configuredTrajectorySafe(std::string & reason) const;

  static double smoothStep(double u);
  static double smoothStepVelocity(double u);
  static double smoothStepAcceleration(double u);
  static double rotationAngle(const Eigen::Matrix3d & R);
  static double decelerationIntegral(double u);
  static double decelerationVelocityScale(double u);
  static double decelerationAccelerationScale(double u);

private:
  void loadConfig(const mc_rtc::Configuration & config);
  static Eigen::Vector3d readVector3(const mc_rtc::Configuration & config,
                                     const std::string & key,
                                     const Eigen::Vector3d & fallback);
  static Eigen::Matrix3d readRotation9(const mc_rtc::Configuration & config,
                                       const std::string & key,
                                       const Eigen::Matrix3d & fallback);

private:
  double controlDt_ = 0.001;
  std::string toolFrame_ = "gen3_robotiq_85_base_link";
  std::shared_ptr<mc_tasks::TransformTask> toolTask_;
  bool toolTaskActive_ = false;
  sva::PTransformd commandedPose_ = sva::PTransformd::Identity();

  double taskStiffness_ = 50.0;
  double taskWeight_ = 6500.0;

  std::string scenarioName_ = "canonical_yz";
  bool faceToFaceYaw180_ = true;
  Eigen::Vector3d robotAVelocity_ = Eigen::Vector3d(0.0, 0.08, 0.0);
  double constantVelocityDuration_ = 4.425;
  double decelerationDuration_ = 0.85;
  double maximumTravel_ = 0.45;

  bool startPoseConfigured_ = true;
  sva::PTransformd scenarioStartPose_ = sva::PTransformd::Identity();
  double prepositionMaximumLinearSpeed_ = 0.08;
  double prepositionMaximumAngularSpeed_ = 0.40;
  double prepositionMinimumDuration_ = 2.0;
  double prepositionTimeout_ = 20.0;
  double prepositionPositionTolerance_ = 0.004;
  double prepositionOrientationTolerance_ = 0.035;
  double prepositionLinearVelocityTolerance_ = 0.010;
  double prepositionAngularVelocityTolerance_ = 0.050;
  double prepositionStableDuration_ = 0.200;

  std::string triggerFile_ = "/tmp/call_robot_b_start";
  bool motionEnabled_ = false;
  double maximumLinearSpeed_ = 0.10;
  double maximumTrackingError_ = 0.06;
  double trackingErrorGrace_ = 0.40;
  Eigen::Vector3d workspaceMin_ = Eigen::Vector3d(-1.0, -1.0, 0.08);
  Eigen::Vector3d workspaceMax_ = Eigen::Vector3d(1.0, 1.0, 1.20);

  double terminalPositionTolerance_ = 0.004;
  double terminalVelocityTolerance_ = 0.010;
  double terminalStableDuration_ = 0.150;
  double terminalTimeout_ = 4.0;
  double velocityFilterAlpha_ = 0.10;
};
