#pragma once

#include <mc_rtc/Configuration.h>
#include <mc_tasks/TransformTask.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <string>

struct HandoverInterceptionController;

/**
 * Deterministic Robot-B trajectory presenter integrated inside Robot A's
 * receiver controller. Robot B has no gripper role and no transfer logic.
 * All scenarios are defined once in the Robot-A/world frame. Robot B only
 * prepositions its tool to the selected world start, passes a measured
 * pose/velocity gate, executes the world-frame start-to-end trajectory once,
 * and holds the exact world endpoint. Robot-B base placement affects only the
 * inverse-kinematics solution and local workspace checks.
 */
struct DualGiverCoordinator
{
  enum class Phase
  {
    DisabledHold,
    Prepositioning,
    StartSettling,
    Ready,
    Executing,
    TerminalSettling,
    Holding,
    Failed
  };

  DualGiverCoordinator(HandoverInterceptionController & ctl,
                       const mc_rtc::Configuration & config);

  void reset(HandoverInterceptionController & ctl);
  void run(HandoverInterceptionController & ctl);
  bool ready() const { return phase_ == Phase::Ready; }
  bool failed() const { return phase_ == Phase::Failed; }
  bool enabled() const { return enabled_; }
  bool motionEnabled() const { return motionEnabled_; }
  bool requestPresentation(HandoverInterceptionController & ctl);
  void safeHold(HandoverInterceptionController & ctl);

  bool presentationScheduleAvailable() const;
  double timeToPresentation() const;
  sva::PTransformd presentationObjectPoseWorld() const;

  const std::string & scenarioName() const { return scenarioName_; }
  const std::string & giverRobotName() const { return giverRobotName_; }
  double trackingError() const;
  double measuredSpeed() const { return filteredLinearVelocity_.norm(); }
  int phaseCode() const { return static_cast<int>(phase_); }

private:
  static Eigen::Vector3d readVector3(const mc_rtc::Configuration & cfg,
                                     const std::string & key,
                                     const Eigen::Vector3d & fallback);
  static double clamp01(double x);
  static double smoothStep(double u);
  static double smoothStepVelocity(double u);
  static double smoothStepAcceleration(double u);
  static double stopIntegral(double u);
  static double stopVelocityScale(double u);
  static double stopAccelerationScale(double u);
  static double rotationAngle(const Eigen::Matrix3d & R);

  void loadConfig(const mc_rtc::Configuration & config);
  void activateTask(HandoverInterceptionController & ctl);
  void commandPose(HandoverInterceptionController & ctl,
                   const sva::PTransformd & target,
                   const Eigen::Vector3d & linearVelocityWorld = Eigen::Vector3d::Zero(),
                   const Eigen::Vector3d & linearAccelerationWorld = Eigen::Vector3d::Zero());
  sva::PTransformd actualToolPose(const HandoverInterceptionController & ctl) const;
  sva::PTransformd giverBasePose(const HandoverInterceptionController & ctl) const;
  static Eigen::Matrix3d rpyToRotation(const Eigen::Vector3d & rpy);
  sva::PTransformd scenarioObjectStartWorld(
      const HandoverInterceptionController & ctl) const;
  sva::PTransformd scenarioObjectEndWorld(
      const HandoverInterceptionController & ctl) const;
  sva::PTransformd toolPoseForObjectPose(
      const HandoverInterceptionController & ctl,
      const sva::PTransformd & objectPoseWorld) const;
  sva::PTransformd objectPoseFromToolPose(
      const HandoverInterceptionController & ctl,
      const sva::PTransformd & toolPoseWorld) const;
  void updateCarriedObjectPose(HandoverInterceptionController & ctl);
  Eigen::Vector3d velocityWorld() const;
  void updateMeasuredMotion(HandoverInterceptionController & ctl);
  void fail(HandoverInterceptionController & ctl, const std::string & reason);
  bool localTargetInsideWorkspace(const HandoverInterceptionController & ctl,
                                  const sva::PTransformd & targetWorld) const;

private:
  bool enabled_ = false;
  bool motionEnabled_ = false;
  bool autoStartWithObservation_ = true;
  bool allowUnvalidatedScenario_ = false;
  bool referenceOnlyPreview_ = false;
  bool carriedObjectEnabled_ = false;
  bool releaseObjectOnReceiverAttach_ = true;
  // Ticker-safe visual coupling: the object orientation remains fixed in the
  // Robot-A/world frame while its grey outer tip follows Robot-B tool position.
  // Robot-B tool orientation is not imposed on the object.
  bool positionCoupledWorldOrientation_ = false;
  // Ticker-only deterministic object propagation. When enabled, the visual
  // object follows the commanded world-frame scenario reference during the
  // execution/terminal phases rather than Robot-B tracking transients. Robot B
  // still follows the grey outer-tip task. Keep false for physical execution.
  bool referenceCoupledObject_ = false;

  std::string giverRobotName_ = "kinova";
  std::string giverToolFrame_ = "tool_frame";
  std::string giverBaseFrame_ = "base_link";
  std::string scenarioName_ = "canonical_yz";
  std::string carriedObjectRobotName_ = "call_object";
  std::string carriedObjectFrameName_ = "call_object";

  Eigen::Vector3d objectOrientationRPYWorld_ =
      Eigen::Vector3d(0.0, 1.57079632679, 0.0);
  Eigen::Vector3d objectToGiverToolTranslation_ =
      Eigen::Vector3d(0.0, 0.0, 0.1556);
  Eigen::Vector3d objectToGiverToolRPY_ = Eigen::Vector3d::Zero();

  Eigen::Vector3d objectStartWorld_ = Eigen::Vector3d(0.55, -0.56, 0.15);
  Eigen::Vector3d objectEndWorld_ = Eigen::Vector3d(0.55, -0.172, 0.15);
  Eigen::Vector3d objectVelocityWorld_ = Eigen::Vector3d(0.0, 0.08, 0.0);
  Eigen::Vector3d workspaceMinLocal_ = Eigen::Vector3d(-1.0, -1.0, 0.08);
  Eigen::Vector3d workspaceMaxLocal_ = Eigen::Vector3d(1.0, 1.0, 1.20);

  double constantVelocityDuration_ = 4.425;
  double decelerationDuration_ = 0.85;
  double maximumTravel_ = 0.50;
  double maximumLinearSpeed_ = 0.10;
  double maximumTrackingError_ = 0.06;
  double trackingErrorGrace_ = 0.40;
  bool enforceTrackingError_ = true;

  double prepositionMaximumLinearSpeed_ = 0.08;
  double prepositionMinimumDuration_ = 1.0;
  double prepositionTimeout_ = 20.0;
  double prepositionPositionTolerance_ = 0.004;
  double prepositionVelocityTolerance_ = 0.010;
  double prepositionStableDuration_ = 0.200;

  double terminalPositionTolerance_ = 0.004;
  double terminalVelocityTolerance_ = 0.010;
  double terminalStableDuration_ = 0.150;
  double terminalTimeout_ = 4.0;
  double velocityFilterAlpha_ = 0.10;

  double taskStiffness_ = 50.0;
  double taskWeight_ = 6500.0;

  std::shared_ptr<mc_tasks::TransformTask> task_;
  bool taskActive_ = false;
  Phase phase_ = Phase::DisabledHold;

  sva::PTransformd holdPose_ = sva::PTransformd::Identity();
  sva::PTransformd startObjectPoseWorld_ = sva::PTransformd::Identity();
  sva::PTransformd endObjectPoseWorld_ = sva::PTransformd::Identity();
  sva::PTransformd targetObjectPoseWorld_ = sva::PTransformd::Identity();
  sva::PTransformd O_T_G_ = sva::PTransformd::Identity();
  sva::PTransformd G_T_O_ = sva::PTransformd::Identity();
  sva::PTransformd startPoseWorld_ = sva::PTransformd::Identity();
  sva::PTransformd prepositionStartPose_ = sva::PTransformd::Identity();
  sva::PTransformd targetPose_ = sva::PTransformd::Identity();

  double prepositionElapsed_ = 0.0;
  double prepositionDuration_ = 0.0;
  double settleElapsed_ = 0.0;
  double stableTime_ = 0.0;
  double executionElapsed_ = 0.0;
  double maximumObservedTrackingError_ = 0.0;

  bool havePreviousPose_ = false;
  Eigen::Vector3d previousPosition_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d previousRotation_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d filteredLinearVelocity_ = Eigen::Vector3d::Zero();
  double filteredAngularSpeed_ = 0.0;
  bool presentationRequested_ = false;
  bool objectReleasedToReceiver_ = false;
  bool objectPoseInitialized_ = false;
  unsigned long long iter_ = 0;
};
