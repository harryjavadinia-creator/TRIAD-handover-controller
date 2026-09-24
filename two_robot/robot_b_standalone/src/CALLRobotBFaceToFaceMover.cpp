#include "CALLRobotBFaceToFaceMover.h"

#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <vector>

CALLRobotBFaceToFaceMover::CALLRobotBFaceToFaceMover(
    mc_rbdyn::RobotModulePtr rm,
    double dt,
    const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config), controlDt_(dt)
{
  loadConfig(config);

  if(!robot().hasFrame(toolFrame_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[RobotB] missing configured tool frame {}", toolFrame_);
  }

  toolTask_ = std::make_shared<mc_tasks::TransformTask>(
      robot().frame(toolFrame_), taskStiffness_, taskWeight_);
  commandedPose_ = actualToolPose();
  toolTask_->target(commandedPose_);
  clearToolTaskReferenceMotion();

  logger().addLogEntry("robot_b_target_x", [this]() {
    return commandedPose_.translation().x();
  });
  logger().addLogEntry("robot_b_target_y", [this]() {
    return commandedPose_.translation().y();
  });
  logger().addLogEntry("robot_b_target_z", [this]() {
    return commandedPose_.translation().z();
  });
  logger().addLogEntry("robot_b_tracking_error", [this]() {
    return toolTask_ ? toolTask_->eval().norm() : -1.0;
  });

  const Eigen::Vector3d vB = robotBVelocity();
  mc_rtc::log::warning(
      "[RobotB] deterministic scenario mover loaded scenario={} "
      "motionEnabled={} predefinedStartConfigured={} "
      "vA=[{:.5f},{:.5f},{:.5f}] vB=[{:.5f},{:.5f},{:.5f}] "
      "constant={:.3f}s decel={:.3f}s task=[stiffness:{:.1f},weight:{:.1f}] "
      "preposition=[v:{:.3f}m/s,w:{:.3f}rad/s,pos:{:.4f}m,ori:{:.4f}rad] "
      "terminal=[pos:{:.4f}m,vel:{:.4f}m/s,dwell:{:.3f}s,timeout:{:.3f}s]. "
      "No communication with Robot A.",
      scenarioName_, motionEnabled_, startPoseConfigured_,
      robotAVelocity_.x(), robotAVelocity_.y(), robotAVelocity_.z(),
      vB.x(), vB.y(), vB.z(),
      constantVelocityDuration_, decelerationDuration_,
      taskStiffness_, taskWeight_, prepositionMaximumLinearSpeed_,
      prepositionMaximumAngularSpeed_, prepositionPositionTolerance_,
      prepositionOrientationTolerance_, terminalPositionTolerance_,
      terminalVelocityTolerance_, terminalStableDuration_, terminalTimeout_);
}

void CALLRobotBFaceToFaceMover::reset(
    const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
  toolTaskActive_ = false;
  commandedPose_ = actualToolPose();
  clearToolTaskReferenceMotion();
  clearTrigger();
}

bool CALLRobotBFaceToFaceMover::run()
{
  return mc_control::fsm::Controller::run();
}

void CALLRobotBFaceToFaceMover::activateToolTask()
{
  if(toolTask_ && !toolTaskActive_)
  {
    solver().addTask(toolTask_);
    toolTaskActive_ = true;
    mc_rtc::log::success("[RobotB] Cartesian tool task activated frame={}", toolFrame_);
  }
}

void CALLRobotBFaceToFaceMover::clearToolTaskReferenceMotion()
{
  if(!toolTask_) { return; }
  const sva::MotionVecd zeroMotion(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  toolTask_->refVelB(zeroMotion);
  toolTask_->refAccel(zeroMotion);
}

void CALLRobotBFaceToFaceMover::commandToolPose(
    const sva::PTransformd & target)
{
  commandedPose_ = target;
  if(toolTask_)
  {
    toolTask_->target(commandedPose_);
    clearToolTaskReferenceMotion();
  }
}

void CALLRobotBFaceToFaceMover::commandToolPoseWithWorldMotion(
    const sva::PTransformd & target,
    const Eigen::Vector3d & linearVelocityWorld,
    const Eigen::Vector3d & linearAccelerationWorld)
{
  commandedPose_ = target;
  if(!toolTask_) { return; }

  toolTask_->target(commandedPose_);

  const Eigen::Matrix3d R_W_T = actualToolPose().rotation().transpose();
  const sva::MotionVecd bodyVelocity(
      Eigen::Vector3d::Zero(),
      R_W_T.transpose() * linearVelocityWorld);
  const sva::MotionVecd bodyAcceleration(
      Eigen::Vector3d::Zero(),
      R_W_T.transpose() * linearAccelerationWorld);

  toolTask_->refVelB(bodyVelocity);
  toolTask_->refAccel(bodyAcceleration);
}

sva::PTransformd CALLRobotBFaceToFaceMover::actualToolPose() const
{
  return robot().frame(toolFrame_).position();
}

Eigen::Vector3d CALLRobotBFaceToFaceMover::robotBVelocity() const
{
  if(!faceToFaceYaw180_) { return robotAVelocity_; }
  return Eigen::Vector3d(
      -robotAVelocity_.x(), -robotAVelocity_.y(), robotAVelocity_.z());
}

bool CALLRobotBFaceToFaceMover::triggerPresent() const
{
  std::ifstream in(triggerFile_);
  return in.good();
}

void CALLRobotBFaceToFaceMover::clearTrigger() const
{
  std::remove(triggerFile_.c_str());
}

bool CALLRobotBFaceToFaceMover::targetInsideWorkspace(
    const Eigen::Vector3d & p) const
{
  return (p.array() >= workspaceMin_.array()).all()
      && (p.array() <= workspaceMax_.array()).all();
}

bool CALLRobotBFaceToFaceMover::configuredTrajectorySafe(
    std::string & reason) const
{
  if(!startPoseConfigured_)
  {
    reason = "scenario_start_pose_not_configured";
    return false;
  }

  const Eigen::Vector3d vB = robotBVelocity();
  if(!vB.allFinite()
     || !scenarioStartPose_.translation().allFinite()
     || !scenarioStartPose_.rotation().allFinite())
  {
    reason = "non_finite_configuration";
    return false;
  }
  if(constantVelocityDuration_ < 0.0 || decelerationDuration_ < 0.0)
  {
    reason = "negative_duration";
    return false;
  }
  if(vB.norm() > maximumLinearSpeed_ + 1e-9)
  {
    reason = "velocity_above_limit";
    return false;
  }
  const double effectiveTime = constantVelocityDuration_
      + 0.5 * decelerationDuration_;
  const double travel = vB.norm() * effectiveTime;
  if(travel > maximumTravel_ + 1e-9)
  {
    reason = "travel_above_limit";
    return false;
  }

  const Eigen::Vector3d finalPosition =
      scenarioStartPose_.translation() + vB * effectiveTime;
  if(!targetInsideWorkspace(scenarioStartPose_.translation())
     || !targetInsideWorkspace(finalPosition))
  {
    reason = "scenario_start_or_final_outside_workspace";
    return false;
  }

  const Eigen::Matrix3d RtR = scenarioStartPose_.rotation().transpose()
      * scenarioStartPose_.rotation();
  if((RtR - Eigen::Matrix3d::Identity()).norm() > 1e-3
     || scenarioStartPose_.rotation().determinant() < 0.99)
  {
    reason = "scenario_start_rotation_not_orthonormal";
    return false;
  }

  reason = "safe";
  return true;
}

double CALLRobotBFaceToFaceMover::smoothStep(double u)
{
  u = std::max(0.0, std::min(1.0, u));
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double u5 = u4 * u;
  return 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
}

double CALLRobotBFaceToFaceMover::smoothStepVelocity(double u)
{
  u = std::max(0.0, std::min(1.0, u));
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  return 30.0 * u2 - 60.0 * u3 + 30.0 * u4;
}

double CALLRobotBFaceToFaceMover::smoothStepAcceleration(double u)
{
  u = std::max(0.0, std::min(1.0, u));
  const double u2 = u * u;
  const double u3 = u2 * u;
  return 60.0 * u - 180.0 * u2 + 120.0 * u3;
}

double CALLRobotBFaceToFaceMover::rotationAngle(const Eigen::Matrix3d & R)
{
  const double c = std::max(-1.0, std::min(1.0, 0.5 * (R.trace() - 1.0)));
  return std::acos(c);
}

double CALLRobotBFaceToFaceMover::decelerationIntegral(double u)
{
  u = std::max(0.0, std::min(1.0, u));
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double u5 = u4 * u;
  const double u6 = u5 * u;
  return u - 2.5 * u4 + 3.0 * u5 - u6;
}

double CALLRobotBFaceToFaceMover::decelerationVelocityScale(double u)
{
  u = std::max(0.0, std::min(1.0, u));
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double u5 = u4 * u;
  return 1.0 - 10.0 * u3 + 15.0 * u4 - 6.0 * u5;
}

double CALLRobotBFaceToFaceMover::decelerationAccelerationScale(double u)
{
  u = std::max(0.0, std::min(1.0, u));
  const double oneMinusU = 1.0 - u;
  return -30.0 * u * u * oneMinusU * oneMinusU;
}

Eigen::Vector3d CALLRobotBFaceToFaceMover::readVector3(
    const mc_rtc::Configuration & config,
    const std::string & key,
    const Eigen::Vector3d & fallback)
{
  if(!config.has(key)) { return fallback; }
  std::vector<double> v;
  config(key, v);
  if(v.size() != 3) { return fallback; }
  return Eigen::Vector3d(v[0], v[1], v[2]);
}

Eigen::Matrix3d CALLRobotBFaceToFaceMover::readRotation9(
    const mc_rtc::Configuration & config,
    const std::string & key,
    const Eigen::Matrix3d & fallback)
{
  if(!config.has(key)) { return fallback; }
  std::vector<double> v;
  config(key, v);
  if(v.size() != 9) { return fallback; }
  Eigen::Matrix3d R;
  R << v[0], v[1], v[2],
       v[3], v[4], v[5],
       v[6], v[7], v[8];
  return R;
}

void CALLRobotBFaceToFaceMover::loadConfig(
    const mc_rtc::Configuration & config)
{
  if(config.has("toolFrame")) { config("toolFrame", toolFrame_); }

  if(config.has("task"))
  {
    auto task = config("task");
    if(task.has("stiffness")) { task("stiffness", taskStiffness_); }
    if(task.has("weight")) { task("weight", taskWeight_); }
  }

  if(config.has("trajectory"))
  {
    auto tr = config("trajectory");
    if(tr.has("scenario")) { tr("scenario", scenarioName_); }
    if(tr.has("faceToFaceYaw180"))
    {
      tr("faceToFaceYaw180", faceToFaceYaw180_);
    }
    robotAVelocity_ = readVector3(tr, "robotAVelocity", robotAVelocity_);
    if(tr.has("constantVelocityDuration"))
    {
      tr("constantVelocityDuration", constantVelocityDuration_);
    }
    if(tr.has("decelerationDuration"))
    {
      tr("decelerationDuration", decelerationDuration_);
    }
    if(tr.has("maximumTravel"))
    {
      tr("maximumTravel", maximumTravel_);
    }
  }

  if(config.has("startPose"))
  {
    auto start = config("startPose");
    if(start.has("configured")) { start("configured", startPoseConfigured_); }
    const Eigen::Vector3d t = readVector3(
        start, "translation", scenarioStartPose_.translation());
    const Eigen::Matrix3d R = readRotation9(
        start, "rotation", scenarioStartPose_.rotation());
    scenarioStartPose_ = sva::PTransformd(R, t);
  }

  if(config.has("preposition"))
  {
    auto pre = config("preposition");
    if(pre.has("maximumLinearSpeed"))
    {
      pre("maximumLinearSpeed", prepositionMaximumLinearSpeed_);
    }
    if(pre.has("maximumAngularSpeed"))
    {
      pre("maximumAngularSpeed", prepositionMaximumAngularSpeed_);
    }
    if(pre.has("minimumDuration"))
    {
      pre("minimumDuration", prepositionMinimumDuration_);
    }
    if(pre.has("timeout")) { pre("timeout", prepositionTimeout_); }
    if(pre.has("positionTolerance"))
    {
      pre("positionTolerance", prepositionPositionTolerance_);
    }
    if(pre.has("orientationTolerance"))
    {
      pre("orientationTolerance", prepositionOrientationTolerance_);
    }
    if(pre.has("linearVelocityTolerance"))
    {
      pre("linearVelocityTolerance", prepositionLinearVelocityTolerance_);
    }
    if(pre.has("angularVelocityTolerance"))
    {
      pre("angularVelocityTolerance", prepositionAngularVelocityTolerance_);
    }
    if(pre.has("stableDuration"))
    {
      pre("stableDuration", prepositionStableDuration_);
    }
  }

  if(config.has("manual"))
  {
    auto manual = config("manual");
    if(manual.has("triggerFile")) { manual("triggerFile", triggerFile_); }
  }

  if(config.has("safety"))
  {
    auto safety = config("safety");
    if(safety.has("motionEnabled"))
    {
      safety("motionEnabled", motionEnabled_);
    }
    if(safety.has("maximumLinearSpeed"))
    {
      safety("maximumLinearSpeed", maximumLinearSpeed_);
    }
    if(safety.has("maximumTrackingError"))
    {
      safety("maximumTrackingError", maximumTrackingError_);
    }
    if(safety.has("trackingErrorGrace"))
    {
      safety("trackingErrorGrace", trackingErrorGrace_);
    }
    workspaceMin_ = readVector3(safety, "workspaceMin", workspaceMin_);
    workspaceMax_ = readVector3(safety, "workspaceMax", workspaceMax_);
  }

  if(config.has("terminal"))
  {
    auto terminal = config("terminal");
    if(terminal.has("positionTolerance"))
    {
      terminal("positionTolerance", terminalPositionTolerance_);
    }
    if(terminal.has("velocityTolerance"))
    {
      terminal("velocityTolerance", terminalVelocityTolerance_);
    }
    if(terminal.has("stableDuration"))
    {
      terminal("stableDuration", terminalStableDuration_);
    }
    if(terminal.has("timeout"))
    {
      terminal("timeout", terminalTimeout_);
    }
    if(terminal.has("velocityFilterAlpha"))
    {
      terminal("velocityFilterAlpha", velocityFilterAlpha_);
    }
  }

  taskStiffness_ = std::max(0.1, taskStiffness_);
  taskWeight_ = std::max(1.0, taskWeight_);
  constantVelocityDuration_ = std::max(0.0, constantVelocityDuration_);
  decelerationDuration_ = std::max(0.0, decelerationDuration_);
  maximumTravel_ = std::max(0.0, maximumTravel_);
  prepositionMaximumLinearSpeed_ = std::max(0.005, prepositionMaximumLinearSpeed_);
  prepositionMaximumAngularSpeed_ = std::max(0.05, prepositionMaximumAngularSpeed_);
  prepositionMinimumDuration_ = std::max(0.2, prepositionMinimumDuration_);
  prepositionTimeout_ = std::max(prepositionMinimumDuration_ + 1.0, prepositionTimeout_);
  prepositionPositionTolerance_ = std::max(0.0, prepositionPositionTolerance_);
  prepositionOrientationTolerance_ = std::max(0.0, prepositionOrientationTolerance_);
  prepositionLinearVelocityTolerance_ = std::max(0.0, prepositionLinearVelocityTolerance_);
  prepositionAngularVelocityTolerance_ = std::max(0.0, prepositionAngularVelocityTolerance_);
  prepositionStableDuration_ = std::max(0.0, prepositionStableDuration_);
  maximumLinearSpeed_ = std::max(0.0, maximumLinearSpeed_);
  maximumTrackingError_ = std::max(0.0, maximumTrackingError_);
  trackingErrorGrace_ = std::max(0.0, trackingErrorGrace_);
  terminalPositionTolerance_ = std::max(0.0, terminalPositionTolerance_);
  terminalVelocityTolerance_ = std::max(0.0, terminalVelocityTolerance_);
  terminalStableDuration_ = std::max(0.0, terminalStableDuration_);
  terminalTimeout_ = std::max(0.1, terminalTimeout_);
  velocityFilterAlpha_ = std::max(0.001, std::min(1.0, velocityFilterAlpha_));
}
