#include "HandoverInterceptionController.h"
#include "FiniteEventPlanSelector.h"
#include "FinitePlanSelector.h"

#include <mc_control/mc_controller.h>
#include <mc_rbdyn/ForceSensor.h>
#include <mc_rtc/config.h>
#include <mc_rtc/gui/Point3D.h>
#include <mc_rtc/gui/Transform.h>
#include <mc_rtc/logging.h>

#include <RBDyn/FK.h>
#include <RBDyn/Jacobian.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/NumericalIntegration.h>
#include <Tasks/QPTasks.h>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
constexpr double PI = 3.14159265358979323846;

inline double clampUnit(double x)
{
  return std::max(-1.0, std::min(1.0, x));
}

inline double clamp01(double x)
{
  return std::max(0.0, std::min(1.0, x));
}

inline double quinticStep(double x)
{
  x = clamp01(x);
  return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

inline double cubicStep(double x)
{
  x = clamp01(x);
  return x * x * (3.0 - 2.0 * x);
}

// A more responsive reach law than a pure quintic while retaining zero
// endpoint velocity. The copied-state preview and physical runtime both use
// this exact profile, so speed is increased without breaking the plan/runtime
// contract.
inline double naturalReachStep(double x)
{
  return 0.65 * cubicStep(x) + 0.35 * quinticStep(x);
}

// Integral from 0 to x of (1 - quinticStep(s)). This is the effective
// constant-twist travel time of a C2-smooth deceleration from unit speed to
// zero. At x=1 the integral is exactly 0.5.
inline double quinticStopIntegral(double x)
{
  x = clamp01(x);
  const double x2 = x * x;
  const double x4 = x2 * x2;
  return x - 2.5 * x4 + 3.0 * x4 * x - x4 * x2;
}
} // namespace

HandoverInterceptionController::HandoverInterceptionController(
    mc_rbdyn::RobotModulePtr rm,
    double dt,
    const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config), controlDt_(dt)
{
  loadHandoverConfig(config);

  mc_rtc::log::info(
      "[HandoverInterception] physical task frame B={} object={}/{} dt={:.6f}",
      toolFrame_, objectRobotName_, objectFrameName_, controlDt_);

  toolTask_ = std::make_shared<mc_tasks::TransformTask>(
      robot().frame(toolFrame_), taskStiffness_, taskWeight_);
  toolTaskActive_ = false;

  refreshObjectPose();
  W_T_O_predicted_ = W_T_O_;
  addMethodologyGui();

  logger().addLogEntry("handover_object_x", [this]() { return W_T_O_.translation().x(); });
  logger().addLogEntry("handover_object_y", [this]() { return W_T_O_.translation().y(); });
  logger().addLogEntry("handover_object_z", [this]() { return W_T_O_.translation().z(); });
  logger().addLogEntry("handover_blue_handle_x", [this]() { return W_T_H_.translation().x(); });
  logger().addLogEntry("handover_blue_handle_y", [this]() { return W_T_H_.translation().y(); });
  logger().addLogEntry("handover_blue_handle_z", [this]() { return W_T_H_.translation().z(); });
  logger().addLogEntry("handover_mouth_x", [this]() { return actualMouthPose().translation().x(); });
  logger().addLogEntry("handover_mouth_y", [this]() { return actualMouthPose().translation().y(); });
  logger().addLogEntry("handover_mouth_z", [this]() { return actualMouthPose().translation().z(); });
  logger().addLogEntry("handover_mouth_gap", [this]() { return liveMouthGap(); });
  logger().addLogEntry("handover_gripper_command", [this]() { return gripperCommand_; });
  logger().addLogEntry("handover_physical_gripper_bridge_enabled", [this]() {
    return physicalGripperBridgeEnabled_ ? 1.0 : 0.0;
  });
  logger().addLogEntry("handover_physical_gripper_command_enabled", [this]() {
    return physicalGripperCommandEnabled_ ? 1.0 : 0.0;
  });
  logger().addLogEntry("handover_physical_gripper_feedback_valid", [this]() {
    return physicalGripperFeedbackValid_ ? 1.0 : 0.0;
  });
  logger().addLogEntry("handover_physical_gripper_measured_percent", [this]() {
    return physicalGripperMeasuredPercent_;
  });
  logger().addLogEntry("handover_gripper_actuated", [this]() {
    return gripperActuationAvailable() ? 1.0 : 0.0;
  });
  logger().addLogEntry("handover_object_attached", [this]() {
    return objectAttached_ ? 1.0 : 0.0;
  });
  logger().addLogEntry("handover_object_observation_active", [this]() {
    return objectObservationActive_ ? 1.0 : 0.0;
  });
  logger().addLogEntry("handover_observed_object_mode", [this]() {
    switch(observedObjectMode_)
    {
      case ObservedObjectMode::Static: return 1.0;
      case ObservedObjectMode::Moving: return 2.0;
      default: return 0.0;
    }
  });
  logger().addLogEntry("handover_object_velocity_x", [this]() {
    return objectLinearVelocityEstimate_.x();
  });
  logger().addLogEntry("handover_object_velocity_y", [this]() {
    return objectLinearVelocityEstimate_.y();
  });
  logger().addLogEntry("handover_object_velocity_z", [this]() {
    return objectLinearVelocityEstimate_.z();
  });
  logger().addLogEntry("handover_object_angular_velocity_x", [this]() {
    return objectAngularVelocityEstimate_.x();
  });
  logger().addLogEntry("handover_object_angular_velocity_y", [this]() {
    return objectAngularVelocityEstimate_.y();
  });
  logger().addLogEntry("handover_object_angular_velocity_z", [this]() {
    return objectAngularVelocityEstimate_.z();
  });
  logger().addLogEntry("handover_predicted_object_x", [this]() {
    return W_T_O_predicted_.translation().x();
  });
  logger().addLogEntry("handover_predicted_object_y", [this]() {
    return W_T_O_predicted_.translation().y();
  });
  logger().addLogEntry("handover_predicted_object_z", [this]() {
    return W_T_O_predicted_.translation().z();
  });
  logger().addLogEntry("handover_perception_latency_enabled", [this]() {
    return perceptionLatencyEnabled() ? 1.0 : 0.0;
  });
  logger().addLogEntry("handover_perception_latency_compensated", [this]() {
    return perceptionLatencyCompensationEnabled() ? 1.0 : 0.0;
  });
  logger().addLogEntry("handover_perception_configured_delay", [this]() {
    return perceptionLatencySeconds();
  });
  logger().addLogEntry("handover_perception_measurement_age", [this]() {
    return objectPerceptionMeasurementAge_;
  });
  logger().addLogEntry("handover_perception_raw_position_error", [this]() {
    return objectPerceptionRawPositionError();
  });
  logger().addLogEntry("handover_perception_estimate_position_error", [this]() {
    return objectPerceptionEstimatePositionError();
  });
  logger().addLogEntry("handover_capture_depth", [this]() { return captureDepth_; });
  logger().addLogEntry("handover_candidate_clearance", [this]() { return selectedCandidateClearance_; });
  logger().addLogEntry("handover_candidate_score", [this]() { return selectedCandidateScore_; });
  logger().addLogEntry("handover_predicted_execution_time", [this]() {
    return selectedCandidatePredictedTime_;
  });
  logger().addLogEntry("handover_predicted_contact_time", [this]() {
    return selectedCandidatePredictedContactTime_;
  });
  logger().addLogEntry("handover_predicted_reach_time", [this]() {
    return selectedCandidatePredictedReachTime_;
  });
  logger().addLogEntry("handover_predicted_approach_time", [this]() {
    return selectedCandidatePredictedApproachTime_;
  });
  logger().addLogEntry("handover_predicted_acquire_time", [this]() {
    return selectedCandidatePredictedAcquireTime_;
  });
  logger().addLogEntry("handover_predicted_transfer_time", [this]() {
    return committedPlanValid()
        ? committedInterceptionPlan_.forceTransferDuration : 0.0;
  });
  logger().addLogEntry("handover_predicted_complete_handover_time", [this]() {
    return selectedCandidatePredictedTime_
         + (committedPlanValid()
            ? committedInterceptionPlan_.forceTransferDuration : 0.0);
  });
  logger().addLogEntry("handover_interception_committed", [this]() {
    return interceptionCommitted_ ? 1.0 : 0.0;
  });
  logger().addLogEntry("handover_time_to_committed_contact", [this]() {
    return timeToCommittedContact();
  });
  logger().addLogEntry("handover_committed_timing_residual", [this]() {
    return committedTimingResidual_;
  });
  logger().addLogEntry("handover_measured_execution_time", [this]() {
    return phaseDuration("reach") + phaseDuration("approach")
         + phaseDuration("acquire") + phaseDuration("transfer")
         + phaseDuration("retreat");
  });
  logger().addLogEntry("handover_to_transit_error", [this]() {
    return (W_T_M_transit_.translation() - actualMouthPose().translation()).norm();
  });
  logger().addLogEntry("handover_to_standoff_error", [this]() {
    return (W_T_M_standoff_.translation() - actualMouthPose().translation()).norm();
  });
  logger().addLogEntry("handover_to_pregrasp_error", [this]() {
    return (W_T_M_pre_.translation() - actualMouthPose().translation()).norm();
  });
  logger().addLogEntry("handover_to_retreat_error", [this]() {
    return (W_T_M_retreat_.translation() - actualMouthPose().translation()).norm();
  });
  logger().addLogEntry("handover_force_transfer_valid", [this]() {
    return forceTransferMeasurement_.valid ? 1.0 : 0.0;
  });
  logger().addLogEntry("handover_force_transfer_synthetic", [this]() {
    return forceTransferMeasurement_.synthetic ? 1.0 : 0.0;
  });
  logger().addLogEntry("handover_force_transfer_simulated_physics", [this]() {
    return forceTransferMeasurement_.simulatedPhysics ? 1.0 : 0.0;
  });
  logger().addLogEntry("handover_force_world_x", [this]() {
    return forceTransferMeasurement_.forceWorld.x();
  });
  logger().addLogEntry("handover_force_world_y", [this]() {
    return forceTransferMeasurement_.forceWorld.y();
  });
  logger().addLogEntry("handover_force_world_z", [this]() {
    return forceTransferMeasurement_.forceWorld.z();
  });
  logger().addLogEntry("handover_moment_world_x", [this]() {
    return forceTransferMeasurement_.coupleWorld.x();
  });
  logger().addLogEntry("handover_moment_world_y", [this]() {
    return forceTransferMeasurement_.coupleWorld.y();
  });
  logger().addLogEntry("handover_moment_world_z", [this]() {
    return forceTransferMeasurement_.coupleWorld.z();
  });
  logger().addLogEntry("handover_transfer_load_force", [this]() {
    return forceTransferMeasurement_.loadForce;
  });
  logger().addLogEntry("handover_transfer_lateral_force", [this]() {
    return forceTransferMeasurement_.lateralForce;
  });
  logger().addLogEntry("handover_transfer_moment_norm", [this]() {
    return forceTransferMeasurement_.momentNorm;
  });
  logger().addLogEntry("handover_transfer_index", [this]() {
    return forceTransferMeasurement_.transferIndex;
  });
  logger().addLogEntry("handover_transfer_rate", [this]() {
    return forceTransferMeasurement_.transferRate;
  });
  logger().addLogEntry("handover_virtual_human_support_force", [this]() {
    return forceTransferMeasurement_.humanSupportForce;
  });
  logger().addLogEntry("handover_virtual_contact_deflection", [this]() {
    return forceTransferMeasurement_.virtualContactDeflection;
  });

  mc_rtc::log::warning(
      "[HandoverInterception] Cartesian task created but inactive. Initial uses joint posture and opens the gripper.");
}

void HandoverInterceptionController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
  detachObject();
  invalidateSelectedCandidate();
  mouthCalibrationValid_ = false;
  gripperGeometryValid_ = false;
  gripperCommand_ = 0.0;
  physicalGripperFeedbackValid_ = false;
  physicalGripperMeasuredPercent_ = 0.0;
  physicalGripperMeasuredVelocityPercent_ = 0.0;
  physicalGripperFeedbackSequence_ = 0;
  physicalGripperFeedbackWarningLogged_ = false;
  gripperClosureAuthorized_ = false;
  gripperAuthorityViolationLogged_ = false;
  controllerTime_ = 0.0;
  objectObservationActive_ = false;
  simulatedObjectMotionActive_ = false;
  simulatedObjectPoseFrozen_ = false;
  havePreviousObjectObservation_ = false;
  objectMotionEstimateValid_ = false;
  observedObjectMode_ = ObservedObjectMode::Unclassified;
  objectObservationSamples_ = 0;
  resetObjectPerceptionBuffer();
  objectLinearVelocityEstimate_.setZero();
  objectAngularVelocityEstimate_.setZero();
  planningObjectSnapshotActive_ = false;
  capturePlanningCommitOnSuccess_ = true;
  interceptionCommitted_ = false;
  committedInterceptionPlan_ = InterceptionPlan{};
  simulatedTruthInterceptionPlan_ = InterceptionPlan{};
  simulatedTruthInterceptionPlanValid_ = false;
  committedContactTime_ = 0.0;
  committedTimingResidual_ = 1e9;
  W_T_O_committedContact_ = sva::PTransformd::Identity();
  committedObjectLinearVelocity_.setZero();
  committedObjectAngularVelocity_.setZero();
  forceTransferMeasurement_ = ForceTransferMeasurement{};
  forceTransferBiasCalibrationActive_ = false;
  forceTransferBiasSamples_ = 0;
  forceTransferBiasForceSum_.setZero();
  forceTransferBiasCoupleSum_.setZero();
  forceTransferBiasForceWorld_.setZero();
  forceTransferBiasCoupleWorld_.setZero();
  forceTransferFilterInitialized_ = false;
  forceTransferFilteredForceWorld_.setZero();
  forceTransferFilteredCoupleWorld_.setZero();
  forceTransferExecutionActive_ = false;
  forceTransferExecutionStartTime_ = 0.0;
  forceTransferPreviousIndex_ = 0.0;
  virtualForceTransferDeflection_ = 0.0;
  virtualForceTransferVelocity_ = 0.0;
  virtualForceTransferBilateralContact_ = false;
  resetPhaseTiming();
  refreshObjectPose();
  W_T_O_predicted_ = W_T_O_;
}

bool HandoverInterceptionController::run()
{
  controllerTime_ += controlDt_;
  refreshPhysicalGripperBridge();
  if(!objectAttached_ && simulatedObjectMotionActive_)
  {
    updateSimulatedObjectMotion();
  }
  // Before transfer the object pose comes from the object robot/perception.
  // After confirmed bilateral acquisition the rigid mouth-object transform is
  // propagated kinematically so the simulator represents the carried object.
  if(objectAttached_) { updateAttachedObjectPose(); }
  else
  {
    refreshObjectPose();
    // Observation determines the committed model. After commitment, continue
    // estimating the *live* twist only for event validation and bounded
    // terminal correction. The committed candidate, contact time and nominal
    // twist remain immutable.
    if(objectObservationActive_ || interceptionCommitted_)
    {
      updateObjectMotionEstimate();
    }
  }
  updateForceTransferMeasurement();
  return mc_control::fsm::Controller::run();
}

void HandoverInterceptionController::beginForceTransferBiasCalibration()
{
  forceTransferBiasCalibrationActive_ = true;
  forceTransferBiasSamples_ = 0;
  forceTransferBiasForceSum_.setZero();
  forceTransferBiasCoupleSum_.setZero();
  forceTransferFilterInitialized_ = false;
  mc_rtc::log::info(
      "[ForceTransferCalibration] bias capture started source={}",
      forceTransferSourceDescription());
}

void HandoverInterceptionController::finishForceTransferBiasCalibration()
{
  if(!forceTransferBiasCalibrationActive_) { return; }
  forceTransferBiasCalibrationActive_ = false;
  if(forceTransferPolicy_.source == "force_sensor" && forceTransferBiasSamples_ > 0)
  {
    forceTransferBiasForceWorld_ = forceTransferBiasForceSum_
        / static_cast<double>(forceTransferBiasSamples_);
    forceTransferBiasCoupleWorld_ = forceTransferBiasCoupleSum_
        / static_cast<double>(forceTransferBiasSamples_);
    mc_rtc::log::success(
        "[ForceTransferCalibration] bias captured samples={} force=[{:.3f},{:.3f},{:.3f}]N moment=[{:.3f},{:.3f},{:.3f}]Nm",
        forceTransferBiasSamples_,
        forceTransferBiasForceWorld_.x(), forceTransferBiasForceWorld_.y(),
        forceTransferBiasForceWorld_.z(), forceTransferBiasCoupleWorld_.x(),
        forceTransferBiasCoupleWorld_.y(), forceTransferBiasCoupleWorld_.z());
  }
  else if(forceTransferPolicy_.source == "synthetic"
          || forceTransferPolicy_.source == "virtual_sensor")
  {
    forceTransferBiasForceWorld_.setZero();
    forceTransferBiasCoupleWorld_.setZero();
    mc_rtc::log::success(
        "[ForceTransferCalibration] simulation source={} uses exact zero bias",
        forceTransferPolicy_.source);
  }
  else
  {
    mc_rtc::log::warning(
        "[ForceTransferCalibration] no valid force-sensor samples captured source={} samples={}",
        forceTransferSourceDescription(), forceTransferBiasSamples_);
  }
  forceTransferFilterInitialized_ = false;
}

void HandoverInterceptionController::beginForceTransferExecution()
{
  forceTransferExecutionActive_ = true;
  forceTransferExecutionStartTime_ = controllerTime_;
  forceTransferPreviousIndex_ = forceTransferMeasurement_.transferIndex;
  virtualForceTransferDeflection_ = 0.0;
  virtualForceTransferVelocity_ = 0.0;
  virtualForceTransferBilateralContact_ = false;
  mc_rtc::log::success(
      "[ForceTransferExecution] started source={} fullSupport={:.3f}N releaseThreshold={:.3f}",
      forceTransferSourceDescription(), forceTransferPolicy_.fullSupportForce,
      forceTransferPolicy_.releaseThreshold);
}

void HandoverInterceptionController::finishForceTransferExecution()
{
  if(!forceTransferExecutionActive_) { return; }
  forceTransferExecutionActive_ = false;
  mc_rtc::log::info(
      "[ForceTransferExecution] stopped index={:.3f} rate={:+.3f}/s load={:.3f}N",
      forceTransferMeasurement_.transferIndex,
      forceTransferMeasurement_.transferRate,
      forceTransferMeasurement_.loadForce);
  virtualForceTransferDeflection_ = 0.0;
  virtualForceTransferVelocity_ = 0.0;
  virtualForceTransferBilateralContact_ = false;
}

void HandoverInterceptionController::setVirtualForceTransferState(
    double contactDeflection,
    double contactVelocity,
    bool bilateralContact)
{
  virtualForceTransferDeflection_ = std::max(0.0, contactDeflection);
  virtualForceTransferVelocity_ = contactVelocity;
  virtualForceTransferBilateralContact_ = bilateralContact;
}

std::string HandoverInterceptionController::forceTransferSourceDescription() const
{
  if(forceTransferPolicy_.source == "force_sensor")
  {
    const std::string robotName = forceTransferPolicy_.sourceRobot.empty()
        ? robot().name() : forceTransferPolicy_.sourceRobot;
    return "force_sensor:" + robotName + "/"
         + (forceTransferPolicy_.sensorName.empty()
            ? std::string("auto") : forceTransferPolicy_.sensorName);
  }
  return forceTransferPolicy_.source;
}

void HandoverInterceptionController::updateForceTransferMeasurement()
{
  ForceTransferMeasurement next;
  next.loadAxisWorld = worldRotation(W_T_O_) * forceTransferPolicy_.loadAxisObject;
  if(next.loadAxisWorld.norm() < 1e-9)
  {
    next.loadAxisWorld = Eigen::Vector3d::UnitZ();
  }
  next.loadAxisWorld.normalize();

  Eigen::Vector3d rawForce = Eigen::Vector3d::Zero();
  Eigen::Vector3d rawCouple = Eigen::Vector3d::Zero();

  if(forceTransferPolicy_.source == "virtual_sensor")
  {
    double virtualLoad = 0.0;
    const double virtualK = committedPlanValid()
        ? committedInterceptionPlan_.forceTransferVirtualContactStiffness
        : forceTransferPolicy_.virtualContactStiffness;
    const double virtualD = committedPlanValid()
        ? committedInterceptionPlan_.forceTransferVirtualContactDamping
        : forceTransferPolicy_.virtualContactDamping;
    const double virtualLoadScale = committedPlanValid()
        ? committedInterceptionPlan_.forceTransferVirtualMaximumLoadScale
        : forceTransferPolicy_.virtualMaximumLoadScale;
    if(forceTransferExecutionActive_ && virtualForceTransferBilateralContact_)
    {
      virtualLoad = virtualK * virtualForceTransferDeflection_
                  + virtualD * virtualForceTransferVelocity_;
    }
    const double maximumVirtualLoad = virtualLoadScale
                                    * forceTransferPolicy_.fullSupportForce;
    virtualLoad = std::max(0.0, std::min(maximumVirtualLoad, virtualLoad));
    rawForce = next.loadAxisWorld * virtualLoad;
    next.simulatedPhysics = true;
    next.valid = true;
    next.humanSupportForce = std::max(
        0.0, forceTransferPolicy_.fullSupportForce - virtualLoad);
    next.virtualContactDeflection = virtualForceTransferDeflection_;
  }
  else if(forceTransferPolicy_.source == "synthetic")
  {
    const double elapsed = forceTransferExecutionActive_
        ? std::max(0.0, controllerTime_ - forceTransferExecutionStartTime_) : 0.0;
    const double u = forceTransferExecutionActive_
        ? elapsed / forceTransferPolicy_.syntheticTransferDuration : 0.0;
    const double share = cubicStep(u);
    rawForce = next.loadAxisWorld
             * (share * forceTransferPolicy_.fullSupportForce);
    next.synthetic = true;
    next.valid = true;
    next.humanSupportForce = std::max(
        0.0, forceTransferPolicy_.fullSupportForce
             - share * forceTransferPolicy_.fullSupportForce);
  }
  else if(forceTransferPolicy_.source == "force_sensor")
  {
    const std::string robotName = forceTransferPolicy_.sourceRobot.empty()
        ? robot().name() : forceTransferPolicy_.sourceRobot;
    if(hasRobot(robotName))
    {
      const auto & sourceRobot = robot(robotName);
      std::string sensorName = forceTransferPolicy_.sensorName;
      if(sensorName.empty() && sourceRobot.forceSensors().size() == 1)
      {
        sensorName = sourceRobot.forceSensors().front().name();
      }
      if(!sensorName.empty() && sourceRobot.hasForceSensor(sensorName))
      {
        const auto & sensor = sourceRobot.forceSensor(sensorName);
        const sva::ForceVecd wrench = forceTransferPolicy_.gravityCompensated
            ? sensor.worldWrenchWithoutGravity(sourceRobot)
            : sensor.worldWrench(sourceRobot);
        rawForce = wrench.force();
        rawCouple = wrench.couple();
        next.valid = rawForce.allFinite() && rawCouple.allFinite();
      }
    }
  }

  if(forceTransferBiasCalibrationActive_ && next.valid
     && forceTransferPolicy_.source == "force_sensor")
  {
    forceTransferBiasForceSum_ += rawForce;
    forceTransferBiasCoupleSum_ += rawCouple;
    ++forceTransferBiasSamples_;
  }

  Eigen::Vector3d correctedForce = rawForce - forceTransferBiasForceWorld_;
  Eigen::Vector3d correctedCouple = rawCouple - forceTransferBiasCoupleWorld_;
  if(next.synthetic || next.simulatedPhysics)
  {
    correctedForce = rawForce;
    correctedCouple = rawCouple;
  }

  if(next.valid)
  {
    if(!forceTransferFilterInitialized_)
    {
      forceTransferFilteredForceWorld_ = correctedForce;
      forceTransferFilteredCoupleWorld_ = correctedCouple;
      forceTransferFilterInitialized_ = true;
    }
    const double tau = forceTransferPolicy_.filterTimeConstant;
    const double alpha = tau <= 1e-12
        ? 1.0 : controlDt_ / (tau + controlDt_);
    forceTransferFilteredForceWorld_ =
        (1.0 - alpha) * forceTransferFilteredForceWorld_
        + alpha * correctedForce;
    forceTransferFilteredCoupleWorld_ =
        (1.0 - alpha) * forceTransferFilteredCoupleWorld_
        + alpha * correctedCouple;

    next.forceWorld = forceTransferFilteredForceWorld_;
    next.coupleWorld = forceTransferFilteredCoupleWorld_;
    next.loadForce = std::max(
        0.0, next.forceWorld.dot(next.loadAxisWorld));
    const Eigen::Vector3d lateral = next.forceWorld
        - next.loadAxisWorld * next.forceWorld.dot(next.loadAxisWorld);
    next.lateralForce = lateral.norm();
    next.momentNorm = next.coupleWorld.norm();
    next.transferIndex = clamp01(
        next.loadForce / forceTransferPolicy_.fullSupportForce);
    next.transferRate = (next.transferIndex - forceTransferPreviousIndex_)
                      / std::max(1e-9, controlDt_);
    if(next.synthetic || next.simulatedPhysics)
    {
      next.humanSupportForce = std::max(
          0.0, forceTransferPolicy_.fullSupportForce - next.loadForce);
    }
    forceTransferPreviousIndex_ = next.transferIndex;
  }
  else
  {
    next.transferRate = 0.0;
    if(!forceTransferExecutionActive_)
    {
      forceTransferFilterInitialized_ = false;
      forceTransferPreviousIndex_ = 0.0;
    }
  }

  forceTransferMeasurement_ = next;
}

void HandoverInterceptionController::resetPhaseTiming()
{
  phaseStartTimes_.clear();
  phaseDurations_.clear();
}

void HandoverInterceptionController::startPhaseTiming(const std::string & phase)
{
  if(phase.empty()) { return; }
  phaseStartTimes_[phase] = controllerTime_;
  phaseDurations_[phase] = 0.0;
}

void HandoverInterceptionController::finishPhaseTiming(const std::string & phase)
{
  const auto it = phaseStartTimes_.find(phase);
  if(it == phaseStartTimes_.end()) { return; }
  phaseDurations_[phase] = std::max(0.0, controllerTime_ - it->second);
  phaseStartTimes_.erase(it);
}

double HandoverInterceptionController::phaseDuration(const std::string & phase) const
{
  const auto it = phaseDurations_.find(phase);
  return it == phaseDurations_.end() ? 0.0 : it->second;
}

void HandoverInterceptionController::logPhaseTimingSummary() const
{
  const double planning = phaseDuration("interception_planning");
  const double reach = phaseDuration("reach");
  const double approach = phaseDuration("approach");
  const double acquire = phaseDuration("acquire");
  const double transfer = phaseDuration("transfer");
  const double retreat = phaseDuration("retreat");
  const double execution = reach + approach + acquire + transfer + retreat;
  const double predicted = selectedCandidatePredictedTime_
      + (committedPlanValid()
         ? committedInterceptionPlan_.forceTransferDuration : 0.0);
  const double error = execution - predicted;
  const double ratio = predicted > 1e-9 ? execution / predicted : 0.0;
  mc_rtc::log::success(
      "[TimingSummary] planning={:.3f}s reach={:.3f}s approach={:.3f}s acquire={:.3f}s transfer={:.3f}s retreat={:.3f}s execution={:.3f}s predicted={:.3f}s error={:+.3f}s ratio={:.3f}",
      planning, reach, approach, acquire, transfer, retreat, execution, predicted,
      error, ratio);
}

// =============================================================================
// Configuration
// =============================================================================

void HandoverInterceptionController::loadHandoverConfig(
    const mc_rtc::Configuration & config)
{
  if(config.has("toolFrame")) { config("toolFrame", toolFrame_); }
  if(config.has("objectRobot")) { config("objectRobot", objectRobotName_); }
  if(config.has("objectFrame")) { config("objectFrame", objectFrameName_); }

  if(config.has("task"))
  {
    auto task = config("task");
    if(task.has("stiffness")) { task("stiffness", taskStiffness_); }
    if(task.has("weight")) { task("weight", taskWeight_); }
  }

  if(config.has("geometry"))
  {
    auto geometry = config("geometry");
    if(geometry.has("sensorRadius")) { geometry("sensorRadius", sensorRadius_); }
    if(geometry.has("sensorHalfLength")) { geometry("sensorHalfLength", sensorHalfLength_); }
    if(geometry.has("handleRadius")) { geometry("handleRadius", handleRadius_); }
    if(geometry.has("handleHalfLength")) { geometry("handleHalfLength", handleHalfLength_); }
    if(geometry.has("safetyMargin")) { geometry("safetyMargin", gripperSafetyMargin_); }
  }

  if(config.has("environment"))
  {
    auto environment = config("environment");
    if(environment.has("groundEnabled")) { environment("groundEnabled", groundEnabled_); }
    if(environment.has("groundZ")) { environment("groundZ", groundZ_); }
    if(environment.has("groundSafetyMargin"))
    {
      environment("groundSafetyMargin", groundSafetyMargin_);
    }
    if(environment.has("armGroundSafetyMargin"))
    {
      environment("armGroundSafetyMargin", armGroundSafetyMargin_);
    }
  }

  if(config.has("capture"))
  {
    auto capture = config("capture");
    if(capture.has("candidateCount")) { capture("candidateCount", candidateCount_); }
    if(capture.has("standoffDistance")) { capture("standoffDistance", candidateStandoffDistance_); }
    if(capture.has("retreatDistance")) { capture("retreatDistance", candidateRetreatDistance_); }
    if(capture.has("clearanceWeight")) { capture("clearanceWeight", candidateClearanceWeight_); }
    if(capture.has("travelWeight")) { capture("travelWeight", candidateTravelWeight_); }
    if(capture.has("rotationWeight")) { capture("rotationWeight", candidateRotationWeight_); }
    if(capture.has("clearanceTieBand")) { capture("clearanceTieBand", candidateClearanceTieBand_); }
    if(capture.has("transitSpeed")) { capture("transitSpeed", candidateTransitSpeed_); }
    if(capture.has("angularSpeed")) { capture("angularSpeed", candidateAngularSpeed_); }
    if(capture.has("sweepSamples")) { capture("sweepSamples", sweepSamples_); }
    if(capture.has("backtrackIterations")) { capture("backtrackIterations", backtrackIterations_); }
  }

  if(config.has("preview"))
  {
    auto preview = config("preview");
    if(preview.has("dt")) { preview("dt", previewDt_); }
    if(preview.has("maxIterationsPerSegment"))
    {
      preview("maxIterationsPerSegment", previewMaxIterationsPerSegment_);
    }
    if(preview.has("linearGain")) { preview("linearGain", previewLinearGain_); }
    if(preview.has("angularGain")) { preview("angularGain", previewAngularGain_); }
    if(preview.has("damping")) { preview("damping", previewDamping_); }
    if(preview.has("postureGain")) { preview("postureGain", previewPostureGain_); }
    if(preview.has("jointLimitWeightGain"))
    {
      preview("jointLimitWeightGain", previewJointLimitWeightGain_);
    }
    if(preview.has("jointLimitAvoidanceGain"))
    {
      preview("jointLimitAvoidanceGain", previewJointLimitAvoidanceGain_);
    }
    if(preview.has("jointLimitActivation"))
    {
      preview("jointLimitActivation", previewJointLimitActivation_);
    }
    if(preview.has("jointLimitVelocityCap"))
    {
      preview("jointLimitVelocityCap", previewJointLimitVelocityCap_);
    }
    if(preview.has("maxLinearSpeed")) { preview("maxLinearSpeed", previewMaxLinearSpeed_); }
    if(preview.has("maxAngularSpeed")) { preview("maxAngularSpeed", previewMaxAngularSpeed_); }
    if(preview.has("positionTolerance")) { preview("positionTolerance", previewPositionTolerance_); }
    if(preview.has("orientationTolerance")) { preview("orientationTolerance", previewOrientationTolerance_); }
    if(preview.has("effortWeight")) { preview("effortWeight", previewEffortWeight_); }
    if(preview.has("rotationWeight")) { preview("rotationWeight", previewRotationWeight_); }
    if(preview.has("clearanceTieBand")) { preview("clearanceTieBand", previewClearanceTieBand_); }
    if(preview.has("costTieBand")) { preview("costTieBand", previewCostTieBand_); }
    if(preview.has("jointLimitMargin")) { preview("jointLimitMargin", previewJointLimitMargin_); }
    if(preview.has("closureSamples")) { preview("closureSamples", previewClosureSamples_); }
    if(preview.has("movingInterception"))
    {
      preview("movingInterception", previewMovingInterception_);
    }
    if(preview.has("movingReachPositionTolerance"))
    {
      preview("movingReachPositionTolerance",
              previewMovingReachPositionTolerance_);
    }
    if(preview.has("movingReachOrientationTolerance"))
    {
      preview("movingReachOrientationTolerance",
              previewMovingReachOrientationTolerance_);
    }
    if(preview.has("movingReachMinimumClearance"))
    {
      preview("movingReachMinimumClearance",
              previewMovingReachMinimumClearance_);
    }
    if(preview.has("movingPositionTolerance"))
    {
      preview("movingPositionTolerance", previewMovingPositionTolerance_);
    }
    if(preview.has("movingOrientationTolerance"))
    {
      preview("movingOrientationTolerance", previewMovingOrientationTolerance_);
    }
    if(preview.has("movingCenteringTolerance"))
    {
      preview("movingCenteringTolerance", previewMovingCenteringTolerance_);
    }
    if(preview.has("movingMaximumRelativeLinearSpeed"))
    {
      preview("movingMaximumRelativeLinearSpeed",
              previewMovingMaximumRelativeLinearSpeed_);
    }
    if(preview.has("movingMaximumRelativeAngularSpeed"))
    {
      preview("movingMaximumRelativeAngularSpeed",
              previewMovingMaximumRelativeAngularSpeed_);
    }
    if(preview.has("movingContactTimingTolerance"))
    {
      preview("movingContactTimingTolerance",
              previewMovingContactTimingTolerance_);
    }
    if(preview.has("movingMaximumContactLateness"))
    {
      preview("movingMaximumContactLateness",
              previewMovingMaximumContactLateness_);
    }
    if(preview.has("movingMaximumScheduleIterations"))
    {
      preview("movingMaximumScheduleIterations",
              previewMovingMaximumScheduleIterations_);
    }
    if(preview.has("movingScheduleGrowth"))
    {
      preview("movingScheduleGrowth", previewMovingScheduleGrowth_);
    }
  }

  previewMovingReachPositionTolerance_ = std::max(
      0.001, previewMovingReachPositionTolerance_);
  previewMovingReachOrientationTolerance_ = std::max(
      0.001, previewMovingReachOrientationTolerance_);
  previewMovingReachMinimumClearance_ = std::max(
      0.0, previewMovingReachMinimumClearance_);
  previewMovingPositionTolerance_ = std::max(
      0.001, previewMovingPositionTolerance_);
  previewMovingOrientationTolerance_ = std::max(
      0.001, previewMovingOrientationTolerance_);
  previewMovingCenteringTolerance_ = std::max(
      0.0, previewMovingCenteringTolerance_);
  previewMovingMaximumRelativeLinearSpeed_ = std::max(
      0.0, previewMovingMaximumRelativeLinearSpeed_);
  previewMovingMaximumRelativeAngularSpeed_ = std::max(
      0.0, previewMovingMaximumRelativeAngularSpeed_);
  previewMovingContactTimingTolerance_ = std::max(
      0.0, previewMovingContactTimingTolerance_);
  previewMovingMaximumContactLateness_ = std::max(
      previewMovingContactTimingTolerance_,
      previewMovingMaximumContactLateness_);
  previewMovingMaximumScheduleIterations_ = std::max(
      1, previewMovingMaximumScheduleIterations_);
  previewMovingScheduleGrowth_ = std::max(1.0, previewMovingScheduleGrowth_);

  if(config.has("decisionCost"))
  {
    auto cost = config("decisionCost");
    if(cost.has("selectionMode"))
    {
      cost("selectionMode", completePlanSelectionMode_);
    }
    if(cost.has("eventSelectionMode"))
    {
      cost("eventSelectionMode", completeEventSelectionMode_);
    }
    if(cost.has("tieTolerance"))
    {
      cost("tieTolerance", decisionCostTieTolerance_);
    }
    if(cost.has("allowPhysicalExecution"))
    {
      cost("allowPhysicalExecution",
           decisionBindingPhysicalExecutionAuthorized_);
    }
    if(cost.has("timeReference"))
    {
      cost("timeReference", decisionTimeReference_);
    }
    if(cost.has("effortReference"))
    {
      cost("effortReference", decisionEffortReference_);
    }
    if(cost.has("pathReference"))
    {
      cost("pathReference", decisionPathReference_);
    }
    if(cost.has("characteristicLength"))
    {
      cost("characteristicLength", decisionCharacteristicLength_);
    }
    if(cost.has("softClearance"))
    {
      cost("softClearance", decisionSoftClearance_);
    }
    if(cost.has("softJointMargin"))
    {
      cost("softJointMargin", decisionSoftJointMargin_);
    }
    if(cost.has("softConditionIndex"))
    {
      cost("softConditionIndex", decisionSoftConditionIndex_);
    }
    if(cost.has("timeWeight")) { cost("timeWeight", decisionTimeWeight_); }
    if(cost.has("effortWeight"))
    {
      cost("effortWeight", decisionEffortWeight_);
    }
    if(cost.has("pathWeight")) { cost("pathWeight", decisionPathWeight_); }
    if(cost.has("rotationWeight"))
    {
      cost("rotationWeight", decisionRotationWeight_);
    }
    if(cost.has("clearanceWeight"))
    {
      cost("clearanceWeight", decisionClearanceWeight_);
    }
    if(cost.has("jointMarginWeight"))
    {
      cost("jointMarginWeight", decisionJointMarginWeight_);
    }
    if(cost.has("conditioningWeight"))
    {
      cost("conditioningWeight", decisionConditioningWeight_);
    }
    if(cost.has("velocityReserveWeight"))
    {
      cost("velocityReserveWeight", decisionVelocityReserveWeight_);
    }
    if(cost.has("metricStride"))
    {
      cost("metricStride", decisionMetricStride_);
    }
  }

  const bool selectionModeValid =
      completePlanSelectionMode_ == "protected_heuristic"
      || completePlanSelectionMode_ == "binding_cost";
  const bool eventSelectionModeValid =
      completeEventSelectionMode_ == "first_admissible_center_out"
      || completeEventSelectionMode_ == "global_time_plan";
  const bool eventSelectionCompatible =
      completeEventSelectionMode_ != "global_time_plan"
      || completePlanSelectionMode_ == "binding_cost";
  const bool decisionReferencesValid =
      std::isfinite(decisionTimeReference_) && decisionTimeReference_ > 0.0
      && std::isfinite(decisionEffortReference_)
      && decisionEffortReference_ > 0.0
      && std::isfinite(decisionPathReference_)
      && decisionPathReference_ > 0.0
      && std::isfinite(decisionCharacteristicLength_)
      && decisionCharacteristicLength_ > 0.0
      && std::isfinite(decisionSoftClearance_)
      && std::isfinite(decisionSoftJointMargin_)
      && decisionSoftJointMargin_ > 0.0
      && std::isfinite(decisionSoftConditionIndex_)
      && decisionSoftConditionIndex_ > 0.0
      && std::isfinite(decisionCostTieTolerance_)
      && decisionCostTieTolerance_ >= 0.0;
  const bool decisionWeightsValid =
      std::isfinite(decisionTimeWeight_) && decisionTimeWeight_ >= 0.0
      && std::isfinite(decisionEffortWeight_)
      && decisionEffortWeight_ >= 0.0
      && std::isfinite(decisionPathWeight_) && decisionPathWeight_ >= 0.0
      && std::isfinite(decisionRotationWeight_)
      && decisionRotationWeight_ >= 0.0
      && std::isfinite(decisionClearanceWeight_)
      && decisionClearanceWeight_ >= 0.0
      && std::isfinite(decisionJointMarginWeight_)
      && decisionJointMarginWeight_ >= 0.0
      && std::isfinite(decisionConditioningWeight_)
      && decisionConditioningWeight_ >= 0.0
      && std::isfinite(decisionVelocityReserveWeight_)
      && decisionVelocityReserveWeight_ >= 0.0;

  decisionTimeReference_ = std::max(1e-6, decisionTimeReference_);
  decisionEffortReference_ = std::max(1e-6, decisionEffortReference_);
  decisionPathReference_ = std::max(1e-6, decisionPathReference_);
  decisionCharacteristicLength_ = std::max(
      1e-4, decisionCharacteristicLength_);
  decisionSoftClearance_ = std::max(
      transitMinimumPredictedClearance_ + 1e-6,
      decisionSoftClearance_);
  decisionSoftJointMargin_ = std::max(1e-6, decisionSoftJointMargin_);
  decisionSoftConditionIndex_ = std::max(
      1e-6, decisionSoftConditionIndex_);
  decisionMetricStride_ = std::max(1, decisionMetricStride_);
  decisionTimeWeight_ = std::max(0.0, decisionTimeWeight_);
  decisionEffortWeight_ = std::max(0.0, decisionEffortWeight_);
  decisionPathWeight_ = std::max(0.0, decisionPathWeight_);
  decisionRotationWeight_ = std::max(0.0, decisionRotationWeight_);
  decisionClearanceWeight_ = std::max(0.0, decisionClearanceWeight_);
  decisionJointMarginWeight_ = std::max(0.0, decisionJointMarginWeight_);
  decisionConditioningWeight_ = std::max(0.0, decisionConditioningWeight_);
  decisionVelocityReserveWeight_ = std::max(
      0.0, decisionVelocityReserveWeight_);
  const double decisionWeightSum = decisionTimeWeight_
      + decisionEffortWeight_ + decisionPathWeight_
      + decisionRotationWeight_ + decisionClearanceWeight_
      + decisionJointMarginWeight_ + decisionConditioningWeight_
      + decisionVelocityReserveWeight_;
  decisionCostConfigurationValid_ = selectionModeValid
      && eventSelectionModeValid && eventSelectionCompatible
      && decisionReferencesValid && decisionWeightsValid
      && std::isfinite(decisionWeightSum) && decisionWeightSum > 1e-12;
  if(decisionWeightSum > 1e-12)
  {
    decisionTimeWeight_ /= decisionWeightSum;
    decisionEffortWeight_ /= decisionWeightSum;
    decisionPathWeight_ /= decisionWeightSum;
    decisionRotationWeight_ /= decisionWeightSum;
    decisionClearanceWeight_ /= decisionWeightSum;
    decisionJointMarginWeight_ /= decisionWeightSum;
    decisionConditioningWeight_ /= decisionWeightSum;
    decisionVelocityReserveWeight_ /= decisionWeightSum;
  }
  decisionCostTieTolerance_ = std::max(0.0, decisionCostTieTolerance_);

  if(!selectionModeValid)
  {
    mc_rtc::log::error(
        "[PlanSelectionConfiguration] unsupported selectionMode={}; allowed=[protected_heuristic,binding_cost]. Binding selection will fail closed",
        completePlanSelectionMode_);
  }
  if(!eventSelectionModeValid || !eventSelectionCompatible)
  {
    mc_rtc::log::error(
        "[PlanSelectionConfiguration] unsupported eventSelectionMode={} for selectionMode={}; allowed=[first_admissible_center_out,global_time_plan(binding_cost_only)]. Selection will fail closed",
        completeEventSelectionMode_, completePlanSelectionMode_);
  }
  mc_rtc::log::warning(
      "[PlanSelectionConfiguration] mode={} costConfigurationValid={} tieTolerance={:.3e} allowPhysicalExecution={} weightSumBeforeNormalization={:.6f} eventTimePolicy={} timeTerm={}",
      completePlanSelectionMode_, decisionCostConfigurationValid_,
      decisionCostTieTolerance_, decisionBindingPhysicalExecutionAuthorized_,
      decisionWeightSum, completeEventSelectionMode_,
      completeEventSelectionMode_ == "global_time_plan"
          ? "search_to_completion" : "execution_only");

  if(config.has("transitPlanning"))
  {
    auto transit = config("transitPlanning");
    if(transit.has("enabled"))
    {
      transit("enabled", transitPlanningEnabled_);
    }
    if(transit.has("routeDirections"))
    {
      transit("routeDirections", transitRouteDirections_);
    }
    if(transit.has("apexOffsets"))
    {
      transit("apexOffsets", transitRouteApexOffsets_);
    }
    if(transit.has("minimumPredictedClearance"))
    {
      transit("minimumPredictedClearance",
              transitMinimumPredictedClearance_);
    }
    if(transit.has("clearancePreferenceBand"))
    {
      transit("clearancePreferenceBand",
              transitClearancePreferenceBand_);
    }
    if(transit.has("maximumPathStretch"))
    {
      transit("maximumPathStretch", transitMaximumPathStretch_);
    }
    if(transit.has("curveLengthSamples"))
    {
      transit("curveLengthSamples", transitCurveLengthSamples_);
    }
  }
  transitRouteDirections_ = std::max(4, transitRouteDirections_);
  for(double & radius : transitRouteApexOffsets_)
  {
    radius = std::max(0.0, radius);
  }
  transitMinimumPredictedClearance_ = std::max(
      previewMovingReachMinimumClearance_,
      transitMinimumPredictedClearance_);
  decisionSoftClearance_ = std::max(
      transitMinimumPredictedClearance_ + 1e-6,
      decisionSoftClearance_);
  transitClearancePreferenceBand_ = std::max(
      0.0, transitClearancePreferenceBand_);
  transitMaximumPathStretch_ = std::max(1.0, transitMaximumPathStretch_);
  transitCurveLengthSamples_ = std::max(4, transitCurveLengthSamples_);

  if(config.has("predictiveReachPolicy"))
  {
    auto reach = config("predictiveReachPolicy");
    if(reach.has("maxLinearTrackingLead"))
      reach("maxLinearTrackingLead", predictiveReachPolicy_.maxLinearTrackingLead);
    if(reach.has("maxAngularTrackingLead"))
      reach("maxAngularTrackingLead", predictiveReachPolicy_.maxAngularTrackingLead);
    if(reach.has("nearLinearTrackingLead"))
      reach("nearLinearTrackingLead", predictiveReachPolicy_.nearLinearTrackingLead);
    if(reach.has("nearAngularTrackingLead"))
      reach("nearAngularTrackingLead", predictiveReachPolicy_.nearAngularTrackingLead);
    if(reach.has("clearanceSlowdownStart"))
      reach("clearanceSlowdownStart", predictiveReachPolicy_.clearanceSlowdownStart);
    if(reach.has("clearanceHardMargin"))
      reach("clearanceHardMargin", predictiveReachPolicy_.clearanceHardMargin);
    if(reach.has("minimumRuntimeClearance"))
      reach("minimumRuntimeClearance", predictiveReachPolicy_.minimumRuntimeClearance);
    if(reach.has("minimumVelocityScale"))
      reach("minimumVelocityScale", predictiveReachPolicy_.minimumVelocityScale);
    if(reach.has("clearanceScaleDropRate"))
      reach("clearanceScaleDropRate", predictiveReachPolicy_.clearanceScaleDropRate);
    if(reach.has("clearanceScaleRiseRate"))
      reach("clearanceScaleRiseRate", predictiveReachPolicy_.clearanceScaleRiseRate);
    if(reach.has("farLinearSpeed"))
      reach("farLinearSpeed", predictiveReachPolicy_.farLinearSpeed);
    if(reach.has("nearLinearSpeed"))
      reach("nearLinearSpeed", predictiveReachPolicy_.nearLinearSpeed);
    if(reach.has("farAngularSpeed"))
      reach("farAngularSpeed", predictiveReachPolicy_.farAngularSpeed);
    if(reach.has("nearAngularSpeed"))
      reach("nearAngularSpeed", predictiveReachPolicy_.nearAngularSpeed);
    if(reach.has("positionTolerance"))
      reach("positionTolerance", predictiveReachPolicy_.positionTolerance);
    if(reach.has("orientationTolerance"))
      reach("orientationTolerance", predictiveReachPolicy_.orientationTolerance);
    if(reach.has("taskStiffness"))
      reach("taskStiffness", predictiveReachPolicy_.taskStiffness);
    if(reach.has("taskWeight"))
      reach("taskWeight", predictiveReachPolicy_.taskWeight);
    if(reach.has("launchTimingTolerance"))
      reach("launchTimingTolerance", predictiveReachPolicy_.launchTimingTolerance);
    if(reach.has("scheduleLatenessTolerance"))
      reach("scheduleLatenessTolerance", predictiveReachPolicy_.scheduleLatenessTolerance);
    if(reach.has("minimumScheduledDuration"))
      reach("minimumScheduledDuration", predictiveReachPolicy_.minimumScheduledDuration);
    if(reach.has("maximumObjectTranslationDeviation"))
      reach("maximumObjectTranslationDeviation", predictiveReachPolicy_.maximumObjectTranslationDeviation);
    if(reach.has("maximumObjectRotationDeviation"))
      reach("maximumObjectRotationDeviation", predictiveReachPolicy_.maximumObjectRotationDeviation);
  }
  auto & reach = predictiveReachPolicy_;
  reach.maxLinearTrackingLead = std::max(0.001, reach.maxLinearTrackingLead);
  reach.maxAngularTrackingLead = std::max(0.001, reach.maxAngularTrackingLead);
  reach.nearLinearTrackingLead = std::min(
      reach.maxLinearTrackingLead, std::max(0.001, reach.nearLinearTrackingLead));
  reach.nearAngularTrackingLead = std::min(
      reach.maxAngularTrackingLead, std::max(0.001, reach.nearAngularTrackingLead));
  reach.clearanceSlowdownStart = std::max(0.001, reach.clearanceSlowdownStart);
  reach.clearanceHardMargin = std::min(
      reach.clearanceSlowdownStart - 1e-4,
      std::max(0.0, reach.clearanceHardMargin));
  reach.minimumRuntimeClearance = std::min(
      reach.clearanceHardMargin, std::max(0.0, reach.minimumRuntimeClearance));
  reach.minimumVelocityScale = std::min(
      1.0, std::max(0.01, reach.minimumVelocityScale));
  reach.clearanceScaleDropRate = std::max(0.01, reach.clearanceScaleDropRate);
  reach.clearanceScaleRiseRate = std::max(0.01, reach.clearanceScaleRiseRate);
  reach.farLinearSpeed = std::max(0.01, reach.farLinearSpeed);
  reach.nearLinearSpeed = std::min(
      reach.farLinearSpeed, std::max(0.005, reach.nearLinearSpeed));
  reach.farAngularSpeed = std::max(0.05, reach.farAngularSpeed);
  reach.nearAngularSpeed = std::min(
      reach.farAngularSpeed, std::max(0.02, reach.nearAngularSpeed));
  reach.positionTolerance = std::max(0.001, reach.positionTolerance);
  reach.orientationTolerance = std::max(0.001, reach.orientationTolerance);
  reach.launchTimingTolerance = std::max(0.001, reach.launchTimingTolerance);
  reach.scheduleLatenessTolerance = std::max(
      reach.launchTimingTolerance, reach.scheduleLatenessTolerance);
  reach.minimumScheduledDuration = std::max(0.05, reach.minimumScheduledDuration);
  reach.maximumObjectTranslationDeviation = std::max(
      0.0, reach.maximumObjectTranslationDeviation);
  reach.maximumObjectRotationDeviation = std::max(
      0.0, reach.maximumObjectRotationDeviation);

  if(config.has("executionTiming"))
  {
    auto timing = config("executionTiming");
    if(timing.has("armScale")) { timing("armScale", timingArmScale_); }
    if(timing.has("effectiveGripperRate"))
    {
      timing("effectiveGripperRate", timingEffectiveGripperRate_);
    }
    if(timing.has("priorityBlend"))
    {
      timing("priorityBlend", timingPriorityBlend_);
    }
    if(timing.has("captureLock"))
    {
      timing("captureLock", timingCaptureLock_);
    }
    if(timing.has("terminalCaptureDwell"))
    {
      timing("terminalCaptureDwell", timingTerminalCaptureDwell_);
    }
    if(timing.has("bilateralDwell"))
    {
      timing("bilateralDwell", timingBilateralDwell_);
    }
    if(timing.has("confirmationDwell"))
    {
      timing("confirmationDwell", timingConfirmationDwell_);
    }
    if(timing.has("confirmationTimeout"))
    {
      timing("confirmationTimeout", timingConfirmationTimeout_);
    }
  }

  timingArmScale_ = std::max(0.1, timingArmScale_);
  timingEffectiveGripperRate_ = std::max(1e-3, timingEffectiveGripperRate_);
  timingPriorityBlend_ = std::max(0.0, timingPriorityBlend_);
  timingCaptureLock_ = std::max(0.0, timingCaptureLock_);
  timingTerminalCaptureDwell_ = std::max(0.0, timingTerminalCaptureDwell_);
  timingBilateralDwell_ = std::max(0.0, timingBilateralDwell_);
  timingConfirmationDwell_ = std::max(0.0, timingConfirmationDwell_);
  timingConfirmationTimeout_ = std::max(
      timingConfirmationDwell_, timingConfirmationTimeout_);

  if(config.has("movingObject"))
  {
    auto moving = config("movingObject");
    if(moving.has("simulateMotion"))
    {
      moving("simulateMotion", simulateMovingObject_);
    }
    if(moving.has("observationDuration"))
    {
      moving("observationDuration", objectObservationDuration_);
    }
    if(moving.has("minimumObservationTime"))
    {
      moving("minimumObservationTime", objectMinimumObservationTime_);
    }
    if(moving.has("minimumObservationSamples"))
    {
      moving("minimumObservationSamples", objectMinimumObservationSamples_);
    }
    if(moving.has("predictionHorizon"))
    {
      moving("predictionHorizon", objectPredictionHorizon_);
    }
    if(moving.has("perceptionLatency"))
    {
      auto latency = moving("perceptionLatency");
      if(latency.has("enabled"))
      {
        latency("enabled", perceptionLatencyEnabled_);
      }
      if(latency.has("delay"))
      {
        latency("delay", perceptionLatencySeconds_);
      }
      if(latency.has("compensate"))
      {
        latency("compensate", perceptionLatencyCompensationEnabled_);
      }
      if(latency.has("bufferDuration"))
      {
        latency("bufferDuration", perceptionLatencyBufferDuration_);
      }
    }
    if(moving.has("velocityFilterTimeConstant"))
    {
      moving("velocityFilterTimeConstant", objectVelocityFilterTimeConstant_);
    }
    if(moving.has("maximumLinearSpeed"))
    {
      moving("maximumLinearSpeed", objectMaximumLinearSpeed_);
    }
    if(moving.has("maximumAngularSpeed"))
    {
      moving("maximumAngularSpeed", objectMaximumAngularSpeed_);
    }
    if(moving.has("maximumSimulatedTravel"))
    {
      moving("maximumSimulatedTravel", objectMaximumSimulatedTravel_);
    }
    if(moving.has("simulatedLinearVelocity"))
    {
      std::vector<double> v;
      moving("simulatedLinearVelocity", v);
      if(v.size() == 3)
      {
        simulatedObjectLinearVelocity_ = Eigen::Vector3d(v[0], v[1], v[2]);
      }
    }
    if(moving.has("simulatedAngularVelocity"))
    {
      std::vector<double> v;
      moving("simulatedAngularVelocity", v);
      if(v.size() == 3)
      {
        simulatedObjectAngularVelocity_ = Eigen::Vector3d(v[0], v[1], v[2]);
      }
    }
    if(moving.has("presentationMode"))
    {
      moving("presentationMode", presentationMode_);
    }
    if(moving.has("presentationDecelerationDuration"))
    {
      moving("presentationDecelerationDuration",
             presentationDecelerationDuration_);
    }
    if(moving.has("presentationAcquisitionWindow"))
    {
      moving("presentationAcquisitionWindow",
             presentationAcquisitionWindow_);
    }
    if(moving.has("presentationCapturePositionTolerance"))
    {
      moving("presentationCapturePositionTolerance",
             presentationCapturePositionTolerance_);
    }
    if(moving.has("presentationCaptureOrientationTolerance"))
    {
      moving("presentationCaptureOrientationTolerance",
             presentationCaptureOrientationTolerance_);
    }
    if(moving.has("presentationCaptureCenterTolerance"))
    {
      moving("presentationCaptureCenterTolerance",
             presentationCaptureCenterTolerance_);
    }
    if(moving.has("presentationMaximumLinearSpeed"))
    {
      moving("presentationMaximumLinearSpeed",
             presentationMaximumLinearSpeed_);
    }
    if(moving.has("presentationMaximumAngularSpeed"))
    {
      moving("presentationMaximumAngularSpeed",
             presentationMaximumAngularSpeed_);
    }
  }

  objectObservationDuration_ = std::max(0.1, objectObservationDuration_);
  objectMinimumObservationTime_ = std::max(0.0, objectMinimumObservationTime_);
  objectMinimumObservationSamples_ = std::max(2, objectMinimumObservationSamples_);
  objectPredictionHorizon_ = std::max(0.0, objectPredictionHorizon_);
  perceptionLatencySeconds_ = std::max(0.0, perceptionLatencySeconds_);
  perceptionLatencyBufferDuration_ = std::max(
      perceptionLatencySeconds_ + 2.0 * controlDt_,
      perceptionLatencyBufferDuration_);
  objectVelocityFilterTimeConstant_ =
      std::max(controlDt_, objectVelocityFilterTimeConstant_);
  objectMaximumLinearSpeed_ = std::max(1e-3, objectMaximumLinearSpeed_);
  objectMaximumAngularSpeed_ = std::max(1e-3, objectMaximumAngularSpeed_);
  objectMaximumSimulatedTravel_ = std::max(0.0, objectMaximumSimulatedTravel_);
  presentationDecelerationDuration_ = std::max(
      2.0 * controlDt_, presentationDecelerationDuration_);
  presentationAcquisitionWindow_ = std::max(
      0.5, presentationAcquisitionWindow_);
  presentationCapturePositionTolerance_ = std::max(
      0.001, presentationCapturePositionTolerance_);
  presentationCaptureOrientationTolerance_ = std::max(
      0.001, presentationCaptureOrientationTolerance_);
  presentationCaptureCenterTolerance_ = std::max(
      0.0, presentationCaptureCenterTolerance_);
  presentationMaximumLinearSpeed_ = std::max(
      0.0, presentationMaximumLinearSpeed_);
  presentationMaximumAngularSpeed_ = std::max(
      0.0, presentationMaximumAngularSpeed_);

  if(config.has("corridor"))
  {
    auto corridor = config("corridor");
    if(corridor.has("safetyMargin")) { corridor("safetyMargin", corridorSafetyMargin_); }
    if(corridor.has("fingerInset")) { corridor("fingerInset", corridorFingerInset_); }
    if(corridor.has("maxAngleDeg"))
    {
      double deg = corridorMaxAngleRad_ * 180.0 / PI;
      corridor("maxAngleDeg", deg);
      corridorMaxAngleRad_ = deg * PI / 180.0;
    }
    if(corridor.has("entryDepth")) { corridor("entryDepth", corridorEntryDepth_); }
    if(corridor.has("palmLimit")) { corridor("palmLimit", corridorPalmLimit_); }
    if(corridor.has("axialTolerance")) { corridor("axialTolerance", corridorAxialTolerance_); }
  }

  auto readVec3 = [](const mc_rtc::Configuration & block,
                     const std::string & key,
                     Eigen::Vector3d & out)
  {
    if(!block.has(key)) { return; }
    std::vector<double> v;
    block(key, v);
    if(v.size() == 3) { out = Eigen::Vector3d(v[0], v[1], v[2]); }
  };

  if(config.has("naturalApproach"))
  {
    auto natural = config("naturalApproach");
    readVec3(natural, "worldUp", worldUp_);
  }

  if(worldUp_.norm() < 1e-9) { worldUp_ = Eigen::Vector3d::UnitZ(); }
  worldUp_.normalize();

  if(config.has("padFrame"))
  {
    auto pad = config("padFrame");
    readVec3(pad, "leftPointInTip", leftPadPointTip_);
    readVec3(pad, "rightPointInTip", rightPadPointTip_);
    if(pad.has("captureDepth")) { pad("captureDepth", captureDepth_); }
  }

  if(config.has("gripper"))
  {
    auto gripper = config("gripper");
    if(gripper.has("openQ")) { gripper("openQ", gripperOpenQ_); }
    if(gripper.has("closeQ")) { gripper("closeQ", gripperCloseQ_); }
    if(gripper.has("closeRate")) { gripper("closeRate", gripperCloseRate_); }
    if(gripper.has("targetTipCenterGap")) { gripper("targetTipCenterGap", gripperTargetGap_); }
    if(gripper.has("maxClosure")) { gripper("maxClosure", gripperMaxClosure_); }
    if(gripper.has("commandLead")) { gripper("commandLead", gripperCommandLead_); }
    if(gripper.has("nearContactCommandLead"))
    {
      gripper("nearContactCommandLead", gripperNearContactCommandLead_);
    }
    if(gripper.has("contactClosureGuard"))
    {
      gripper("contactClosureGuard", gripperContactClosureGuard_);
    }
    if(gripper.has("minimumCloseRate"))
    {
      gripper("minimumCloseRate", gripperMinimumCloseRate_);
    }
    if(gripper.has("slowDistance")) { gripper("slowDistance", gripperSlowDistance_); }
    if(gripper.has("contactTolerance"))
    {
      gripper("contactTolerance", gripperContactTolerance_);
    }
    if(gripper.has("penetrationTolerance"))
    {
      gripper("penetrationTolerance", gripperPenetrationTolerance_);
    }
    if(gripper.has("padCenteringTolerance"))
    {
      gripper("padCenteringTolerance", padCenteringTolerance_);
    }
    if(gripper.has("jointWeightNormal")) { gripper("jointWeightNormal", gripperJointWeightNormal_); }
    if(gripper.has("jointWeightHigh")) { gripper("jointWeightHigh", gripperJointWeightHigh_); }
    if(gripper.has("postureStiffnessNormal")) { gripper("postureStiffnessNormal", gripperPostureStiffnessNormal_); }
    if(gripper.has("postureStiffnessHigh")) { gripper("postureStiffnessHigh", gripperPostureStiffnessHigh_); }
    if(gripper.has("postureWeight")) { gripper("postureWeight", gripperPostureWeight_); }
    if(gripper.has("physicalBridge"))
    {
      auto physical = gripper("physicalBridge");
      if(physical.has("enabled")) { physical("enabled", physicalGripperBridgeEnabled_); }
      if(physical.has("commandEnabled"))
      {
        physical("commandEnabled", physicalGripperCommandEnabled_);
      }
      if(physical.has("openPercent"))
      {
        physical("openPercent", physicalGripperOpenPercent_);
      }
      if(physical.has("closePercent"))
      {
        physical("closePercent", physicalGripperClosePercent_);
      }
      if(physical.has("maxPercent"))
      {
        physical("maxPercent", physicalGripperMaxPercent_);
      }
      if(physical.has("requireFeedback"))
      {
        physical("requireFeedback", physicalGripperRequireFeedback_);
      }
    }
  }
  physicalGripperOpenPercent_ = std::max(
      0.0, std::min(99.0, physicalGripperOpenPercent_));
  physicalGripperClosePercent_ = std::max(
      physicalGripperOpenPercent_ + 1e-3,
      std::min(100.0, physicalGripperClosePercent_));
  physicalGripperMaxPercent_ = std::max(
      physicalGripperClosePercent_,
      std::min(100.0, physicalGripperMaxPercent_));

  if(config.has("acquisition"))
  {
    auto acquisition = config("acquisition");
    if(acquisition.has("farCenterTolerance"))
    {
      acquisition("farCenterTolerance", acquisitionFarCenterTolerance_);
    }
    if(acquisition.has("nearCenterTolerance"))
    {
      acquisition("nearCenterTolerance", acquisitionNearCenterTolerance_);
    }
    if(acquisition.has("centerTightenDistance"))
    {
      acquisition("centerTightenDistance", acquisitionCenterTightenDistance_);
    }
  }

  if(config.has("readyMotion"))
  {
    auto ready = config("readyMotion");
    if(ready.has("stiffness")) { ready("stiffness", readyPostureStiffness_); }
    if(ready.has("weight")) { ready("weight", readyPostureWeight_); }
  }

  if(config.has("transfer"))
  {
    auto transfer = config("transfer");
    if(transfer.has("simulateKinematicAttachment"))
    {
      transfer("simulateKinematicAttachment", simulateKinematicAttachment_);
    }
    if(transfer.has("source"))
    {
      transfer("source", forceTransferPolicy_.source);
    }
    if(transfer.has("sourceRobot"))
    {
      transfer("sourceRobot", forceTransferPolicy_.sourceRobot);
    }
    if(transfer.has("sensorName"))
    {
      transfer("sensorName", forceTransferPolicy_.sensorName);
    }
    if(transfer.has("gravityCompensated"))
    {
      transfer("gravityCompensated", forceTransferPolicy_.gravityCompensated);
    }
    readVec3(transfer, "loadAxisObject", forceTransferPolicy_.loadAxisObject);
    if(transfer.has("objectMass"))
    {
      transfer("objectMass", forceTransferPolicy_.objectMass);
    }
    const bool fullSupportConfigured = transfer.has("fullSupportForce");
    if(fullSupportConfigured)
    {
      transfer("fullSupportForce", forceTransferPolicy_.fullSupportForce);
    }
    else
    {
      forceTransferPolicy_.fullSupportForce =
          std::max(1e-6, forceTransferPolicy_.objectMass * 9.81);
    }
    if(transfer.has("filterTimeConstant"))
    {
      transfer("filterTimeConstant", forceTransferPolicy_.filterTimeConstant);
    }
    if(transfer.has("syntheticTransferDuration"))
    {
      transfer("syntheticTransferDuration", forceTransferPolicy_.syntheticTransferDuration);
    }
    if(transfer.has("desiredTransferDuration"))
    {
      transfer("desiredTransferDuration", forceTransferPolicy_.desiredTransferDuration);
    }
    if(transfer.has("virtualContactStiffness"))
    {
      transfer("virtualContactStiffness", forceTransferPolicy_.virtualContactStiffness);
    }
    if(transfer.has("virtualContactDamping"))
    {
      transfer("virtualContactDamping", forceTransferPolicy_.virtualContactDamping);
    }
    if(transfer.has("virtualMaximumLoadScale"))
    {
      transfer("virtualMaximumLoadScale", forceTransferPolicy_.virtualMaximumLoadScale);
    }
    if(transfer.has("releaseThreshold"))
    {
      transfer("releaseThreshold", forceTransferPolicy_.releaseThreshold);
    }
    if(transfer.has("releaseRateTolerance"))
    {
      transfer("releaseRateTolerance", forceTransferPolicy_.releaseRateTolerance);
    }
    if(transfer.has("releaseDwell"))
    {
      transfer("releaseDwell", forceTransferPolicy_.releaseDwell);
    }
    if(transfer.has("transferTimeout"))
    {
      transfer("transferTimeout", forceTransferPolicy_.transferTimeout);
    }
    if(transfer.has("maximumForce"))
    {
      transfer("maximumForce", forceTransferPolicy_.maximumForce);
    }
    if(transfer.has("maximumMoment"))
    {
      transfer("maximumMoment", forceTransferPolicy_.maximumMoment);
    }
    if(transfer.has("admittanceMass"))
    {
      transfer("admittanceMass", forceTransferPolicy_.admittanceMass);
    }
    if(transfer.has("admittanceDamping"))
    {
      transfer("admittanceDamping", forceTransferPolicy_.admittanceDamping);
    }
    if(transfer.has("admittanceStiffness"))
    {
      transfer("admittanceStiffness", forceTransferPolicy_.admittanceStiffness);
    }
    if(transfer.has("maximumAdmittanceSpeed"))
    {
      transfer("maximumAdmittanceSpeed", forceTransferPolicy_.maximumAdmittanceSpeed);
    }
    if(transfer.has("maximumAdmittanceOffset"))
    {
      transfer("maximumAdmittanceOffset", forceTransferPolicy_.maximumAdmittanceOffset);
    }
  }

  if(forceTransferPolicy_.loadAxisObject.norm() < 1e-9)
  {
    forceTransferPolicy_.loadAxisObject = Eigen::Vector3d::UnitZ();
  }
  forceTransferPolicy_.loadAxisObject.normalize();
  forceTransferPolicy_.objectMass = std::max(0.0, forceTransferPolicy_.objectMass);
  forceTransferPolicy_.fullSupportForce = std::max(1e-6, forceTransferPolicy_.fullSupportForce);
  forceTransferPolicy_.filterTimeConstant = std::max(0.0, forceTransferPolicy_.filterTimeConstant);
  forceTransferPolicy_.syntheticTransferDuration = std::max(0.05, forceTransferPolicy_.syntheticTransferDuration);
  forceTransferPolicy_.desiredTransferDuration = std::max(0.05, forceTransferPolicy_.desiredTransferDuration);
  forceTransferPolicy_.virtualContactStiffness = std::max(1e-6, forceTransferPolicy_.virtualContactStiffness);
  forceTransferPolicy_.virtualContactDamping = std::max(0.0, forceTransferPolicy_.virtualContactDamping);
  forceTransferPolicy_.virtualMaximumLoadScale = std::max(1.0, forceTransferPolicy_.virtualMaximumLoadScale);
  forceTransferPolicy_.releaseThreshold = clamp01(forceTransferPolicy_.releaseThreshold);
  forceTransferPolicy_.releaseRateTolerance = std::max(0.0, forceTransferPolicy_.releaseRateTolerance);
  forceTransferPolicy_.releaseDwell = std::max(0.0, forceTransferPolicy_.releaseDwell);
  forceTransferPolicy_.transferTimeout = std::max(forceTransferPolicy_.releaseDwell, forceTransferPolicy_.transferTimeout);
  forceTransferPolicy_.maximumForce = std::max(0.0, forceTransferPolicy_.maximumForce);
  forceTransferPolicy_.maximumMoment = std::max(0.0, forceTransferPolicy_.maximumMoment);
  forceTransferPolicy_.admittanceMass = std::max(1e-6, forceTransferPolicy_.admittanceMass);
  forceTransferPolicy_.admittanceDamping = std::max(0.0, forceTransferPolicy_.admittanceDamping);
  forceTransferPolicy_.admittanceStiffness = std::max(0.0, forceTransferPolicy_.admittanceStiffness);
  forceTransferPolicy_.maximumAdmittanceSpeed = std::max(0.0, forceTransferPolicy_.maximumAdmittanceSpeed);
  forceTransferPolicy_.maximumAdmittanceOffset = std::max(0.0, forceTransferPolicy_.maximumAdmittanceOffset);

  readyPosture_ =
  {
    {"gen3_joint_1", {-0.044821}},
    {"gen3_joint_2", { 0.784738}},
    {"gen3_joint_3", { 2.757292}},
    {"gen3_joint_4", {-1.694771}},
    {"gen3_joint_5", {-0.390082}},
    {"gen3_joint_6", { 1.026258}},
    {"gen3_joint_7", { 1.899765}}
  };

  auto parsePoseBlock = [this, &config](const std::string & key,
                                        Eigen::Vector3d & translation,
                                        Eigen::Vector3d & rpy)
  {
    if(!config.has(key)) { return; }
    auto block = config(key);
    if(block.has("translation"))
    {
      std::vector<double> v;
      block("translation", v);
      if(v.size() == 3) { translation = Eigen::Vector3d(v[0], v[1], v[2]); }
    }
    if(block.has("rpy"))
    {
      std::vector<double> v;
      block("rpy", v);
      if(v.size() == 3) { rpy = Eigen::Vector3d(v[0], v[1], v[2]); }
    }
  };

  Eigen::Vector3d objectTranslation(0.55, 0.0, 0.55);
  Eigen::Vector3d objectRPY(0.0, 1.5708, 0.0);
  parsePoseBlock("object", objectTranslation, objectRPY);

  Eigen::Vector3d O_H_translation(0.0, 0.0, -0.0869);
  Eigen::Vector3d O_H_rpy(0.0, 0.0, 0.0);
  parsePoseBlock("O_T_H", O_H_translation, O_H_rpy);

  Eigen::Vector3d B_M_translation(0.0, 0.0, 0.0983262);
  Eigen::Vector3d B_M_rpy(-1.5707963267948966, 0.0, 0.0);
  parsePoseBlock("B_T_M", B_M_translation, B_M_rpy);

  W_T_O_config_ = makePose(objectTranslation, objectRPY);
  W_T_O_ = W_T_O_config_;
  O_T_H_ = makePose(O_H_translation, O_H_rpy);
  B_T_M_config_ = makePose(B_M_translation, B_M_rpy);
  B_T_M_control_ = B_T_M_config_;
  W_T_H_ = compose(W_T_O_, O_T_H_);
}

double HandoverInterceptionController::acquisitionCenterTolerance(
    double minPadClearance) const
{
  const double nearTol = std::max(0.0, acquisitionNearCenterTolerance_);
  const double farTol = std::max(nearTol, acquisitionFarCenterTolerance_);
  const double nearDistance = gripperContactTolerance_;
  const double farDistance = std::max(
      nearDistance + 1e-6, acquisitionCenterTightenDistance_);
  const double alpha = std::max(0.0, std::min(1.0,
      (minPadClearance - nearDistance) / (farDistance - nearDistance)));
  return nearTol + alpha * (farTol - nearTol);
}

// =============================================================================
// Correct physical-pose / SpaceVecAlg conversions
// =============================================================================

Eigen::Matrix3d HandoverInterceptionController::rpyToWorldRotation(
    const Eigen::Vector3d & rpy) const
{
  return (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

Eigen::Matrix3d HandoverInterceptionController::worldRotation(
    const sva::PTransformd & X_0_F) const
{
  return X_0_F.rotation().transpose();
}

sva::PTransformd HandoverInterceptionController::fromWorldPose(
    const Eigen::Matrix3d & R_W_F,
    const Eigen::Vector3d & p_W_F) const
{
  return sva::PTransformd(R_W_F.transpose(), p_W_F);
}

sva::PTransformd HandoverInterceptionController::makePose(
    const Eigen::Vector3d & translation,
    const Eigen::Vector3d & rpy) const
{
  return fromWorldPose(rpyToWorldRotation(rpy), translation);
}

Eigen::Vector3d HandoverInterceptionController::pointToWorld(
    const sva::PTransformd & W_T_F,
    const Eigen::Vector3d & p_F) const
{
  return W_T_F.translation() + worldRotation(W_T_F) * p_F;
}

Eigen::Vector3d HandoverInterceptionController::pointToLocal(
    const sva::PTransformd & W_T_F,
    const Eigen::Vector3d & p_W) const
{
  return W_T_F.rotation() * (p_W - W_T_F.translation());
}

sva::PTransformd HandoverInterceptionController::compose(
    const sva::PTransformd & W_T_A,
    const sva::PTransformd & A_T_B) const
{
  const Eigen::Matrix3d R_W_A = worldRotation(W_T_A);
  const Eigen::Matrix3d R_A_B = worldRotation(A_T_B);
  const Eigen::Matrix3d R_W_B = R_W_A * R_A_B;
  const Eigen::Vector3d p_W_B = W_T_A.translation() + R_W_A * A_T_B.translation();
  return fromWorldPose(R_W_B, p_W_B);
}

sva::PTransformd HandoverInterceptionController::relativePose(
    const sva::PTransformd & W_T_A,
    const sva::PTransformd & W_T_B) const
{
  const Eigen::Matrix3d R_W_A = worldRotation(W_T_A);
  const Eigen::Matrix3d R_W_B = worldRotation(W_T_B);
  const Eigen::Matrix3d R_A_B = R_W_A.transpose() * R_W_B;
  const Eigen::Vector3d p_A_B = R_W_A.transpose()
                              * (W_T_B.translation() - W_T_A.translation());
  return fromWorldPose(R_A_B, p_A_B);
}

sva::PTransformd HandoverInterceptionController::interpolatePose(
    const sva::PTransformd & A,
    const sva::PTransformd & B,
    double alpha) const
{
  alpha = std::max(0.0, std::min(1.0, alpha));
  const Eigen::Vector3d p = (1.0 - alpha) * A.translation() + alpha * B.translation();

  Eigen::Quaterniond qA(worldRotation(A));
  Eigen::Quaterniond qB(worldRotation(B));
  qA.normalize();
  qB.normalize();
  if(qA.dot(qB) < 0.0) { qB.coeffs() *= -1.0; }
  const Eigen::Matrix3d R = qA.slerp(alpha, qB).normalized().toRotationMatrix();
  return fromWorldPose(R, p);
}

sva::PTransformd HandoverInterceptionController::reachCurvePose(
    const sva::PTransformd & start,
    const sva::PTransformd & standoff,
    const Eigen::Vector3d & curveOffsetWorld,
    double progress) const
{
  const double u = clamp01(progress);
  const Eigen::Vector3d p0 = start.translation();
  const Eigen::Vector3d p3 = standoff.translation();
  const Eigen::Vector3d chord = p3 - p0;
  // Both controls receive 4/3 of the requested midpoint displacement, so the
  // cubic reaches exactly curveOffsetWorld away from the direct chord at u=.5.
  const Eigen::Vector3d controlOffset = (4.0 / 3.0) * curveOffsetWorld;
  const Eigen::Vector3d p1 = p0 + chord / 3.0 + controlOffset;
  const Eigen::Vector3d p2 = p0 + 2.0 * chord / 3.0 + controlOffset;
  const double omu = 1.0 - u;
  const Eigen::Vector3d p = omu * omu * omu * p0
      + 3.0 * omu * omu * u * p1
      + 3.0 * omu * u * u * p2
      + u * u * u * p3;

  Eigen::Quaterniond q0(worldRotation(start));
  Eigen::Quaterniond q3(worldRotation(standoff));
  q0.normalize();
  q3.normalize();
  if(q0.dot(q3) < 0.0) { q3.coeffs() *= -1.0; }
  const Eigen::Matrix3d R = q0.slerp(u, q3).normalized().toRotationMatrix();
  return fromWorldPose(R, p);
}

double HandoverInterceptionController::reachCurveLength(
    const sva::PTransformd & start,
    const sva::PTransformd & standoff,
    const Eigen::Vector3d & curveOffsetWorld) const
{
  const int samples = std::max(4, transitCurveLengthSamples_);
  double length = 0.0;
  Eigen::Vector3d previous = start.translation();
  for(int k = 1; k <= samples; ++k)
  {
    const double u = static_cast<double>(k) / static_cast<double>(samples);
    const Eigen::Vector3d current = reachCurvePose(
        start, standoff, curveOffsetWorld, u).translation();
    length += (current - previous).norm();
    previous = current;
  }
  return length;
}

std::vector<std::pair<std::string, Eigen::Vector3d>>
HandoverInterceptionController::transitRouteBank(
    const sva::PTransformd & start,
    const sva::PTransformd & standoff) const
{
  std::vector<std::pair<std::string, Eigen::Vector3d>> routes;
  routes.emplace_back("direct", Eigen::Vector3d::Zero());
  if(!transitPlanningEnabled_) { return routes; }

  Eigen::Vector3d chord = standoff.translation() - start.translation();
  if(chord.norm() < 1e-6) { return routes; }
  chord.normalize();

  Eigen::Vector3d e1 = worldUp_ - chord * chord.dot(worldUp_);
  if(e1.norm() < 1e-6)
  {
    e1 = Eigen::Vector3d::UnitX()
       - chord * chord.dot(Eigen::Vector3d::UnitX());
  }
  if(e1.norm() < 1e-6)
  {
    e1 = Eigen::Vector3d::UnitY()
       - chord * chord.dot(Eigen::Vector3d::UnitY());
  }
  e1.normalize();
  Eigen::Vector3d e2 = chord.cross(e1);
  if(e2.norm() < 1e-6) { return routes; }
  e2.normalize();

  const int directions = std::max(4, transitRouteDirections_);
  for(double radius : transitRouteApexOffsets_)
  {
    radius = std::max(0.0, radius);
    if(radius <= 1e-9) { continue; }
    const int mm = static_cast<int>(std::lround(1000.0 * radius));
    for(int k = 0; k < directions; ++k)
    {
      const double angle = 2.0 * PI * static_cast<double>(k)
                         / static_cast<double>(directions);
      const Eigen::Vector3d offset = radius
          * (std::cos(angle) * e1 + std::sin(angle) * e2);
      routes.emplace_back(
          "ring" + std::to_string(mm) + "mm_"
              + std::to_string(k) + "of" + std::to_string(directions),
          offset);
    }
  }
  return routes;
}

double HandoverInterceptionController::orientationError(
    const sva::PTransformd & A,
    const sva::PTransformd & B) const
{
  Eigen::Quaterniond qA(worldRotation(A));
  Eigen::Quaterniond qB(worldRotation(B));
  qA.normalize();
  qB.normalize();
  return 2.0 * std::acos(clampUnit(std::abs(qA.dot(qB))));
}

sva::PTransformd HandoverInterceptionController::boundedPoseStep(
    const sva::PTransformd & current,
    const sva::PTransformd & goal,
    double maxTranslation,
    double maxRotation) const
{
  const Eigen::Vector3d e = goal.translation() - current.translation();
  const double dist = e.norm();
  Eigen::Vector3d p = current.translation();
  if(dist > 1e-12)
  {
    p += std::min(std::max(0.0, maxTranslation), dist) * e / dist;
  }

  Eigen::Quaterniond qCurrent(worldRotation(current));
  Eigen::Quaterniond qGoal(worldRotation(goal));
  qCurrent.normalize();
  qGoal.normalize();
  if(qCurrent.dot(qGoal) < 0.0) { qGoal.coeffs() *= -1.0; }
  const double angle = 2.0 * std::acos(clampUnit(std::abs(qCurrent.dot(qGoal))));
  const double alpha = angle > 1e-12
                     ? std::min(1.0, std::max(0.0, maxRotation) / angle)
                     : 1.0;
  const Eigen::Matrix3d R = qCurrent.slerp(alpha, qGoal).normalized().toRotationMatrix();
  return fromWorldPose(R, p);
}

sva::PTransformd HandoverInterceptionController::advancePoseReference(
    const sva::PTransformd & reference,
    const sva::PTransformd & goal,
    double maxLinearSpeed,
    double maxAngularSpeed) const
{
  return boundedPoseStep(reference, goal,
                         std::max(0.0, maxLinearSpeed) * controlDt_,
                         std::max(0.0, maxAngularSpeed) * controlDt_);
}

void HandoverInterceptionController::worldPoseTwist(
    const sva::PTransformd & from,
    const sva::PTransformd & to,
    double dt,
    Eigen::Vector3d & linearVelocityWorld,
    Eigen::Vector3d & angularVelocityWorld) const
{
  const double safeDt = std::max(1e-9, dt);
  linearVelocityWorld = (to.translation() - from.translation()) / safeDt;

  const Eigen::Matrix3d dR = worldRotation(to)
                            * worldRotation(from).transpose();
  Eigen::AngleAxisd aa(dR);
  angularVelocityWorld.setZero();
  if(std::isfinite(aa.angle()) && std::abs(aa.angle()) > 1e-12)
  {
    angularVelocityWorld = aa.axis() * aa.angle() / safeDt;
  }
}

sva::PTransformd HandoverInterceptionController::propagatePoseConstantTwist(
    const sva::PTransformd & referencePose,
    double dt,
    const Eigen::Vector3d & linearVelocityWorld,
    const Eigen::Vector3d & angularVelocityWorld) const
{
  const Eigen::Vector3d p = referencePose.translation()
                          + dt * linearVelocityWorld;
  Eigen::Matrix3d R = worldRotation(referencePose);
  const double omegaNorm = angularVelocityWorld.norm();
  if(omegaNorm > 1e-9 && std::abs(dt) > 1e-12)
  {
    const Eigen::Matrix3d dR = Eigen::AngleAxisd(
        omegaNorm * dt, angularVelocityWorld / omegaNorm).toRotationMatrix();
    R = dR * R;
  }
  return fromWorldPose(R, p);
}

HandoverInterceptionController::InterceptionPlan
HandoverInterceptionController::makeInterceptionPlan(
    const CaptureCandidate & candidate,
    const sva::PTransformd & W_T_O_presentation,
    double presentationTime,
    double reachDuration,
    double approachDuration,
    double acquireDuration,
    double retreatDuration) const
{
  InterceptionPlan plan;
  plan.valid = true;
  plan.candidateName = candidate.name;
  plan.objectMode = observedObjectMode_;
  plan.transitRouteName = candidate.transitRouteName;
  plan.reachCurveOffsetWorld = candidate.reachCurveOffsetWorld;
  plan.presentationMode = presentationMode_;
  plan.presentationTime = presentationTime;
  plan.reachDuration = std::max(2.0 * previewDt_, reachDuration);
  plan.approachDuration = std::max(2.0 * previewDt_, approachDuration);
  plan.acquireDuration = std::max(2.0 * previewDt_, acquireDuration);
  plan.confirmationDuration = timingBilateralDwell_
                            + timingConfirmationTimeout_;
  plan.forceTransferDuration = forceTransferPolicy_.desiredTransferDuration
                             + forceTransferPolicy_.releaseDwell;
  plan.forceTransferTimeout = forceTransferPolicy_.transferTimeout;
  plan.forceTransferFullSupportForce = forceTransferPolicy_.fullSupportForce;
  plan.forceTransferReleaseThreshold = forceTransferPolicy_.releaseThreshold;
  plan.forceTransferReleaseRateTolerance = forceTransferPolicy_.releaseRateTolerance;
  plan.forceTransferReleaseDwell = forceTransferPolicy_.releaseDwell;
  plan.forceTransferMaximumForce = forceTransferPolicy_.maximumForce;
  plan.forceTransferMaximumMoment = forceTransferPolicy_.maximumMoment;
  plan.forceTransferAdmittanceMass = forceTransferPolicy_.admittanceMass;
  plan.forceTransferAdmittanceDamping = forceTransferPolicy_.admittanceDamping;
  plan.forceTransferAdmittanceStiffness = forceTransferPolicy_.admittanceStiffness;
  plan.forceTransferMaximumAdmittanceSpeed = forceTransferPolicy_.maximumAdmittanceSpeed;
  plan.forceTransferMaximumAdmittanceOffset = forceTransferPolicy_.maximumAdmittanceOffset;
  plan.forceTransferVirtualContactStiffness = forceTransferPolicy_.virtualContactStiffness;
  plan.forceTransferVirtualContactDamping = forceTransferPolicy_.virtualContactDamping;
  plan.forceTransferVirtualMaximumLoadScale = forceTransferPolicy_.virtualMaximumLoadScale;
  plan.forceTransferSource = forceTransferPolicy_.source;
  plan.retreatDuration = std::max(0.0, retreatDuration);
  plan.contactClosure = std::max(0.0, candidate.contactClosure);

  // The fingers stay fully open through prediction, reach, presentation hold
  // and insertion. The continuous CaptureTransfer controller owns closure,
  // bilateral confirmation and force-aware load takeover.

  const double minimumStaticWindow = plan.acquireDuration
      + plan.confirmationDuration + 1.0;
  plan.acquisitionWindowDuration = plan.presentationMode
      ? std::max(presentationAcquisitionWindow_, minimumStaticWindow) : 0.0;

  if(plan.presentationMode)
  {
    // The giver finishes a smooth deceleration exactly when the receiver
    // reaches the committed standoff. The object then remains presented while
    // the guarded insertion and short final closure are executed.
    plan.standoffTime = plan.presentationTime;
    plan.acquireStartTime = plan.standoffTime + plan.approachDuration;
    plan.contactTime = plan.acquireStartTime + plan.acquireDuration;
    plan.acquisitionDeadlineTime = plan.acquireStartTime
                                 + plan.acquisitionWindowDuration;
    plan.reachStartTime = plan.standoffTime - plan.reachDuration;
    if(plan.objectMode == ObservedObjectMode::Static)
    {
      plan.decelerationDuration = 0.0;
      plan.decelerationStartTime = plan.presentationTime;
    }
    else
    {
      plan.decelerationDuration = presentationDecelerationDuration_;
      plan.decelerationStartTime = plan.presentationTime
                                 - plan.decelerationDuration;
    }
  }
  else
  {
    // Advanced stress-test mode retained for constant-speed moving contact.
    plan.contactTime = presentationTime;
    plan.acquireStartTime = plan.contactTime - plan.acquireDuration;
    plan.standoffTime = plan.acquireStartTime - plan.approachDuration;
    plan.reachStartTime = plan.standoffTime - plan.reachDuration;
    plan.presentationTime = plan.contactTime;
    plan.decelerationStartTime = plan.contactTime;
    plan.decelerationDuration = 0.0;
    plan.acquisitionDeadlineTime = plan.contactTime;
  }
  plan.confirmationEndTime = plan.acquisitionDeadlineTime
                           + plan.confirmationDuration;

  plan.objectAtPresentation = W_T_O_presentation;
  plan.objectAtContact = W_T_O_presentation;
  plan.mouthAtReachStart = planningStartMouthPose_;
  plan.O_T_M_standoff = relativePose(
      W_T_O_presentation, candidate.W_T_M_standoff);
  plan.O_T_M_capture = relativePose(
      W_T_O_presentation, candidate.W_T_M_pre);
  plan.O_T_M_retreat = relativePose(
      W_T_O_presentation, candidate.W_T_M_retreat);
  plan.objectLinearVelocity = objectLinearVelocityEstimate_;
  plan.objectAngularVelocity = objectAngularVelocityEstimate_;
  return plan;
}

sva::PTransformd HandoverInterceptionController::predictPresentationPose(
    double horizon) const
{
  horizon = std::max(0.0, horizon);
  if(observedObjectMode_ == ObservedObjectMode::Static) { return W_T_O_; }
  if(!presentationMode_) { return predictObjectPose(horizon); }

  // The deceleration is scheduled to end at the hypothesized presentation
  // event. Its integrated velocity is one half of a same-duration constant-
  // speed segment, hence the effective travel time below.
  const double decel = std::min(
      horizon, std::max(0.0, presentationDecelerationDuration_));
  const double effectiveTravel = std::max(0.0, horizon - 0.5 * decel);
  return propagatePoseConstantTwist(
      W_T_O_, effectiveTravel,
      objectLinearVelocityEstimate_, objectAngularVelocityEstimate_);
}

sva::PTransformd HandoverInterceptionController::interceptionObjectPoseAt(
    const InterceptionPlan & plan,
    double absoluteTime) const
{
  if(!plan.presentationMode)
  {
    return propagatePoseConstantTwist(
        plan.objectAtContact, absoluteTime - plan.contactTime,
        plan.objectLinearVelocity, plan.objectAngularVelocity);
  }

  if(absoluteTime >= plan.presentationTime)
  {
    return plan.objectAtPresentation;
  }

  const double duration = std::max(1e-9, plan.decelerationDuration);
  const sva::PTransformd objectAtDecelerationStart =
      propagatePoseConstantTwist(
          plan.objectAtPresentation, -0.5 * duration,
          plan.objectLinearVelocity, plan.objectAngularVelocity);

  if(absoluteTime <= plan.decelerationStartTime)
  {
    return propagatePoseConstantTwist(
        objectAtDecelerationStart,
        absoluteTime - plan.decelerationStartTime,
        plan.objectLinearVelocity, plan.objectAngularVelocity);
  }

  const double u = clamp01(
      (absoluteTime - plan.decelerationStartTime) / duration);
  const double effectiveTravel = duration * quinticStopIntegral(u);
  return propagatePoseConstantTwist(
      objectAtDecelerationStart, effectiveTravel,
      plan.objectLinearVelocity, plan.objectAngularVelocity);
}

void HandoverInterceptionController::interceptionObjectTwistAt(
    const InterceptionPlan & plan,
    double absoluteTime,
    Eigen::Vector3d & linearVelocityWorld,
    Eigen::Vector3d & angularVelocityWorld) const
{
  double scale = 1.0;
  if(plan.presentationMode)
  {
    if(absoluteTime >= plan.presentationTime)
    {
      scale = 0.0;
    }
    else if(absoluteTime > plan.decelerationStartTime)
    {
      const double duration = std::max(1e-9, plan.decelerationDuration);
      const double u = clamp01(
          (absoluteTime - plan.decelerationStartTime) / duration);
      scale = 1.0 - quinticStep(u);
    }
  }
  linearVelocityWorld = scale * plan.objectLinearVelocity;
  angularVelocityWorld = scale * plan.objectAngularVelocity;
}

bool HandoverInterceptionController::presentationCaptureAdmitted(
    double positionError,
    double rotationError,
    const HandoverSafetyReport & report,
    double objectLinearSpeed,
    double objectAngularSpeed) const
{
  return std::isfinite(positionError) && std::isfinite(rotationError)
      && positionError <= presentationCapturePositionTolerance_
      && rotationError <= presentationCaptureOrientationTolerance_
      && report.minClearance >= -1e-9
      && report.padCenteringError <= presentationCaptureCenterTolerance_
      && objectLinearSpeed <= presentationMaximumLinearSpeed_
      && objectAngularSpeed <= presentationMaximumAngularSpeed_;
}

sva::PTransformd HandoverInterceptionController::interceptionMouthPoseAt(
    const InterceptionPlan & plan,
    double absoluteTime) const
{
  const sva::PTransformd objectPose = interceptionObjectPoseAt(
      plan, absoluteTime);

  if(absoluteTime <= plan.standoffTime)
  {
    const double u = plan.reachDuration > 1e-9
        ? (absoluteTime - plan.reachStartTime) / plan.reachDuration : 1.0;
    const sva::PTransformd objectAtStandoff = interceptionObjectPoseAt(
        plan, plan.standoffTime);
    const sva::PTransformd standoff = compose(
        objectAtStandoff, plan.O_T_M_standoff);
    return reachCurvePose(
        plan.mouthAtReachStart, standoff, plan.reachCurveOffsetWorld,
        naturalReachStep(u));
  }

  if(absoluteTime <= plan.acquireStartTime)
  {
    const double u = plan.approachDuration > 1e-9
        ? (absoluteTime - plan.standoffTime) / plan.approachDuration : 1.0;
    const sva::PTransformd O_T_M = interpolatePose(
        plan.O_T_M_standoff, plan.O_T_M_capture, quinticStep(u));
    return compose(objectPose, O_T_M);
  }

  return compose(objectPose, plan.O_T_M_capture);
}

HandoverInterceptionController::MovingReference
HandoverInterceptionController::interceptionReferenceAt(
    const InterceptionPlan & plan,
    double absoluteTime,
    double sampleDt) const
{
  MovingReference reference;
  reference.absoluteTime = absoluteTime;
  reference.timeToContact = plan.contactTime - absoluteTime;
  reference.objectPose = interceptionObjectPoseAt(plan, absoluteTime);
  interceptionObjectTwistAt(
      plan, absoluteTime,
      reference.objectLinearVelocityWorld,
      reference.objectAngularVelocityWorld);
  reference.mouthPose = interceptionMouthPoseAt(plan, absoluteTime);

  if(absoluteTime <= plan.standoffTime)
  {
    reference.phase = InterceptionPhase::Reach;
    reference.phaseProgress = plan.reachDuration > 1e-9
        ? clamp01((absoluteTime - plan.reachStartTime) / plan.reachDuration)
        : 1.0;
  }
  else if(absoluteTime <= plan.acquireStartTime)
  {
    reference.phase = InterceptionPhase::Approach;
    reference.phaseProgress = plan.approachDuration > 1e-9
        ? clamp01((absoluteTime - plan.standoffTime) / plan.approachDuration)
        : 1.0;
  }
  else if(absoluteTime <= plan.contactTime)
  {
    reference.phase = InterceptionPhase::Acquire;
    reference.phaseProgress = plan.acquireDuration > 1e-9
        ? clamp01((absoluteTime - plan.acquireStartTime) / plan.acquireDuration)
        : 1.0;
  }
  else
  {
    reference.phase = InterceptionPhase::Confirm;
    reference.phaseProgress = plan.confirmationDuration > 1e-9
        ? clamp01((absoluteTime - plan.contactTime) / plan.confirmationDuration)
        : 1.0;
  }

  if(plan.presentationMode)
  {
    // Predictive presentation is open-gripper until Acquire. This reference
    // is used for planning/logging; physical closure authority is enforced
    // separately by commandGripper().
    reference.gripperClosure = 0.0;
    if(absoluteTime > plan.acquireStartTime)
    {
      const double closureElapsed = absoluteTime - plan.acquireStartTime;
      reference.gripperClosure = std::min(
          plan.contactClosure, timingEffectiveGripperRate_ * closureElapsed);
    }
  }
  else
  {
    const double closureDuration = plan.contactClosure > 0.0
        ? plan.contactClosure / std::max(1e-6, timingEffectiveGripperRate_)
        : 0.0;
    const double closureStartTime = plan.contactTime - closureDuration;
    if(absoluteTime > closureStartTime && closureDuration > 0.0)
    {
      reference.gripperClosure = std::min(
          plan.contactClosure, timingEffectiveGripperRate_
            * (absoluteTime - closureStartTime));
    }
  }

  const double dt = std::max(1e-6, sampleDt);
  const sva::PTransformd nextMouth = interceptionMouthPoseAt(
      plan, absoluteTime + dt);
  worldPoseTwist(reference.mouthPose, nextMouth, dt,
                 reference.mouthLinearVelocityWorld,
                 reference.mouthAngularVelocityWorld);
  return reference;
}

HandoverInterceptionController::ContactConfirmationTransition
HandoverInterceptionController::advanceContactConfirmation(
    ContactConfirmationState & state,
    bool validContactEvent,
    bool bilateralContactInsideTube,
    double dt,
    double bilateralDwell,
    double confirmationDwell,
    double confirmationTimeout) const
{
  ContactConfirmationTransition transition;
  dt = std::max(0.0, dt);
  bilateralDwell = std::max(0.0, bilateralDwell);
  confirmationDwell = std::max(0.0, confirmationDwell);
  confirmationTimeout = std::max(confirmationDwell, confirmationTimeout);

  if(state.confirmed || state.timedOut)
  {
    transition.becameConfirmed = state.confirmed;
    transition.timedOut = state.timedOut;
    return transition;
  }

  if(!validContactEvent)
  {
    state.bilateralStableTime = 0.0;
    return transition;
  }

  if(!state.confirming)
  {
    if(bilateralContactInsideTube)
    {
      state.bilateralStableTime += dt;
    }
    else
    {
      state.bilateralStableTime = 0.0;
    }

    if(state.bilateralStableTime + 1e-12 >= bilateralDwell)
    {
      state.confirming = true;
      state.confirmationStableTime = 0.0;
      state.confirmationElapsed = 0.0;
      transition.enteredConfirming = true;
    }
    return transition;
  }

  state.confirmationElapsed += dt;
  if(bilateralContactInsideTube)
  {
    state.confirmationStableTime += dt;
  }
  else
  {
    // Runtime and preview share the same reset-on-loss rule.
    state.confirmationStableTime = 0.0;
  }

  if(state.confirmationStableTime + 1e-12 >= confirmationDwell)
  {
    state.confirmed = true;
    transition.becameConfirmed = true;
  }
  else if(state.confirmationElapsed + 1e-12 >= confirmationTimeout)
  {
    state.timedOut = true;
    transition.timedOut = true;
  }
  return transition;
}

bool HandoverInterceptionController::validateInterceptionPlan(
    const InterceptionPlan & plan,
    std::string * reason,
    bool verbose) const
{
  auto fail = [&](const std::string & why)
  {
    if(reason) { *reason = why; }
    if(verbose)
    {
      mc_rtc::log::error(
          "[MethodologyLock] invalid interception plan candidate={} reason={}",
          plan.candidateName, why);
    }
    return false;
  };

  if(!plan.valid) { return fail("plan_not_marked_valid"); }
  if(plan.objectMode == ObservedObjectMode::Unclassified)
  {
    return fail("object_mode_unclassified");
  }
  const double values[] = {plan.presentationTime,
      plan.decelerationStartTime, plan.decelerationDuration,
      plan.contactTime, plan.reachStartTime, plan.standoffTime,
      plan.acquireStartTime, plan.acquisitionDeadlineTime,
      plan.confirmationEndTime, plan.reachDuration, plan.approachDuration,
      plan.acquireDuration, plan.acquisitionWindowDuration,
      plan.confirmationDuration, plan.forceTransferDuration,
      plan.forceTransferTimeout, plan.forceTransferFullSupportForce,
      plan.forceTransferReleaseThreshold,
      plan.forceTransferReleaseRateTolerance,
      plan.forceTransferReleaseDwell,
      plan.forceTransferMaximumForce, plan.forceTransferMaximumMoment,
      plan.forceTransferAdmittanceMass,
      plan.forceTransferAdmittanceDamping,
      plan.forceTransferAdmittanceStiffness,
      plan.forceTransferMaximumAdmittanceSpeed,
      plan.forceTransferMaximumAdmittanceOffset,
      plan.forceTransferVirtualContactStiffness,
      plan.forceTransferVirtualContactDamping,
      plan.forceTransferVirtualMaximumLoadScale,
      plan.retreatDuration, plan.contactClosure};
  for(double value : values)
  {
    if(!std::isfinite(value)) { return fail("non_finite_plan_scalar"); }
  }
  auto poseFinite = [&](const sva::PTransformd & pose)
  {
    return pose.translation().allFinite() && worldRotation(pose).allFinite();
  };
  if(!poseFinite(plan.objectAtPresentation)
     || !poseFinite(plan.objectAtContact)
     || !poseFinite(plan.mouthAtReachStart)
     || !poseFinite(plan.O_T_M_standoff)
     || !poseFinite(plan.O_T_M_capture)
     || !poseFinite(plan.O_T_M_retreat)
     || !plan.reachCurveOffsetWorld.allFinite()
     || !plan.objectLinearVelocity.allFinite()
     || !plan.objectAngularVelocity.allFinite())
  {
    return fail("non_finite_plan_transform_or_twist");
  }
  if(plan.reachDuration <= 0.0 || plan.approachDuration <= 0.0
     || plan.acquireDuration <= 0.0)
  {
    return fail("non_positive_precontact_duration");
  }
  if(plan.forceTransferSource != "virtual_sensor"
     && plan.forceTransferSource != "synthetic"
     && plan.forceTransferSource != "force_sensor"
     && plan.forceTransferSource != "disabled")
  {
    return fail("unsupported_force_transfer_source");
  }
  if(plan.forceTransferDuration <= 0.0
     || plan.forceTransferTimeout < plan.forceTransferDuration
     || plan.forceTransferFullSupportForce <= 0.0
     || plan.forceTransferReleaseThreshold < 0.0
     || plan.forceTransferReleaseThreshold > 1.0
     || plan.forceTransferReleaseRateTolerance < 0.0
     || plan.forceTransferReleaseDwell < 0.0
     || plan.forceTransferMaximumForce <= 0.0
     || plan.forceTransferMaximumMoment <= 0.0
     || plan.forceTransferAdmittanceMass <= 0.0
     || plan.forceTransferMaximumAdmittanceSpeed < 0.0
     || plan.forceTransferMaximumAdmittanceOffset < 0.0
     || plan.forceTransferVirtualContactStiffness <= 0.0
     || plan.forceTransferVirtualContactDamping < 0.0
     || plan.forceTransferVirtualMaximumLoadScale < 1.0)
  {
    return fail("invalid_force_transfer_contract");
  }
  if(!(plan.reachStartTime < plan.standoffTime
       && plan.standoffTime < plan.acquireStartTime
       && plan.acquireStartTime < plan.contactTime
       && plan.contactTime <= plan.acquisitionDeadlineTime
       && plan.acquisitionDeadlineTime < plan.confirmationEndTime))
  {
    return fail("phase_time_ordering");
  }
  if(plan.presentationMode)
  {
    if(std::abs(plan.standoffTime - plan.presentationTime) > 1e-9)
    {
      return fail("presentation_not_at_standoff");
    }
    if(plan.acquisitionWindowDuration <= 0.0
       || std::abs(plan.acquisitionDeadlineTime
                  - (plan.acquireStartTime + plan.acquisitionWindowDuration))
              > 1e-9)
    {
      return fail("presentation_acquisition_window");
    }
    if(plan.objectMode == ObservedObjectMode::Static)
    {
      if(std::abs(plan.decelerationDuration) > 1e-12
         || std::abs(plan.decelerationStartTime - plan.presentationTime) > 1e-9
         || plan.objectLinearVelocity.norm() > 1e-9
         || plan.objectAngularVelocity.norm() > 1e-9)
      {
        return fail("static_presentation_contract");
      }
    }
    else
    {
      if(plan.decelerationDuration <= 0.0
         || std::abs(plan.decelerationStartTime
                    - (plan.presentationTime - plan.decelerationDuration))
                > 1e-9)
      {
        return fail("presentation_deceleration_timing");
      }
      if(plan.decelerationStartTime < plan.reachStartTime - 1e-9)
      {
        return fail("deceleration_before_reach_start");
      }
    }
  }
  else if(std::abs(plan.acquisitionDeadlineTime - plan.contactTime) > 1e-9
          || std::abs(plan.acquisitionWindowDuration) > 1e-12)
  {
    return fail("moving_contact_deadline_mismatch");
  }

  const double sumError = std::abs(
      (plan.contactTime - plan.reachStartTime)
      - (plan.reachDuration + plan.approachDuration + plan.acquireDuration));
  if(sumError > 1e-9) { return fail("phase_duration_sum"); }
  const sva::PTransformd objectAtPresentation = interceptionObjectPoseAt(
      plan, plan.presentationTime);
  const sva::PTransformd objectAtContact = interceptionObjectPoseAt(
      plan, plan.contactTime);
  if((objectAtPresentation.translation()
      - plan.objectAtPresentation.translation()).norm() > 1e-10
     || orientationError(objectAtPresentation,
                         plan.objectAtPresentation) > 1e-10)
  {
    return fail("presentation_anchor_not_fixed");
  }
  if((objectAtContact.translation()
      - plan.objectAtContact.translation()).norm() > 1e-10
     || orientationError(objectAtContact, plan.objectAtContact) > 1e-10)
  {
    return fail("contact_anchor_not_fixed");
  }

  if(plan.presentationMode)
  {
    if((plan.objectAtPresentation.translation()
        - plan.objectAtContact.translation()).norm() > 1e-12
       || orientationError(plan.objectAtPresentation,
                           plan.objectAtContact) > 1e-12)
    {
      return fail("presentation_contact_anchor_mismatch");
    }
    Eigen::Vector3d vPresentation, wPresentation;
    interceptionObjectTwistAt(plan, plan.presentationTime,
                              vPresentation, wPresentation);
    if(vPresentation.norm() > 1e-12 || wPresentation.norm() > 1e-12)
    {
      return fail("presentation_not_quasi_static");
    }
  }

  const double times[] = {plan.reachStartTime,
      plan.decelerationStartTime, plan.presentationTime,
      plan.acquireStartTime, plan.contactTime};
  sva::PTransformd previousPose = interceptionObjectPoseAt(plan, times[0]);
  for(size_t i = 1; i < 5; ++i)
  {
    const sva::PTransformd currentPose = interceptionObjectPoseAt(
        plan, times[i]);
    const Eigen::Vector3d actualTranslation =
        currentPose.translation() - previousPose.translation();
    if(plan.objectLinearVelocity.squaredNorm() > 1e-12
       && actualTranslation.dot(plan.objectLinearVelocity) < -1e-10)
    {
      return fail("object_motion_not_monotonic");
    }
    previousPose = currentPose;
  }

  if(plan.presentationMode)
  {
    const sva::PTransformd afterPresentation = interceptionObjectPoseAt(
        plan, plan.presentationTime + std::max(0.1, plan.approachDuration));
    if((afterPresentation.translation()
        - plan.objectAtPresentation.translation()).norm() > 1e-10
       || orientationError(afterPresentation,
                           plan.objectAtPresentation) > 1e-10)
    {
      return fail("presentation_hold_invariant");
    }
  }

  const double eps = 1e-6;
  const double boundaries[] = {plan.standoffTime, plan.acquireStartTime};
  for(double boundary : boundaries)
  {
    const sva::PTransformd before = interceptionMouthPoseAt(
        plan, boundary - eps);
    const sva::PTransformd at = interceptionMouthPoseAt(plan, boundary);
    const sva::PTransformd after = interceptionMouthPoseAt(
        plan, boundary + eps);
    if((before.translation() - at.translation()).norm() > 1e-4
       || (after.translation() - at.translation()).norm() > 1e-4
       || orientationError(before, at) > 1e-4
       || orientationError(after, at) > 1e-4)
    {
      return fail("mouth_reference_discontinuity");
    }
  }

  if(verbose)
  {
    const auto reach = interceptionReferenceAt(
        plan, plan.reachStartTime, previewDt_);
    const auto decel = interceptionReferenceAt(
        plan, plan.decelerationStartTime, previewDt_);
    const auto presentation = interceptionReferenceAt(
        plan, plan.presentationTime, previewDt_);
    const auto acquire = interceptionReferenceAt(
        plan, plan.acquireStartTime, previewDt_);
    const auto contact = interceptionReferenceAt(
        plan, plan.contactTime, previewDt_);
    const Eigen::Vector3d pr = reach.objectPose.translation();
    const Eigen::Vector3d pd = decel.objectPose.translation();
    const Eigen::Vector3d pp = presentation.objectPose.translation();
    const Eigen::Vector3d pa = acquire.objectPose.translation();
    const Eigen::Vector3d pc = contact.objectPose.translation();
    mc_rtc::log::success(
        "[PresentationPlan] candidate={} route={} objectMode={} immutable=true predictiveStatic=true presentationMode={} times=[reach:{:.3f},decel:{:.3f},present:{:.3f},acquire:{:.3f},nominalContact:{:.3f},deadline:{:.3f}] objectReach=[{:.3f},{:.3f},{:.3f}] objectDecel=[{:.3f},{:.3f},{:.3f}] objectPresentation=[{:.3f},{:.3f},{:.3f}] objectAcquire=[{:.3f},{:.3f},{:.3f}] objectContact=[{:.3f},{:.3f},{:.3f}] speedPresentation={:.5f} acquisitionWindow={:.3f}s openThroughInsertion=true fullClosure={:.3f} anchorError={:.3e}",
        plan.candidateName, plan.transitRouteName,
        plan.objectMode == ObservedObjectMode::Static ? "STATIC" : "MOVING",
        plan.presentationMode,
        plan.reachStartTime, plan.decelerationStartTime,
        plan.presentationTime, plan.acquireStartTime, plan.contactTime,
        plan.acquisitionDeadlineTime,
        pr.x(), pr.y(), pr.z(), pd.x(), pd.y(), pd.z(),
        pp.x(), pp.y(), pp.z(), pa.x(), pa.y(), pa.z(),
        pc.x(), pc.y(), pc.z(),
        presentation.objectLinearVelocityWorld.norm(),
        plan.acquisitionWindowDuration,
        plan.contactClosure,
        (contact.objectPose.translation()
         - plan.objectAtContact.translation()).norm());
  }
  if(reason) { *reason = "valid"; }
  return true;
}

HandoverInterceptionController::MovingReference
HandoverInterceptionController::committedReferenceAt(
    double absoluteTime,
    double sampleDt,
    bool useBoundedLiveCorrection,
    double maxTranslationCorrection,
    double maxRotationCorrection) const
{
  MovingReference reference = interceptionReferenceAt(
      committedInterceptionPlan_, absoluteTime, sampleDt);
  if(useBoundedLiveCorrection
     && reference.phase != InterceptionPhase::Reach)
  {
    const sva::PTransformd nominalObject = reference.objectPose;
    const sva::PTransformd correctedObject = boundedPoseStep(
        nominalObject, W_T_O_,
        std::max(0.0, maxTranslationCorrection),
        std::max(0.0, maxRotationCorrection));
    const sva::PTransformd O_T_M = relativePose(
        nominalObject, reference.mouthPose);
    reference.objectPose = correctedObject;
    reference.mouthPose = compose(correctedObject, O_T_M);
  }
  return reference;
}

// =============================================================================
// Object and physical mouth geometry
// =============================================================================

void HandoverInterceptionController::resetObjectPerceptionBuffer()
{
  objectPerceptionBuffer_.clear();
  objectPerceptionMeasurementTime_ = controllerTime_;
  objectPerceptionMeasurementAge_ = 0.0;
  lastObjectPerceptionTruthSampleTime_ = -1.0;
  objectPerceptionMeasurementValid_ = false;
  W_T_O_truth_ = W_T_O_config_;
  W_T_O_perceptionMeasurement_ = W_T_O_config_;
}

void HandoverInterceptionController::recordObjectPerceptionTruthSample(
    const sva::PTransformd & truthPose)
{
  W_T_O_truth_ = truthPose;
  if(lastObjectPerceptionTruthSampleTime_ < 0.0
     || controllerTime_ > lastObjectPerceptionTruthSampleTime_ + 1e-12)
  {
    objectPerceptionBuffer_.push_back({controllerTime_, truthPose});
    lastObjectPerceptionTruthSampleTime_ = controllerTime_;
  }
  else if(!objectPerceptionBuffer_.empty())
  {
    objectPerceptionBuffer_.back().pose = truthPose;
  }

  const double keepAfter = controllerTime_
      - std::max(perceptionLatencyBufferDuration_,
                 perceptionLatencySeconds_ + 2.0 * controlDt_);
  while(objectPerceptionBuffer_.size() > 2
        && objectPerceptionBuffer_[1].time < keepAfter)
  {
    objectPerceptionBuffer_.pop_front();
  }
}

void HandoverInterceptionController::selectDelayedObjectMeasurement()
{
  if(objectPerceptionBuffer_.empty())
  {
    W_T_O_perceptionMeasurement_ = W_T_O_truth_;
    objectPerceptionMeasurementTime_ = controllerTime_;
    objectPerceptionMeasurementAge_ = 0.0;
    objectPerceptionMeasurementValid_ = true;
    return;
  }

  if(!perceptionLatencyEnabled())
  {
    W_T_O_perceptionMeasurement_ = objectPerceptionBuffer_.back().pose;
    objectPerceptionMeasurementTime_ = objectPerceptionBuffer_.back().time;
    objectPerceptionMeasurementAge_ = std::max(
        0.0, controllerTime_ - objectPerceptionMeasurementTime_);
    objectPerceptionMeasurementValid_ = true;
    return;
  }

  const double targetTime = controllerTime_ - perceptionLatencySeconds_;
  if(targetTime <= objectPerceptionBuffer_.front().time)
  {
    W_T_O_perceptionMeasurement_ = objectPerceptionBuffer_.front().pose;
    objectPerceptionMeasurementTime_ = objectPerceptionBuffer_.front().time;
  }
  else if(targetTime >= objectPerceptionBuffer_.back().time)
  {
    W_T_O_perceptionMeasurement_ = objectPerceptionBuffer_.back().pose;
    objectPerceptionMeasurementTime_ = objectPerceptionBuffer_.back().time;
  }
  else
  {
    auto upper = objectPerceptionBuffer_.begin();
    while(upper != objectPerceptionBuffer_.end()
          && upper->time < targetTime)
    {
      ++upper;
    }
    const auto lower = std::prev(upper);
    const double span = std::max(1e-12, upper->time - lower->time);
    const double alpha = clamp01((targetTime - lower->time) / span);
    W_T_O_perceptionMeasurement_ = interpolatePose(
        lower->pose, upper->pose, alpha);
    objectPerceptionMeasurementTime_ = targetTime;
  }
  objectPerceptionMeasurementAge_ = std::max(
      0.0, controllerTime_ - objectPerceptionMeasurementTime_);
  objectPerceptionMeasurementValid_ = true;
}

void HandoverInterceptionController::applyObjectPerceptionEstimate()
{
  if(!objectPerceptionMeasurementValid_)
  {
    W_T_O_ = W_T_O_truth_;
  }
  else if(perceptionLatencyCompensationEnabled())
  {
    W_T_O_ = W_T_O_perceptionMeasurement_;
    const double age = objectPerceptionMeasurementAge_;
    Eigen::Vector3d p = W_T_O_.translation()
        + age * objectLinearVelocityEstimate_;
    Eigen::Matrix3d R = worldRotation(W_T_O_);
    const double omegaNorm = objectAngularVelocityEstimate_.norm();
    if(omegaNorm > 1e-9 && age > 0.0)
    {
      const Eigen::Matrix3d dR = Eigen::AngleAxisd(
          omegaNorm * age,
          objectAngularVelocityEstimate_ / omegaNorm).toRotationMatrix();
      R = dR * R;
    }
    W_T_O_ = fromWorldPose(R, p);
  }
  else
  {
    W_T_O_ = W_T_O_perceptionMeasurement_;
  }
  W_T_H_ = compose(W_T_O_, O_T_H_);
}

void HandoverInterceptionController::refreshObjectPose()
{
  sva::PTransformd latest = W_T_O_config_;

  if(simulatedObjectMotionActive_ || simulatedObjectPoseFrozen_)
  {
    latest = W_T_O_simulated_;
  }
  else if(robots().hasRobot(objectRobotName_))
  {
    auto & object = robots().robot(objectRobotName_);
    if(object.hasFrame(objectFrameName_))
    {
      latest = object.frame(objectFrameName_).position();
    }
    else
    {
      static bool warned = false;
      if(!warned)
      {
        mc_rtc::log::warning(
            "[ObjectGeometry] robot {} has no frame {}; using configured object pose",
            objectRobotName_, objectFrameName_);
        warned = true;
      }
    }
  }

  recordObjectPerceptionTruthSample(latest);
  selectDelayedObjectMeasurement();
  applyObjectPerceptionEstimate();
}

void HandoverInterceptionController::beginObjectObservation()
{
  objectObservationActive_ = true;
  simulatedObjectMotionActive_ = false;
  simulatedObjectPoseFrozen_ = false;
  havePreviousObjectObservation_ = false;
  objectMotionEstimateValid_ = false;
  observedObjectMode_ = ObservedObjectMode::Unclassified;
  objectObservationSamples_ = 0;
  objectObservationStartTime_ = controllerTime_;
  previousObjectObservationTime_ = controllerTime_;
  refreshObjectPose();
  previousObjectObservationTime_ = objectPerceptionMeasurementTime_;
  W_T_O_observationStart_ = W_T_O_;
  W_T_O_simulated_ = W_T_O_truth_;
  simulatedObjectMotionActive_ = simulateMovingObject_;
  W_T_O_previousObservation_ = W_T_O_;
  W_T_O_predicted_ = W_T_O_;
  objectLinearVelocityEstimate_.setZero();
  objectAngularVelocityEstimate_.setZero();

  mc_rtc::log::warning(
      "[PerceptionLatency] mode={} configuredDelay={:.3f}s measurementAge={:.3f}s compensate={} predictionHorizon={:.3f}s horizonRetuned=false controlThreadSleep=false",
      perceptionLatencyModeName(), perceptionLatencySeconds(),
      objectPerceptionMeasurementAge_,
      perceptionLatencyCompensationEnabled(), objectPredictionHorizon_);

  mc_rtc::log::warning(
      "[ObjectEstimator] observation armed duration={:.2f}s predictionHorizon={:.2f}s simulate={} vSim=[{:.3f},{:.3f},{:.3f}]m/s wSim=[{:.3f},{:.3f},{:.3f}]rad/s",
      objectObservationDuration_, objectPredictionHorizon_,
      simulateMovingObject_, simulatedObjectLinearVelocity_.x(),
      simulatedObjectLinearVelocity_.y(), simulatedObjectLinearVelocity_.z(),
      simulatedObjectAngularVelocity_.x(),
      simulatedObjectAngularVelocity_.y(),
      simulatedObjectAngularVelocity_.z());
}

void HandoverInterceptionController::endObjectObservation(
    bool freezeSimulatedObject)
{
  if(simulateMovingObject_)
  {
    // V4A.2 distinguishes "observation ended" from "object stopped". When
    // freezeSimulatedObject is false, the deterministic object trajectory
    // continues while the physical robot remains stationary and the future
    // contact event is solved entirely on copied robot states.
    simulatedObjectMotionActive_ = !freezeSimulatedObject;
    simulatedObjectPoseFrozen_ = freezeSimulatedObject;
    W_T_O_ = W_T_O_simulated_;
    W_T_H_ = compose(W_T_O_, O_T_H_);
  }
  objectObservationActive_ = false;
  refreshObjectPose();
  W_T_O_predicted_ = predictObjectPose(objectPredictionHorizon_);
}

double HandoverInterceptionController::objectObservationElapsed() const
{
  if(!objectObservationActive_)
  {
    return std::max(0.0, previousObjectObservationTime_
                         - objectObservationStartTime_);
  }
  return std::max(0.0, controllerTime_ - objectObservationStartTime_);
}

double HandoverInterceptionController::objectObservationDisplacement() const
{
  return (W_T_O_.translation()
        - W_T_O_observationStart_.translation()).norm();
}

sva::PTransformd HandoverInterceptionController::predictObjectPose(
    double horizon) const
{
  horizon = std::max(0.0, horizon);
  Eigen::Vector3d p = W_T_O_.translation()
                    + horizon * objectLinearVelocityEstimate_;
  Eigen::Matrix3d R = worldRotation(W_T_O_);
  const double omegaNorm = objectAngularVelocityEstimate_.norm();
  if(omegaNorm > 1e-9 && horizon > 0.0)
  {
    const Eigen::Matrix3d dR = Eigen::AngleAxisd(
        omegaNorm * horizon,
        objectAngularVelocityEstimate_ / omegaNorm).toRotationMatrix();
    R = dR * R;
  }
  return fromWorldPose(R, p);
}

// =============================================================================
// Committed moving-object reference (V4A.6 methodology lock)
// =============================================================================

sva::PTransformd HandoverInterceptionController::committedObjectPoseAt(
    double absoluteTime) const
{
  if(!committedPlanValid()) { return W_T_O_; }
  return interceptionObjectPoseAt(
      committedInterceptionPlan_, absoluteTime);
}

sva::PTransformd
HandoverInterceptionController::committedObjectPoseWithBoundedCorrection(
    double absoluteTime,
    double maxTranslationCorrection,
    double maxRotationCorrection) const
{
  const sva::PTransformd nominal = committedObjectPoseAt(absoluteTime);
  return boundedPoseStep(nominal, W_T_O_,
                         std::max(0.0, maxTranslationCorrection),
                         std::max(0.0, maxRotationCorrection));
}

sva::PTransformd HandoverInterceptionController::committedStandoffTargetAt(
    double absoluteTime) const
{
  if(!committedPlanValid()) { return W_T_M_standoff_; }
  return compose(committedObjectPoseAt(absoluteTime),
                 committedInterceptionPlan_.O_T_M_standoff);
}

sva::PTransformd HandoverInterceptionController::committedCaptureTargetAt(
    double absoluteTime) const
{
  if(!committedPlanValid()) { return W_T_M_pre_; }
  return compose(committedObjectPoseAt(absoluteTime),
                 committedInterceptionPlan_.O_T_M_capture);
}

sva::PTransformd HandoverInterceptionController::committedApproachTargetAt(
    double absoluteTime,
    double relativeProgress,
    bool useBoundedLiveCorrection,
    double maxTranslationCorrection,
    double maxRotationCorrection) const
{
  if(!committedPlanValid()) { return W_T_M_pre_; }
  const sva::PTransformd O_T_M = interpolatePose(
      committedInterceptionPlan_.O_T_M_standoff,
      committedInterceptionPlan_.O_T_M_capture, relativeProgress);
  const sva::PTransformd W_T_O_reference = useBoundedLiveCorrection
      ? committedObjectPoseWithBoundedCorrection(
            absoluteTime, maxTranslationCorrection, maxRotationCorrection)
      : committedObjectPoseAt(absoluteTime);
  return compose(W_T_O_reference, O_T_M);
}

double HandoverInterceptionController::committedObjectPositionErrorAt(
    double absoluteTime) const
{
  return (W_T_O_.translation()
        - committedObjectPoseAt(absoluteTime).translation()).norm();
}

double HandoverInterceptionController::committedObjectOrientationErrorAt(
    double absoluteTime) const
{
  return orientationError(W_T_O_, committedObjectPoseAt(absoluteTime));
}

void HandoverInterceptionController::updateSimulatedObjectMotion()
{
  if(!simulatedObjectMotionActive_ || objectAttached_) { return; }
  if(!robots().hasRobot(objectRobotName_)) { return; }

  sva::PTransformd target = sva::PTransformd::Identity();
  if(committedPlanValid() && committedInterceptionPlan_.presentationMode)
  {
    // The visible simulation truth follows the same committed timing and
    // deceleration law, but its trajectory is anchored to the actual object
    // pose at commit. It is never re-anchored to a delayed measurement.
    const auto & truthPlan = simulatedTruthInterceptionPlanValid_
        ? simulatedTruthInterceptionPlan_ : committedInterceptionPlan_;
    target = interceptionObjectPoseAt(truthPlan, controllerTime_);
  }
  else
  {
    const double t = std::max(
        0.0, controllerTime_ - objectObservationStartTime_);
    Eigen::Vector3d travel = simulatedObjectLinearVelocity_ * t;
    const double travelNorm = travel.norm();
    if(objectMaximumSimulatedTravel_ > 0.0
       && travelNorm > objectMaximumSimulatedTravel_)
    {
      travel *= objectMaximumSimulatedTravel_ / travelNorm;
    }

    Eigen::Matrix3d R = worldRotation(W_T_O_observationStart_);
    const double omegaNorm = simulatedObjectAngularVelocity_.norm();
    if(omegaNorm > 1e-9)
    {
      const Eigen::Matrix3d dR = Eigen::AngleAxisd(
          omegaNorm * t,
          simulatedObjectAngularVelocity_ / omegaNorm).toRotationMatrix();
      R = dR * R;
    }
    target = fromWorldPose(
        R, W_T_O_observationStart_.translation() + travel);
  }
  W_T_O_simulated_ = target;
  try
  {
    robots().robot(objectRobotName_).posW(target);
  }
  catch(const std::exception & e)
  {
    simulatedObjectMotionActive_ = false;
    mc_rtc::log::error(
        "[ObjectEstimator] simulated object pose update failed: {}", e.what());
  }
}

void HandoverInterceptionController::updateObjectMotionEstimate()
{
  if(!objectObservationActive_ && !interceptionCommitted_) { return; }
  if(!objectPerceptionMeasurementValid_) { return; }

  if(!havePreviousObjectObservation_)
  {
    havePreviousObjectObservation_ = true;
    W_T_O_previousObservation_ = W_T_O_perceptionMeasurement_;
    previousObjectObservationTime_ = objectPerceptionMeasurementTime_;
    applyObjectPerceptionEstimate();
    W_T_O_predicted_ = predictObjectPose(objectPredictionHorizon_);
    return;
  }

  const double sampleDt = objectPerceptionMeasurementTime_
      - previousObjectObservationTime_;
  if(sampleDt <= 1e-9)
  {
    applyObjectPerceptionEstimate();
    W_T_O_predicted_ = predictObjectPose(objectPredictionHorizon_);
    return;
  }

  const Eigen::Vector3d rawLinear =
      (W_T_O_perceptionMeasurement_.translation()
       - W_T_O_previousObservation_.translation()) / sampleDt;

  const Eigen::Matrix3d RCurrent = worldRotation(
      W_T_O_perceptionMeasurement_);
  const Eigen::Matrix3d RPrevious = worldRotation(
      W_T_O_previousObservation_);
  const Eigen::Matrix3d dR = RCurrent * RPrevious.transpose();
  Eigen::AngleAxisd aa(dR);
  Eigen::Vector3d rawAngular = Eigen::Vector3d::Zero();
  if(std::isfinite(aa.angle()) && aa.axis().allFinite())
  {
    rawAngular = aa.axis() * aa.angle() / sampleDt;
  }

  const bool finite = rawLinear.allFinite() && rawAngular.allFinite();
  const bool bounded = rawLinear.norm() <= objectMaximumLinearSpeed_
                    && rawAngular.norm() <= objectMaximumAngularSpeed_;
  if(finite && bounded)
  {
    const double alpha = std::max(0.0, std::min(1.0,
        sampleDt / (objectVelocityFilterTimeConstant_ + sampleDt)));
    objectLinearVelocityEstimate_ =
        (1.0 - alpha) * objectLinearVelocityEstimate_ + alpha * rawLinear;
    objectAngularVelocityEstimate_ =
        (1.0 - alpha) * objectAngularVelocityEstimate_ + alpha * rawAngular;
    ++objectObservationSamples_;
  }

  W_T_O_previousObservation_ = W_T_O_perceptionMeasurement_;
  previousObjectObservationTime_ = objectPerceptionMeasurementTime_;
  applyObjectPerceptionEstimate();
  W_T_O_predicted_ = predictObjectPose(objectPredictionHorizon_);

  objectMotionEstimateValid_ =
      objectObservationElapsed() >= objectMinimumObservationTime_
      && objectObservationSamples_ >= objectMinimumObservationSamples_;
}

void HandoverInterceptionController::refreshSelectedWorldTargets()
{
  if(!candidateSelected_) { return; }
  W_T_M_transit_ = compose(W_T_O_, O_T_M_transit_);
  W_T_M_standoff_ = compose(W_T_O_, O_T_M_standoff_);
  W_T_M_pre_ = compose(W_T_O_, O_T_M_pre_);
}

void HandoverInterceptionController::setPlanningObjectSnapshot(
    const sva::PTransformd & W_T_O_snapshot)
{
  planningObjectSnapshotActive_ = true;
  W_T_O_planningSnapshot_ = W_T_O_snapshot;
}

void HandoverInterceptionController::applyPlanningObjectSnapshot()
{
  if(!planningObjectSnapshotActive_) { return; }
  W_T_O_ = W_T_O_planningSnapshot_;
  W_T_H_ = compose(W_T_O_, O_T_H_);
  W_T_O_predicted_ = W_T_O_;
}

void HandoverInterceptionController::clearPlanningObjectSnapshot()
{
  planningObjectSnapshotActive_ = false;
  refreshObjectPose();
  W_T_O_predicted_ = predictObjectPose(objectPredictionHorizon_);
}

bool HandoverInterceptionController::attachObjectToMouth()
{
  if(objectAttached_) { return true; }
  HandoverSafetyReport report;
  if(!bilateralPadContactReached(&report, true))
  {
    mc_rtc::log::error(
        "[ObjectAttach] refusing attachment without verified bilateral contact left={:.4f} right={:.4f} centerErr={:.4f}",
        report.leftPadClearance, report.rightPadClearance,
        report.padCenteringError);
    return false;
  }

  M_T_O_attached_ = relativePose(actualMouthPose(), W_T_O_);
  if(interceptionCommitted_)
  {
    // Preserve the already-certified object-relative retreat geometry and
    // anchor it at the actual object pose at attachment. This is a deterministic
    // continuation of the committed plan, not a new candidate or a replan.
    W_T_M_retreat_ = compose(W_T_O_, O_T_M_retreat_);
  }
  objectAttached_ = true;
  updateAttachedObjectPose();
  mc_rtc::log::success(
      "[ObjectAttach] rigid mouth-object transform established; carried-object retreat is now active");
  return true;
}

bool HandoverInterceptionController::lockStaticPresentationToCurrentObject()
{
  if(!committedPlanValid() || !candidateSelected_ || objectAttached_)
  {
    mc_rtc::log::error(
        "[TerminalAnchor] cannot lock stopped presentation: committed candidate unavailable or object already attached");
    return false;
  }

  refreshObjectPose();
  const sva::PTransformd nominalObject =
      committedInterceptionPlan_.objectAtPresentation;
  const double translationCorrection =
      (W_T_O_.translation() - nominalObject.translation()).norm();
  const double rotationCorrection = orientationError(W_T_O_, nominalObject);

  W_T_M_transit_ = compose(W_T_O_, O_T_M_standoff_);
  W_T_M_standoff_ = W_T_M_transit_;
  W_T_M_pre_ = compose(W_T_O_, O_T_M_pre_);
  W_T_M_acquired_ = W_T_M_pre_;
  W_T_M_retreat_ = compose(W_T_O_, O_T_M_retreat_);

  const Eigen::Vector3d p = W_T_M_pre_.translation();
  mc_rtc::log::success(
      "[TerminalAnchor] committed object-relative grasp locked to measured stopped presentation translationCorrection={:.4f} rotationCorrection={:.4f} capture=[{:.3f},{:.3f},{:.3f}] candidateUnchanged=true",
      translationCorrection, rotationCorrection, p.x(), p.y(), p.z());
  return true;
}

void HandoverInterceptionController::detachObject()
{
  objectAttached_ = false;
  M_T_O_attached_ = sva::PTransformd::Identity();
}

void HandoverInterceptionController::updateAttachedObjectPose()
{
  if(!objectAttached_) { return; }
  W_T_O_ = compose(actualMouthPose(), M_T_O_attached_);
  W_T_H_ = compose(W_T_O_, O_T_H_);

  if(!simulateKinematicAttachment_ || !robots().hasRobot(objectRobotName_))
  {
    return;
  }

  try
  {
    // This updates the additional free-flyer/fixed object robot and its forward
    // kinematics so RViz displays the object as carried after transfer.
    robots().robot(objectRobotName_).posW(W_T_O_);
  }
  catch(const std::exception & e)
  {
    static bool warned = false;
    if(!warned)
    {
      warned = true;
      mc_rtc::log::warning(
          "[TransferConfirm] object robot pose could not be updated kinematically: {}",
          e.what());
    }
  }
}

sva::PTransformd HandoverInterceptionController::actualBasePose() const
{
  return robot().frame(toolFrame_).position();
}

bool HandoverInterceptionController::livePadCenters(
    Eigen::Vector3d & pL_W,
    Eigen::Vector3d & pR_W) const
{
  static const std::string leftTip = "gen3_robotiq_85_left_finger_tip_link";
  static const std::string rightTip = "gen3_robotiq_85_right_finger_tip_link";
  if(!robot().hasFrame(leftTip) || !robot().hasFrame(rightTip)) { return false; }

  pL_W = pointToWorld(robot().frame(leftTip).position(), leftPadPointTip_);
  pR_W = pointToWorld(robot().frame(rightTip).position(), rightPadPointTip_);
  return true;
}

sva::PTransformd HandoverInterceptionController::actualMouthPose() const
{
  static const std::string leftTip = "gen3_robotiq_85_left_finger_tip_link";
  static const std::string rightTip = "gen3_robotiq_85_right_finger_tip_link";

  const sva::PTransformd W_T_B = actualBasePose();
  Eigen::Vector3d pL, pR;
  if(livePadCenters(pL, pR))
  {
    const Eigen::Vector3d pM = 0.5 * (pL + pR);
    const Eigen::Matrix3d R_W_B = worldRotation(W_T_B);

    Eigen::Vector3d xM = pL - pR; // right -> left, finger-closing axis
    if(xM.norm() < 1e-9) { xM = R_W_B.col(0); }
    xM.normalize();

    Eigen::Vector3d yM = -R_W_B.col(2); // outward toward gripper base
    yM -= xM * xM.dot(yM);
    if(yM.norm() < 1e-9) { yM = R_W_B.col(1); }
    yM.normalize();

    Eigen::Vector3d zM = xM.cross(yM);
    if(zM.norm() < 1e-9) { zM = R_W_B.col(1); }
    zM.normalize();
    yM = zM.cross(xM).normalized();

    Eigen::Matrix3d R_W_M;
    R_W_M.col(0) = xM;
    R_W_M.col(1) = yM;
    R_W_M.col(2) = zM;
    return fromWorldPose(R_W_M, pM);
  }

  return mouthPoseFromBasePose(W_T_B);
}

bool HandoverInterceptionController::calibrateMouthControlFrame()
{
  static const std::string leftTip = "gen3_robotiq_85_left_finger_tip_link";
  static const std::string rightTip = "gen3_robotiq_85_right_finger_tip_link";
  if(!robot().hasFrame(leftTip) || !robot().hasFrame(rightTip))
  {
    B_T_M_control_ = B_T_M_config_;
    mouthCalibrationValid_ = false;
    mc_rtc::log::error(
        "[MouthCalibration] fingertip frames are unavailable; refusing candidate execution instead of guessing");
    return false;
  }

  const sva::PTransformd W_T_B = actualBasePose();
  const sva::PTransformd W_T_M = actualMouthPose();
  B_T_M_control_ = relativePose(W_T_B, W_T_M);
  mouthCalibrationValid_ = true;
  calibratedOpenHalfGap_ = liveMouthHalfGap();

  const Eigen::Vector3d p = B_T_M_control_.translation();
  mc_rtc::log::success(
      "[MouthCalibration] calibrated physical B_T_M p_B=[{:.5f},{:.5f},{:.5f}] openHalfGap={:.5f}",
      p.x(), p.y(), p.z(), calibratedOpenHalfGap_);
  return true;
}

sva::PTransformd HandoverInterceptionController::mouthPoseFromBasePose(
    const sva::PTransformd & W_T_B) const
{
  sva::PTransformd B_T_M = mouthCalibrationValid_ ? B_T_M_control_ : B_T_M_config_;
  Eigen::Vector3d pL, pR;
  if(mouthCalibrationValid_ && livePadCenters(pL, pR))
  {
    // Keep the command map consistent with the current gripper aperture.
    B_T_M = relativePose(actualBasePose(), actualMouthPose());
  }
  return compose(W_T_B, B_T_M);
}

sva::PTransformd HandoverInterceptionController::basePoseFromMouthPose(
    const sva::PTransformd & W_T_M) const
{
  sva::PTransformd B_T_M = mouthCalibrationValid_ ? B_T_M_control_ : B_T_M_config_;
  Eigen::Vector3d pL, pR;
  if(mouthCalibrationValid_ && livePadCenters(pL, pR))
  {
    // The B->pad-mouth transform changes during Robotiq closure. Use the live
    // inverse so the arm holds the capture frame while the fingers move.
    B_T_M = relativePose(actualBasePose(), actualMouthPose());
  }
  const Eigen::Matrix3d R_W_M = worldRotation(W_T_M);
  const Eigen::Matrix3d R_B_M = worldRotation(B_T_M);
  const Eigen::Vector3d p_B_M = B_T_M.translation();

  const Eigen::Matrix3d R_W_B = R_W_M * R_B_M.transpose();
  const Eigen::Vector3d p_W_B = W_T_M.translation() - R_W_B * p_B_M;
  return fromWorldPose(R_W_B, p_W_B);
}

void HandoverInterceptionController::clearToolTaskReferenceMotion()
{
  if(!toolTask_) { return; }
  const sva::MotionVecd zeroMotion(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  toolTask_->refVelB(zeroMotion);
  toolTask_->refAccel(zeroMotion);
}

void HandoverInterceptionController::commandBaseTarget(
    const sva::PTransformd & W_T_B_cmd)
{
  if(!toolTask_) { return; }
  toolTask_->target(W_T_B_cmd);
  clearToolTaskReferenceMotion();
}

void HandoverInterceptionController::commandMouthTarget(
    const sva::PTransformd & W_T_M_cmd)
{
  commandBaseTarget(basePoseFromMouthPose(W_T_M_cmd));
}

void HandoverInterceptionController::commandBaseTargetWithWorldVelocity(
    const sva::PTransformd & W_T_B_cmd,
    const Eigen::Vector3d & linearVelocityWorld,
    const Eigen::Vector3d & angularVelocityWorld)
{
  if(!toolTask_) { return; }
  toolTask_->target(W_T_B_cmd);
  const Eigen::Matrix3d R_W_B = worldRotation(actualBasePose());
  const sva::MotionVecd bodyVelocity(
      R_W_B.transpose() * angularVelocityWorld,
      R_W_B.transpose() * linearVelocityWorld);
  toolTask_->refVelB(bodyVelocity);
  toolTask_->refAccel(sva::MotionVecd(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
}

void HandoverInterceptionController::commandMouthTargetWithWorldVelocity(
    const sva::PTransformd & W_T_M_cmd,
    const Eigen::Vector3d & linearVelocityWorld,
    const Eigen::Vector3d & angularVelocityWorld)
{
  const sva::PTransformd W_T_B_cmd = basePoseFromMouthPose(W_T_M_cmd);
  const Eigen::Vector3d linearVelocityBaseWorld =
      linearVelocityWorld
      + angularVelocityWorld.cross(
          W_T_B_cmd.translation() - W_T_M_cmd.translation());
  commandBaseTargetWithWorldVelocity(
      W_T_B_cmd, linearVelocityBaseWorld, angularVelocityWorld);
}

double HandoverInterceptionController::liveMouthGap() const
{
  Eigen::Vector3d pL, pR;
  if(livePadCenters(pL, pR)) { return (pL - pR).norm(); }
  return 2.0 * calibratedOpenHalfGap_;
}

double HandoverInterceptionController::measuredGripperClosure() const
{
  // On hardware, closure evidence comes directly from the physical Robotiq
  // feedback channel. The reconstructed six-joint model remains the geometry
  // representation, but it is not the authority for physical acquisition.
  if(physicalGripperBridgeEnabled_)
  {
    if(!physicalGripperFeedbackValid_) { return 0.0; }
    const double span = std::max(
        1e-6, physicalGripperClosePercent_ - physicalGripperOpenPercent_);
    return std::max(
        0.0,
        std::min(
            1.0,
            (physicalGripperMeasuredPercent_ - physicalGripperOpenPercent_)
                / span));
  }

  const std::string jointName = "gen3_robotiq_85_left_knuckle_joint";
  try
  {
    const int idx = robot().mb().jointIndexByName(jointName);
    if(idx < 0 || static_cast<size_t>(idx) >= robot().mbc().q.size()
       || robot().mbc().q[static_cast<size_t>(idx)].empty())
    {
      return gripperCommand_;
    }
    const double q = robot().mbc().q[static_cast<size_t>(idx)][0];
    const double den = gripperCloseQ_ - gripperOpenQ_;
    if(std::abs(den) < 1e-9) { return gripperCommand_; }
    return std::max(0.0, std::min(1.0, (q - gripperOpenQ_) / den));
  }
  catch(const std::exception &)
  {
    return gripperCommand_;
  }
}

bool HandoverInterceptionController::hasJoint(const std::string & jointName) const
{
  try
  {
    return robot().mb().jointIndexByName(jointName) >= 0;
  }
  catch(const std::exception &)
  {
    return false;
  }
}

bool HandoverInterceptionController::hasActuatedJoint(
    const std::string & jointName) const
{
  try
  {
    const int idx = robot().mb().jointIndexByName(jointName);
    return idx >= 0
        && static_cast<size_t>(idx) < robot().mbc().q.size()
        && !robot().mbc().q[static_cast<size_t>(idx)].empty();
  }
  catch(const std::exception &)
  {
    return false;
  }
}

bool HandoverInterceptionController::gripperActuationAvailable() const
{
  return hasActuatedJoint("gen3_robotiq_85_left_knuckle_joint");
}

std::string HandoverInterceptionController::gripperActuationStatus() const
{
  if(gripperActuationAvailable())
  {
    return "actuated Robotiq joint model available";
  }
  return "Robotiq joints are fixed/non-actuated in the loaded RobotModule";
}

bool HandoverInterceptionController::refreshGripperGeometry(bool verbose)
{
  const sva::PTransformd W_T_B = actualBasePose();
  gripperSamplesB_.clear();

  auto addLocal = [this](const std::string & name,
                         const Eigen::Vector3d & p_B,
                         double radius,
                         bool hardAgainstBlue)
  {
    GripperSample s;
    s.name = name;
    s.frame = toolFrame_;
    s.p_F = p_B;
    s.p_B = p_B;
    s.radius = radius;
    s.hardAgainstBlue = hardAgainstBlue;
    gripperSamplesB_.push_back(s);
  };

  auto addFrameLocal = [this, &W_T_B](
      const std::string & label,
      const std::string & frame,
      const Eigen::Vector3d & p_F,
      double radius,
      bool hardAgainstBlue)
  {
    if(!robot().hasFrame(frame)) { return; }
    const Eigen::Vector3d p_W = pointToWorld(robot().frame(frame).position(), p_F);
    GripperSample s;
    s.name = label;
    s.frame = frame;
    s.p_F = p_F;
    s.p_B = pointToLocal(W_T_B, p_W);
    s.radius = radius;
    s.hardAgainstBlue = hardAgainstBlue;
    gripperSamplesB_.push_back(s);
  };

  // Base/palm proxies cover the uploaded robotiq_base.stl while avoiding the
  // former single oversized sphere. All are hard obstacles.
  addLocal("base_rear", Eigen::Vector3d(0.0, 0.0, 0.010), 0.032, true);
  addLocal("palm_front_center", Eigen::Vector3d(0.0, 0.0, 0.045), 0.020, true);
  for(double x : {-0.026, 0.026})
  {
    for(double y : {-0.022, 0.022})
    {
      addLocal(std::string("palm_corner_") + (x > 0 ? "p" : "m")
                   + (y > 0 ? "p" : "m"),
               Eigen::Vector3d(x, y, 0.048), 0.013, true);
    }
  }

  const std::string lK = "gen3_robotiq_85_left_knuckle_link";
  const std::string rK = "gen3_robotiq_85_right_knuckle_link";
  const std::string lIK = "gen3_robotiq_85_left_inner_knuckle_link";
  const std::string rIK = "gen3_robotiq_85_right_inner_knuckle_link";
  const std::string lF = "gen3_robotiq_85_left_finger_link";
  const std::string rF = "gen3_robotiq_85_right_finger_link";
  const std::string lT = "gen3_robotiq_85_left_finger_tip_link";
  const std::string rT = "gen3_robotiq_85_right_finger_tip_link";

  // Mesh-derived knuckle proxies. Coordinates come from the uploaded STL
  // bounds and are transformed through the live link frames at every refresh.
  for(const auto & side : std::vector<std::pair<std::string, std::string>>{{"left", lK}, {"right", rK}})
  {
    const double sx = side.first == "left" ? 1.0 : -1.0;
    addFrameLocal(side.first + "_knuckle_a", side.second,
                  Eigen::Vector3d(0.006 * sx, 0.0, 0.0), 0.009, true);
    addFrameLocal(side.first + "_knuckle_b", side.second,
                  Eigen::Vector3d(0.024 * sx, 0.0, 0.0), 0.009, true);
  }
  for(const auto & side : std::vector<std::pair<std::string, std::string>>{{"left", lIK}, {"right", rIK}})
  {
    const double sx = side.first == "left" ? 1.0 : -1.0;
    addFrameLocal(side.first + "_inner_knuckle_a", side.second,
                  Eigen::Vector3d(0.006 * sx, 0.0, 0.010), 0.010, true);
    addFrameLocal(side.first + "_inner_knuckle_b", side.second,
                  Eigen::Vector3d(0.026 * sx, 0.0, 0.034), 0.010, true);
  }

  // Dense finger-link proxies from left/right_finger.stl. These are all hard
  // against the blue handle. A 5 mm sphere lattice is deliberately denser than
  // the former root/midpoint approximation that allowed visible penetration.
  for(const auto & side : std::vector<std::pair<std::string, std::string>>{{"left", lF}, {"right", rF}})
  {
    int id = 0;
    for(double x : {-0.009, 0.0, 0.009})
    {
      for(double y : {-0.007, 0.007})
      {
        for(double z : {0.004, 0.023, 0.043})
        {
          addFrameLocal(side.first + "_finger_" + std::to_string(id++),
                        side.second, Eigen::Vector3d(x, y, z), 0.0055, true);
        }
      }
    }
  }

  // Fingertip-body proxies from left/right_finger_tip.stl. The inner contact
  // face itself (left x-min / right x-max) is intentionally excluded from the
  // hard lattice and represented by the two zero-penetration pad points below.
  for(const auto & side : std::vector<std::pair<std::string, std::string>>{{"left", lT}, {"right", rT}})
  {
    const bool left = side.first == "left";
    const double innerSign = left ? -1.0 : 1.0;
    int id = 0;
    for(double xAbs : {0.003, 0.013})
    {
      const double x = innerSign * xAbs;
      for(double y : {-0.008, 0.008})
      {
        for(double z : {0.003, 0.025, 0.047})
        {
          addFrameLocal(side.first + "_tip_body_" + std::to_string(id++),
                        side.second, Eigen::Vector3d(x, y, z), 0.0045, true);
        }
      }
    }
    // Pad shoulders/end caps remain hard so the cylinder cannot escape around
    // or through the top/bottom of the contact face.
    addFrameLocal(side.first + "_pad_shoulder_low", side.second,
                  Eigen::Vector3d(innerSign * 0.0215, 0.0, 0.006), 0.0035, true);
    addFrameLocal(side.first + "_pad_shoulder_high", side.second,
                  Eigen::Vector3d(innerSign * 0.0215, 0.0, 0.050), 0.0035, true);
  }

  // Exact inner-pad contact-face points extracted from the uploaded collision
  // meshes. These are the only gripper samples permitted to become tangent to
  // the blue cylinder; they are never permitted to pass through it.
  Eigen::Vector3d pPadL_W, pPadR_W;
  if(livePadCenters(pPadL_W, pPadR_W))
  {
    addFrameLocal("left_inner_pad_contact", lT, leftPadPointTip_, 0.0010, false);
    addFrameLocal("right_inner_pad_contact", rT, rightPadPointTip_, 0.0010, false);
  }

  gripperGeometryValid_ = gripperSamplesB_.size() >= 60;
  if(!gripperGeometryValid_)
  {
    mc_rtc::log::error(
        "[GripperGeometry] only {} safety samples were constructed; exact Robotiq link frames are missing",
        gripperSamplesB_.size());
    return false;
  }

  if(verbose)
  {
    mc_rtc::log::success(
        "[GripperGeometry] calibrated {} live mesh-derived safety samples; exact pad-face gap={:.5f}",
        gripperSamplesB_.size(), liveMouthGap());
  }
  return true;
}

bool HandoverInterceptionController::prepareCaptureSelection()
{
  refreshObjectPose();
  const bool mouthOK = calibrateMouthControlFrame();
  const bool geometryOK = refreshGripperGeometry();
  return mouthOK && geometryOK;
}

// =============================================================================
// Object-aware swept-volume and grasp-corridor safety
// =============================================================================

Eigen::Vector3d HandoverInterceptionController::objectAxis() const
{
  Eigen::Vector3d axis = worldRotation(W_T_O_) * Eigen::Vector3d::UnitZ();
  if(axis.norm() < 1e-9) { axis = Eigen::Vector3d::UnitZ(); }
  return axis.normalized();
}

Eigen::Vector3d HandoverInterceptionController::blueHandleAxis() const
{
  return -objectAxis();
}

double HandoverInterceptionController::pointSegmentDistance(
    const Eigen::Vector3d & p,
    const Eigen::Vector3d & a,
    const Eigen::Vector3d & b) const
{
  const Eigen::Vector3d ab = b - a;
  const double den = ab.squaredNorm();
  if(den < 1e-12) { return (p - a).norm(); }
  const double t = std::max(0.0, std::min(1.0, (p - a).dot(ab) / den));
  return (p - (a + t * ab)).norm();
}

void HandoverInterceptionController::updateWorstClearance(
    HandoverSafetyReport & report,
    double clearance,
    const std::string & sample,
    const std::string & obstacle) const
{
  if(clearance < report.minClearance)
  {
    report.minClearance = clearance;
    report.sample = sample;
    report.obstacle = obstacle;
  }
  if(clearance < 0.0) { report.safe = false; }
}

bool HandoverInterceptionController::graspCorridorSafe(
    const sva::PTransformd & W_T_M,
    HandoverSafetyReport & report) const
{
  const Eigen::Vector3d pH_M = pointToLocal(W_T_M, W_T_H_.translation());
  Eigen::Vector3d aH_M = worldRotation(W_T_M).transpose() * blueHandleAxis();
  if(aH_M.norm() < 1e-9) { aH_M = Eigen::Vector3d::UnitZ(); }
  aH_M.normalize();

  const double alignment = clampUnit(std::abs(aH_M.dot(Eigen::Vector3d::UnitZ())));
  const double angle = std::acos(alignment);
  const double usableHalfGap = liveMouthHalfGap() - corridorFingerInset_;

  // x_M: finger-closing direction, y_M: outward toward the gripper base,
  // z_M: blue-handle axis. At standoff the handle is at negative y_M; at
  // capture it reaches y_M=0. Positive y_M would put it behind the mouth.
  const double lateralClear = usableHalfGap
                            - (std::abs(pH_M.x()) + handleRadius_ + corridorSafetyMargin_);
  const double entryClear = pH_M.y() + corridorEntryDepth_;
  const double palmClear = corridorPalmLimit_ - pH_M.y();
  const double axialClear = corridorAxialTolerance_ - std::abs(pH_M.z());
  const double angleClear = corridorMaxAngleRad_ - angle;

  report.corridorLateralClearance = lateralClear;
  report.corridorEntryClearance = entryClear;
  report.corridorPalmClearance = palmClear;
  report.corridorAxialClearance = axialClear;
  report.corridorAngleClearance = angleClear;

  updateWorstClearance(report, lateralClear, "mouth_corridor", "blue_handle_lateral");
  updateWorstClearance(report, entryClear, "mouth_corridor", "blue_handle_entry_depth");
  updateWorstClearance(report, palmClear, "mouth_corridor", "blue_handle_palm_depth");
  updateWorstClearance(report, axialClear, "mouth_corridor", "blue_handle_axial_offset");

  // Convert angular margin to a length-like value only for common ranking.
  updateWorstClearance(report,
                       0.05 * angleClear,
                       "mouth_corridor",
                       "blue_handle_axis_alignment");

  return lateralClear >= 0.0
      && entryClear >= 0.0
      && palmClear >= 0.0
      && axialClear >= 0.0
      && angleClear >= 0.0;
}

bool HandoverInterceptionController::gripperPoseSafe(
    const sva::PTransformd & W_T_M,
    HandoverSafetyReport & report,
    bool requireCorridor) const
{
  return gripperBasePoseSafe(
      basePoseFromMouthPose(W_T_M), W_T_M, report, requireCorridor);
}

bool HandoverInterceptionController::gripperBasePoseSafe(
    const sva::PTransformd & W_T_B,
    const sva::PTransformd & W_T_M,
    HandoverSafetyReport & report,
    bool requireCorridor) const
{
  if(!mouthCalibrationValid_ || !gripperGeometryValid_)
  {
    report.safe = false;
    report.minClearance = -1e9;
    report.sample = "uncalibrated_geometry";
    report.obstacle = "methodology_precondition";
    return false;
  }

  const Eigen::Vector3d axis = objectAxis();
  const Eigen::Vector3d pO = W_T_O_.translation();
  const double centerOffset = std::abs(O_T_H_.translation().z());

  const Eigen::Vector3d sensorA = pO - axis * sensorHalfLength_;
  const Eigen::Vector3d sensorB = pO + axis * sensorHalfLength_;

  const Eigen::Vector3d humanCenter = pO + axis * centerOffset;
  const Eigen::Vector3d humanA = humanCenter - axis * handleHalfLength_;
  const Eigen::Vector3d humanB = humanCenter + axis * handleHalfLength_;

  const Eigen::Vector3d blueCenter = W_T_H_.translation();
  const Eigen::Vector3d blueA = blueCenter - axis * handleHalfLength_;
  const Eigen::Vector3d blueB = blueCenter + axis * handleHalfLength_;

  for(const auto & sample : gripperSamplesB_)
  {
    const Eigen::Vector3d pW = pointToWorld(W_T_B, sample.p_B);

    if(groundEnabled_)
    {
      const double groundClear = pW.z() - groundZ_
          - (sample.radius + groundSafetyMargin_);
      if(report.groundClearance < -1e8 || groundClear < report.groundClearance)
      {
        report.groundClearance = groundClear;
      }
      updateWorstClearance(report, groundClear, sample.name, "ground_plane");
    }

    const double sensorClear = pointSegmentDistance(pW, sensorA, sensorB)
        - (sensorRadius_ + sample.radius + gripperSafetyMargin_);
    updateWorstClearance(report, sensorClear, sample.name, "sensor_core");

    const double humanClear = pointSegmentDistance(pW, humanA, humanB)
        - (handleRadius_ + sample.radius + gripperSafetyMargin_);
    updateWorstClearance(report, humanClear, sample.name, "human_grey_handle");

    if(!requireCorridor || sample.hardAgainstBlue)
    {
      const double blueClear = pointSegmentDistance(pW, blueA, blueB)
          - (handleRadius_ + sample.radius + gripperSafetyMargin_);
      updateWorstClearance(report, blueClear, sample.name, "robot_blue_handle");
    }
  }

  if(requireCorridor) { graspCorridorSafe(W_T_M, report); }
  return report.safe;
}

bool HandoverInterceptionController::sweptGripperPoseSafe(
    const sva::PTransformd & W_T_M_from,
    const sva::PTransformd & W_T_M_to,
    HandoverSafetyReport & report,
    bool requireCorridor) const
{
  report = HandoverSafetyReport{};
  report.safe = true;
  report.minClearance = std::numeric_limits<double>::infinity();

  const int N = std::max(2, sweepSamples_);
  for(int k = 0; k <= N; ++k)
  {
    const double alpha = static_cast<double>(k) / static_cast<double>(N);
    const sva::PTransformd W_T_M = interpolatePose(W_T_M_from, W_T_M_to, alpha);
    gripperPoseSafe(W_T_M, report, requireCorridor);
  }

  return report.safe;
}

bool HandoverInterceptionController::filterSafeMouthCommand(
    const sva::PTransformd & W_T_M_current,
    const sva::PTransformd & W_T_M_proposed,
    sva::PTransformd & W_T_M_safe,
    HandoverSafetyReport & report,
    bool requireCorridor)
{
  HandoverSafetyReport currentGround;
  currentGround.safe = true;
  currentGround.minClearance = std::numeric_limits<double>::infinity();
  if(!wholeRobotGroundSafe(currentGround))
  {
    W_T_M_safe = W_T_M_current;
    currentGround.acceptedScale = 0.0;
    report = currentGround;
    return false;
  }

  double scale = 1.0;
  HandoverSafetyReport last;

  for(int i = 0; i < std::max(1, backtrackIterations_); ++i)
  {
    const sva::PTransformd candidate =
        interpolatePose(W_T_M_current, W_T_M_proposed, scale);
    HandoverSafetyReport r;
    if(sweptGripperPoseSafe(W_T_M_current, candidate, r, requireCorridor))
    {
      r.acceptedScale = scale;
      W_T_M_safe = candidate;
      report = r;
      return true;
    }
    last = r;
    scale *= 0.5;
  }

  W_T_M_safe = W_T_M_current;
  last.acceptedScale = 0.0;
  report = last;
  return false;
}

bool HandoverInterceptionController::evaluateCurrentPoseSafety(
    HandoverSafetyReport & report,
    bool requireCorridor)
{
  report = HandoverSafetyReport{};
  report.safe = true;
  report.minClearance = std::numeric_limits<double>::infinity();
  const sva::PTransformd W_T_M = actualMouthPose();
  gripperBasePoseSafe(actualBasePose(), W_T_M, report, requireCorridor);
  wholeRobotGroundSafe(report);
  return report.safe;
}

bool HandoverInterceptionController::wholeRobotGroundSafe(
    HandoverSafetyReport & report) const
{
  if(!groundEnabled_) { return true; }

  // Online whole-arm monitor. These conservative frame-centred spheres are
  // not used to bias candidate direction; they are a hard static-environment
  // constraint. The gripper is checked separately with its denser samples.
  const std::vector<std::pair<std::string, double>> linkSamples =
  {
    {"gen3_half_arm_1_link", 0.045},
    {"gen3_half_arm_2_link", 0.045},
    {"gen3_forearm_link", 0.042},
    {"gen3_spherical_wrist_1_link", 0.038},
    {"gen3_spherical_wrist_2_link", 0.038},
    {"gen3_bracelet_link", 0.035},
    {"gen3_end_effector_link", 0.032}
  };

  bool sawSample = false;
  for(const auto & item : linkSamples)
  {
    if(!robot().hasFrame(item.first)) { continue; }
    sawSample = true;
    const double clear = robot().frame(item.first).position().translation().z()
        - groundZ_ - (item.second + armGroundSafetyMargin_);
    if(report.groundClearance < -1e8 || clear < report.groundClearance)
    {
      report.groundClearance = clear;
    }
    updateWorstClearance(report, clear, item.first, "ground_plane");
  }

  // Missing optional frames must not invalidate the controller, but the
  // gripper-plane test still remains active.
  return !sawSample || report.safe;
}

bool HandoverInterceptionController::evaluateCurrentClosureSafety(
    HandoverSafetyReport & report,
    bool enforceCentering,
    bool allowDesignatedPadContact) const
{
  report = HandoverSafetyReport{};
  report.safe = true;
  report.minClearance = std::numeric_limits<double>::infinity();

  if(!mouthCalibrationValid_ || !gripperGeometryValid_)
  {
    report.safe = false;
    report.sample = "uncalibrated_geometry";
    report.obstacle = "closure_precondition";
    report.minClearance = -1e9;
    return false;
  }

  const sva::PTransformd W_T_B = actualBasePose();
  const sva::PTransformd W_T_M = actualMouthPose();
  const Eigen::Vector3d axis = objectAxis();
  const Eigen::Vector3d pO = W_T_O_.translation();
  const double centerOffset = std::abs(O_T_H_.translation().z());

  const Eigen::Vector3d sensorA = pO - axis * sensorHalfLength_;
  const Eigen::Vector3d sensorB = pO + axis * sensorHalfLength_;
  const Eigen::Vector3d humanCenter = pO + axis * centerOffset;
  const Eigen::Vector3d humanA = humanCenter - axis * handleHalfLength_;
  const Eigen::Vector3d humanB = humanCenter + axis * handleHalfLength_;
  const Eigen::Vector3d blueCenter = W_T_H_.translation();
  const Eigen::Vector3d blueA = blueCenter - axis * handleHalfLength_;
  const Eigen::Vector3d blueB = blueCenter + axis * handleHalfLength_;

  for(const auto & sample : gripperSamplesB_)
  {
    const Eigen::Vector3d pW = sampleWorldPoint(sample, robot().mbc());

    if(groundEnabled_)
    {
      const double clear = pW.z() - groundZ_
          - (sample.radius + groundSafetyMargin_);
      if(report.groundClearance < -1e8 || clear < report.groundClearance)
      {
        report.groundClearance = clear;
      }
      updateWorstClearance(report, clear, sample.name, "ground_plane");
    }

    const double sensorClear = pointSegmentDistance(pW, sensorA, sensorB)
        - (sensorRadius_ + sample.radius + gripperSafetyMargin_);
    updateWorstClearance(report, sensorClear, sample.name, "sensor_core");

    const double humanClear = pointSegmentDistance(pW, humanA, humanB)
        - (handleRadius_ + sample.radius + gripperSafetyMargin_);
    updateWorstClearance(report, humanClear, sample.name, "human_grey_handle");

    if(sample.hardAgainstBlue)
    {
      const double blueClear = pointSegmentDistance(pW, blueA, blueB)
          - (handleRadius_ + sample.radius + gripperSafetyMargin_);
      updateWorstClearance(report, blueClear, sample.name, "robot_blue_handle");
    }
  }

  wholeRobotGroundSafe(report);

  Eigen::Vector3d pL_W, pR_W;
  if(!livePadCenters(pL_W, pR_W))
  {
    report.safe = false;
    updateWorstClearance(report, -1e9, "pad_frames", "closure_precondition");
    return false;
  }

  const Eigen::Vector3d pL_M = pointToLocal(W_T_M, pL_W);
  const Eigen::Vector3d pR_M = pointToLocal(W_T_M, pR_W);
  const Eigen::Vector3d pH_M = pointToLocal(W_T_M, W_T_H_.translation());
  const double xPositive = std::max(pL_M.x(), pR_M.x());
  const double xNegative = std::min(pL_M.x(), pR_M.x());

  report.leftPadClearance = xPositive - pH_M.x() - handleRadius_;
  report.rightPadClearance = pH_M.x() - xNegative - handleRadius_;
  report.signedPadCenteringError =
      0.5 * (report.rightPadClearance - report.leftPadClearance);
  report.padCenteringError = std::abs(report.signedPadCenteringError);

  // Hybrid contact contract:
  // - before bilateral contact is established, the inner pads are ordinary
  //   collision geometry and may only use the small numerical penetration
  //   tolerance;
  // - during in-loop grasp confirmation, these two designated pad/blue-handle
  //   pairs are active grasp contacts. Their signed-distance residual is then
  //   bounded by the already-defined contact band, while every other gripper,
  //   object, arm, human-handle and environment pair remains a hard safety
  //   constraint.
  //
  // This changes the contact mode, not the selected grasp, route, target or
  // controller gains. It also does not authorize palm/finger/body contact.
  const double designatedPadAllowance = allowDesignatedPadContact
      ? gripperContactTolerance_ : gripperPenetrationTolerance_;
  updateWorstClearance(report,
                       report.leftPadClearance + designatedPadAllowance,
                       "left_inner_pad", "robot_blue_handle");
  updateWorstClearance(report,
                       report.rightPadClearance + designatedPadAllowance,
                       "right_inner_pad", "robot_blue_handle");
  if(enforceCentering)
  {
    updateWorstClearance(report,
                         padCenteringTolerance_ - report.padCenteringError,
                         "pad_pair", "blue_handle_centering");
  }

  Eigen::Vector3d aH_M = worldRotation(W_T_M).transpose() * blueHandleAxis();
  if(aH_M.norm() < 1e-9) { aH_M = Eigen::Vector3d::UnitZ(); }
  aH_M.normalize();
  const double angle = std::acos(
      clampUnit(std::abs(aH_M.dot(Eigen::Vector3d::UnitZ()))));
  const double entryClear = pH_M.y() + corridorEntryDepth_;
  const double palmClear = corridorPalmLimit_ - pH_M.y();
  const double axialClear = corridorAxialTolerance_ - std::abs(pH_M.z());
  const double angleClear = corridorMaxAngleRad_ - angle;

  report.corridorEntryClearance = entryClear;
  report.corridorPalmClearance = palmClear;
  report.corridorAxialClearance = axialClear;
  report.corridorAngleClearance = angleClear;
  updateWorstClearance(report, entryClear, "mouth_corridor", "blue_handle_entry_depth");
  updateWorstClearance(report, palmClear, "mouth_corridor", "blue_handle_palm_depth");
  updateWorstClearance(report, axialClear, "mouth_corridor", "blue_handle_axial_offset");
  updateWorstClearance(report, 0.05 * angleClear,
                       "mouth_corridor", "blue_handle_axis_alignment");

  const double designatedPadLowerBound = allowDesignatedPadContact
      ? -gripperContactTolerance_ : -gripperPenetrationTolerance_;
  report.bilateralPadContact = report.safe
      && report.leftPadClearance <= gripperContactTolerance_
      && report.rightPadClearance <= gripperContactTolerance_
      && report.leftPadClearance >= designatedPadLowerBound
      && report.rightPadClearance >= designatedPadLowerBound
      && report.padCenteringError <= padCenteringTolerance_;

  return report.safe;
}

bool HandoverInterceptionController::bilateralPadContactReached(
    HandoverSafetyReport * report,
    bool allowDesignatedPadContact) const
{
  HandoverSafetyReport local;
  const bool safe = evaluateCurrentClosureSafety(
      local, true, allowDesignatedPadContact);
  if(report) { *report = local; }
  return safe && local.bilateralPadContact;
}


// =============================================================================
// Internal whole-arm preview used by plan-once candidate selection
// =============================================================================

Eigen::Vector3d HandoverInterceptionController::sampleWorldPoint(
    const GripperSample & sample,
    const rbd::MultiBodyConfig & mbc) const
{
  try
  {
    const int bodyIndex = robot().mb().bodyIndexByName(sample.frame);
    if(bodyIndex >= 0 && static_cast<size_t>(bodyIndex) < mbc.bodyPosW.size())
    {
      return pointToWorld(mbc.bodyPosW[static_cast<size_t>(bodyIndex)], sample.p_F);
    }
  }
  catch(const std::exception &)
  {
  }
  return pointToWorld(previewBasePose(mbc), sample.p_B);
}

sva::PTransformd HandoverInterceptionController::previewBasePose(
    const rbd::MultiBodyConfig & mbc) const
{
  try
  {
    const int idx = robot().mb().bodyIndexByName(toolFrame_);
    if(idx >= 0 && static_cast<size_t>(idx) < mbc.bodyPosW.size())
    {
      return mbc.bodyPosW[static_cast<size_t>(idx)];
    }
  }
  catch(const std::exception &)
  {
  }
  return actualBasePose();
}

bool HandoverInterceptionController::previewPadCenters(
    const rbd::MultiBodyConfig & mbc,
    Eigen::Vector3d & pL_W,
    Eigen::Vector3d & pR_W) const
{
  try
  {
    const int l = robot().mb().bodyIndexByName(
        "gen3_robotiq_85_left_finger_tip_link");
    const int r = robot().mb().bodyIndexByName(
        "gen3_robotiq_85_right_finger_tip_link");
    if(l < 0 || r < 0
       || static_cast<size_t>(l) >= mbc.bodyPosW.size()
       || static_cast<size_t>(r) >= mbc.bodyPosW.size())
    {
      return false;
    }
    pL_W = pointToWorld(mbc.bodyPosW[static_cast<size_t>(l)], leftPadPointTip_);
    pR_W = pointToWorld(mbc.bodyPosW[static_cast<size_t>(r)], rightPadPointTip_);
    return true;
  }
  catch(const std::exception &)
  {
    return false;
  }
}

sva::PTransformd HandoverInterceptionController::previewMouthPose(
    const rbd::MultiBodyConfig & mbc) const
{
  const sva::PTransformd W_T_B = previewBasePose(mbc);
  Eigen::Vector3d pL, pR;
  if(!previewPadCenters(mbc, pL, pR))
  {
    return mouthPoseFromBasePose(W_T_B);
  }

  const Eigen::Vector3d pM = 0.5 * (pL + pR);
  const Eigen::Matrix3d R_W_B = worldRotation(W_T_B);
  Eigen::Vector3d xM = pL - pR;
  if(xM.norm() < 1e-9) { xM = R_W_B.col(0); }
  xM.normalize();
  Eigen::Vector3d yM = -R_W_B.col(2);
  yM -= xM * xM.dot(yM);
  if(yM.norm() < 1e-9) { yM = R_W_B.col(1); }
  yM.normalize();
  Eigen::Vector3d zM = xM.cross(yM);
  if(zM.norm() < 1e-9) { zM = R_W_B.col(1); }
  zM.normalize();
  yM = zM.cross(xM).normalized();
  Eigen::Matrix3d R_W_M;
  R_W_M.col(0) = xM;
  R_W_M.col(1) = yM;
  R_W_M.col(2) = zM;
  return fromWorldPose(R_W_M, pM);
}

sva::PTransformd HandoverInterceptionController::previewBasePoseFromMouthPose(
    const sva::PTransformd & W_T_M,
    const rbd::MultiBodyConfig & mbc) const
{
  // Use the copied gripper configuration, including its current aperture, to
  // map the desired pad-midpoint mouth pose back to the controlled base frame.
  // Runtime performs the same map from the live aperture. This avoids
  // certifying closure with the open-gripper B->M transform.
  const sva::PTransformd W_T_B_current = previewBasePose(mbc);
  const sva::PTransformd W_T_M_current = previewMouthPose(mbc);
  const sva::PTransformd B_T_M = relativePose(
      W_T_B_current, W_T_M_current);
  const Eigen::Matrix3d R_W_M = worldRotation(W_T_M);
  const Eigen::Matrix3d R_B_M = worldRotation(B_T_M);
  const Eigen::Vector3d p_B_M = B_T_M.translation();
  const Eigen::Matrix3d R_W_B = R_W_M * R_B_M.transpose();
  const Eigen::Vector3d p_W_B = W_T_M.translation() - R_W_B * p_B_M;
  return fromWorldPose(R_W_B, p_W_B);
}

bool HandoverInterceptionController::wholeRobotGroundSafe(
    const rbd::MultiBodyConfig & mbc,
    HandoverSafetyReport & report) const
{
  if(!groundEnabled_) { return true; }
  const std::vector<std::pair<std::string, double>> linkSamples =
  {
    {"gen3_half_arm_1_link", 0.045},
    {"gen3_half_arm_2_link", 0.045},
    {"gen3_forearm_link", 0.042},
    {"gen3_spherical_wrist_1_link", 0.038},
    {"gen3_spherical_wrist_2_link", 0.038},
    {"gen3_bracelet_link", 0.035},
    {"gen3_end_effector_link", 0.032}
  };

  bool saw = false;
  for(const auto & item : linkSamples)
  {
    try
    {
      const int idx = robot().mb().bodyIndexByName(item.first);
      if(idx < 0 || static_cast<size_t>(idx) >= mbc.bodyPosW.size()) { continue; }
      saw = true;
      const double clear = mbc.bodyPosW[static_cast<size_t>(idx)].translation().z()
          - groundZ_ - (item.second + armGroundSafetyMargin_);
      if(report.groundClearance < -1e8 || clear < report.groundClearance)
      {
        report.groundClearance = clear;
      }
      updateWorstClearance(report, clear, item.first, "ground_plane");
    }
    catch(const std::exception &)
    {
    }
  }
  return !saw || report.safe;
}

bool HandoverInterceptionController::carriedObjectGroundSafe(
    const sva::PTransformd & W_T_O_carried,
    HandoverSafetyReport & report) const
{
  if(!groundEnabled_) { return true; }

  Eigen::Vector3d axis = worldRotation(W_T_O_carried) * Eigen::Vector3d::UnitZ();
  if(axis.norm() < 1e-9) { axis = Eigen::Vector3d::UnitZ(); }
  axis.normalize();
  const Eigen::Vector3d pO = W_T_O_carried.translation();
  const double centerOffset = std::abs(O_T_H_.translation().z());
  const Eigen::Vector3d humanCenter = pO + axis * centerOffset;
  const Eigen::Vector3d blueCenter = compose(W_T_O_carried, O_T_H_).translation();

  auto checkCylinder = [this, &report, &axis](
      const Eigen::Vector3d & center,
      double halfLength,
      double radius,
      const std::string & name)
  {
    const double lowest = center.z()
        - std::abs(axis.z()) * halfLength - radius;
    const double clear = lowest - groundZ_ - groundSafetyMargin_;
    if(report.groundClearance < -1e8 || clear < report.groundClearance)
    {
      report.groundClearance = clear;
    }
    updateWorstClearance(report, clear, name, "ground_plane");
  };

  checkCylinder(pO, sensorHalfLength_, sensorRadius_, "carried_sensor_core");
  checkCylinder(humanCenter, handleHalfLength_, handleRadius_,
                "carried_human_handle");
  checkCylinder(blueCenter, handleHalfLength_, handleRadius_,
                "carried_blue_handle");
  return report.safe;
}

bool HandoverInterceptionController::carriedObjectArmSafe(
    const rbd::MultiBodyConfig & mbc,
    const sva::PTransformd & W_T_O_carried,
    HandoverSafetyReport & report) const
{
  Eigen::Vector3d axis = worldRotation(W_T_O_carried) * Eigen::Vector3d::UnitZ();
  if(axis.norm() < 1e-9) { axis = Eigen::Vector3d::UnitZ(); }
  axis.normalize();
  const Eigen::Vector3d pO = W_T_O_carried.translation();
  const double centerOffset = std::abs(O_T_H_.translation().z());
  const Eigen::Vector3d humanCenter = pO + axis * centerOffset;
  const Eigen::Vector3d blueCenter = compose(W_T_O_carried, O_T_H_).translation();
  const Eigen::Vector3d sensorA = pO - axis * sensorHalfLength_;
  const Eigen::Vector3d sensorB = pO + axis * sensorHalfLength_;
  const Eigen::Vector3d humanA = humanCenter - axis * handleHalfLength_;
  const Eigen::Vector3d humanB = humanCenter + axis * handleHalfLength_;
  const Eigen::Vector3d blueA = blueCenter - axis * handleHalfLength_;
  const Eigen::Vector3d blueB = blueCenter + axis * handleHalfLength_;

  // The bracelet/end-effector/gripper are intentionally close to the blue
  // handle and are handled by the exact gripper geometry. The proximal arm is
  // checked here against the carried object during the certified retreat.
  const std::vector<std::pair<std::string, double>> links =
  {
    {"gen3_half_arm_1_link", 0.045},
    {"gen3_half_arm_2_link", 0.045},
    {"gen3_forearm_link", 0.042},
    {"gen3_spherical_wrist_1_link", 0.038},
    {"gen3_spherical_wrist_2_link", 0.038}
  };

  for(const auto & item : links)
  {
    try
    {
      const int idx = robot().mb().bodyIndexByName(item.first);
      if(idx < 0 || static_cast<size_t>(idx) >= mbc.bodyPosW.size()) { continue; }
      const Eigen::Vector3d p = mbc.bodyPosW[static_cast<size_t>(idx)].translation();
      updateWorstClearance(report,
          pointSegmentDistance(p, sensorA, sensorB)
            - (item.second + sensorRadius_ + gripperSafetyMargin_),
          item.first, "carried_sensor_core");
      updateWorstClearance(report,
          pointSegmentDistance(p, humanA, humanB)
            - (item.second + handleRadius_ + gripperSafetyMargin_),
          item.first, "carried_human_handle");
      updateWorstClearance(report,
          pointSegmentDistance(p, blueA, blueB)
            - (item.second + handleRadius_ + gripperSafetyMargin_),
          item.first, "carried_blue_handle");
    }
    catch(const std::exception &)
    {
    }
  }
  return report.safe;
}

bool HandoverInterceptionController::carriedObjectArmSafe(
    const sva::PTransformd & W_T_O_carried,
    HandoverSafetyReport & report) const
{
  return carriedObjectArmSafe(robot().mbc(), W_T_O_carried, report);
}

bool HandoverInterceptionController::previewAttachedRetreatSafe(
    const rbd::MultiBodyConfig & mbc,
    const sva::PTransformd & W_T_O_carried,
    HandoverSafetyReport & report) const
{
  report = HandoverSafetyReport{};
  report.safe = true;
  report.minClearance = std::numeric_limits<double>::infinity();

  for(const auto & sample : gripperSamplesB_)
  {
    if(!groundEnabled_) { break; }
    // Use the copied closed-gripper link transforms. Reconstructing every
    // sample from the cached live/open B->M calibration can certify a retreat
    // geometry that is different from the copied state being evaluated.
    const Eigen::Vector3d pW = sampleWorldPoint(sample, mbc);
    const double clear = pW.z() - groundZ_
        - (sample.radius + groundSafetyMargin_);
    if(report.groundClearance < -1e8 || clear < report.groundClearance)
    {
      report.groundClearance = clear;
    }
    updateWorstClearance(report, clear, sample.name, "ground_plane");
  }

  wholeRobotGroundSafe(mbc, report);
  carriedObjectGroundSafe(W_T_O_carried, report);
  carriedObjectArmSafe(mbc, W_T_O_carried, report);
  return report.safe;
}

bool HandoverInterceptionController::evaluateAttachedRetreatSafety(
    HandoverSafetyReport & report) const
{
  if(!objectAttached_)
  {
    report = HandoverSafetyReport{};
    report.safe = false;
    report.sample = "object_not_attached";
    report.obstacle = "retreat_precondition";
    report.minClearance = -1e9;
    return false;
  }
  return previewAttachedRetreatSafe(robot().mbc(), W_T_O_, report);
}

bool HandoverInterceptionController::filterSafeAttachedRetreatCommand(
    const sva::PTransformd & W_T_M_current,
    const sva::PTransformd & W_T_M_proposed,
    sva::PTransformd & W_T_M_safe,
    HandoverSafetyReport & report) const
{
  HandoverSafetyReport currentReport;
  if(!evaluateAttachedRetreatSafety(currentReport))
  {
    W_T_M_safe = W_T_M_current;
    currentReport.acceptedScale = 0.0;
    report = currentReport;
    return false;
  }

  double scale = 1.0;
  HandoverSafetyReport last = currentReport;
  for(int i = 0; i < std::max(1, backtrackIterations_); ++i)
  {
    const sva::PTransformd candidate = interpolatePose(
        W_T_M_current, W_T_M_proposed, scale);
    HandoverSafetyReport r;
    r.safe = true;
    r.minClearance = std::numeric_limits<double>::infinity();

    const int N = std::max(2, sweepSamples_);
    for(int k = 0; k <= N; ++k)
    {
      const double a = static_cast<double>(k) / static_cast<double>(N);
      const sva::PTransformd W_T_M = interpolatePose(
          W_T_M_current, candidate, a);
      const sva::PTransformd W_T_B = basePoseFromMouthPose(W_T_M);
      const sva::PTransformd W_T_O_carried = compose(W_T_M, M_T_O_attached_);

      for(const auto & sample : gripperSamplesB_)
      {
        if(!groundEnabled_) { break; }
        const Eigen::Vector3d pW = pointToWorld(W_T_B, sample.p_B);
        const double clear = pW.z() - groundZ_
            - (sample.radius + groundSafetyMargin_);
        if(r.groundClearance < -1e8 || clear < r.groundClearance)
        {
          r.groundClearance = clear;
        }
        updateWorstClearance(r, clear, sample.name, "ground_plane");
      }
      carriedObjectGroundSafe(W_T_O_carried, r);
    }

    // Current whole-arm/object clearance is checked every servo cycle. The
    // proposed task-space step is additionally swept for gripper/object ground.
    carriedObjectArmSafe(W_T_O_, r);
    if(r.safe)
    {
      r.acceptedScale = scale;
      W_T_M_safe = candidate;
      report = r;
      return true;
    }
    last = r;
    scale *= 0.5;
  }

  W_T_M_safe = W_T_M_current;
  last.acceptedScale = 0.0;
  report = last;
  return false;
}

std::map<std::string, std::vector<double>>
HandoverInterceptionController::armPostureFromMbc(
    const rbd::MultiBodyConfig & mbc) const
{
  std::map<std::string, std::vector<double>> out;
  for(int i = 1; i <= 7; ++i)
  {
    const std::string name = "gen3_joint_" + std::to_string(i);
    try
    {
      const int idx = robot().mb().jointIndexByName(name);
      if(idx >= 0 && static_cast<size_t>(idx) < mbc.q.size()
         && !mbc.q[static_cast<size_t>(idx)].empty())
      {
        out[name] = {mbc.q[static_cast<size_t>(idx)][0]};
      }
    }
    catch(const std::exception &)
    {
    }
  }
  return out;
}

bool HandoverInterceptionController::previewJointLimitsSafe(
    const rbd::MultiBodyConfig & mbc,
    std::string & reason) const
{
  const auto & ql = robot().ql();
  const auto & qu = robot().qu();
  const auto & mb = robot().mb();
  const size_t n = std::min(mbc.q.size(), std::min(ql.size(), qu.size()));
  for(size_t j = 0; j < n; ++j)
  {
    const std::string name = mb.joint(static_cast<int>(j)).name();
    const size_t m = std::min(mbc.q[j].size(), std::min(ql[j].size(), qu[j].size()));
    for(size_t k = 0; k < m; ++k)
    {
      const double lo = ql[j][k];
      const double hi = qu[j][k];
      const double q = mbc.q[j][k];
      if(!std::isfinite(lo) || !std::isfinite(hi)) { continue; }
      // The arm keeps an interior safety margin. The Robotiq gripper does not:
      // fully open (q = lower limit) and fully closed (q = upper limit) are
      // legitimate commanded configurations and must remain preview-feasible.
      const bool gripperJoint = name.rfind("gen3_robotiq_85_", 0) == 0;
      const double margin = (!gripperJoint
                             && hi - lo > 4.0 * previewJointLimitMargin_)
                          ? previewJointLimitMargin_ : 0.0;
      constexpr double numericalTolerance = 1e-9;
      if(q < lo + margin - numericalTolerance
         || q > hi - margin + numericalTolerance)
      {
        reason = "joint_limit/" + name;
        return false;
      }
    }
  }
  return true;
}

bool HandoverInterceptionController::previewConfigurationSafe(
    const rbd::MultiBodyConfig & mbc,
    const sva::PTransformd & W_T_M,
    bool requireCorridor,
    HandoverSafetyReport & report) const
{
  report = HandoverSafetyReport{};
  report.safe = true;
  report.minClearance = std::numeric_limits<double>::infinity();
  gripperBasePoseSafe(previewBasePose(mbc), W_T_M, report, requireCorridor);
  wholeRobotGroundSafe(mbc, report);
  return report.safe;
}

void HandoverInterceptionController::setPreviewGripperClosure(
    rbd::MultiBodyConfig & mbc,
    double closure) const
{
  closure = std::max(0.0, std::min(1.0, closure));
  const double q = gripperOpenQ_ + closure * (gripperCloseQ_ - gripperOpenQ_);
  const std::vector<std::pair<std::string, double>> commands =
  {
    {"gen3_robotiq_85_left_knuckle_joint", q},
    {"gen3_robotiq_85_right_knuckle_joint", -q},
    {"gen3_robotiq_85_left_inner_knuckle_joint", q},
    {"gen3_robotiq_85_right_inner_knuckle_joint", -q},
    {"gen3_robotiq_85_left_finger_tip_joint", -q},
    {"gen3_robotiq_85_right_finger_tip_joint", q}
  };
  for(const auto & cmd : commands)
  {
    try
    {
      const int idx = robot().mb().jointIndexByName(cmd.first);
      if(idx >= 0 && static_cast<size_t>(idx) < mbc.q.size()
         && !mbc.q[static_cast<size_t>(idx)].empty())
      {
        mbc.q[static_cast<size_t>(idx)][0] = cmd.second;
      }
    }
    catch(const std::exception &)
    {
    }
  }
  rbd::forwardKinematics(robot().mb(), mbc);
}

bool HandoverInterceptionController::previewDynamicClosureSafety(
    const rbd::MultiBodyConfig & mbc,
    HandoverSafetyReport & report,
    bool allowDesignatedPadContact) const
{
  report = HandoverSafetyReport{};
  report.safe = true;
  report.minClearance = std::numeric_limits<double>::infinity();

  const sva::PTransformd W_T_M = previewMouthPose(mbc);
  const Eigen::Vector3d axis = objectAxis();
  const Eigen::Vector3d pO = W_T_O_.translation();
  const double centerOffset = std::abs(O_T_H_.translation().z());
  const Eigen::Vector3d sensorA = pO - axis * sensorHalfLength_;
  const Eigen::Vector3d sensorB = pO + axis * sensorHalfLength_;
  const Eigen::Vector3d humanCenter = pO + axis * centerOffset;
  const Eigen::Vector3d humanA = humanCenter - axis * handleHalfLength_;
  const Eigen::Vector3d humanB = humanCenter + axis * handleHalfLength_;
  const Eigen::Vector3d blueCenter = W_T_H_.translation();
  const Eigen::Vector3d blueA = blueCenter - axis * handleHalfLength_;
  const Eigen::Vector3d blueB = blueCenter + axis * handleHalfLength_;

  for(const auto & sample : gripperSamplesB_)
  {
    const Eigen::Vector3d pW = sampleWorldPoint(sample, mbc);
    if(groundEnabled_)
    {
      const double clear = pW.z() - groundZ_
          - (sample.radius + groundSafetyMargin_);
      if(report.groundClearance < -1e8 || clear < report.groundClearance)
      {
        report.groundClearance = clear;
      }
      updateWorstClearance(report, clear, sample.name, "ground_plane");
    }
    updateWorstClearance(report,
        pointSegmentDistance(pW, sensorA, sensorB)
          - (sensorRadius_ + sample.radius + gripperSafetyMargin_),
        sample.name, "sensor_core");
    updateWorstClearance(report,
        pointSegmentDistance(pW, humanA, humanB)
          - (handleRadius_ + sample.radius + gripperSafetyMargin_),
        sample.name, "human_grey_handle");
    if(sample.hardAgainstBlue)
    {
      updateWorstClearance(report,
          pointSegmentDistance(pW, blueA, blueB)
            - (handleRadius_ + sample.radius + gripperSafetyMargin_),
          sample.name, "robot_blue_handle");
    }
  }
  wholeRobotGroundSafe(mbc, report);

  Eigen::Vector3d pL_W, pR_W;
  if(!previewPadCenters(mbc, pL_W, pR_W))
  {
    updateWorstClearance(report, -1e9, "pad_frames", "closure_precondition");
    return false;
  }
  const Eigen::Vector3d pL_M = pointToLocal(W_T_M, pL_W);
  const Eigen::Vector3d pR_M = pointToLocal(W_T_M, pR_W);
  const Eigen::Vector3d pH_M = pointToLocal(W_T_M, W_T_H_.translation());
  const double xPositive = std::max(pL_M.x(), pR_M.x());
  const double xNegative = std::min(pL_M.x(), pR_M.x());
  report.leftPadClearance = xPositive - pH_M.x() - handleRadius_;
  report.rightPadClearance = pH_M.x() - xNegative - handleRadius_;
  report.signedPadCenteringError =
      0.5 * (report.rightPadClearance - report.leftPadClearance);
  report.padCenteringError = std::abs(report.signedPadCenteringError);
  const double designatedPadAllowance = allowDesignatedPadContact
      ? gripperContactTolerance_ : gripperPenetrationTolerance_;
  updateWorstClearance(report,
      report.leftPadClearance + designatedPadAllowance,
      "left_inner_pad", "robot_blue_handle");
  updateWorstClearance(report,
      report.rightPadClearance + designatedPadAllowance,
      "right_inner_pad", "robot_blue_handle");
  const double minPadClearance = std::min(
      report.leftPadClearance, report.rightPadClearance);
  const double acquisitionCenterTol =
      acquisitionCenterTolerance(minPadClearance);
  updateWorstClearance(report,
      acquisitionCenterTol - report.padCenteringError,
      "pad_pair", "blue_handle_acquisition_tube");

  Eigen::Vector3d aH_M = worldRotation(W_T_M).transpose() * blueHandleAxis();
  if(aH_M.norm() < 1e-9) { aH_M = Eigen::Vector3d::UnitZ(); }
  aH_M.normalize();
  const double angle = std::acos(
      clampUnit(std::abs(aH_M.dot(Eigen::Vector3d::UnitZ()))));
  const double entryClear = pH_M.y() + corridorEntryDepth_;
  const double palmClear = corridorPalmLimit_ - pH_M.y();
  const double axialClear = corridorAxialTolerance_ - std::abs(pH_M.z());
  const double angleClear = corridorMaxAngleRad_ - angle;
  report.corridorEntryClearance = entryClear;
  report.corridorPalmClearance = palmClear;
  report.corridorAxialClearance = axialClear;
  report.corridorAngleClearance = angleClear;
  updateWorstClearance(report, entryClear, "mouth_corridor", "blue_handle_entry_depth");
  updateWorstClearance(report, palmClear, "mouth_corridor", "blue_handle_palm_depth");
  updateWorstClearance(report, axialClear, "mouth_corridor", "blue_handle_axial_offset");
  updateWorstClearance(report, 0.05 * angleClear,
                       "mouth_corridor", "blue_handle_axis_alignment");

  const double designatedPadLowerBound = allowDesignatedPadContact
      ? -gripperContactTolerance_ : -gripperPenetrationTolerance_;
  report.bilateralPadContact = report.safe
      && report.leftPadClearance <= gripperContactTolerance_
      && report.rightPadClearance <= gripperContactTolerance_
      && report.leftPadClearance >= designatedPadLowerBound
      && report.rightPadClearance >= designatedPadLowerBound
      && report.padCenteringError <= padCenteringTolerance_;
  return report.safe;
}

HandoverInterceptionController::PreviewStepStatus
HandoverInterceptionController::previewReachStep(
    rbd::MultiBodyConfig & mbc,
    const sva::PTransformd & W_T_M_goal,
    bool requireCorridor,
    bool attachedRetreat,
    int & segmentIteration,
    PreviewResult & result,
    const Eigen::Vector3d & linearFeedforwardWorld,
    const Eigen::Vector3d & angularFeedforwardWorld,
    bool allowConvergence,
    const std::map<std::string, std::vector<double>> * postureTarget,
    bool collectDecisionMetrics) const
{
  if(segmentIteration >= previewMaxIterationsPerSegment_)
  {
    result.reason = "ik_preview_no_convergence";
    return PreviewStepStatus::Failed;
  }

  const auto & mb = robot().mb();
  rbd::Jacobian jac(mb, toolFrame_);
  const sva::PTransformd W_T_B_goal = previewBasePoseFromMouthPose(
      W_T_M_goal, mbc);
  const sva::PTransformd W_T_B = previewBasePose(mbc);
  const sva::PTransformd W_T_M = previewMouthPose(mbc);
  const Eigen::Vector3d pErr = W_T_B_goal.translation() - W_T_B.translation();
  const Eigen::Matrix3d Rerr = worldRotation(W_T_B_goal)
                             * worldRotation(W_T_B).transpose();
  Eigen::AngleAxisd aa(Rerr);
  Eigen::Vector3d rErr = Eigen::Vector3d::Zero();
  if(std::isfinite(aa.angle()) && aa.angle() > 1e-10)
  {
    rErr = aa.axis() * aa.angle();
  }

  if(allowConvergence
     && pErr.norm() <= previewPositionTolerance_
     && rErr.norm() <= previewOrientationTolerance_)
  {
    HandoverSafetyReport finalReport;
    const sva::PTransformd W_T_O_carried = compose(W_T_M, planningM_T_O_);
    const bool finalSafe = attachedRetreat
        ? previewAttachedRetreatSafe(mbc, W_T_O_carried, finalReport)
        : previewConfigurationSafe(mbc, W_T_M, requireCorridor, finalReport);
    if(!finalSafe)
    {
      result.reason = finalReport.sample + "/" + finalReport.obstacle;
      result.limitingSample = finalReport.sample;
      result.limitingObstacle = finalReport.obstacle;
      result.minClearance = std::min(result.minClearance,
                                     finalReport.minClearance);
      return PreviewStepStatus::Failed;
    }
    result.minClearance = std::min(result.minClearance,
                                   finalReport.minClearance);
    return PreviewStepStatus::Succeeded;
  }

  Eigen::Vector3d v = linearFeedforwardWorld + previewLinearGain_ * pErr;
  if(v.norm() > previewMaxLinearSpeed_)
  {
    v *= previewMaxLinearSpeed_ / v.norm();
  }
  Eigen::Vector3d w = angularFeedforwardWorld + previewAngularGain_ * rErr;
  if(w.norm() > previewMaxAngularSpeed_)
  {
    w *= previewMaxAngularSpeed_ / w.norm();
  }
  Eigen::Matrix<double, 6, 1> twist;
  twist.head<3>() = w;
  twist.tail<3>() = v;

  const Eigen::MatrixXd Jcompact = jac.jacobian(mb, mbc);
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, mb.nrDof());
  jac.fullJacobian(mb, Jcompact, J);

  if(collectDecisionMetrics
     && segmentIteration % decisionMetricStride_ == 0)
  {
    // Angular Jacobian rows are multiplied by a characteristic length so the
    // 6D condition index is dimensionless and translation/rotation are
    // comparable. One is isotropic; zero is singular.
    Eigen::MatrixXd Jmetric = J;
    Jmetric.topRows(3) *= decisionCharacteristicLength_;
    const Eigen::Matrix<double, 6, 6> gram = Jmetric * Jmetric.transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig(gram);
    if(eig.info() == Eigen::Success)
    {
      const double lambdaMax = std::max(
          1e-12, eig.eigenvalues().maxCoeff());
      const double lambdaMin = std::max(
          0.0, eig.eigenvalues().minCoeff());
      const double conditionIndex = std::sqrt(lambdaMin / lambdaMax);
      result.minimumConditionIndex = std::min(
          result.minimumConditionIndex, conditionIndex);
    }
    else
    {
      result.minimumConditionIndex = 0.0;
    }
  }

  // Joint-limit-aware weighted DLS. A joint loses mobility smoothly as it
  // approaches its safe interval boundary, so the task is redistributed
  // through the redundant arm instead of driving one wrist joint into a hard
  // limit and falsely declaring the whole grasp unreachable.
  Eigen::VectorXd mobility = Eigen::VectorXd::Ones(mb.nrDof());
  Eigen::VectorXd qLimitAvoidance = Eigen::VectorXd::Zero(mb.nrDof());
  const auto & ql = robot().ql();
  const auto & qu = robot().qu();
  for(size_t j = 0; j < mb.joints().size() && j < mbc.q.size()
      && j < ql.size() && j < qu.size(); ++j)
  {
    const std::string jointName = mb.joint(static_cast<int>(j)).name();
    if(jointName.rfind("gen3_robotiq_85_", 0) == 0) { continue; }
    const int dofPos = mb.jointPosInDof(static_cast<int>(j));
    const size_t m = std::min(mbc.q[j].size(),
                             std::min(ql[j].size(), qu[j].size()));
    for(size_t k = 0; k < m; ++k)
    {
      const int d = dofPos + static_cast<int>(k);
      if(d < 0 || d >= mobility.size()) { continue; }
      const double lo = ql[j][k];
      const double hi = qu[j][k];
      if(!std::isfinite(lo) || !std::isfinite(hi)) { continue; }
      const double margin = (hi - lo > 4.0 * previewJointLimitMargin_)
                          ? previewJointLimitMargin_ : 0.0;
      const double safeLo = lo + margin;
      const double safeHi = hi - margin;
      const double half = 0.5 * (safeHi - safeLo);
      if(half <= 1e-6) { continue; }
      const double center = 0.5 * (safeLo + safeHi);
      const double s = (mbc.q[j][k] - center) / half;
      const double absS = std::abs(s);
      if(collectDecisionMetrics)
      {
        result.minimumJointMarginRatio = std::min(
            result.minimumJointMarginRatio, std::max(0.0, 1.0 - absS));
      }
      const double barrierDen = std::max(1e-3, 1.0 - s * s);
      const double pressure = std::pow(absS, 6)
                            / (barrierDen * barrierDen);
      mobility[d] = std::max(
          0.02, 1.0 / (1.0 + previewJointLimitWeightGain_ * pressure));

      if(absS > previewJointLimitActivation_)
      {
        const double active = std::max(
            1e-6, 1.0 - previewJointLimitActivation_);
        const double u = std::min(
            1.0, (absS - previewJointLimitActivation_) / active);
        double v = -std::copysign(
            previewJointLimitAvoidanceGain_ * u * u
              / std::max(0.05, 1.0 - absS),
            s);
        v = std::max(-previewJointLimitVelocityCap_,
                     std::min(previewJointLimitVelocityCap_, v));
        qLimitAvoidance[d] = v;
      }
    }
  }

  const Eigen::MatrixXd WInv = mobility.asDiagonal();
  Eigen::Matrix<double, 6, 6> A = J * WInv * J.transpose();
  A.diagonal().array() += previewDamping_ * previewDamping_;
  const Eigen::MatrixXd Jpinv = WInv * J.transpose() * A.ldlt().solve(
      Eigen::Matrix<double, 6, 6>::Identity());
  Eigen::VectorXd qdot = Jpinv * twist;

  // Match the low-priority execution posture while adding a configuration-
  // space barrier. This objective contains no world-direction preference.
  Eigen::VectorXd qPosture = Eigen::VectorXd::Zero(mb.nrDof());
  for(size_t j = 0; j < mb.joints().size() && j < mbc.q.size(); ++j)
  {
    const auto & posture = postureTarget ? *postureTarget : readyPosture_;
    const auto it = posture.find(mb.joint(static_cast<int>(j)).name());
    if(it == posture.end() || it->second.empty() || mbc.q[j].empty())
    {
      continue;
    }
    const int dofPos = mb.jointPosInDof(static_cast<int>(j));
    if(dofPos >= 0 && dofPos < qPosture.size())
    {
      qPosture[dofPos] = previewPostureGain_
                       * (it->second[0] - mbc.q[j][0]);
    }
  }
  const Eigen::MatrixXd nullspace = Eigen::MatrixXd::Identity(
      mb.nrDof(), mb.nrDof()) - Jpinv * J;
  qdot += nullspace * (qPosture + qLimitAvoidance);

  const Eigen::VectorXd vl = rbd::dofToVector(mb, robot().vl());
  const Eigen::VectorXd vu = rbd::dofToVector(mb, robot().vu());
  if(vl.size() == qdot.size() && vu.size() == qdot.size())
  {
    for(Eigen::Index i = 0; i < qdot.size(); ++i)
    {
      if(std::isfinite(vl[i])) { qdot[i] = std::max(qdot[i], vl[i]); }
      if(std::isfinite(vu[i])) { qdot[i] = std::min(qdot[i], vu[i]); }
    }
  }

  // Fraction-to-boundary scaling guarantees that numerical integration never
  // steps beyond the same safe joint interval used by feasibility checking.
  double boundaryScale = 1.0;
  for(size_t j = 0; j < mb.joints().size() && j < mbc.q.size()
      && j < ql.size() && j < qu.size(); ++j)
  {
    const std::string jointName = mb.joint(static_cast<int>(j)).name();
    if(jointName.rfind("gen3_robotiq_85_", 0) == 0) { continue; }
    const int dofPos = mb.jointPosInDof(static_cast<int>(j));
    const size_t m = std::min(mbc.q[j].size(),
                             std::min(ql[j].size(), qu[j].size()));
    for(size_t k = 0; k < m; ++k)
    {
      const int d = dofPos + static_cast<int>(k);
      if(d < 0 || d >= qdot.size()) { continue; }
      const double lo = ql[j][k];
      const double hi = qu[j][k];
      if(!std::isfinite(lo) || !std::isfinite(hi)) { continue; }
      const double margin = (hi - lo > 4.0 * previewJointLimitMargin_)
                          ? previewJointLimitMargin_ : 0.0;
      const double safeLo = lo + margin;
      const double safeHi = hi - margin;
      const double dq = qdot[d];
      if(dq > 1e-12)
      {
        boundaryScale = std::min(
            boundaryScale,
            0.98 * std::max(0.0, safeHi - mbc.q[j][k])
              / (previewDt_ * dq));
      }
      else if(dq < -1e-12)
      {
        boundaryScale = std::min(
            boundaryScale,
            0.98 * std::max(0.0, mbc.q[j][k] - safeLo)
              / (-previewDt_ * dq));
      }
    }
  }
  qdot *= std::max(0.0, std::min(1.0, boundaryScale));

  if(collectDecisionMetrics
     && vl.size() == qdot.size() && vu.size() == qdot.size())
  {
    for(Eigen::Index i = 0; i < qdot.size(); ++i)
    {
      const double directionalLimit = qdot[i] >= 0.0 ? vu[i] : -vl[i];
      if(std::isfinite(directionalLimit) && directionalLimit > 1e-9)
      {
        result.maximumJointVelocityUtilization = std::max(
            result.maximumJointVelocityUtilization,
            std::abs(qdot[i]) / directionalLimit);
      }
    }
  }

  mbc.alpha = rbd::vectorToDof(mb, qdot);
  for(auto & aD : mbc.alphaD) { std::fill(aD.begin(), aD.end(), 0.0); }
  rbd::integration(mb, mbc, previewDt_);
  rbd::forwardKinematics(mb, mbc);
  ++segmentIteration;
  result.duration += previewDt_;
  result.effort += qdot.squaredNorm() * previewDt_;

  std::string limitReason;
  if(!previewJointLimitsSafe(mbc, limitReason))
  {
    result.reason = limitReason;
    return PreviewStepStatus::Failed;
  }

  HandoverSafetyReport report;
  const sva::PTransformd newMouth = previewMouthPose(mbc);
  const sva::PTransformd W_T_O_carried = compose(newMouth, planningM_T_O_);
  const bool stepSafe = attachedRetreat
      ? previewAttachedRetreatSafe(mbc, W_T_O_carried, report)
      : previewConfigurationSafe(mbc, newMouth, requireCorridor, report);
  if(!stepSafe)
  {
    result.reason = report.sample + "/" + report.obstacle;
    result.limitingSample = report.sample;
    result.limitingObstacle = report.obstacle;
    result.minClearance = std::min(result.minClearance, report.minClearance);
    return PreviewStepStatus::Failed;
  }
  result.minClearance = std::min(result.minClearance, report.minClearance);
  return PreviewStepStatus::Running;
}

bool HandoverInterceptionController::previewReachSegment(
    rbd::MultiBodyConfig & mbc,
    const sva::PTransformd & W_T_M_goal,
    bool requireCorridor,
    bool attachedRetreat,
    PreviewResult & result,
    bool collectDecisionMetrics) const
{
  int segmentIteration = 0;
  while(true)
  {
    const PreviewStepStatus status = previewReachStep(
        mbc, W_T_M_goal, requireCorridor, attachedRetreat,
        segmentIteration, result, Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), true, nullptr, collectDecisionMetrics);
    if(status == PreviewStepStatus::Succeeded) { return true; }
    if(status == PreviewStepStatus::Failed) { return false; }
  }
}

HandoverInterceptionController::PreviewStepStatus
HandoverInterceptionController::previewClosureStep(
    rbd::MultiBodyConfig & mbc,
    int & closureIndex,
    PreviewResult & result) const
{
  const int samples = std::max(20, previewClosureSamples_);
  if(closureIndex > samples)
  {
    result.reason = "closure/no_bilateral_contact";
    return PreviewStepStatus::Failed;
  }

  const double closure = gripperMaxClosure_
      * static_cast<double>(closureIndex) / static_cast<double>(samples);
  setPreviewGripperClosure(mbc, closure);

  HandoverSafetyReport report;
  bool closureSafe = previewDynamicClosureSafety(mbc, report, false);
  if(!closureSafe)
  {
    // Match the e2e194d runtime Closing transition: only the two designated
    // inner-pad/blue-handle pairs may use the existing contact band, and only
    // when the copied state already establishes bilateral contact. Every
    // other contact and geometric constraint remains hard.
    HandoverSafetyReport designatedContactReport;
    const bool designatedContactSafe = previewDynamicClosureSafety(
        mbc, designatedContactReport, true);
    if(designatedContactSafe && designatedContactReport.bilateralPadContact)
    {
      report = designatedContactReport;
      closureSafe = true;
      mc_rtc::log::info(
          "[PreviewContactEntry] designated bilateral pad contact admitted with runtime contact band left={:.4f} right={:.4f} centerErr={:.4f} allOtherConstraintsHard=true",
          report.leftPadClearance, report.rightPadClearance,
          report.padCenteringError);
    }
  }
  if(!closureSafe)
  {
    result.reason = "closure/" + report.sample + "/" + report.obstacle;
    result.limitingSample = report.sample;
    result.limitingObstacle = report.obstacle;
    result.minClearance = std::min(result.minClearance, report.minClearance);
    return PreviewStepStatus::Failed;
  }
  result.minClearance = std::min(result.minClearance, report.minClearance);

  // Do not reject at the first discrete sample where only one pad has
  // entered the contact tolerance. With a finite closure discretisation, two
  // geometrically valid symmetric contacts commonly cross that threshold one
  // sample apart. Continue the virtual closure while the hard penetration and
  // centering constraints in previewDynamicClosureSafety remain satisfied.
  // A genuinely off-centre grasp is still rejected before visible
  // penetration, whereas a sub-millimetre sampling offset may reach the next
  // safe bilateral-contact sample.
  if(report.bilateralPadContact)
  {
    result.contactClosure = closure;
    result.duration += closure / std::max(1e-3, gripperCloseRate_);
    return PreviewStepStatus::Succeeded;
  }

  ++closureIndex;
  return PreviewStepStatus::Running;
}


bool HandoverInterceptionController::previewTerminalCaptureDwell(
    rbd::MultiBodyConfig & mbc,
    double duration,
    PreviewResult & result) const
{
  const int steps = std::max(0, static_cast<int>(std::ceil(
      std::max(0.0, duration) / std::max(1e-9, previewDt_))));
  setPreviewGripperClosure(mbc, 0.0);
  for(int k = 0; k < steps; ++k)
  {
    for(auto & a : mbc.alpha) { std::fill(a.begin(), a.end(), 0.0); }
    for(auto & aD : mbc.alphaD) { std::fill(aD.begin(), aD.end(), 0.0); }
    rbd::forwardKinematics(robot().mb(), mbc);

    HandoverSafetyReport report;
    const sva::PTransformd mouth = previewMouthPose(mbc);
    if(!previewConfigurationSafe(mbc, mouth, true, report))
    {
      result.reason = "terminal_capture_dwell/" + report.sample
                    + "/" + report.obstacle;
      result.limitingSample = report.sample;
      result.limitingObstacle = report.obstacle;
      result.minClearance = std::min(result.minClearance,
                                     report.minClearance);
      return false;
    }
    result.minClearance = std::min(result.minClearance,
                                   report.minClearance);
    result.duration += previewDt_;
  }
  return true;
}


bool HandoverInterceptionController::previewVelocityGateParityShadow(
    rbd::MultiBodyConfig mbc,
    const CaptureCandidate & candidate,
    double & duration,
    double & finalPositionError,
    double & finalOrientationError,
    double & finalLinearSpeed,
    double & finalAngularSpeed,
    double & finalStableTime,
    std::string & reason,
    bool verbose) const
{
  // Copied-state terminal timing approximation for a complete route. It keeps
  // hard feasibility unchanged, but its duration contributes to the plan cost
  // and can affect the committed action in binding_cost mode. It must not be
  // described as exact runtime-policy parity: this bounded shadow has its own
  // constants and numerical integration.
  constexpr double farLinearSpeed = 0.38;
  constexpr double nearLinearSpeed = 0.14;
  constexpr double nearDistance = 0.025;
  constexpr double angularSpeed = 1.50;
  constexpr double linearAcceleration = 1.60;
  constexpr double angularAcceleration = 4.50;
  constexpr double linearDeceleration = 2.00;
  constexpr double angularDeceleration = 4.50;
  constexpr double brakeMargin = 0.008;
  constexpr double spatialLinearLookAhead = 0.018;
  constexpr double minimumLinearLookAhead = 0.004;
  constexpr double lookAheadTaperDistance = 0.012;
  constexpr double linearFeedforwardRatio = 0.70;
  constexpr double angularFeedforwardRatio = 0.70;
  constexpr double maxLinearLead = 0.028;
  constexpr double maxAngularLead = 0.12;
  constexpr double positionTolerance = 0.003;
  constexpr double orientationTolerance = 0.025;
  constexpr double angularActivationMargin = 0.015;
  constexpr double centeringTolerance = 0.00045;
  constexpr double linearSpeedTolerance = 0.040;
  constexpr double angularSpeedTolerance = 0.080;
  constexpr double stableDwell = 0.080;
  constexpr double velocityFilter = 0.20;
  constexpr double maximumDuration = 4.0;

  const auto clamp01Shadow = [](double x) {
    return std::max(0.0, std::min(1.0, x));
  };
  const auto smoothStepShadow = [&](double x) {
    const double u = clamp01Shadow(x);
    return u * u * (3.0 - 2.0 * u);
  };
  const auto rampSpeedShadow = [](double current, double desired,
                                  double acceleration, double dt) {
    const double step = std::max(0.0, acceleration)
                      * std::max(0.0, dt);
    return current + std::max(-step,
                              std::min(step, desired - current));
  };
  const auto taperedLookAheadShadow = [&](double maximum, double minimum,
                                           double remaining,
                                           double tolerance,
                                           double taperDistance) {
    const double maxValue = std::max(0.0, maximum);
    const double minValue = std::min(maxValue, std::max(0.0, minimum));
    const double taperStart = std::max(tolerance + 1e-6, taperDistance);
    if(remaining >= taperStart) { return maxValue; }
    const double alpha = clamp01Shadow(
        (remaining - tolerance)
        / std::max(1e-9, taperStart - tolerance));
    return minValue + alpha * (maxValue - minValue);
  };

  duration = 0.0;
  finalPositionError = std::numeric_limits<double>::infinity();
  finalOrientationError = std::numeric_limits<double>::infinity();
  finalLinearSpeed = std::numeric_limits<double>::infinity();
  finalAngularSpeed = std::numeric_limits<double>::infinity();
  finalStableTime = 0.0;
  reason = "not_started";

  setPreviewGripperClosure(mbc, 0.0);
  const sva::PTransformd goal = candidate.W_T_M_pre;
  const sva::PTransformd startPose = previewMouthPose(mbc);
  sva::PTransformd previous = startPose;
  sva::PTransformd reference = startPose;
  const Eigen::Vector3d linearPath =
      goal.translation() - startPose.translation();
  const double pathDistance = linearPath.norm();
  Eigen::Vector3d linearPathDirection = Eigen::Vector3d::Zero();
  if(pathDistance > 1e-9)
  {
    linearPathDirection = linearPath / pathDistance;
  }
  const double pathAngle = orientationError(startPose, goal);
  const bool linearProgressCoupled = pathDistance > positionTolerance;
  const bool angularProgressCoupled =
      pathAngle > orientationTolerance + angularActivationMargin;

  Eigen::Vector3d angularPathDirection = Eigen::Vector3d::Zero();
  if(angularProgressCoupled)
  {
    const Eigen::Matrix3d Rerr = worldRotation(goal)
                               * worldRotation(startPose).transpose();
    Eigen::AngleAxisd aa(Rerr);
    if(std::isfinite(aa.angle()) && aa.angle() > 1e-10)
    {
      angularPathDirection = aa.axis();
    }
  }

  double commandedLinearSpeed = 0.0;
  double commandedAngularSpeed = 0.0;
  double filteredLinearSpeed = 0.0;
  double filteredAngularSpeed = 0.0;
  double stableTime = 0.0;
  double measuredProgressAnchor = 0.0;
  double referenceProgress = 0.0;
  bool haveVelocityEstimate = false;
  bool linearBrakingActive = false;
  bool angularBrakingActive = false;
  double dynamicBrakeDistance = 0.0;
  double dynamicBrakeAngle = 0.0;
  int segmentIteration = 0;
  PreviewResult shadow;
  shadow.minClearance = std::numeric_limits<double>::infinity();
  const int maximumSteps = std::max(1, static_cast<int>(std::ceil(
      maximumDuration / std::max(1e-9, previewDt_))));
  const double effectiveFilter = 1.0 - std::pow(
      1.0 - velocityFilter,
      previewDt_ / std::max(1e-9, controlDt_));

  for(int step = 0; step < maximumSteps; ++step)
  {
    const sva::PTransformd current = previewMouthPose(mbc);
    Eigen::Vector3d linearVelocity;
    Eigen::Vector3d angularVelocity;
    worldPoseTwist(previous, current, previewDt_,
                   linearVelocity, angularVelocity);
    previous = current;
    const double measuredLinearSpeed = linearVelocity.norm();
    const double measuredAngularSpeed = angularVelocity.norm();
    if(!haveVelocityEstimate)
    {
      filteredLinearSpeed = measuredLinearSpeed;
      filteredAngularSpeed = measuredAngularSpeed;
      haveVelocityEstimate = true;
    }
    else
    {
      filteredLinearSpeed = (1.0 - effectiveFilter) * filteredLinearSpeed
                          + effectiveFilter * measuredLinearSpeed;
      filteredAngularSpeed = (1.0 - effectiveFilter) * filteredAngularSpeed
                           + effectiveFilter * measuredAngularSpeed;
    }

    const double dist = (goal.translation() - current.translation()).norm();
    const double angle = orientationError(current, goal);
    HandoverSafetyReport captureReport;
    const bool captureSafe = previewDynamicClosureSafety(mbc, captureReport);
    shadow.minClearance = std::min(
        shadow.minClearance, captureReport.minClearance);
    const bool positionReady = dist <= positionTolerance;
    const bool orientationReady = angle <= orientationTolerance;
    const bool centeringReady = captureSafe
        && captureReport.padCenteringError <= centeringTolerance;
    const bool linearVelocityReady = filteredLinearSpeed
        <= linearSpeedTolerance;
    const bool angularVelocityReady = filteredAngularSpeed
        <= angularSpeedTolerance;
    const bool geometryReady = positionReady && orientationReady
                            && centeringReady;
    const bool velocityReady = linearVelocityReady && angularVelocityReady;

    finalPositionError = dist;
    finalOrientationError = angle;
    finalLinearSpeed = filteredLinearSpeed;
    finalAngularSpeed = filteredAngularSpeed;
    finalStableTime = stableTime;

    if(geometryReady && velocityReady)
    {
      stableTime += previewDt_;
      finalStableTime = stableTime;
      if(stableTime + 1e-12 >= stableDwell)
      {
        duration = shadow.duration;
        reason = "success";
        return true;
      }
    }
    else
    {
      stableTime = 0.0;
    }

    Eigen::Vector3d linearFeedforwardWorld = Eigen::Vector3d::Zero();
    Eigen::Vector3d angularFeedforwardWorld = Eigen::Vector3d::Zero();
    double terminalFeedforwardScale = 0.0;
    double terminalAngularFeedforwardScale = 0.0;

    if(geometryReady)
    {
      commandedLinearSpeed = rampSpeedShadow(
          commandedLinearSpeed, 0.0, linearDeceleration, previewDt_);
      commandedAngularSpeed = rampSpeedShadow(
          commandedAngularSpeed, 0.0, angularDeceleration, previewDt_);
      reference = goal;
    }
    else
    {
      const double speedAlpha = nearDistance > 1e-9
          ? clamp01Shadow(dist / nearDistance) : 1.0;
      const double nominalLinearSpeed = nearLinearSpeed
          + speedAlpha * (farLinearSpeed - nearLinearSpeed);
      const double filteredLinearSquared = filteredLinearSpeed
                                         * filteredLinearSpeed;
      const double terminalLinearSquared = linearSpeedTolerance
                                          * linearSpeedTolerance;
      const double requiredLinearBrakeDistance = positionTolerance
          + brakeMargin
          + std::max(0.0, filteredLinearSquared - terminalLinearSquared)
              / (2.0 * linearDeceleration);
      const double filteredAngularSquared = filteredAngularSpeed
                                          * filteredAngularSpeed;
      const double terminalAngularSquared = angularSpeedTolerance
                                           * angularSpeedTolerance;
      const double requiredAngularBrakeAngle = orientationTolerance
          + std::max(0.0, filteredAngularSquared - terminalAngularSquared)
              / (2.0 * angularDeceleration);

      if(!linearBrakingActive && dist <= requiredLinearBrakeDistance)
      {
        linearBrakingActive = true;
        dynamicBrakeDistance = std::max(
            lookAheadTaperDistance, requiredLinearBrakeDistance);
      }
      if(angularProgressCoupled && !angularBrakingActive
         && angle <= requiredAngularBrakeAngle)
      {
        angularBrakingActive = true;
        dynamicBrakeAngle = std::max(
            orientationTolerance + 0.001, requiredAngularBrakeAngle);
      }
      if(linearBrakingActive)
      {
        dynamicBrakeDistance = std::max(
            dynamicBrakeDistance, requiredLinearBrakeDistance);
      }
      if(angularBrakingActive)
      {
        dynamicBrakeAngle = std::max(
            dynamicBrakeAngle, requiredAngularBrakeAngle);
      }

      const double terminalLinearBudget = std::max(
          0.0, dist - positionTolerance - brakeMargin);
      const double terminalAngularBudget = std::max(
          0.0, angle - orientationTolerance);
      const double brakingLinearSpeed = std::sqrt(std::max(
          0.0, terminalLinearSquared
               + 2.0 * linearDeceleration * terminalLinearBudget));
      const double brakingAngularSpeed = std::sqrt(std::max(
          0.0, terminalAngularSquared
               + 2.0 * angularDeceleration * terminalAngularBudget));
      const double desiredLinearSpeed = linearBrakingActive
          ? std::min(nominalLinearSpeed, brakingLinearSpeed)
          : nominalLinearSpeed;
      const double desiredAngularSpeed = angularBrakingActive
          ? std::min(angularSpeed, brakingAngularSpeed)
          : angularSpeed;

      commandedLinearSpeed = rampSpeedShadow(
          commandedLinearSpeed, desiredLinearSpeed,
          linearBrakingActive ? linearDeceleration : linearAcceleration,
          previewDt_);
      commandedAngularSpeed = rampSpeedShadow(
          commandedAngularSpeed, desiredAngularSpeed,
          angularBrakingActive ? angularDeceleration : angularAcceleration,
          previewDt_);

      double measuredLinearProgress = 1.0;
      if(pathDistance > 1e-9)
      {
        measuredLinearProgress = clamp01Shadow(
            (current.translation() - startPose.translation()).dot(linearPath)
            / std::max(1e-12, linearPath.squaredNorm()));
      }
      const double measuredAngularProgress = pathAngle > 1e-9
          ? clamp01Shadow(1.0 - angle / pathAngle) : 1.0;
      double measuredProgress = 1.0;
      if(linearProgressCoupled && angularProgressCoupled)
      {
        measuredProgress = std::min(measuredLinearProgress,
                                    measuredAngularProgress);
      }
      else if(linearProgressCoupled) { measuredProgress = measuredLinearProgress; }
      else if(angularProgressCoupled) { measuredProgress = measuredAngularProgress; }
      measuredProgressAnchor = std::max(
          measuredProgressAnchor, measuredProgress);

      const double activeLinearTaperDistance = linearBrakingActive
          ? std::max(lookAheadTaperDistance, dynamicBrakeDistance)
          : lookAheadTaperDistance;
      const double activeLinearLookAhead = taperedLookAheadShadow(
          spatialLinearLookAhead, minimumLinearLookAhead,
          dist, positionTolerance, activeLinearTaperDistance);
      double lookAheadProgress = pathDistance > 1e-9
          ? activeLinearLookAhead / pathDistance : 1.0;
      lookAheadProgress = clamp01Shadow(lookAheadProgress);
      const double targetProgress = std::min(
          1.0, std::max(referenceProgress,
                        measuredProgressAnchor + lookAheadProgress));
      const sva::PTransformd spatialTarget = interpolatePose(
          startPose, goal, targetProgress);
      reference = boundedPoseStep(
          reference, spatialTarget,
          commandedLinearSpeed * previewDt_,
          commandedAngularSpeed * previewDt_);

      double referenceLinearProgress = 1.0;
      if(pathDistance > 1e-9)
      {
        referenceLinearProgress = clamp01Shadow(
            (reference.translation() - startPose.translation()).dot(linearPath)
            / std::max(1e-12, linearPath.squaredNorm()));
      }
      referenceProgress = std::max(referenceProgress,
                                   referenceLinearProgress);

      terminalFeedforwardScale = 1.0;
      if(linearBrakingActive
         && dynamicBrakeDistance > positionTolerance + 1e-9)
      {
        terminalFeedforwardScale = smoothStepShadow(
            (dist - positionTolerance)
            / (dynamicBrakeDistance - positionTolerance));
      }
      if(pathDistance > positionTolerance && dist > positionTolerance)
      {
        linearFeedforwardWorld = terminalFeedforwardScale
                               * linearFeedforwardRatio
                               * commandedLinearSpeed
                               * linearPathDirection;
      }

      terminalAngularFeedforwardScale = 1.0;
      if(angularBrakingActive
         && dynamicBrakeAngle > orientationTolerance + 1e-9)
      {
        terminalAngularFeedforwardScale = smoothStepShadow(
            (angle - orientationTolerance)
            / (dynamicBrakeAngle - orientationTolerance));
      }
      if(angularProgressCoupled && angle > orientationTolerance)
      {
        angularFeedforwardWorld = terminalAngularFeedforwardScale
                                * angularFeedforwardRatio
                                * commandedAngularSpeed
                                * angularPathDirection;
      }
    }

    if(verbose && (step % 5 == 0 || (geometryReady && velocityReady)))
    {
      mc_rtc::log::info(
          "[VelocityGateParityFeedforward] candidate={} route={} t={:.3f}s posReady={} oriReady={} captureSafe={} centeringReady={} linearVelocityReady={} angularVelocityReady={} stable={:.3f}/{:.3f}s dist={:.4f}/{:.4f} angle={:.4f}/{:.4f} centerErr={:.4f}/{:.4f} speed=[v:{:.4f}/{:.4f},w:{:.4f}/{:.4f}] command=[v:{:.3f},w:{:.3f}] feedforward=[v:{:.3f},w:{:.3f}] brake=[linear:{},angular:{}] scale=[{:.3f},{:.3f}]",
          candidate.name, candidate.transitRouteName, shadow.duration,
          positionReady, orientationReady, captureSafe, centeringReady,
          linearVelocityReady, angularVelocityReady,
          stableTime, stableDwell, dist, positionTolerance,
          angle, orientationTolerance,
          captureReport.padCenteringError, centeringTolerance,
          filteredLinearSpeed, linearSpeedTolerance,
          filteredAngularSpeed, angularSpeedTolerance,
          commandedLinearSpeed, commandedAngularSpeed,
          linearFeedforwardWorld.norm(), angularFeedforwardWorld.norm(),
          linearBrakingActive, angularBrakingActive,
          terminalFeedforwardScale, terminalAngularFeedforwardScale);
    }

    const sva::PTransformd boundedReference = boundedPoseStep(
        current, reference, maxLinearLead, maxAngularLead);
    const PreviewStepStatus status = previewReachStep(
        mbc, boundedReference, true, false,
        segmentIteration, shadow,
        linearFeedforwardWorld, angularFeedforwardWorld, false,
        &candidate.plannedArmPosture, true);
    if(status == PreviewStepStatus::Failed)
    {
      duration = shadow.duration;
      reason = "tracking/" + shadow.reason;
      return false;
    }
  }

  duration = shadow.duration;
  finalStableTime = stableTime;
  reason = "timeout";
  return false;
}

bool HandoverInterceptionController::previewClosureSweep(
    rbd::MultiBodyConfig & mbc,
    PreviewResult & result) const
{
  int closureIndex = 0;
  while(true)
  {
    const PreviewStepStatus status = previewClosureStep(
        mbc, closureIndex, result);
    if(status == PreviewStepStatus::Succeeded) { return true; }
    if(status == PreviewStepStatus::Failed) { return false; }
  }
}

bool HandoverInterceptionController::verifyPredictiveStaticCandidate(
    CaptureCandidate & candidate,
    const PreviewResult & staticResult,
    const sva::PTransformd & W_T_O_presentation)
{
  const auto routes = transitRouteBank(
      planningStartMouthPose_, candidate.W_T_M_standoff);
  bool found = false;
  CaptureCandidate best;
  std::string lastFailure = "no_route_tested";

  for(const auto & route : routes)
  {
    CaptureCandidate trial = candidate;
    if(!verifyPredictiveRouteCandidate(
           trial, staticResult, W_T_O_presentation,
           route.first, route.second))
    {
      lastFailure = trial.failureReason;
      mc_rtc::log::info(
          "[TransitRoute] candidate={} route={} infeasible reason={} reachClear={:.4f}",
          candidate.name, route.first, trial.failureReason,
          trial.predictiveReachClearance);
      continue;
    }

    planningCompletePlanAuditCandidates_.push_back(trial);

    const double routeCost = trial.estimatedTime
        + previewEffortWeight_ * trial.predictedEffort
        + previewRotationWeight_ * trial.rotation;
    const double bestCost = best.estimatedTime
        + previewEffortWeight_ * best.predictedEffort
        + previewRotationWeight_ * best.rotation;
    bool choose = !found;
    if(found)
    {
      const double clearanceGain = trial.predictiveReachClearance
                                 - best.predictiveReachClearance;
      const bool clearlySafer = clearanceGain
          > transitClearancePreferenceBand_;
      const bool comparableClearance = std::abs(clearanceGain)
          <= transitClearancePreferenceBand_;
      choose = clearlySafer
          || (comparableClearance && routeCost < bestCost);
    }
    if(choose)
    {
      best = trial;
      found = true;
    }
  }

  if(!found)
  {
    candidate.failureReason = "transit_route_bank/" + lastFailure;
    return false;
  }

  candidate = best;
  mc_rtc::log::success(
      "[TransitRoute] SELECTED candidate={} route={} curveOffset=[{:.3f},{:.3f},{:.3f}] pathLength={:.3f}m reachClear={:.4f}m completePlan=true",
      candidate.name, candidate.transitRouteName,
      candidate.reachCurveOffsetWorld.x(),
      candidate.reachCurveOffsetWorld.y(),
      candidate.reachCurveOffsetWorld.z(),
      candidate.transitPathLength, candidate.predictiveReachClearance);
  return true;
}

bool HandoverInterceptionController::verifyPredictiveRouteCandidate(
    CaptureCandidate & candidate,
    const PreviewResult & staticResult,
    const sva::PTransformd & W_T_O_presentation,
    const std::string & routeName,
    const Eigen::Vector3d & curveOffsetWorld)
{
  candidate.transitRouteName = routeName;
  candidate.reachCurveOffsetWorld = curveOffsetWorld;
  if(!previewMovingInterception_) { return true; }
  if(!presentationMode_)
  {
    candidate.failureReason = "predictive_static/presentation_mode_required";
    return false;
  }
  if(staticResult.contactClosure <= 0.0
     || !std::isfinite(staticResult.contactClosure))
  {
    candidate.failureReason = "predictive_static/no_contact_closure";
    return false;
  }

  // Immutable presentation anchor. The virtual object below is mutable, but
  // this value copy can never alias it.
  const sva::PTransformd presentationAnchor = W_T_O_presentation;
  const sva::PTransformd savedObject = W_T_O_;
  const sva::PTransformd savedHandle = W_T_H_;
  const sva::PTransformd savedPlanningAttachment = planningM_T_O_;
  auto restorePlanningWorld = [&]()
  {
    W_T_O_ = savedObject;
    W_T_H_ = savedHandle;
    planningM_T_O_ = savedPlanningAttachment;
  };
  auto setVirtualObject = [&](const sva::PTransformd & W_T_O_virtual)
  {
    W_T_O_ = W_T_O_virtual;
    W_T_H_ = compose(W_T_O_, O_T_H_);
  };

  candidate.contactClosure = staticResult.contactClosure;
  const double baseReachDuration = std::max(
      2.0 * previewDt_,
      timingArmScale_ * staticResult.reachStandoffDuration);
  const double chordLength = (candidate.W_T_M_standoff.translation()
      - planningStartMouthPose_.translation()).norm();
  candidate.transitPathLength = reachCurveLength(
      planningStartMouthPose_, candidate.W_T_M_standoff,
      candidate.reachCurveOffsetWorld);
  candidate.W_T_M_transit = reachCurvePose(
      planningStartMouthPose_, candidate.W_T_M_standoff,
      candidate.reachCurveOffsetWorld, 0.5);
  const double pathStretch = candidate.transitPathLength
      / std::max(0.05, chordLength);
  if(pathStretch > transitMaximumPathStretch_)
  {
    candidate.failureReason = "transit_route/path_stretch_limit";
    restorePlanningWorld();
    return false;
  }
  const double reachDuration = baseReachDuration * std::max(1.0, pathStretch);
  const double approachDuration = timingTerminalCaptureDwell_ + std::max(
      2.0 * previewDt_,
      timingArmScale_ * staticResult.reachCaptureDuration);
  const double acquireDuration = timingPriorityBlend_ + timingCaptureLock_
      + staticResult.contactClosure
          / std::max(1e-6, timingEffectiveGripperRate_);
  const double retreatDuration = std::max(
      0.0, timingArmScale_ * staticResult.retreatDuration);

  const InterceptionPlan plan = makeInterceptionPlan(
      candidate, presentationAnchor, 0.0,
      reachDuration, approachDuration, acquireDuration, retreatDuration);
  std::string invariantReason;
  if(!validateInterceptionPlan(plan, &invariantReason, false))
  {
    candidate.failureReason = "predictive_static/" + invariantReason;
    restorePlanningWorld();
    return false;
  }

  // -----------------------------------------------------------------------
  // 1) Runtime-policy-equivalent predictive reach. The copied-state rollout
  //    uses the same clearance governor, rate cap and measured-pose lead tube
  //    as ExecuteCommittedReach, then integrates whole-arm IK on that bounded
  //    command. This is a certification model, not a second controller.
  // -----------------------------------------------------------------------
  // Same frozen decision-state discipline as startNextPlanningCandidate():
  // this route's local rollout must not seed from a live robot().mbc() that
  // depends on how many other candidates/routes were already processed in
  // this search. See resetGlobalTimePlanSearch().
  rbd::MultiBodyConfig mbc = globalTimePlanFrozenRobotStateValid_
      ? globalTimePlanFrozenRobotState_ : robot().mbc();
  for(auto & a : mbc.alpha) { std::fill(a.begin(), a.end(), 0.0); }
  for(auto & aD : mbc.alphaD) { std::fill(aD.begin(), aD.end(), 0.0); }
  setPreviewGripperClosure(mbc, 0.0);

  PreviewResult reachResult;
  reachResult.minClearance = std::numeric_limits<double>::infinity();
  int reachIteration = 0;
  const int reachSteps = std::max(1, static_cast<int>(std::ceil(
      plan.reachDuration / previewDt_)));
  const PredictiveReachPolicy & policy = predictiveReachPolicy_;
  sva::PTransformd commandReference = plan.mouthAtReachStart;
  double clearanceScale = 1.0;
  bool transitPostureSaved = false;

  for(int k = 0; k < reachSteps; ++k)
  {
    const double t = std::min(
        plan.standoffTime, plan.reachStartTime + k * previewDt_);
    const double tNext = std::min(
        plan.standoffTime, t + previewDt_);
    const MovingReference refNext = interceptionReferenceAt(
        plan, tNext, previewDt_);

    setPreviewGripperClosure(mbc, 0.0);
    setVirtualObject(refNext.objectPose);
    const sva::PTransformd currentMouth = previewMouthPose(mbc);

    HandoverSafetyReport currentReport;
    if(!previewConfigurationSafe(mbc, currentMouth, false, currentReport))
    {
      candidate.failureReason = "predictive_static/reach_current/"
                              + currentReport.sample + "/"
                              + currentReport.obstacle;
      candidate.predictiveReachClearance = currentReport.minClearance;
      restorePlanningWorld();
      return false;
    }
    reachResult.minClearance = std::min(
        reachResult.minClearance, currentReport.minClearance);
    if(currentReport.minClearance < policy.minimumRuntimeClearance)
    {
      candidate.failureReason = "predictive_static/runtime_clearance_reserve";
      candidate.predictiveReachClearance = currentReport.minClearance;
      restorePlanningWorld();
      return false;
    }

    double rawClearanceScale = 1.0;
    if(currentReport.minClearance < policy.clearanceSlowdownStart)
    {
      const double denominator = std::max(
          1e-6, policy.clearanceSlowdownStart - policy.clearanceHardMargin);
      const double u = std::min(1.0, std::max(0.0,
          (currentReport.minClearance - policy.clearanceHardMargin)
              / denominator));
      const double smooth = u * u * (3.0 - 2.0 * u);
      rawClearanceScale = policy.minimumVelocityScale
          + (1.0 - policy.minimumVelocityScale) * smooth;
    }
    const double scaleRate = rawClearanceScale < clearanceScale
        ? policy.clearanceScaleDropRate : policy.clearanceScaleRiseRate;
    const double maximumScaleChange = scaleRate * previewDt_;
    const double scaleError = rawClearanceScale - clearanceScale;
    clearanceScale += std::max(
        -maximumScaleChange, std::min(maximumScaleChange, scaleError));
    clearanceScale = std::min(1.0, std::max(
        policy.minimumVelocityScale, clearanceScale));

    const double linearSpeedLimit = policy.nearLinearSpeed
        + clearanceScale * (policy.farLinearSpeed - policy.nearLinearSpeed);
    const double angularSpeedLimit = policy.nearAngularSpeed
        + clearanceScale * (policy.farAngularSpeed - policy.nearAngularSpeed);
    const double linearLeadLimit = policy.nearLinearTrackingLead
        + clearanceScale
            * (policy.maxLinearTrackingLead - policy.nearLinearTrackingLead);
    const double angularLeadLimit = policy.nearAngularTrackingLead
        + clearanceScale
            * (policy.maxAngularTrackingLead - policy.nearAngularTrackingLead);

    const sva::PTransformd rateLimitedReference = boundedPoseStep(
        commandReference, refNext.mouthPose,
        linearSpeedLimit * previewDt_, angularSpeedLimit * previewDt_);
    const sva::PTransformd nextCommand = boundedPoseStep(
        currentMouth, rateLimitedReference,
        linearLeadLimit, angularLeadLimit);

    HandoverSafetyReport sweptReport;
    if(!sweptGripperPoseSafe(
           currentMouth, nextCommand, sweptReport, false))
    {
      candidate.failureReason = "predictive_static/reach_command/"
                              + sweptReport.sample + "/"
                              + sweptReport.obstacle;
      candidate.predictiveReachClearance = sweptReport.minClearance;
      restorePlanningWorld();
      return false;
    }
    reachResult.minClearance = std::min(
        reachResult.minClearance, sweptReport.minClearance);

    Eigen::Vector3d ffLinear = Eigen::Vector3d::Zero();
    Eigen::Vector3d ffAngular = Eigen::Vector3d::Zero();
    const sva::PTransformd baseCommandNow = previewBasePoseFromMouthPose(
        commandReference, mbc);
    const sva::PTransformd baseCommandNext = previewBasePoseFromMouthPose(
        nextCommand, mbc);
    worldPoseTwist(baseCommandNow, baseCommandNext, previewDt_,
                   ffLinear, ffAngular);

    const PreviewStepStatus status = previewReachStep(
        mbc, nextCommand, false, false, reachIteration, reachResult,
        ffLinear, ffAngular, false,
        &candidate.plannedStandoffArmPosture, true);
    if(status == PreviewStepStatus::Failed)
    {
      candidate.failureReason = "predictive_static/reach/"
                              + reachResult.reason;
      candidate.predictiveReachClearance = reachResult.minClearance;
      restorePlanningWorld();
      return false;
    }
    commandReference = nextCommand;

    if(!transitPostureSaved && refNext.phaseProgress >= 0.5)
    {
      candidate.plannedTransitArmPosture = armPostureFromMbc(mbc);
      transitPostureSaved = true;
    }
  }

  candidate.predictiveReachClearance = reachResult.minClearance;
  if(!std::isfinite(reachResult.minClearance)
     || reachResult.minClearance < transitMinimumPredictedClearance_)
  {
    candidate.failureReason =
        "predictive_static/robust_transit_clearance_reserve";
    candidate.minClearance = std::min(
        candidate.minClearance, reachResult.minClearance);
    restorePlanningWorld();
    return false;
  }

  setVirtualObject(presentationAnchor);
  setPreviewGripperClosure(mbc, 0.0);
  const sva::PTransformd reachedStandoff = previewMouthPose(mbc);
  const double reachPositionError = (
      candidate.W_T_M_standoff.translation()
      - reachedStandoff.translation()).norm();
  const double reachOrientationError = orientationError(
      reachedStandoff, candidate.W_T_M_standoff);
  if(reachPositionError > policy.positionTolerance
     || reachOrientationError > policy.orientationTolerance)
  {
    candidate.failureReason = "predictive_static/reach_tracking";
    candidate.minClearance = std::min(
        candidate.minClearance, reachResult.minClearance);
    restorePlanningWorld();
    return false;
  }
  if(!transitPostureSaved)
  {
    candidate.plannedTransitArmPosture = armPostureFromMbc(mbc);
  }
  candidate.plannedStandoffArmPosture = armPostureFromMbc(mbc);
  const rbd::MultiBodyConfig timingAuditStartMbc = mbc;

  // -----------------------------------------------------------------------
  // 2) Exact proven static terminal controller from the reached standoff:
  //    open insertion -> full closure -> bilateral contact -> retreat.
  // -----------------------------------------------------------------------
  PreviewResult terminalResult;
  terminalResult.minClearance = std::numeric_limits<double>::infinity();
  double phaseStart = terminalResult.duration;

  if(!previewReachSegment(
         mbc, candidate.W_T_M_pre, true, false, terminalResult, true))
  {
    candidate.failureReason = "predictive_static/static_approach/"
                            + terminalResult.reason;
    candidate.minClearance = std::min(
        candidate.minClearance,
        std::min(reachResult.minClearance,
                 terminalResult.minClearance));
    restorePlanningWorld();
    return false;
  }
  terminalResult.reachCaptureDuration = terminalResult.duration - phaseStart;

  if(candidate.name == "axisP_side_337deg"
     && candidate.transitRouteName == "direct")
  {
    const sva::PTransformd previewCapturePose = previewMouthPose(mbc);
    const double positionError = (candidate.W_T_M_pre.translation()
        - previewCapturePose.translation()).norm();
    const double rotationError = orientationError(
        previewCapturePose, candidate.W_T_M_pre);
    HandoverSafetyReport configurationReport;
    const bool configurationSafe = previewConfigurationSafe(
        mbc, previewCapturePose, true, configurationReport);
    HandoverSafetyReport closureReport;
    const bool closureSafe = previewDynamicClosureSafety(mbc, closureReport);
    const bool positionReady = positionError <= previewPositionTolerance_;
    const bool orientationReady = rotationError <= previewOrientationTolerance_;
    const bool centeringReady = closureSafe
        && closureReport.padCenteringError <= padCenteringTolerance_;
    const bool entryReady = closureReport.corridorEntryClearance >= -1e-9;
    const bool palmReady = closureReport.corridorPalmClearance >= -1e-9;
    const bool axialReady = closureReport.corridorAxialClearance >= -1e-9;
    const bool axisReady = closureReport.corridorAngleClearance >= -1e-9;

    mc_rtc::log::info(
        "[R1TerminalGateAudit] source=preview candidate={} route={} positionReady={} orientationReady={} configurationSafe={} closureSafe={} centeringReady={} entryReady={} palmReady={} axialReady={} axisReady={} staticVelocityReady=true dwellRequested={:.3f}s posErr={:.4f}/{:.4f} oriErr={:.4f}/{:.4f} centerErr={:.4f}/{:.4f} corridor=[entry:{:.4f},palm:{:.4f},axial:{:.4f},axis:{:.4f}] configClear={:.4f} configLimiting={}/{} closureClear={:.4f} closureLimiting={}/{}",
        candidate.name, candidate.transitRouteName, positionReady,
        orientationReady, configurationSafe, closureSafe, centeringReady,
        entryReady, palmReady, axialReady, axisReady,
        timingTerminalCaptureDwell_, positionError, previewPositionTolerance_,
        rotationError, previewOrientationTolerance_,
        closureReport.padCenteringError, padCenteringTolerance_,
        closureReport.corridorEntryClearance,
        closureReport.corridorPalmClearance,
        closureReport.corridorAxialClearance,
        closureReport.corridorAngleClearance,
        configurationReport.minClearance, configurationReport.sample,
        configurationReport.obstacle, closureReport.minClearance,
        closureReport.sample, closureReport.obstacle);
  }

  if(!previewTerminalCaptureDwell(
         mbc, timingTerminalCaptureDwell_, terminalResult))
  {
    candidate.failureReason = "predictive_static/terminal_velocity_gate/"
                            + terminalResult.reason;
    candidate.minClearance = std::min(
        candidate.minClearance,
        std::min(reachResult.minClearance, terminalResult.minClearance));
    restorePlanningWorld();
    return false;
  }
  phaseStart = terminalResult.duration;
  candidate.plannedArmPosture = armPostureFromMbc(mbc);

  if(!previewClosureSweep(mbc, terminalResult))
  {
    candidate.failureReason = "predictive_static/static_acquire/"
                            + terminalResult.reason;
    candidate.minClearance = std::min(
        candidate.minClearance,
        std::min(reachResult.minClearance,
                 terminalResult.minClearance));
    restorePlanningWorld();
    return false;
  }
  terminalResult.closureDuration = terminalResult.duration - phaseStart;
  candidate.contactClosure = terminalResult.contactClosure;

  const sva::PTransformd M_T_O = relativePose(
      previewMouthPose(mbc), presentationAnchor);
  PreviewResult retreatResult;
  retreatResult.minClearance = std::numeric_limits<double>::infinity();
  int retreatIteration = 0;
  while(true)
  {
    const sva::PTransformd oldPlanning = planningM_T_O_;
    planningM_T_O_ = M_T_O;
    const PreviewStepStatus status = previewReachStep(
        mbc, candidate.W_T_M_retreat, false, true,
        retreatIteration, retreatResult, Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), true, nullptr, true);
    planningM_T_O_ = oldPlanning;
    if(status == PreviewStepStatus::Succeeded) { break; }
    if(status == PreviewStepStatus::Failed)
    {
      candidate.failureReason = "predictive_static/static_retreat/"
                              + retreatResult.reason;
      candidate.minClearance = std::min(
          candidate.minClearance,
          std::min(reachResult.minClearance,
                   std::min(terminalResult.minClearance,
                            retreatResult.minClearance)));
      restorePlanningWorld();
      return false;
    }
  }
  candidate.predictiveRetreatClearance = retreatResult.minClearance;
  candidate.plannedRetreatArmPosture = armPostureFromMbc(mbc);

  const double anchorTranslationError = (
      presentationAnchor.translation()
      - plan.objectAtPresentation.translation()).norm();
  const double anchorRotationError = orientationError(
      presentationAnchor, plan.objectAtPresentation);
  if(anchorTranslationError > 1e-12 || anchorRotationError > 1e-12)
  {
    candidate.failureReason = "predictive_static/presentation_anchor_mutated";
    restorePlanningWorld();
    return false;
  }

  candidate.previewFeasible = true;
  candidate.failureReason = "feasible_predictive_static_handover";
  candidate.minClearance = std::min(
      candidate.minClearance,
      std::min(reachResult.minClearance,
               std::min(terminalResult.minClearance,
                        retreatResult.minClearance)));
  candidate.rawRolloutTime = reachResult.duration + terminalResult.duration
                           + retreatResult.duration;
  candidate.predictedReachTime = plan.reachDuration;
  candidate.predictedPresentationTime = plan.reachDuration;
  candidate.predictedApproachTime = timingTerminalCaptureDwell_
      + timingArmScale_ * terminalResult.reachCaptureDuration;
  candidate.predictedAcquireTime = timingPriorityBlend_ + timingCaptureLock_
      + candidate.contactClosure
          / std::max(1e-6, timingEffectiveGripperRate_);
  candidate.predictedContactTime = candidate.predictedPresentationTime
                                 + candidate.predictedApproachTime
                                 + candidate.predictedAcquireTime;
  const double predictedRetreat = timingArmScale_ * retreatResult.duration;
  candidate.predictedArmTime = candidate.predictedReachTime
                             + candidate.predictedApproachTime
                             + predictedRetreat;
  candidate.predictedClosureTime = candidate.contactClosure
                                 / std::max(1e-6,
                                            timingEffectiveGripperRate_);
  candidate.predictedFixedTime = timingPriorityBlend_ + timingCaptureLock_
                               + timingBilateralDwell_
                               + timingConfirmationDwell_;
  candidate.estimatedTime = candidate.predictedContactTime
                          + timingBilateralDwell_
                          + timingConfirmationDwell_
                          + predictedRetreat;

  // Preserve the protected R1 prediction used by the legacy selector before
  // the accepted copied terminal duration is bound into reporting/commit
  // fields. This prevents a timing-model improvement from silently changing
  // the discrete candidate decision at this checkpoint.
  candidate.legacyPredictedApproachTime = candidate.predictedApproachTime;
  candidate.legacyPredictedContactTime = candidate.predictedContactTime;
  candidate.legacyEstimatedTime = candidate.estimatedTime;

  candidate.predictedEffort = reachResult.effort + terminalResult.effort
                            + retreatResult.effort;
  candidate.minimumJointMarginRatio = std::min(
      reachResult.minimumJointMarginRatio,
      std::min(terminalResult.minimumJointMarginRatio,
               retreatResult.minimumJointMarginRatio));
  candidate.minimumConditionIndex = std::min(
      reachResult.minimumConditionIndex,
      std::min(terminalResult.minimumConditionIndex,
               retreatResult.minimumConditionIndex));
  candidate.maximumJointVelocityUtilization = std::max(
      reachResult.maximumJointVelocityUtilization,
      std::max(terminalResult.maximumJointVelocityUtilization,
               retreatResult.maximumJointVelocityUtilization));

  // Run the validated copied terminal controller only after the complete
  // reach-insert-close-retreat route has already passed every hard check.
  // Its duration may update prediction fields, but never hard feasibility or
  // the protected R1 ranking at this checkpoint.
  candidate.terminalTimingAuditRan = true;
  const bool verboseTimingAudit = candidate.name == "axisP_side_337deg"
      && candidate.transitRouteName == "direct";
  candidate.terminalTimingAuditSuccess = previewVelocityGateParityShadow(
      timingAuditStartMbc, candidate,
      candidate.terminalTimingAuditDuration,
      candidate.terminalTimingFinalPositionError,
      candidate.terminalTimingFinalOrientationError,
      candidate.terminalTimingFinalLinearSpeed,
      candidate.terminalTimingFinalAngularSpeed,
      candidate.terminalTimingFinalStableTime,
      candidate.terminalTimingAuditReason, verboseTimingAudit);
  candidate.terminalVelocityUtilization = std::max(
      candidate.terminalTimingFinalLinearSpeed / 0.040,
      candidate.terminalTimingFinalAngularSpeed / 0.080);

  const double legacyApproach = candidate.legacyPredictedApproachTime;
  const double legacyContact = candidate.legacyPredictedContactTime;
  const double legacyExecution = candidate.legacyEstimatedTime;
  if(candidate.terminalTimingAuditSuccess)
  {
    candidate.terminalTimingPredictionBound = true;
    candidate.predictedApproachTime = candidate.terminalTimingAuditDuration;
    candidate.predictedContactTime = candidate.predictedPresentationTime
        + candidate.predictedApproachTime
        + candidate.predictedAcquireTime;
    candidate.predictedArmTime = candidate.predictedReachTime
        + candidate.predictedApproachTime + predictedRetreat;
    candidate.estimatedTime = candidate.predictedContactTime
        + timingBilateralDwell_ + timingConfirmationDwell_
        + predictedRetreat;

    // Keep the audit mirrors for backward-compatible diagnostics. From this
    // checkpoint onward they equal the accepted bound prediction fields.
    candidate.auditPredictedContactTime = candidate.predictedContactTime;
    candidate.auditEstimatedTime = candidate.estimatedTime;
  }

  mc_rtc::log::success(
      "[MeasuredTerminalPredictionBinding] candidate={} route={} applied={} legacyApproach={:.3f}s measuredApproach={:.3f}s boundApproach={:.3f}s contactDelta={:+.3f}s executionDelta={:+.3f}s hardFeasibilityUnchanged=true protectedHeuristicInputUnchanged=true bindingCostInputUpdated={} stateSequenceUnchanged=true",
      candidate.name, candidate.transitRouteName,
      candidate.terminalTimingPredictionBound, legacyApproach,
      candidate.terminalTimingAuditDuration,
      candidate.predictedApproachTime,
      candidate.predictedContactTime - legacyContact,
      candidate.estimatedTime - legacyExecution,
      completePlanSelectionMode_ == "binding_cost");

  computeCompletePlanAuditCost(candidate);
  mc_rtc::log::success(
      "[CompletePlanTimingAudit] candidate={} route={} success={} captureReadyDuration={:.3f}s legacyApproach={:.3f}s boundApproach={:.3f}s delta={:+.3f}s final=[dist:{:.4f},angle:{:.4f},v:{:.4f},w:{:.4f},stable:{:.3f}] reason={} completeRoute=true predictionBinding={} hardFeasibilityUnchanged=true selectionMode={} canAffectCommittedSelection={}",
      candidate.name, candidate.transitRouteName,
      candidate.terminalTimingAuditSuccess,
      candidate.terminalTimingAuditDuration, legacyApproach,
      candidate.predictedApproachTime,
      candidate.terminalTimingAuditDuration - legacyApproach,
      candidate.terminalTimingFinalPositionError,
      candidate.terminalTimingFinalOrientationError,
      candidate.terminalTimingFinalLinearSpeed,
      candidate.terminalTimingFinalAngularSpeed,
      candidate.terminalTimingFinalStableTime,
      candidate.terminalTimingAuditReason,
      candidate.terminalTimingPredictionBound,
      completePlanSelectionMode_,
      completePlanSelectionMode_ == "binding_cost");
  mc_rtc::log::success(
      "[CompletePlanCost] candidate={} route={} valid={} J={:.6f} weightsNormalized=true auditTime={:.3f}s effort={:.3f} path={:.3f}m rotation={:.3f} clearance=[reach:{:.4f},retreat:{:.4f}] jointMargin={:.4f} conditionIndex={:.4f} velocityUtil={:.4f} velocityLaw=rho4 finiteBoundary=true terms=[T:{:.4f},E:{:.4f},L:{:.4f},R:{:.4f},C:{:.4f},Q:{:.4f},K:{:.4f},V:{:.4f}] hardFeasibilityUnchanged=true selectionMode={} binding={}",
      candidate.name, candidate.transitRouteName,
      candidate.completeCostAuditValid, candidate.completeCostAudit,
      candidate.auditEstimatedTime, candidate.predictedEffort,
      candidate.transitPathLength, candidate.rotation,
      candidate.predictiveReachClearance,
      candidate.predictiveRetreatClearance,
      candidate.minimumJointMarginRatio,
      candidate.minimumConditionIndex,
      std::max(candidate.maximumJointVelocityUtilization,
               candidate.terminalVelocityUtilization),
      candidate.costTime, candidate.costEffort, candidate.costPath,
      candidate.costRotation, candidate.costClearance,
      candidate.costJointMargin, candidate.costConditioning,
      candidate.costVelocityReserve, completePlanSelectionMode_,
      completePlanSelectionMode_ == "binding_cost");

  if(verboseTimingAudit)
  {
    mc_rtc::log::success(
        "[VelocityGateTimingApproximationSummary] candidate={} route={} success={} duration={:.3f}s reason={} final=[dist:{:.4f},angle:{:.4f},v:{:.4f},w:{:.4f},stable:{:.3f}] runtimePolicyEquivalent=false hardFeasibilityUnchanged=true feedsBindingCost={}",
        candidate.name, candidate.transitRouteName,
        candidate.terminalTimingAuditSuccess,
        candidate.terminalTimingAuditDuration,
        candidate.terminalTimingAuditReason,
        candidate.terminalTimingFinalPositionError,
        candidate.terminalTimingFinalOrientationError,
        candidate.terminalTimingFinalLinearSpeed,
        candidate.terminalTimingFinalAngularSpeed,
        candidate.terminalTimingFinalStableTime,
        completePlanSelectionMode_ == "binding_cost");
    mc_rtc::log::success(
        "[VelocityGateTimingApproximationComparison] candidate={} route={} shadowSuccess={} shadowDuration={:.3f}s legacyDuration={:.3f}s delta={:+.3f}s runtimePolicyEquivalent=false",
        candidate.name, candidate.transitRouteName,
        candidate.terminalTimingAuditSuccess,
        candidate.terminalTimingAuditDuration, legacyApproach,
        candidate.terminalTimingAuditDuration - legacyApproach);
  }

  mc_rtc::log::success(
      "[PredictiveStaticPreview] candidate={} route={} certified runtimePolicyApproximation=true runtimePolicyEquivalent=false movingReach=true openInsertion=true fullAcquire=true retreat=true reach={:.3f}s legacyApproach={:.3f}s approach={:.3f}s auditApproach={:.3f}s acquire={:.3f}s contactAfterPresentation={:.3f}s auditContactAfterPresentation={:.3f}s closure={:.3f} reachError=[{:.4f},{:.4f}] reachClear={:.4f} requiredReachClear={:.4f} retreatClear={:.4f} terminalClear={:.4f} overallClear={:.4f} J={:.6f}",
      candidate.name, candidate.transitRouteName, candidate.predictedReachTime,
      candidate.legacyPredictedApproachTime,
      candidate.predictedApproachTime,
      candidate.terminalTimingAuditDuration,
      candidate.predictedAcquireTime,
      candidate.predictedContactTime - candidate.predictedPresentationTime,
      candidate.auditPredictedContactTime - candidate.predictedPresentationTime,
      candidate.contactClosure, reachPositionError,
      reachOrientationError, candidate.predictiveReachClearance,
      transitMinimumPredictedClearance_,
      candidate.predictiveRetreatClearance,
      terminalResult.minClearance, candidate.minClearance,
      candidate.completeCostAudit);

  restorePlanningWorld();
  return true;
}

bool HandoverInterceptionController::previewCandidate(
    CaptureCandidate & candidate,
    const sva::PTransformd & W_T_M_actual)
{
  rbd::MultiBodyConfig mbc = robot().mbc();
  for(auto & a : mbc.alpha) { std::fill(a.begin(), a.end(), 0.0); }
  for(auto & aD : mbc.alphaD) { std::fill(aD.begin(), aD.end(), 0.0); }
  setPreviewGripperClosure(mbc, 0.0);

  PreviewResult result;
  result.minClearance = std::numeric_limits<double>::infinity();
  candidate.rotation = orientationError(W_T_M_actual, candidate.W_T_M_standoff);
  double phaseStart = result.duration;

  if(!previewReachSegment(
         mbc, candidate.W_T_M_standoff, false, false, result))
  {
    candidate.previewFeasible = false;
    candidate.failureReason = result.reason;
    candidate.minClearance = result.minClearance;
    candidate.estimatedTime = result.duration;
    candidate.predictedEffort = result.effort;
    return false;
  }
  result.reachStandoffDuration = result.duration - phaseStart;
  phaseStart = result.duration;
  candidate.plannedStandoffArmPosture = armPostureFromMbc(mbc);

  if(!previewReachSegment(mbc, candidate.W_T_M_pre, true, false, result))
  {
    candidate.previewFeasible = false;
    candidate.failureReason = result.reason;
    candidate.minClearance = result.minClearance;
    candidate.estimatedTime = result.duration;
    candidate.predictedEffort = result.effort;
    return false;
  }
  result.reachCaptureDuration = result.duration - phaseStart;
  if(!previewTerminalCaptureDwell(
         mbc, timingTerminalCaptureDwell_, result))
  {
    candidate.previewFeasible = false;
    candidate.failureReason = result.reason;
    candidate.minClearance = result.minClearance;
    candidate.estimatedTime = result.duration;
    candidate.predictedEffort = result.effort;
    return false;
  }
  phaseStart = result.duration;
  if(!previewClosureSweep(mbc, result))
  {
    candidate.previewFeasible = false;
    candidate.failureReason = result.reason;
    candidate.minClearance = result.minClearance;
    candidate.estimatedTime = result.duration;
    candidate.predictedEffort = result.effort;
    return false;
  }

  result.closureDuration = result.duration - phaseStart;
  phaseStart = result.duration;
  candidate.plannedArmPosture = armPostureFromMbc(mbc);
  // The object is rigidly attached at the contact configuration for the
  // virtual retreat. mutable planner state is avoided here by evaluating the
  // same relative transform directly through a local full retreat loop.
  const sva::PTransformd M_T_O = relativePose(previewMouthPose(mbc), W_T_O_);
  int retreatIteration = 0;
  while(true)
  {
    const sva::PTransformd oldPlanning = planningM_T_O_;
    planningM_T_O_ = M_T_O;
    const PreviewStepStatus status = previewReachStep(
        mbc, candidate.W_T_M_retreat, false, true, retreatIteration, result);
    planningM_T_O_ = oldPlanning;
    if(status == PreviewStepStatus::Succeeded) { break; }
    if(status == PreviewStepStatus::Failed)
    {
      candidate.previewFeasible = false;
      candidate.failureReason = result.reason;
      candidate.minClearance = result.minClearance;
      candidate.estimatedTime = result.duration;
      candidate.predictedEffort = result.effort;
      return false;
    }
  }

  result.retreatDuration = result.duration - phaseStart;
  candidate.previewFeasible = true;
  candidate.failureReason = "feasible";
  candidate.minClearance = result.minClearance;
  candidate.rawRolloutTime = result.duration;
  candidate.estimatedTime = predictedExecutionTime(result);
  candidate.predictedReachTime = timingArmScale_
      * result.reachStandoffDuration;
  candidate.predictedApproachTime = timingArmScale_
      * result.reachCaptureDuration;
  candidate.predictedAcquireTime = timingPriorityBlend_ + timingCaptureLock_
      + (result.contactClosure > 0.0
         ? result.contactClosure / timingEffectiveGripperRate_ : 0.0);
  candidate.predictedArmTime = timingArmScale_ * (
      result.reachStandoffDuration + result.reachCaptureDuration
      + result.retreatDuration);
  candidate.predictedClosureTime = result.contactClosure > 0.0
      ? result.contactClosure / timingEffectiveGripperRate_ : 0.0;
  candidate.predictedFixedTime = timingPriorityBlend_ + timingCaptureLock_
                               + timingBilateralDwell_
                               + timingConfirmationDwell_;
  candidate.predictedEffort = result.effort;
  candidate.contactClosure = result.contactClosure;
  candidate.plannedRetreatArmPosture = armPostureFromMbc(mbc);
  candidate.score = candidate.estimatedTime
                  + previewEffortWeight_ * result.effort
                  + previewRotationWeight_ * candidate.rotation;
  return true;
}

// =============================================================================
// Candidate generation and selection
// =============================================================================

HandoverInterceptionController::CaptureCandidate
HandoverInterceptionController::buildCandidate(
    double angleRad,
    double axisSign,
    const sva::PTransformd & W_T_M_actual,
    const Eigen::Vector3d & baseOutward) const
{
  CaptureCandidate c;
  const Eigen::Vector3d pH = W_T_H_.translation();
  const Eigen::Vector3d zM = (axisSign >= 0.0 ? 1.0 : -1.0)
                            * blueHandleAxis();

  Eigen::Vector3d yM = Eigen::AngleAxisd(angleRad, zM) * baseOutward;
  yM -= zM * zM.dot(yM);
  if(yM.norm() < 1e-9) { yM = Eigen::Vector3d::UnitY(); }
  yM.normalize();

  Eigen::Vector3d xM = yM.cross(zM);
  if(xM.norm() < 1e-9) { xM = Eigen::Vector3d::UnitZ(); }
  xM.normalize();
  yM = zM.cross(xM).normalized();

  Eigen::Matrix3d R_W_M;
  R_W_M.col(0) = xM;
  R_W_M.col(1) = yM;
  R_W_M.col(2) = zM;

  // The controlled mouth frame is the midpoint of model-derived inner-pad
  // references. A small positive captureDepth keeps the blue handle slightly
  // in front of that reference (negative y_M), away from the palm.
  const Eigen::Vector3d pCapture = pH + yM * captureDepth_;
  const Eigen::Vector3d pStand = pCapture + yM * candidateStandoffDistance_;
  c.W_T_M_transit = fromWorldPose(R_W_M, pStand);
  c.W_T_M_standoff = fromWorldPose(R_W_M, pStand);
  c.W_T_M_pre = fromWorldPose(R_W_M, pCapture);
  // Retreat follows the same certified outward ray used for insertion. This is
  // candidate-relative rather than a world-direction preference.
  c.W_T_M_retreat = fromWorldPose(
      R_W_M, pCapture + yM * candidateRetreatDistance_);
  // Direction is diagnostic only. Above, below and lateral approaches are
  // all generated; the internal whole-arm rollout decides feasibility before
  // the physical robot moves.
  c.verticalComponent = clampUnit(yM.dot(worldUp_));

  int deg = static_cast<int>(std::lround(angleRad * 180.0 / PI));
  while(deg < 0) { deg += 360; }
  while(deg >= 360) { deg -= 360; }
  c.name = std::string(axisSign >= 0.0 ? "axisP_side_" : "axisN_side_")
         + std::to_string(deg) + "deg";
  return c;
}

HandoverInterceptionController::CapturePlanningStatus
HandoverInterceptionController::beginCapturePlanning(bool commitOnSuccess)
{
  detachObject();
  invalidateSelectedCandidate();
  capturePlanningCommitOnSuccess_ = commitOnSuccess;
  if(planningObjectSnapshotActive_) { applyPlanningObjectSnapshot(); }
  else { refreshObjectPose(); }
  planningM_T_O_ = sva::PTransformd::Identity();

  if(!mouthCalibrationValid_ || !gripperGeometryValid_)
  {
    mc_rtc::log::error(
        "[PlanCapture] mouth/gripper geometry was not calibrated; planning aborted safely");
    capturePlanningStatus_ = CapturePlanningStatus::Failure;
    return capturePlanningStatus_;
  }

  // Third frozen-decision-state consumer: in global-time-plan mode this
  // reference pose is otherwise resampled live once per hypothesis (this
  // function runs once per event hypothesis), which leaks the same
  // elapsed-time nuisance variable into candidate geometry itself, upstream
  // of the two already-frozen preview seed sites. previewMouthPose() is the
  // existing MBC-parameterized equivalent of actualMouthPose() (same frame
  // construction, parameterized on an explicit mbc instead of live robot()).
  planningStartMouthPose_ = globalTimePlanFrozenRobotStateValid_
      ? previewMouthPose(globalTimePlanFrozenRobotState_) : actualMouthPose();
  const Eigen::Vector3d pH = W_T_H_.translation();
  const Eigen::Vector3d zH = blueHandleAxis();

  // Candidate zero is the direct object-to-current-mouth outward direction.
  // It is robot-relative. The complete ring is still tested with identical
  // hard constraints, so this is not a directional preference or penalty.
  planningBaseOutward_ = planningStartMouthPose_.translation() - pH;
  planningBaseOutward_ -= zH * zH.dot(planningBaseOutward_);
  if(planningBaseOutward_.norm() < 1e-6)
  {
    planningBaseOutward_ = worldUp_ - zH * zH.dot(worldUp_);
  }
  if(planningBaseOutward_.norm() < 1e-6)
  {
    planningBaseOutward_ = Eigen::Vector3d::UnitY() - zH * zH.y();
  }
  if(planningBaseOutward_.norm() < 1e-6)
  {
    planningBaseOutward_ = Eigen::Vector3d::UnitX() - zH * zH.x();
  }
  planningBaseOutward_.normalize();

  planningCandidateIndex_ = 0;
  // The cylindrical handle axis is undirected. Both +axis and -axis tool
  // alignments are physically equivalent grasps but can belong to very
  // different wrist branches. Evaluate both internally before any motion.
  planningCandidateCount_ = 2 * std::max(4, candidateCount_);
  planningFoundFeasible_ = false;
  planningBestCandidate_ = CaptureCandidate{};
  planningCompletePlanAuditCandidates_.clear();
  planningCostSelectionValid_ = false;
  planningCostSelectionCommitAdmissible_ = false;
  planningGlobalSelectionActive_ = false;
  planningCostSelectionReason_ = "not_run";
  planningCompletePlanCount_ = 0;
  planningCostValidCount_ = 0;
  planningTimingAdmissibleCount_ = 0;
  planningMinimumAdmissibleCost_ = 1e9;
  planningSelectedObjectiveCost_ = 1e9;
  planningMbc_.reset();
  capturePlanningStatus_ = CapturePlanningStatus::Running;

  mc_rtc::log::warning(
      "[PlanCapture] beginning bounded internal rollout of {} complete whole-arm candidates; robot remains stationary until one plan is selected",
      planningCandidateCount_);

  if(!startNextPlanningCandidate())
  {
    return finalizeCapturePlanning();
  }
  return capturePlanningStatus_;
}

bool HandoverInterceptionController::startNextPlanningCandidate()
{
  if(planningCandidateIndex_ >= planningCandidateCount_) { return false; }

  const int ringCount = std::max(4, candidateCount_);
  const int axisIndex = planningCandidateIndex_ / ringCount;
  const int sideIndex = planningCandidateIndex_ % ringCount;
  const double axisSign = axisIndex == 0 ? 1.0 : -1.0;
  const double angle = 2.0 * PI * static_cast<double>(sideIndex)
                     / static_cast<double>(ringCount);
  planningCurrentCandidate_ = buildCandidate(
      angle, axisSign, planningStartMouthPose_, planningBaseOutward_);
  planningCurrentCandidate_.rotation = orientationError(
      planningStartMouthPose_, planningCurrentCandidate_.W_T_M_standoff);

  // In global-time-plan mode every candidate in this search must preview
  // from the same decision state, not from whatever live robot().mbc()
  // happens to exist when this candidate's turn arrives (which otherwise
  // depends on enumeration order/workload). See resetGlobalTimePlanSearch().
  planningMbc_ = std::make_unique<rbd::MultiBodyConfig>(
      globalTimePlanFrozenRobotStateValid_
          ? globalTimePlanFrozenRobotState_ : robot().mbc());
  for(auto & a : planningMbc_->alpha) { std::fill(a.begin(), a.end(), 0.0); }
  for(auto & aD : planningMbc_->alphaD) { std::fill(aD.begin(), aD.end(), 0.0); }
  setPreviewGripperClosure(*planningMbc_, 0.0);

  planningResult_ = PreviewResult{};
  planningResult_.minClearance = std::numeric_limits<double>::infinity();
  planningPhaseStartDuration_ = 0.0;
  planningPhase_ = PlanningPhase::ReachStandoff;
  planningSegmentIteration_ = 0;
  planningClosureIndex_ = 0;

  mc_rtc::log::info(
      "[PlanCapture] previewing {} ({}/{}) entirely in controller memory",
      planningCurrentCandidate_.name, planningCandidateIndex_ + 1,
      planningCandidateCount_);
  return true;
}

double HandoverInterceptionController::predictedExecutionTime(
    const PreviewResult & result) const
{
  const double armRaw = result.reachStandoffDuration
                      + result.reachCaptureDuration
                      + result.retreatDuration;
  const double arm = timingArmScale_ * std::max(0.0, armRaw);
  const double closure = std::max(0.0, result.contactClosure)
                       / timingEffectiveGripperRate_;
  const double fixed = timingPriorityBlend_ + timingCaptureLock_
                     + timingBilateralDwell_ + timingConfirmationDwell_;
  return arm + closure + fixed;
}

double HandoverInterceptionController::normalizedUpperBarrier(
    double value, double hard, double soft) const
{
  if(!std::isfinite(value)) { return 1e6; }
  soft = std::max(hard + 1e-9, soft);
  if(value >= soft) { return 0.0; }
  if(value <= hard) { return 1e6; }
  const double reserve = (value - hard) / (soft - hard);
  return -std::log(std::max(1e-12, reserve));
}

double HandoverInterceptionController::normalizedLowerBarrier(
    double value, double soft) const
{
  if(!std::isfinite(value)) { return 1e6; }
  soft = std::max(1e-9, soft);
  if(value >= soft) { return 0.0; }
  if(value <= 0.0) { return 1e6; }
  return -std::log(std::max(1e-12, value / soft));
}

void HandoverInterceptionController::computeCompletePlanAuditCost(
    CaptureCandidate & candidate) const
{
  candidate.completeCostAuditValid = candidate.previewFeasible
      && candidate.terminalTimingAuditSuccess
      && std::isfinite(candidate.auditEstimatedTime)
      && std::isfinite(candidate.predictedEffort)
      && std::isfinite(candidate.transitPathLength)
      && std::isfinite(candidate.predictiveReachClearance)
      && std::isfinite(candidate.predictiveRetreatClearance)
      && std::isfinite(candidate.minimumJointMarginRatio)
      && std::isfinite(candidate.minimumConditionIndex)
      && std::isfinite(candidate.maximumJointVelocityUtilization)
      && std::isfinite(candidate.terminalVelocityUtilization);
  if(!candidate.completeCostAuditValid)
  {
    candidate.completeCostAudit = 1e9;
    return;
  }

  candidate.costTime = std::max(0.0, candidate.auditEstimatedTime)
                     / decisionTimeReference_;
  candidate.costEffort = std::max(0.0, candidate.predictedEffort)
                       / decisionEffortReference_;
  candidate.costPath = std::max(0.0, candidate.transitPathLength)
                     / decisionPathReference_;
  const double normalizedRotation = std::max(0.0, candidate.rotation) / PI;
  candidate.costRotation = normalizedRotation * normalizedRotation;

  // Contact pairs are intentionally absent from this soft reserve. The cost
  // uses the minimum non-contact reserve over predictive reach and attached
  // retreat, while terminal bilateral contact remains a hard certified event.
  const double operationalClearance = std::min(
      candidate.predictiveReachClearance,
      candidate.predictiveRetreatClearance);
  candidate.costClearance = normalizedUpperBarrier(
      operationalClearance,
      transitMinimumPredictedClearance_, decisionSoftClearance_);
  candidate.costJointMargin = normalizedLowerBarrier(
      candidate.minimumJointMarginRatio, decisionSoftJointMargin_);
  candidate.costConditioning = normalizedLowerBarrier(
      candidate.minimumConditionIndex, decisionSoftConditionIndex_);

  // Hard joint-velocity compliance has already been certified by the copied
  // rollout. This is therefore a soft preference only: keep it finite at the
  // admissible boundary instead of recreating a hidden hard constraint.
  // Frozen seven-term objective: V uses joint-velocity utilization only.
  // terminalVelocityUtilization is deliberately excluded from this soft
  // preference term (validated: it never dominated the previous combined
  // max() across the full diagnostic campaign, including a dedicated
  // terminal-settling stress test); it remains computed above and continues
  // to gate terminal-timing-audit success/failure and execution checks
  // exactly as before, unaffected by this exclusion.
  const double velocityUtilization = clamp01(
      candidate.maximumJointVelocityUtilization);
  const double velocityUtilizationSquared = velocityUtilization
                                           * velocityUtilization;
  candidate.costVelocityReserve = velocityUtilizationSquared
                                * velocityUtilizationSquared;

  // Frozen seven-term binding preference objective (T,E,L,C,Q,K,V). R
  // (candidate.costRotation, computed above) is retained as a diagnostic
  // quantity only and is deliberately excluded from this binding sum: it
  // never independently changed the selected winner across the full
  // ablation/robustness campaign, so it no longer contributes to candidate
  // ranking, the global argmin, or winner selection. decisionRotationWeight_
  // is retained in configuration for provenance/diagnostics only and is
  // intentionally never referenced below.
  candidate.completeCostAudit =
        decisionTimeWeight_ * candidate.costTime
      + decisionEffortWeight_ * candidate.costEffort
      + decisionPathWeight_ * candidate.costPath
      + decisionClearanceWeight_ * candidate.costClearance
      + decisionJointMarginWeight_ * candidate.costJointMargin
      + decisionConditioningWeight_ * candidate.costConditioning
      + decisionVelocityReserveWeight_ * candidate.costVelocityReserve;
}

double HandoverInterceptionController::predictedContactTime(
    const PreviewResult & result) const
{
  // First bilateral pad contact is the interception event. Confirmation and
  // retreat happen after this event and therefore must not shift the predicted
  // meeting time.
  const double armRaw = result.reachStandoffDuration
                      + result.reachCaptureDuration;
  const double arm = timingArmScale_ * std::max(0.0, armRaw);
  const double closure = std::max(0.0, result.contactClosure)
                       / timingEffectiveGripperRate_;
  return arm + closure + timingPriorityBlend_ + timingCaptureLock_;
}

void HandoverInterceptionController::finishCurrentPlanningCandidate(bool feasible)
{
  CaptureCandidate & c = planningCurrentCandidate_;
  c.previewFeasible = feasible;
  c.failureReason = feasible ? "feasible" : planningResult_.reason;
  c.minClearance = planningResult_.minClearance;
  c.rawRolloutTime = planningResult_.duration;
  c.estimatedTime = feasible ? predictedExecutionTime(planningResult_)
                             : planningResult_.duration;
  c.predictedContactTime = feasible ? predictedContactTime(planningResult_)
                                    : planningResult_.duration;
  c.predictedReachTime = timingArmScale_
      * planningResult_.reachStandoffDuration;
  c.predictedPresentationTime = c.predictedReachTime;
  c.predictedApproachTime = timingArmScale_
      * planningResult_.reachCaptureDuration;
  c.predictedAcquireTime = timingPriorityBlend_ + timingCaptureLock_
      + (planningResult_.contactClosure > 0.0
         ? planningResult_.contactClosure / timingEffectiveGripperRate_ : 0.0);
  c.predictedArmTime = timingArmScale_ * (
      planningResult_.reachStandoffDuration
      + planningResult_.reachCaptureDuration
      + planningResult_.retreatDuration);
  c.predictedClosureTime = planningResult_.contactClosure > 0.0
      ? planningResult_.contactClosure / timingEffectiveGripperRate_ : 0.0;
  c.predictedFixedTime = timingPriorityBlend_ + timingCaptureLock_
                       + timingBilateralDwell_ + timingConfirmationDwell_;
  c.predictedEffort = planningResult_.effort;
  c.contactClosure = planningResult_.contactClosure;

  const bool movingVerificationRequested =
      previewMovingInterception_ && planningObjectSnapshotActive_
      && objectMotionEstimateValid_;
  if(feasible && movingVerificationRequested)
  {
    feasible = verifyPredictiveStaticCandidate(
        c, planningResult_, W_T_O_);
    c.previewFeasible = feasible;
    if(!feasible && c.failureReason == "feasible")
    {
      c.failureReason = "methodology_lock/unknown";
    }
  }

  if(feasible && planningMbc_ && !movingVerificationRequested)
  {
    c.plannedRetreatArmPosture = armPostureFromMbc(*planningMbc_);
  }

  if(feasible)
  {
    const double protectedSelectionTime =
        c.terminalTimingPredictionBound
        && std::isfinite(c.legacyEstimatedTime)
        ? c.legacyEstimatedTime : c.estimatedTime;
    c.score = protectedSelectionTime
            + previewEffortWeight_ * c.predictedEffort
            + previewRotationWeight_ * c.rotation;
    mc_rtc::log::info(
        "[PlanCapture] {} route={} feasible reach+insert+close+retreat predictedTime={:.3f}s selectionTime={:.3f}s predictedContact={:.3f}s rawRollout={:.3f}s arm={:.3f}s closure={:.3f}s fixed={:.3f}s effort={:.3f} rot={:.3f} reachClear={:.4f} overallClear={:.4f} contactClosure={:.3f} cost={:.4f} vertical={:.3f}",
        c.name, c.transitRouteName, c.estimatedTime,
        protectedSelectionTime, c.predictedContactTime, c.rawRolloutTime,
        c.predictedArmTime,
        c.predictedClosureTime, c.predictedFixedTime, c.predictedEffort,
        c.rotation, c.predictiveReachClearance, c.minClearance,
        c.contactClosure, c.score, c.verticalComponent);

    bool choose = !planningFoundFeasible_;
    if(planningFoundFeasible_)
    {
      const double reachClearanceGain = c.predictiveReachClearance
          - planningBestCandidate_.predictiveReachClearance;
      const bool clearlySafer = reachClearanceGain
          > transitClearancePreferenceBand_;
      const bool comparableSafety = std::abs(reachClearanceGain)
          <= transitClearancePreferenceBand_;
      choose = clearlySafer
          || (comparableSafety && c.score < planningBestCandidate_.score);
    }
    if(choose)
    {
      planningFoundFeasible_ = true;
      planningBestCandidate_ = c;
    }
  }
  else
  {
    mc_rtc::log::info(
        "[PlanCapture] {} infeasible reason={} elapsedPreview={:.3f}s clear={:.4f} vertical={:.3f}",
        c.name, c.failureReason, c.estimatedTime,
        c.minClearance, c.verticalComponent);
  }

  ++planningCandidateIndex_;
  planningMbc_.reset();
}

bool HandoverInterceptionController::selectPlanningBestForCommit(
    double remainingToPresentation,
    double minimumReachEntryLead,
    double minimumSafeCommitLead)
{
  planningCostSelectionValid_ = false;
  planningCostSelectionCommitAdmissible_ = false;
  planningGlobalSelectionActive_ = false;
  planningCostSelectionReason_ = "not_run";
  planningCompletePlanCount_ = planningCompletePlanAuditCandidates_.size();
  planningCostValidCount_ = 0;
  planningTimingAdmissibleCount_ = 0;
  planningMinimumAdmissibleCost_ = 1e9;
  planningSelectedObjectiveCost_ = 1e9;
  selectedGlobalMotionCost_ = 1e9;
  selectedGlobalScheduleWait_ = 0.0;

  if(!planningFoundFeasible_)
  {
    planningCostSelectionReason_ = "no_geometrically_feasible_plan";
    mc_rtc::log::error(
        "[PlanSelection] mode={} success=false reason={} no plan may be committed",
        completePlanSelectionMode_, planningCostSelectionReason_);
    return false;
  }

  if(completePlanSelectionMode_ == "protected_heuristic")
  {
    planningCostSelectionValid_ = true;
    planningCostSelectionCommitAdmissible_ = true;
    planningCostSelectionReason_ = "protected_heuristic_selected";
    planningSelectedObjectiveCost_ = planningBestCandidate_.completeCostAudit;
    mc_rtc::log::warning(
        "[PlanSelection] mode=protected_heuristic candidate={} route={} costBinding=false historicalSelectorPreserved=true",
        planningBestCandidate_.name,
        planningBestCandidate_.transitRouteName);
    return true;
  }

  if(completePlanSelectionMode_ != "binding_cost"
     || !decisionCostConfigurationValid_)
  {
    planningCostSelectionReason_ = "invalid_binding_cost_configuration";
    mc_rtc::log::error(
        "[BindingCostSelection] success=false reason={} mode={} costConfigurationValid={} no fallback permitted",
        planningCostSelectionReason_, completePlanSelectionMode_,
        decisionCostConfigurationValid_);
    return false;
  }

  if(physicalGripperBridgeEnabled_
     && !decisionBindingPhysicalExecutionAuthorized_)
  {
    planningCostSelectionReason_ = "physical_binding_execution_not_authorized";
    mc_rtc::log::error(
        "[BindingCostSelection] success=false reason={} physicalBridgeEnabled=true physicalCommandEnabled={} allowPhysicalExecution=false no fallback permitted",
        planningCostSelectionReason_, physicalGripperCommandEnabled_);
    return false;
  }

  std::vector<call_handover::FinitePlanRecord> records;
  records.reserve(planningCompletePlanAuditCandidates_.size());
  for(std::size_t i = 0;
      i < planningCompletePlanAuditCandidates_.size(); ++i)
  {
    const auto & candidate = planningCompletePlanAuditCandidates_[i];
    call_handover::FinitePlanRecord record;
    record.sourceIndex = i;
    record.costValid = candidate.completeCostAuditValid;
    record.cost = candidate.completeCostAudit;
    record.predictedPresentationTime = candidate.predictedPresentationTime;
    record.clearance = candidate.predictiveReachClearance;
    record.candidateName = candidate.name;
    record.routeName = candidate.transitRouteName;
    records.push_back(record);
  }

  const bool enforceTiming = remainingToPresentation >= 0.0;
  const auto selection = call_handover::selectFinitePlan(
      records, decisionCostTieTolerance_, enforceTiming,
      remainingToPresentation, minimumReachEntryLead,
      minimumSafeCommitLead);
  planningCompletePlanCount_ = selection.completePlanCount;
  planningCostValidCount_ = selection.costValidCount;
  planningTimingAdmissibleCount_ = selection.timingAdmissibleCount;
  planningMinimumAdmissibleCost_ = selection.minimumAdmissibleCost;
  planningCostSelectionReason_ = selection.reason;

  if(!selection.success
     || selection.selectedRecord >= planningCompletePlanAuditCandidates_.size())
  {
    mc_rtc::log::error(
        "[BindingCostSelection] success=false reason={} completePlans={} costValidPlans={} timingAdmissiblePlans={} no fallback permitted",
        planningCostSelectionReason_, planningCompletePlanCount_,
        planningCostValidCount_, planningTimingAdmissibleCount_);
    return false;
  }

  planningBestCandidate_ =
      planningCompletePlanAuditCandidates_[selection.selectedRecord];
  planningSelectedObjectiveCost_ = planningBestCandidate_.completeCostAudit;
  selectedGlobalMotionCost_ = planningBestCandidate_.completeCostAudit;
  planningCostSelectionValid_ = true;
  planningCostSelectionCommitAdmissible_ = selection.commitAdmissible;

  if(selection.commitAdmissible)
  {
    const bool selectedEqualsMinimum =
        std::isfinite(planningMinimumAdmissibleCost_)
        && std::abs(planningSelectedObjectiveCost_
                    - planningMinimumAdmissibleCost_)
               <= decisionCostTieTolerance_;
    mc_rtc::log::success(
        "[BindingCostSelection] success=true commitAdmissible=true completePlans={} costValidPlans={} timingAdmissiblePlans={} candidate={} route={} selectedJ={:.9f} minimumAdmissibleJ={:.9f} selectedWithinMinimumTolerance={} tieTolerance={:.3e} tieBreak=[reachTime,clearance,candidate,route,index]",
        planningCompletePlanCount_, planningCostValidCount_,
        planningTimingAdmissibleCount_, planningBestCandidate_.name,
        planningBestCandidate_.transitRouteName,
        planningBestCandidate_.completeCostAudit,
        planningMinimumAdmissibleCost_, selectedEqualsMinimum,
        decisionCostTieTolerance_);
    if(!selectedEqualsMinimum)
    {
      planningCostSelectionValid_ = false;
      planningCostSelectionCommitAdmissible_ = false;
      planningCostSelectionReason_ = "selected_cost_does_not_equal_minimum";
      return false;
    }
  }
  else
  {
    mc_rtc::log::warning(
        "[BindingCostSelection] success=true commitAdmissible=false reason={} completePlans={} costValidPlans={} timingAdmissiblePlans=0 timingReferenceCandidate={} route={} predictedReach={:.3f}s; candidate may drive event-time refinement but cannot be committed",
        planningCostSelectionReason_, planningCompletePlanCount_,
        planningCostValidCount_, planningBestCandidate_.name,
        planningBestCandidate_.transitRouteName,
        planningBestCandidate_.predictedPresentationTime);
  }
  return true;
}

void HandoverInterceptionController::resetGlobalTimePlanSearch(
    double searchEpoch, bool globalTimePlanModeActive)
{
  globalEventPlanAlternatives_.clear();
  globalTimePlanSearchEpoch_ = searchEpoch;
  if(globalTimePlanModeActive)
  {
    globalTimePlanFrozenRobotState_ = robot().mbc();
    globalTimePlanFrozenRobotStateValid_ = true;
  }
  else
  {
    globalTimePlanFrozenRobotStateValid_ = false;
  }
  planningGlobalSelectionActive_ = false;
  planningSelectedObjectiveCost_ = 1e9;
  selectedGlobalMotionCost_ = 1e9;
  selectedGlobalScheduleWait_ = 0.0;
  selectedGlobalEventLead_ = 0.0;
  selectedGlobalEventPresentationTime_ = 0.0;
  selectedGlobalMinimumReachEntryLead_ = 0.0;
  selectedGlobalMinimumSafeCommitLead_ = 0.0;
  selectedGlobalHypothesisIndex_ = 0;
  globalEvaluatedHypotheses_ = 0;
  globalConfiguredHypotheses_ = 0;
  globalScheduleComplete_ = false;
  selectedGlobalObjectPresentationPose_ = sva::PTransformd::Identity();
  selectedGlobalPlanningStartMouthPose_ = sva::PTransformd::Identity();
}

bool HandoverInterceptionController::captureCurrentEventPlanAlternatives(
    std::size_t hypothesisIndex,
    double eventLeadFromSearchEpoch,
    double eventPresentationTime,
    const sva::PTransformd & W_T_O_presentation)
{
  if(!globalTimePlanSelectionEnabled()
     || completePlanSelectionMode_ != "binding_cost"
     || !decisionCostConfigurationValid_)
  {
    planningCostSelectionReason_ = "invalid_global_time_plan_configuration";
    mc_rtc::log::error(
        "[GlobalTimePlanCapture] success=false reason={} mode={} eventSelectionMode={} no fallback permitted",
        planningCostSelectionReason_, completePlanSelectionMode_,
        completeEventSelectionMode_);
    return false;
  }
  if(!planningFoundFeasible_
     || planningCompletePlanAuditCandidates_.empty())
  {
    planningCostSelectionReason_ = "no_complete_plans_at_event";
    return false;
  }
  if(!std::isfinite(globalTimePlanSearchEpoch_)
     || !std::isfinite(eventLeadFromSearchEpoch)
     || !std::isfinite(eventPresentationTime)
     || std::abs((eventPresentationTime - globalTimePlanSearchEpoch_)
                 - eventLeadFromSearchEpoch) > 1e-6)
  {
    planningCostSelectionReason_ = "inconsistent_global_event_epoch";
    mc_rtc::log::error(
        "[GlobalTimePlanCapture] success=false reason={} hypothesis={} eventLead={:.9f} presentationTime={:.9f} epoch={:.9f}",
        planningCostSelectionReason_, hypothesisIndex,
        eventLeadFromSearchEpoch, eventPresentationTime,
        globalTimePlanSearchEpoch_);
    return false;
  }

  const std::size_t before = globalEventPlanAlternatives_.size();
  std::size_t costInvalidCount = 0;
  for(const auto & candidate : planningCompletePlanAuditCandidates_)
  {
    GlobalEventPlanAlternative alternative;
    alternative.candidate = candidate;
    alternative.hypothesisIndex = hypothesisIndex;
    alternative.eventLeadFromSearchEpoch = eventLeadFromSearchEpoch;
    alternative.eventPresentationTime = eventPresentationTime;
    alternative.scheduleWaitBeforeReach = eventLeadFromSearchEpoch
        - candidate.predictedPresentationTime;
    alternative.predictedSearchToCompletionTime =
        alternative.scheduleWaitBeforeReach + candidate.auditEstimatedTime;
    alternative.globalObjectiveCost =
        call_handover::extendMotionCostToSearchEpoch(
            candidate.completeCostAudit, decisionTimeWeight_,
            decisionTimeReference_, eventLeadFromSearchEpoch,
            candidate.predictedPresentationTime);
    alternative.planningStartMouthPose = planningStartMouthPose_;
    alternative.W_T_O_presentation = W_T_O_presentation;

    const bool valid = candidate.completeCostAuditValid
        && std::isfinite(alternative.globalObjectiveCost)
        && std::isfinite(alternative.predictedSearchToCompletionTime);
    mc_rtc::log::success(
        "[GlobalPlanCost] hypothesis={} eventLead={:.3f}s presentationTime={:.3f}s candidate={} route={} valid={} motionJ={:.9f} scheduleWait={:+.3f}s searchToCompletion={:.3f}s globalJ={:.9f} timeTerm=search_to_completion hardFeasibilityUnchanged=true",
        hypothesisIndex, eventLeadFromSearchEpoch, eventPresentationTime,
        candidate.name, candidate.transitRouteName, valid,
        candidate.completeCostAudit,
        alternative.scheduleWaitBeforeReach,
        alternative.predictedSearchToCompletionTime,
        alternative.globalObjectiveCost);

    // Exclude only this candidate. A cost-invalid sibling must never remove
    // an otherwise hard-feasible, cost-valid candidate from the global pool.
    if(!valid)
    {
      ++costInvalidCount;
      continue;
    }
    globalEventPlanAlternatives_.push_back(alternative);
  }

  const std::size_t completePlansAdded =
      globalEventPlanAlternatives_.size() - before;
  if(completePlansAdded == 0)
  {
    // Distinguish this from a genuine geometry/IK rejection: every candidate
    // at this hypothesis was a hard-feasible complete plan, but none had a
    // valid complete-plan cost. The search still continues to the next
    // bounded hypothesis exactly as it does after a geometry rejection.
    mc_rtc::log::warning(
        "[GlobalTimePlanCostAuditReject] hypothesis={} eventLead={:.3f}s completePlans={} costInvalidPlans={} reason=all_candidates_cost_invalid deferredCommit=true; searching another future event",
        hypothesisIndex, eventLeadFromSearchEpoch,
        planningCompletePlanAuditCandidates_.size(), costInvalidCount);
  }

  mc_rtc::log::success(
      "[GlobalTimePlanCapture] success=true hypothesis={} eventLead={:.3f}s completePlansAdded={} cumulativeCompletePlans={} deferredCommit=true robotMotion=false",
      hypothesisIndex, eventLeadFromSearchEpoch, completePlansAdded,
      globalEventPlanAlternatives_.size());
  return true;
}

bool HandoverInterceptionController::selectGlobalTimePlanForCommit(
    double now,
    double minimumReachEntryLead,
    double minimumSafeCommitLead,
    std::size_t evaluatedHypotheses,
    std::size_t configuredHypotheses,
    bool scheduleComplete)
{
  planningCostSelectionValid_ = false;
  planningCostSelectionCommitAdmissible_ = false;
  planningGlobalSelectionActive_ = false;
  planningCostSelectionReason_ = "not_run";
  planningSelectedObjectiveCost_ = 1e9;
  planningMinimumAdmissibleCost_ = 1e9;
  globalEvaluatedHypotheses_ = evaluatedHypotheses;
  globalConfiguredHypotheses_ = configuredHypotheses;
  globalScheduleComplete_ = scheduleComplete;

  if(!globalTimePlanSelectionEnabled()
     || completePlanSelectionMode_ != "binding_cost"
     || !decisionCostConfigurationValid_)
  {
    planningCostSelectionReason_ = "invalid_global_time_plan_configuration";
    mc_rtc::log::error(
        "[GlobalTimePlanSelection] success=false reason={} no fallback permitted",
        planningCostSelectionReason_);
    return false;
  }
  if(physicalGripperBridgeEnabled_
     && !decisionBindingPhysicalExecutionAuthorized_)
  {
    planningCostSelectionReason_ = "physical_global_execution_not_authorized";
    mc_rtc::log::error(
        "[GlobalTimePlanSelection] success=false reason={} physicalBridgeEnabled=true allowPhysicalExecution=false no fallback permitted",
        planningCostSelectionReason_);
    return false;
  }
  if(!scheduleComplete || configuredHypotheses == 0
     || evaluatedHypotheses != configuredHypotheses)
  {
    planningCostSelectionReason_ = "incomplete_bounded_event_schedule";
    mc_rtc::log::error(
        "[GlobalTimePlanSelection] success=false reason={} scheduleComplete={} evaluatedHypotheses={} configuredHypotheses={} no fallback permitted",
        planningCostSelectionReason_, scheduleComplete,
        evaluatedHypotheses, configuredHypotheses);
    return false;
  }

  std::vector<call_handover::FiniteEventPlanRecord> records;
  records.reserve(globalEventPlanAlternatives_.size());
  for(std::size_t i = 0; i < globalEventPlanAlternatives_.size(); ++i)
  {
    const auto & alternative = globalEventPlanAlternatives_[i];
    call_handover::FiniteEventPlanRecord record;
    record.sourceIndex = i;
    record.hypothesisIndex = alternative.hypothesisIndex;
    record.costValid = alternative.candidate.completeCostAuditValid;
    record.motionCost = alternative.candidate.completeCostAudit;
    record.globalCost = alternative.globalObjectiveCost;
    record.eventLead = alternative.eventLeadFromSearchEpoch;
    record.eventPresentationTime = alternative.eventPresentationTime;
    record.predictedPresentationDuration =
        alternative.candidate.predictedPresentationTime;
    record.predictedExecutionDuration =
        alternative.candidate.auditEstimatedTime;
    record.clearance = alternative.candidate.predictiveReachClearance;
    record.candidateName = alternative.candidate.name;
    record.routeName = alternative.candidate.transitRouteName;
    records.push_back(record);
  }

  // The timing diagnostics below are captured by selectFiniteEventPlan()
  // itself, during its own admissibility evaluation, not recomputed here.
  // This is solely so the final admissible set F_J ∩ F_timing and its
  // argmin can be reconstructed independently from the log.
  std::vector<call_handover::FiniteEventPlanTimingDiagnostic>
      timingDiagnostics;
  const auto selection = call_handover::selectFiniteEventPlan(
      records, now, minimumReachEntryLead, minimumSafeCommitLead,
      decisionCostTieTolerance_, &timingDiagnostics);
  for(const auto & diagnostic : timingDiagnostics)
  {
    mc_rtc::log::info(
        "[GlobalPlanTimingAdmissibility] hypothesis={} candidate={} route={} costValid={} globalJ={:.9f} eventPresentationTime={:.6f}s now={:.6f}s remaining={:.6f}s minimumSafeCommitLead={:.6f}s predictedPresentationDuration={:.6f}s minimumReachEntryLead={:.6f}s eventWindowAdmissible={} timingAdmissible={}",
        diagnostic.hypothesisIndex, diagnostic.candidateName,
        diagnostic.routeName, diagnostic.costValid, diagnostic.globalCost,
        now + diagnostic.remaining, now, diagnostic.remaining,
        diagnostic.minimumSafeCommitLead,
        diagnostic.predictedPresentationDuration,
        diagnostic.minimumReachEntryLead, diagnostic.eventWindowAdmissible,
        diagnostic.timingAdmissible);
  }
  planningCompletePlanCount_ = selection.completePlanCount;
  planningCostValidCount_ = selection.costValidCount;
  planningTimingAdmissibleCount_ = selection.timingAdmissibleCount;
  planningMinimumAdmissibleCost_ =
      selection.minimumAdmissibleGlobalCost;
  planningCostSelectionReason_ = selection.reason;

  if(!selection.success
     || selection.selectedRecord >= globalEventPlanAlternatives_.size())
  {
    mc_rtc::log::error(
        "[GlobalTimePlanSelection] success=false reason={} scheduleComplete=true evaluatedHypotheses={} configuredHypotheses={} completePlans={} costValidPlans={} timingAdmissiblePlans={} no fallback permitted",
        planningCostSelectionReason_, evaluatedHypotheses,
        configuredHypotheses, planningCompletePlanCount_,
        planningCostValidCount_, planningTimingAdmissibleCount_);
    return false;
  }

  const auto & selected =
      globalEventPlanAlternatives_[selection.selectedRecord];
  planningBestCandidate_ = selected.candidate;
  planningFoundFeasible_ = true;
  planningSelectedObjectiveCost_ = selected.globalObjectiveCost;
  selectedGlobalMotionCost_ = selected.candidate.completeCostAudit;
  selectedGlobalScheduleWait_ = selected.scheduleWaitBeforeReach;
  selectedGlobalEventLead_ = selected.eventLeadFromSearchEpoch;
  selectedGlobalEventPresentationTime_ = selected.eventPresentationTime;
  selectedGlobalMinimumReachEntryLead_ = minimumReachEntryLead;
  selectedGlobalMinimumSafeCommitLead_ = minimumSafeCommitLead;
  selectedGlobalHypothesisIndex_ = selected.hypothesisIndex;
  selectedGlobalObjectPresentationPose_ = selected.W_T_O_presentation;
  selectedGlobalPlanningStartMouthPose_ = selected.planningStartMouthPose;
  planningCostSelectionValid_ = true;
  planningCostSelectionCommitAdmissible_ = selection.commitAdmissible;
  planningGlobalSelectionActive_ = true;

  const bool selectedEqualsMinimum =
      std::isfinite(planningSelectedObjectiveCost_)
      && std::isfinite(planningMinimumAdmissibleCost_)
      && std::abs(planningSelectedObjectiveCost_
                  - planningMinimumAdmissibleCost_)
             <= decisionCostTieTolerance_;
  if(!selectedEqualsMinimum)
  {
    planningCostSelectionValid_ = false;
    planningCostSelectionCommitAdmissible_ = false;
    planningGlobalSelectionActive_ = false;
    planningCostSelectionReason_ =
        "selected_global_cost_does_not_equal_minimum";
    return false;
  }

  mc_rtc::log::success(
      "[GlobalTimePlanSelection] success=true scheduleComplete=true evaluatedHypotheses={} configuredHypotheses={} completePlans={} costValidPlans={} timingAdmissiblePlans={} hypothesis={} eventLead={:.3f}s presentationTime={:.3f}s candidate={} route={} motionJ={:.9f} scheduleWait={:+.3f}s selectedGlobalJ={:.9f} minimumGlobalJ={:.9f} selectedWithinMinimumTolerance=true tieTolerance={:.3e} tieBreak=[completion,event,candidateReach,clearance,candidate,route,hypothesis,index]",
      evaluatedHypotheses, configuredHypotheses,
      planningCompletePlanCount_, planningCostValidCount_,
      planningTimingAdmissibleCount_, selected.hypothesisIndex,
      selected.eventLeadFromSearchEpoch, selected.eventPresentationTime,
      selected.candidate.name, selected.candidate.transitRouteName,
      selected.candidate.completeCostAudit,
      selected.scheduleWaitBeforeReach,
      selected.globalObjectiveCost, planningMinimumAdmissibleCost_,
      decisionCostTieTolerance_);
  return true;
}

bool HandoverInterceptionController::commitGlobalTimePlanSelection()
{
  if(!planningGlobalSelectionActive_ || !planningCostSelectionValid_
     || !planningCostSelectionCommitAdmissible_)
  {
    mc_rtc::log::error(
        "[GlobalTimePlanCommitProof] committed=false reason=global_selection_not_admissible detail={} no fallback permitted",
        planningCostSelectionReason_);
    return false;
  }

  const double remaining = selectedGlobalEventPresentationTime_
                         - controllerTime_;
  const bool timingAdmissible =
      remaining + 1e-12 >= selectedGlobalMinimumSafeCommitLead_
      && planningBestCandidate_.predictedPresentationTime
             + selectedGlobalMinimumReachEntryLead_
             <= remaining + 1e-12;
  if(!timingAdmissible)
  {
    planningCostSelectionCommitAdmissible_ = false;
    planningCostSelectionReason_ = "global_winner_expired_before_commit";
    mc_rtc::log::error(
        "[GlobalTimePlanCommitProof] committed=false reason={} remaining={:.6f}s predictedReach={:.6f}s minimumReachEntryLead={:.6f}s minimumSafeCommitLead={:.6f}s no fallback permitted",
        planningCostSelectionReason_, remaining,
        planningBestCandidate_.predictedPresentationTime,
        selectedGlobalMinimumReachEntryLead_,
        selectedGlobalMinimumSafeCommitLead_);
    return false;
  }

  const sva::PTransformd refreshedPresentation =
      predictPresentationPose(remaining);
  const double translationDrift =
      (refreshedPresentation.translation()
       - selectedGlobalObjectPresentationPose_.translation()).norm();
  const double rotationDrift = orientationError(
      refreshedPresentation, selectedGlobalObjectPresentationPose_);
  if(translationDrift
         > predictiveReachPolicy_.maximumObjectTranslationDeviation
     || rotationDrift
         > predictiveReachPolicy_.maximumObjectRotationDeviation)
  {
    planningCostSelectionCommitAdmissible_ = false;
    planningCostSelectionReason_ = "global_event_prediction_drift";
    mc_rtc::log::error(
        "[GlobalTimePlanCommitProof] committed=false reason={} translationDrift={:.6f}/{:.6f} rotationDrift={:.6f}/{:.6f} no fallback permitted",
        planningCostSelectionReason_, translationDrift,
        predictiveReachPolicy_.maximumObjectTranslationDeviation,
        rotationDrift,
        predictiveReachPolicy_.maximumObjectRotationDeviation);
    return false;
  }

  const double residual = planningBestCandidate_.predictedPresentationTime
                        - remaining;
  planningStartMouthPose_ = selectedGlobalPlanningStartMouthPose_;
  commitCandidate(planningBestCandidate_,
                  selectedGlobalObjectPresentationPose_, true,
                  selectedGlobalEventPresentationTime_, residual);
  return committedPlanValid();
}

HandoverInterceptionController::CapturePlanningStatus
HandoverInterceptionController::finalizeCapturePlanning()
{
  planningMbc_.reset();
  if(!planningFoundFeasible_)
  {
    mc_rtc::log::warning(
        "[PlanCapture] no complete collision-free, joint-feasible reach-insert-close-retreat plan exists at the current future-event snapshot; the stationary event solver may inspect another bounded hypothesis");
    capturePlanningStatus_ = CapturePlanningStatus::Failure;
    return capturePlanningStatus_;
  }

  const CaptureCandidate protectedBest = planningBestCandidate_;

  const CaptureCandidate * auditBest = nullptr;
  size_t auditValidCount = 0;
  for(const auto & candidate : planningCompletePlanAuditCandidates_)
  {
    if(!candidate.completeCostAuditValid) { continue; }
    ++auditValidCount;
    if(!auditBest
       || candidate.completeCostAudit < auditBest->completeCostAudit)
    {
      auditBest = &candidate;
    }
  }
  if(auditBest)
  {
    const bool sameAction = auditBest->name == protectedBest.name
        && auditBest->transitRouteName == protectedBest.transitRouteName;
    mc_rtc::log::success(
        "[CompletePlanCostSet] completeRoutes={} costValidRoutes={} protected=[{}:{} score:{:.6f} selectionTime:{:.3f}] predictedTime={:.3f} unconstrainedCostBest=[{}:{} J:{:.6f} time:{:.3f}] sameAction={} configuredMode={} finalTimingAdmissionPending=true",
        planningCompletePlanAuditCandidates_.size(), auditValidCount,
        protectedBest.name, protectedBest.transitRouteName,
        protectedBest.score,
        protectedBest.terminalTimingPredictionBound
            ? protectedBest.legacyEstimatedTime
            : protectedBest.estimatedTime,
        protectedBest.estimatedTime,
        auditBest->name, auditBest->transitRouteName,
        auditBest->completeCostAudit, auditBest->auditEstimatedTime,
        sameAction, completePlanSelectionMode_);
  }
  else
  {
    mc_rtc::log::warning(
        "[CompletePlanCostSet] completeRoutes={} costValidRoutes=0 cost unavailable configuredMode={}",
        planningCompletePlanAuditCandidates_.size(),
        completePlanSelectionMode_);
  }

  // Prepare an unconstrained selection now. The moving-event state calls this
  // method again with the remaining event time so the binding argmin is taken
  // only over plans that can still satisfy the final launch reserve. In
  // global_time_plan mode this local single-hypothesis result is diagnostic
  // only: a hard-feasible hypothesis must still reach global pooling even
  // when this local selection is cost-invalid, so only the absence of any
  // geometrically feasible plan (checked above) fails the hypothesis here.
  const bool localSelectionSucceeded = selectPlanningBestForCommit();
  if(!localSelectionSucceeded && !globalTimePlanSelectionEnabled())
  {
    capturePlanningStatus_ = CapturePlanningStatus::Failure;
    return capturePlanningStatus_;
  }

  if(localSelectionSucceeded)
  {
    const CaptureCandidate & best = planningBestCandidate_;

    if(capturePlanningCommitOnSuccess_)
    {
      commitCandidate(best, W_T_O_, false, 0.0, 0.0);
    }
    else
    {
      const Eigen::Vector3d po = W_T_O_.translation();
      mc_rtc::log::success(
          "[PlanCaptureProbe] BEST {} route={} at presentation object=[{:.3f},{:.3f},{:.3f}] predictedPresentation={:.3f}s predictedContact={:.3f}s predictedExecution={:.3f}s reachClear={:.4f} score={:.4f}",
          best.name, best.transitRouteName, po.x(), po.y(), po.z(),
          best.predictedPresentationTime, best.predictedContactTime,
          best.estimatedTime, best.predictiveReachClearance, best.score);
    }
  }

  capturePlanningStatus_ = CapturePlanningStatus::Success;
  return capturePlanningStatus_;
}

void HandoverInterceptionController::commitCandidate(
    const CaptureCandidate & best,
    const sva::PTransformd & W_T_O_reference,
    bool asInterception,
    double committedPresentationTime,
    double timingResidual)
{
  if(completePlanSelectionMode_ == "binding_cost")
  {
    const bool selectedEqualsMinimum = best.completeCostAuditValid
        && planningCostSelectionValid_
        && planningCostSelectionCommitAdmissible_
        && std::isfinite(planningSelectedObjectiveCost_)
        && std::isfinite(planningMinimumAdmissibleCost_)
        && std::abs(planningSelectedObjectiveCost_
                    - planningMinimumAdmissibleCost_)
               <= decisionCostTieTolerance_;
    if(!selectedEqualsMinimum)
    {
      candidateSelected_ = false;
      mc_rtc::log::error(
          "[BindingCostCommitProof] committed=false reason=selection_proof_failed candidate={} route={} motionJ={:.9f} selectedJ={:.9f} minimumAdmissibleJ={:.9f} selectionValid={} commitAdmissible={} globalSelection={} no fallback permitted",
          best.name, best.transitRouteName, best.completeCostAudit,
          planningSelectedObjectiveCost_,
          planningMinimumAdmissibleCost_, planningCostSelectionValid_,
          planningCostSelectionCommitAdmissible_,
          planningGlobalSelectionActive_);
      return;
    }
  }

  candidateSelected_ = true;
  selectedCandidateName_ = best.name;
  selectedCandidateClearance_ = best.minClearance;
  selectedCandidateScore_ = best.score;
  selectedCandidatePredictedTime_ = best.estimatedTime;
  selectedCandidatePredictedPresentationTime_ =
      best.predictedPresentationTime;
  selectedCandidatePredictedContactTime_ = best.predictedContactTime;
  selectedCandidatePredictedReachTime_ = best.predictedReachTime;
  selectedCandidatePredictedApproachTime_ = best.predictedApproachTime;
  selectedCandidatePredictedAcquireTime_ = best.predictedAcquireTime;
  selectedCandidatePredictedEffort_ = best.predictedEffort;
  selectedCandidateContactClosure_ = best.contactClosure;
  selectedTransitArmPosture_ = best.plannedTransitArmPosture;
  selectedStandoffArmPosture_ = best.plannedStandoffArmPosture;
  selectedArmPosture_ = best.plannedArmPosture;
  selectedRetreatArmPosture_ = best.plannedRetreatArmPosture;

  W_T_M_transit_ = best.W_T_M_transit;
  W_T_M_standoff_ = best.W_T_M_standoff;
  W_T_M_pre_ = best.W_T_M_pre;
  W_T_M_acquired_ = best.W_T_M_pre;
  W_T_M_retreat_ = best.W_T_M_retreat;
  O_T_M_transit_ = relativePose(W_T_O_reference, W_T_M_transit_);
  O_T_M_standoff_ = relativePose(W_T_O_reference, W_T_M_standoff_);
  O_T_M_pre_ = relativePose(W_T_O_reference, W_T_M_pre_);
  O_T_M_retreat_ = relativePose(W_T_O_reference, W_T_M_retreat_);

  interceptionCommitted_ = false;
  committedInterceptionPlan_ = InterceptionPlan{};
  simulatedTruthInterceptionPlan_ = InterceptionPlan{};
  simulatedTruthInterceptionPlanValid_ = false;
  if(asInterception)
  {
    const double retreatDuration = std::max(
        0.0, best.estimatedTime - best.predictedContactTime
             - timingBilateralDwell_ - timingConfirmationDwell_);
    const InterceptionPlan plan = makeInterceptionPlan(
        best, W_T_O_reference, committedPresentationTime,
        best.predictedReachTime, best.predictedApproachTime,
        best.predictedAcquireTime, retreatDuration);
    std::string invariantReason;
    if(!validateInterceptionPlan(plan, &invariantReason, true))
    {
      candidateSelected_ = false;
      if(completePlanSelectionMode_ == "binding_cost")
      {
        mc_rtc::log::error(
            "[BindingCostCommitProof] committed=false reason=interception_plan_validation candidate={} route={} detail={}",
            best.name, best.transitRouteName, invariantReason);
      }
      mc_rtc::log::error(
          "[InterceptionCommit] methodology-lock validation failed candidate={} reason={}; no physical motion",
          best.name, invariantReason);
      return;
    }

    // Atomic one-shot commit: copy the already validated value object, then
    // expose legacy fields only as read-only mirrors for logging/GUI code.
    committedInterceptionPlan_ = plan;
    interceptionCommitted_ = true;
    committedContactTime_ = plan.contactTime;
    committedTimingResidual_ = timingResidual;
    W_T_O_committedContact_ = plan.objectAtContact;
    committedObjectLinearVelocity_ = plan.objectLinearVelocity;
    committedObjectAngularVelocity_ = plan.objectAngularVelocity;

    if(simulateMovingObject_)
    {
      simulatedTruthInterceptionPlan_ = plan;
      const sva::PTransformd nominalAtCommit = interceptionObjectPoseAt(
          plan, controllerTime_);
      const sva::PTransformd nominal_T_truth = relativePose(
          nominalAtCommit, W_T_O_truth_);
      simulatedTruthInterceptionPlan_.objectAtPresentation = compose(
          plan.objectAtPresentation, nominal_T_truth);
      simulatedTruthInterceptionPlan_.objectAtContact =
          simulatedTruthInterceptionPlan_.objectAtPresentation;
      simulatedTruthInterceptionPlanValid_ = true;
      mc_rtc::log::success(
          "[PerceptionLatencyTruthAnchor] mode={} estimateToTruthTranslation={:.4f}m estimateToTruthRotation={:.4f}rad sameCommittedTiming=true sameDecelerationLaw=true noTruthReanchorToDelayedMeasurement=true",
          perceptionLatencyModeName(),
          (nominalAtCommit.translation() - W_T_O_truth_.translation()).norm(),
          orientationError(nominalAtCommit, W_T_O_truth_));
    }

    O_T_M_standoff_ = plan.O_T_M_standoff;
    O_T_M_pre_ = plan.O_T_M_capture;
    O_T_M_retreat_ = plan.O_T_M_retreat;

    havePreviousObjectObservation_ = true;
    W_T_O_previousObservation_ = W_T_O_perceptionMeasurement_;
    previousObjectObservationTime_ = objectPerceptionMeasurementTime_;
  }
  else
  {
    committedContactTime_ = 0.0;
    committedTimingResidual_ = 0.0;
    W_T_O_committedContact_ = W_T_O_reference;
  }

  if(completePlanSelectionMode_ == "binding_cost"
     && (!asInterception || interceptionCommitted_))
  {
    mc_rtc::log::success(
        "[BindingCostCommitProof] committed=true candidate={} route={} selectedJ={:.9f} minimumAdmissibleJ={:.9f} selectedWithinMinimumTolerance=true tieTolerance={:.3e} completePlans={} costValidPlans={} timingAdmissiblePlans={} motionJ={:.9f} eventTimePolicy={}",
        best.name, best.transitRouteName, planningSelectedObjectiveCost_,
        planningMinimumAdmissibleCost_, decisionCostTieTolerance_,
        planningCompletePlanCount_, planningCostValidCount_,
        planningTimingAdmissibleCount_, best.completeCostAudit,
        planningGlobalSelectionActive_
            ? "global_time_plan" : "first_admissible_center_out");
    if(planningGlobalSelectionActive_ && asInterception
       && interceptionCommitted_)
    {
      mc_rtc::log::success(
          "[GlobalTimePlanCommitProof] committed=true scheduleComplete={} evaluatedHypotheses={} configuredHypotheses={} completePlans={} costValidPlans={} timingAdmissiblePlans={} hypothesis={} eventLead={:.3f}s presentationTime={:.3f}s candidate={} route={} motionJ={:.9f} scheduleWait={:+.3f}s selectedGlobalJ={:.9f} minimumGlobalJ={:.9f} selectedWithinMinimumTolerance=true tieTolerance={:.3e} objective=time_grasp_route noRetiming=true noReplanning=true",
          globalScheduleComplete_, globalEvaluatedHypotheses_,
          globalConfiguredHypotheses_, planningCompletePlanCount_,
          planningCostValidCount_, planningTimingAdmissibleCount_,
          selectedGlobalHypothesisIndex_, selectedGlobalEventLead_,
          selectedGlobalEventPresentationTime_, best.name,
          best.transitRouteName, best.completeCostAudit,
          selectedGlobalScheduleWait_, planningSelectedObjectiveCost_,
          planningMinimumAdmissibleCost_, decisionCostTieTolerance_);
    }
  }

  const Eigen::Vector3d ps = W_T_M_standoff_.translation();
  const Eigen::Vector3d pp = W_T_M_pre_.translation();
  const Eigen::Vector3d pr = W_T_M_retreat_.translation();
  if(asInterception && interceptionCommitted_)
  {
    const Eigen::Vector3d po = W_T_O_reference.translation();
    mc_rtc::log::success(
        "[PresentationCommit] COMMITTED candidate={} route={} mode={} immutablePlan=true predictiveStatic=true presentationTime={:.3f}s timeToPresentation={:.3f}s nominalContactTime={:.3f}s timeToNominalContact={:.3f}s acquisitionDeadline={:.3f}s acquisitionWindow={:.3f}s residual={:+.3f}s objectPresentation=[{:.3f},{:.3f},{:.3f}] decelerationStart={:.3f}s reachStart={:.3f}s acquireStart={:.3f}s predictedPresentationDuration={:.3f}s predictedContactDuration={:.3f}s reach={:.3f}s approach={:.3f}s nominalAcquireToContact={:.3f}s openThroughInsertion=true fullClosure={:.3f} predictedExecution={:.3f}s clear={:.4f} standoff=[{:.3f},{:.3f},{:.3f}] capture=[{:.3f},{:.3f},{:.3f}] retreat=[{:.3f},{:.3f},{:.3f}]",
        selectedCandidateName_, committedInterceptionPlan_.transitRouteName,
        observedObjectModeName(),
        committedInterceptionPlan_.presentationTime,
        committedInterceptionPlan_.presentationTime - controllerTime_,
        committedContactTime_, committedContactTime_ - controllerTime_,
        committedInterceptionPlan_.acquisitionDeadlineTime,
        committedInterceptionPlan_.acquisitionWindowDuration,
        committedTimingResidual_, po.x(), po.y(), po.z(),
        committedInterceptionPlan_.decelerationStartTime,
        committedInterceptionPlan_.reachStartTime,
        committedInterceptionPlan_.acquireStartTime,
        selectedCandidatePredictedPresentationTime_,
        selectedCandidatePredictedContactTime_,
        selectedCandidatePredictedReachTime_,
        selectedCandidatePredictedApproachTime_,
        selectedCandidatePredictedAcquireTime_,
        committedInterceptionPlan_.contactClosure,
        selectedCandidatePredictedTime_, selectedCandidateClearance_,
        ps.x(), ps.y(), ps.z(), pp.x(), pp.y(), pp.z(),
        pr.x(), pr.y(), pr.z());
    mc_rtc::log::success(
        "[ForceTransferContractCommit] source={} immutable=true fullSupport={:.3f}N desiredDuration={:.3f}s releaseThreshold={:.3f} releaseRateTolerance={:.3f}/s releaseDwell={:.3f}s timeout={:.3f}s maxForce={:.3f}N maxMoment={:.3f}Nm admittance=[M:{:.3f},D:{:.3f},K:{:.3f},vMax:{:.3f},xMax:{:.3f}] virtualContact=[K:{:.3f},D:{:.3f},loadScale:{:.3f}] noRuntimeReselection=true",
        committedInterceptionPlan_.forceTransferSource,
        committedInterceptionPlan_.forceTransferFullSupportForce,
        committedInterceptionPlan_.forceTransferDuration
          - committedInterceptionPlan_.forceTransferReleaseDwell,
        committedInterceptionPlan_.forceTransferReleaseThreshold,
        committedInterceptionPlan_.forceTransferReleaseRateTolerance,
        committedInterceptionPlan_.forceTransferReleaseDwell,
        committedInterceptionPlan_.forceTransferTimeout,
        committedInterceptionPlan_.forceTransferMaximumForce,
        committedInterceptionPlan_.forceTransferMaximumMoment,
        committedInterceptionPlan_.forceTransferAdmittanceMass,
        committedInterceptionPlan_.forceTransferAdmittanceDamping,
        committedInterceptionPlan_.forceTransferAdmittanceStiffness,
        committedInterceptionPlan_.forceTransferMaximumAdmittanceSpeed,
        committedInterceptionPlan_.forceTransferMaximumAdmittanceOffset,
        committedInterceptionPlan_.forceTransferVirtualContactStiffness,
        committedInterceptionPlan_.forceTransferVirtualContactDamping,
        committedInterceptionPlan_.forceTransferVirtualMaximumLoadScale);
  }
  else if(!asInterception)
  {
    mc_rtc::log::success(
        "[PlanCapture] SELECTED {} before motion: complete reach-insert-close-retreat predictedTime={:.3f}s predictedContact={:.3f}s rawRollout={:.3f}s arm={:.3f}s closure={:.3f}s fixed={:.3f}s effort={:.3f} clear={:.4f} contactClosure={:.3f} standoff=[{:.3f},{:.3f},{:.3f}] capture=[{:.3f},{:.3f},{:.3f}] retreat=[{:.3f},{:.3f},{:.3f}]",
        selectedCandidateName_, selectedCandidatePredictedTime_,
        selectedCandidatePredictedContactTime_, best.rawRolloutTime,
        best.predictedArmTime, best.predictedClosureTime,
        best.predictedFixedTime, selectedCandidatePredictedEffort_,
        selectedCandidateClearance_, best.contactClosure,
        ps.x(), ps.y(), ps.z(), pp.x(), pp.y(), pp.z(),
        pr.x(), pr.y(), pr.z());
  }
}

bool HandoverInterceptionController::commitPlanningBestAsInterception(
    double committedPresentationTime,
    double timingResidual,
    const sva::PTransformd & W_T_O_presentation)
{
  if(!planningFoundFeasible_)
  {
    mc_rtc::log::error(
        "[InterceptionCommit] no feasible planning candidate is available");
    return false;
  }
  if(completePlanSelectionMode_ == "binding_cost"
     && (!planningCostSelectionValid_
         || !planningCostSelectionCommitAdmissible_))
  {
    mc_rtc::log::error(
        "[InterceptionCommit] binding-cost selection is not commit-admissible reason={}; no physical motion",
        planningCostSelectionReason_);
    return false;
  }
  commitCandidate(planningBestCandidate_, W_T_O_presentation, true,
                  committedPresentationTime, timingResidual);
  return committedPlanValid();
}

HandoverInterceptionController::CapturePlanningStatus
HandoverInterceptionController::stepCapturePlanning(int maxInternalSteps)
{
  if(capturePlanningStatus_ != CapturePlanningStatus::Running)
  {
    return capturePlanningStatus_;
  }

  const int budget = std::max(1, maxInternalSteps);
  for(int step = 0; step < budget
      && capturePlanningStatus_ == CapturePlanningStatus::Running; ++step)
  {
    if(!planningMbc_)
    {
      if(!startNextPlanningCandidate())
      {
        return finalizeCapturePlanning();
      }
    }

    PreviewStepStatus status = PreviewStepStatus::Failed;
    if(planningPhase_ == PlanningPhase::ReachStandoff)
    {
      status = previewReachStep(
          *planningMbc_, planningCurrentCandidate_.W_T_M_standoff,
          false, false, planningSegmentIteration_, planningResult_);
      if(status == PreviewStepStatus::Succeeded)
      {
        planningResult_.reachStandoffDuration =
            planningResult_.duration - planningPhaseStartDuration_;
        planningPhaseStartDuration_ = planningResult_.duration;
        // Preserve the redundancy branch at the end of the standoff segment.
        // Reach execution must not be pulled toward the later capture posture.
        planningCurrentCandidate_.plannedStandoffArmPosture =
            armPostureFromMbc(*planningMbc_);
        planningPhase_ = PlanningPhase::ReachCapture;
        planningSegmentIteration_ = 0;
        continue;
      }
    }
    else if(planningPhase_ == PlanningPhase::ReachCapture)
    {
      status = previewReachStep(
          *planningMbc_, planningCurrentCandidate_.W_T_M_pre,
          true, false, planningSegmentIteration_, planningResult_);
      if(status == PreviewStepStatus::Succeeded)
      {
        planningResult_.reachCaptureDuration =
            planningResult_.duration - planningPhaseStartDuration_;
        planningPhaseStartDuration_ = planningResult_.duration;
        planningPhase_ = PlanningPhase::Closure;
        planningClosureIndex_ = 0;
        continue;
      }
    }
    else if(planningPhase_ == PlanningPhase::Closure)
    {
      status = previewClosureStep(
          *planningMbc_, planningClosureIndex_, planningResult_);
      if(status == PreviewStepStatus::Succeeded)
      {
        planningResult_.closureDuration =
            planningResult_.duration - planningPhaseStartDuration_;
        planningPhaseStartDuration_ = planningResult_.duration;
        // Save the capture redundancy branch before retreat changes it.
        planningCurrentCandidate_.plannedArmPosture =
            armPostureFromMbc(*planningMbc_);
        planningM_T_O_ = relativePose(
            previewMouthPose(*planningMbc_), W_T_O_);
        planningPhase_ = PlanningPhase::Retreat;
        planningSegmentIteration_ = 0;
        continue;
      }
    }
    else
    {
      status = previewReachStep(
          *planningMbc_, planningCurrentCandidate_.W_T_M_retreat,
          false, true, planningSegmentIteration_, planningResult_);
      if(status == PreviewStepStatus::Succeeded)
      {
        planningResult_.retreatDuration =
            planningResult_.duration - planningPhaseStartDuration_;
        planningPhaseStartDuration_ = planningResult_.duration;
        finishCurrentPlanningCandidate(true);
        if(planningCandidateIndex_ >= planningCandidateCount_)
        {
          return finalizeCapturePlanning();
        }
        continue;
      }
    }

    if(status == PreviewStepStatus::Failed)
    {
      finishCurrentPlanningCandidate(false);
      if(planningCandidateIndex_ >= planningCandidateCount_)
      {
        return finalizeCapturePlanning();
      }
    }
  }
  return capturePlanningStatus_;
}

void HandoverInterceptionController::invalidateSelectedCandidate()
{
  candidateSelected_ = false;
  selectedCandidateName_ = "none";
  selectedCandidateClearance_ = -1e9;
  selectedCandidateScore_ = 1e9;
  selectedCandidatePredictedTime_ = 1e9;
  selectedCandidatePredictedPresentationTime_ = 1e9;
  selectedCandidatePredictedContactTime_ = 1e9;
  selectedCandidatePredictedReachTime_ = 1e9;
  selectedCandidatePredictedApproachTime_ = 1e9;
  selectedCandidatePredictedAcquireTime_ = 1e9;
  selectedCandidatePredictedEffort_ = 1e9;
  selectedCandidateContactClosure_ = -1.0;
  selectedTransitArmPosture_.clear();
  selectedStandoffArmPosture_.clear();
  selectedArmPosture_.clear();
  selectedRetreatArmPosture_.clear();
  W_T_M_acquired_ = sva::PTransformd::Identity();
  W_T_M_retreat_ = sva::PTransformd::Identity();
  O_T_M_retreat_ = sva::PTransformd::Identity();
  interceptionCommitted_ = false;
  committedContactTime_ = 0.0;
  committedTimingResidual_ = 1e9;
}

// =============================================================================
// Task, posture and gripper commands
// =============================================================================

void HandoverInterceptionController::activateToolTask()
{
  if(!toolTask_) { return; }

  auto postureTask = getPostureTask(robot().name());
  if(postureTask)
  {
    postureTask->stiffness(gripperPostureStiffnessNormal_);
    postureTask->weight(gripperPostureWeight_);
  }

  toolTask_->stiffness(taskStiffness_);
  toolTask_->weight(taskWeight_);
  if(!toolTaskActive_)
  {
    solver().addTask(toolTask_);
    toolTaskActive_ = true;
    mc_rtc::log::info(
        "[HandoverInterception] Cartesian base task activated for mouth-frame control");
  }
}

void HandoverInterceptionController::deactivateToolTask()
{
  if(toolTask_ && toolTaskActive_)
  {
    solver().removeTask(toolTask_);
    toolTaskActive_ = false;
  }
}

void HandoverInterceptionController::setToolTaskGains(double stiffness, double weight)
{
  if(!toolTask_) { return; }
  toolTask_->stiffness(std::max(0.1, stiffness));
  toolTask_->weight(std::max(1.0, weight));
}

void HandoverInterceptionController::setGripperJointPriority(bool highPriority)
{
  setGripperJointPriority(highPriority ? 1.0 : 0.0);
}

void HandoverInterceptionController::setGripperJointPriority(double priorityBlend)
{
  auto postureTask = getPostureTask(robot().name());
  if(!postureTask) { return; }

  const double alpha = std::max(0.0, std::min(1.0, priorityBlend));
  const double jointWeightReference =
      (1.0 - alpha) * gripperJointWeightNormal_
      + alpha * gripperJointWeightHigh_;
  const double jointStiffness =
      (1.0 - alpha) * gripperPostureStiffnessNormal_
      + alpha * gripperPostureStiffnessHigh_;

  // Keep the arm posture objective completely unchanged. The previous
  // implementation changed the shared posture task stiffness/weight when the
  // gripper priority changed, which created a discontinuity in the arm QP at
  // GuardedApproach -> Acquire. Only gripper joint parameters are changed now.
  // Normalize the per-joint multiplier so the effective gripper weight remains
  // equal to the historical setting regardless of the current arm-task weight.
  const double armTaskWeight = std::max(1e-9, postureTask->weight());
  const double desiredEffectiveWeight =
      std::max(1e-9, gripperPostureWeight_) * jointWeightReference;
  const double normalizedJointWeight = desiredEffectiveWeight / armTaskWeight;

  std::map<std::string, double> weights;
  std::vector<tasks::qp::JointStiffness> stiffness;
  const std::vector<std::string> joints = {
      "gen3_robotiq_85_left_knuckle_joint",
      "gen3_robotiq_85_right_knuckle_joint",
      "gen3_robotiq_85_left_inner_knuckle_joint",
      "gen3_robotiq_85_right_inner_knuckle_joint",
      "gen3_robotiq_85_left_finger_tip_joint",
      "gen3_robotiq_85_right_finger_tip_joint"};
  for(const auto & joint : joints)
  {
    if(!hasActuatedJoint(joint)) { continue; }
    weights[joint] = normalizedJointWeight;
    stiffness.emplace_back(joint, jointStiffness);
  }
  if(!weights.empty()) { postureTask->jointWeights(weights); }
  if(!stiffness.empty()) { postureTask->jointStiffness(solver(), stiffness); }
}

void HandoverInterceptionController::commandArmPosture(
    const std::map<std::string, std::vector<double>> & target)
{
  if(target.empty()) { return; }
  auto postureTask = getPostureTask(robot().name());
  if(!postureTask)
  {
    mc_rtc::log::error("[ArmPosture] posture task unavailable");
    return;
  }
  // During Cartesian execution this is the low-priority redundancy objective.
  // These are the same global gains previously left after normal gripper
  // priority, but they are no longer overwritten when the gripper closes.
  postureTask->stiffness(std::max(0.1, gripperPostureStiffnessNormal_));
  postureTask->weight(std::max(1.0, gripperPostureWeight_));
  postureTask->target(target);
}

std::map<std::string, std::vector<double>>
HandoverInterceptionController::interpolateArmPosture(
    const std::map<std::string, std::vector<double>> & from,
    const std::map<std::string, std::vector<double>> & to,
    double alpha) const
{
  alpha = clamp01(alpha);
  std::map<std::string, std::vector<double>> out = to;
  for(const auto & item : from)
  {
    const auto jt = to.find(item.first);
    if(jt == to.end())
    {
      out[item.first] = item.second;
      continue;
    }
    const size_t n = std::min(item.second.size(), jt->second.size());
    std::vector<double> values = jt->second;
    for(size_t k = 0; k < n; ++k)
    {
      values[k] = (1.0 - alpha) * item.second[k]
                + alpha * jt->second[k];
    }
    out[item.first] = values;
  }
  return out;
}

void HandoverInterceptionController::commandReadyArmPosture()
{
  commandReadyArmPosture(readyPosture_);
}

void HandoverInterceptionController::commandReadyArmPosture(
    const std::map<std::string, std::vector<double>> & target)
{
  if(target.empty()) { return; }
  auto postureTask = getPostureTask(robot().name());
  if(!postureTask)
  {
    mc_rtc::log::error("[ArmPosture] posture task unavailable");
    return;
  }
  postureTask->stiffness(std::max(0.1, readyPostureStiffness_));
  postureTask->weight(std::max(1.0, readyPostureWeight_));
  postureTask->target(target);
}

void HandoverInterceptionController::commandSelectedStandoffPosture()
{
  commandArmPosture(selectedStandoffArmPosture_);
}

void HandoverInterceptionController::commandSelectedReachPosture(
    double progress,
    const std::map<std::string, std::vector<double>> & startPosture)
{
  progress = clamp01(progress);
  const auto & transit = selectedTransitArmPosture_.empty()
      ? selectedStandoffArmPosture_ : selectedTransitArmPosture_;
  if(progress <= 0.5)
  {
    commandArmPosture(interpolateArmPosture(
        startPosture, transit, 2.0 * progress));
  }
  else
  {
    commandArmPosture(interpolateArmPosture(
        transit, selectedStandoffArmPosture_, 2.0 * progress - 1.0));
  }
}

void HandoverInterceptionController::commandSelectedArmPosture()
{
  commandArmPosture(selectedArmPosture_);
}

void HandoverInterceptionController::commandSelectedRetreatPosture()
{
  commandArmPosture(selectedRetreatArmPosture_);
}

std::map<std::string, std::vector<double>>
HandoverInterceptionController::currentArmPosture() const
{
  return armPostureFromMbc(robot().mbc());
}

void HandoverInterceptionController::setGripperClosureAuthorized(
    bool authorized)
{
  gripperClosureAuthorized_ = authorized;
  if(authorized) { gripperAuthorityViolationLogged_ = false; }
}

void HandoverInterceptionController::refreshPhysicalGripperBridge()
{
  if(!physicalGripperBridgeEnabled_)
  {
    physicalGripperFeedbackValid_ = false;
    return;
  }

  auto & ds = datastore();
  constexpr const char * validKey =
      "HandoverInterceptionController::gripperFeedbackValid";
  constexpr const char * positionKey =
      "HandoverInterceptionController::gripperMeasuredPercent";
  constexpr const char * velocityKey =
      "HandoverInterceptionController::gripperMeasuredVelocityPercent";
  constexpr const char * sequenceKey =
      "HandoverInterceptionController::gripperFeedbackSequence";

  bool valid = false;
  try
  {
    valid = ds.has(validKey) && ds.get<bool>(validKey);
    if(valid && ds.has(positionKey) && ds.has(velocityKey))
    {
      const double position = ds.get<double>(positionKey);
      const double velocity = ds.get<double>(velocityKey);
      if(std::isfinite(position) && std::isfinite(velocity))
      {
        physicalGripperMeasuredPercent_ =
            std::max(0.0, std::min(100.0, position));
        physicalGripperMeasuredVelocityPercent_ = velocity;
        if(ds.has(sequenceKey))
        {
          physicalGripperFeedbackSequence_ = ds.get<uint64_t>(sequenceKey);
        }
      }
      else
      {
        valid = false;
      }
    }
    else if(valid)
    {
      valid = false;
    }
  }
  catch(...)
  {
    valid = false;
  }

  physicalGripperFeedbackValid_ = valid;
  if(!valid && physicalGripperRequireFeedback_ && !physicalGripperFeedbackWarningLogged_)
  {
    physicalGripperFeedbackWarningLogged_ = true;
    mc_rtc::log::error(
        "[PhysicalGripperBridge] feedback unavailable; physical Robotiq commands remain disabled");
  }
  else if(valid)
  {
    physicalGripperFeedbackWarningLogged_ = false;
  }
}

void HandoverInterceptionController::commandGripper(double close)
{
  close = std::max(0.0, std::min(1.0, close));
  if(close > 1e-9 && !gripperClosureAuthorized_)
  {
    if(!gripperAuthorityViolationLogged_)
    {
      gripperAuthorityViolationLogged_ = true;
      mc_rtc::log::error(
          "[GripperAuthority] unauthorized early closure command {:.3f} clamped to fully open; only Acquire/Retreat may close",
          close);
    }
    close = 0.0;
  }
  else if(close <= 1e-9)
  {
    gripperAuthorityViolationLogged_ = false;
  }
  gripperCommand_ = close;

  auto & ds = datastore();
  const std::string closeKey = "HandoverInterceptionController::gripperClose";
  const std::string openPercentKey =
      "HandoverInterceptionController::gripperOpenPercent";
  const std::string closePercentKey =
      "HandoverInterceptionController::gripperClosePercent";
  const std::string maxKey = "HandoverInterceptionController::gripperMaxPercent";
  const std::string enabledKey = "HandoverInterceptionController::gripperCommandEnabled";
  const bool physicalCommandAllowed =
      physicalGripperBridgeEnabled_
      && physicalGripperCommandEnabled_
      && (!physicalGripperRequireFeedback_ || physicalGripperFeedbackValid_);
  if(!ds.has(closeKey)) { ds.make<double>(closeKey, close); }
  else { ds.assign(closeKey, close); }
  if(!ds.has(openPercentKey))
  {
    ds.make<double>(openPercentKey, physicalGripperOpenPercent_);
  }
  else { ds.assign(openPercentKey, physicalGripperOpenPercent_); }
  if(!ds.has(closePercentKey))
  {
    ds.make<double>(closePercentKey, physicalGripperClosePercent_);
  }
  else { ds.assign(closePercentKey, physicalGripperClosePercent_); }
  if(!ds.has(maxKey)) { ds.make<double>(maxKey, physicalGripperMaxPercent_); }
  else { ds.assign(maxKey, physicalGripperMaxPercent_); }
  if(!ds.has(enabledKey)) { ds.make<bool>(enabledKey, physicalCommandAllowed); }
  else { ds.assign(enabledKey, physicalCommandAllowed); }

  const double q = gripperOpenQ_ + close * (gripperCloseQ_ - gripperOpenQ_);
  std::map<std::string, std::vector<double>> target;

  auto add = [this, &target](const std::string & name, double value)
  {
    if(hasActuatedJoint(name)) { target[name] = {value}; }
  };

  add("gen3_robotiq_85_left_knuckle_joint", q);
  add("gen3_robotiq_85_right_knuckle_joint", -q);
  add("gen3_robotiq_85_left_inner_knuckle_joint", q);
  add("gen3_robotiq_85_right_inner_knuckle_joint", -q);
  add("gen3_robotiq_85_left_finger_tip_joint", -q);
  add("gen3_robotiq_85_right_finger_tip_joint", q);

  if(!target.empty())
  {
    auto postureTask = getPostureTask(robot().name());
    if(postureTask) { postureTask->target(target); }
  }
  else
  {
    static bool warnedFixedModel = false;
    if(!warnedFixedModel)
    {
      warnedFixedModel = true;
      mc_rtc::log::error(
          "[GripperActuation] {}. Closure cannot move in ticker until the actuated kinova_gen3_2f85 RobotModule is loaded",
          gripperActuationStatus());
    }
  }
}

bool HandoverInterceptionController::geometricGripReached() const
{
  // Static geometric success now requires bilateral inner-pad tangency,
  // correct centering and no penetration of any hard gripper or ground volume.
  return bilateralPadContactReached(nullptr);
}

// =============================================================================
// GUI
// =============================================================================

void HandoverInterceptionController::addMethodologyGui()
{
  if(!gui()) { return; }

  gui()->addElement(
      {"Handover", "Methodology"},
      mc_rtc::gui::Transform("OBJECT O", [this]() { return W_T_O_; }),
      mc_rtc::gui::Transform("PREDICTED OBJECT O(t+h)", [this]() {
        return W_T_O_predicted_;
      }),
      mc_rtc::gui::Transform("COMMITTED OBJECT AT CONTACT", [this]() {
        return W_T_O_committedContact_;
      }),
      mc_rtc::gui::Transform("COMMITTED OBJECT NOW", [this]() {
        return interceptionCommitted_ ? committedObjectPoseAt(controllerTime_)
                                      : W_T_O_;
      }),
      mc_rtc::gui::Transform("COMMITTED MOVING CAPTURE", [this]() {
        return interceptionCommitted_ ? committedCaptureTargetAt(controllerTime_)
                                      : W_T_M_pre_;
      }),
      mc_rtc::gui::Transform("BLUE HANDLE H", [this]() { return W_T_H_; }),
      mc_rtc::gui::Transform("ACTUAL MOUTH M", [this]() { return actualMouthPose(); }),
      mc_rtc::gui::Transform("TRANSIT STANDOFF", [this]() { return W_T_M_transit_; }),
      mc_rtc::gui::Transform("ALIGNED STANDOFF", [this]() { return W_T_M_standoff_; }),
      mc_rtc::gui::Transform("CAPTURE PREGRASP", [this]() { return W_T_M_pre_; }),
      mc_rtc::gui::Transform("CERTIFIED RETREAT", [this]() { return W_T_M_retreat_; }),
      mc_rtc::gui::Point3D("BLUE HANDLE CENTER", [this]() -> Eigen::Vector3d {
        return W_T_H_.translation();
      }),
      mc_rtc::gui::Point3D("PREDICTED OBJECT CENTER", [this]() -> Eigen::Vector3d {
        return W_T_O_predicted_.translation();
      }),
      mc_rtc::gui::Point3D("COMMITTED CONTACT CENTER", [this]() -> Eigen::Vector3d {
        return W_T_O_committedContact_.translation();
      }),
      mc_rtc::gui::Point3D("MOUTH CENTER", [this]() -> Eigen::Vector3d {
        return actualMouthPose().translation();
      }),
      mc_rtc::gui::Point3D("LEFT PAD REFERENCE", [this]() -> Eigen::Vector3d {
        Eigen::Vector3d pL, pR;
        return livePadCenters(pL, pR) ? pL : Eigen::Vector3d::Zero();
      }),
      mc_rtc::gui::Point3D("RIGHT PAD REFERENCE", [this]() -> Eigen::Vector3d {
        Eigen::Vector3d pL, pR;
        return livePadCenters(pL, pR) ? pR : Eigen::Vector3d::Zero();
      }));
}
