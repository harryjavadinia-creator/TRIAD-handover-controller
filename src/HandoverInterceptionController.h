#pragma once

#include "IndependentGiverModel.h"
#include "ControlAwareGraspSupervisor.h"
#include "ReceivingGraspFamily.h"
#include "PredictiveInterception.h"
#include "DualGiverCoordinator.h"

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/TransformTask.h>

#include <RBDyn/Jacobian.h>
#include <RBDyn/MultiBodyConfig.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <iterator>
#include <map>
#include <memory>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <utility>
#include <vector>

struct HandoverSafetyReport
{
  bool safe = false;
  double minClearance = -1e9;
  double acceptedScale = 0.0;
  std::string sample = "none";
  std::string obstacle = "none";

  double corridorLateralClearance = -1e9;
  double corridorPalmClearance = -1e9;
  double corridorEntryClearance = -1e9;
  double corridorAxialClearance = -1e9;
  double corridorAngleClearance = -1e9;

  double groundClearance = -1e9;
  double leftPadClearance = 1e9;
  double rightPadClearance = 1e9;
  // Signed handle offset along the mouth closing axis. Positive means the
  // mouth must translate in +x_M to re-center the handle.
  double signedPadCenteringError = 0.0;
  double padCenteringError = 1e9;
  bool bilateralPadContact = false;
};

/**
 * Plan-once, execute-once force-aware handover controller.
 *
 * Methodology:
 *   stationary observation of object pose/twist and transfer-sensor bias
 *   -> complete copied-state grasp/route/capture/retreat certification
 *   -> select one fully feasible object-relative action and commit once
 *   -> execute the committed reach and guarded capture once
 *   -> keep one continuous capture controller through bilateral contact
 *   -> regulate the committed load-transfer policy from measured wrench
 *   -> admit attachment/retreat only after stable support-transfer evidence
 *   -> execute the pre-certified carried-object retreat and hold.
 *
 * The candidate, grasp, route, transfer contract and retreat never change
 * after commitment. Runtime pose, gripper and wrench feedback only regulate
 * execution of that single committed action. Any unexpected violation enters
 * fail-safe hold; there is no retry, return-home trial or candidate reselection.
 */
struct HandoverInterceptionController : public mc_control::fsm::Controller
{
public:
  enum class CapturePlanningStatus
  {
    Idle,
    Running,
    Success,
    Failure
  };

  /**
   * Observation-time classification selected before any physical motion.
   * Static and moving objects share the same complete candidate evaluator and
   * one-shot execution chain; only the spacetime event construction differs.
   */
  enum class ObservedObjectMode
  {
    Unclassified,
    Static,
    Moving
  };

  enum class InterceptionPhase
  {
    Reach,
    Approach,
    Acquire,
    Confirm
  };

  /**
   * Immutable single-source-of-truth for one candidate and one contact event.
   *
   * The contact anchor, object twist, object-relative grasp geometry and all
   * phase times are copied into this value object. Neither copied-state
   * preview nor runtime execution is allowed to rewrite them.
   */
  struct InterceptionPlan
  {
    bool valid = false;
    std::string candidateName = "none";
    // Immutable route-bank choice for the predictive reach. Zero offset is
    // the original direct path; non-zero offsets are generated symmetrically.
    std::string transitRouteName = "direct";
    Eigen::Vector3d reachCurveOffsetWorld = Eigen::Vector3d::Zero();

    // Observation-time mode is part of the immutable commitment. A static
    // presentation has zero committed twist and no deceleration phase; a
    // moving presentation preserves the validated V6.2 deceleration event.
    ObservedObjectMode objectMode = ObservedObjectMode::Unclassified;

    // The primary event is a cooperative presentation: the object finishes
    // a smooth deceleration at presentationTime and remains quasi-static
    // through approach, closure and confirmation. contactTime is only the
    // nominal bilateral-contact estimate used for feedforward and diagnostics.
    // A real contact is accepted anywhere inside the bounded acquisition
    // window [acquireStartTime, acquisitionDeadlineTime].
    bool presentationMode = false;
    double presentationTime = 0.0;
    double decelerationStartTime = 0.0;
    double decelerationDuration = 0.0;
    double contactTime = 0.0;
    double reachStartTime = 0.0;
    double standoffTime = 0.0;
    double acquireStartTime = 0.0;
    double acquisitionDeadlineTime = 0.0;
    double confirmationEndTime = 0.0;

    double reachDuration = 0.0;
    double approachDuration = 0.0;
    double acquireDuration = 0.0;
    double acquisitionWindowDuration = 0.0;
    double confirmationDuration = 0.0;
    double forceTransferDuration = 0.0;
    double forceTransferTimeout = 0.0;
    double forceTransferFullSupportForce = 0.0;
    double forceTransferReleaseThreshold = 0.0;
    double forceTransferReleaseRateTolerance = 0.0;
    double forceTransferReleaseDwell = 0.0;
    double forceTransferMaximumForce = 0.0;
    double forceTransferMaximumMoment = 0.0;
    double forceTransferAdmittanceMass = 0.0;
    double forceTransferAdmittanceDamping = 0.0;
    double forceTransferAdmittanceStiffness = 0.0;
    double forceTransferMaximumAdmittanceSpeed = 0.0;
    double forceTransferMaximumAdmittanceOffset = 0.0;
    double forceTransferVirtualContactStiffness = 0.0;
    double forceTransferVirtualContactDamping = 0.0;
    double forceTransferVirtualMaximumLoadScale = 0.0;
    std::string forceTransferSource = "disabled";
    double retreatDuration = 0.0;
    double contactClosure = 0.0;

    sva::PTransformd objectAtPresentation = sva::PTransformd::Identity();
    sva::PTransformd objectAtContact = sva::PTransformd::Identity();
    sva::PTransformd mouthAtReachStart = sva::PTransformd::Identity();
    sva::PTransformd O_T_M_standoff = sva::PTransformd::Identity();
    sva::PTransformd O_T_M_capture = sva::PTransformd::Identity();
    sva::PTransformd O_T_M_retreat = sva::PTransformd::Identity();

    Eigen::Vector3d objectLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d objectAngularVelocity = Eigen::Vector3d::Zero();

    // TRIAD V2 conditional presentation model. The object follows the
    // independent constant-twist prediction up to presentationTime and is
    // treated as presented at rest from then on: the certificate is "if the
    // object is presented at the predicted pose, the complete action is
    // feasible". There is no modelled deceleration (decelerationDuration = 0);
    // the premise is verified from measurements at the commit gate. Always
    // false in V1.
    bool conditionalPresentationV2 = false;
  };

  struct PredictiveReachPolicy
  {
    double maxLinearTrackingLead = 0.035;
    double maxAngularTrackingLead = 0.18;
    double nearLinearTrackingLead = 0.012;
    double nearAngularTrackingLead = 0.07;
    double clearanceSlowdownStart = 0.035;
    double clearanceHardMargin = 0.016;
    double minimumRuntimeClearance = 0.008;
    double minimumVelocityScale = 0.25;
    double clearanceScaleDropRate = 10.0;
    double clearanceScaleRiseRate = 4.0;
    double farLinearSpeed = 0.38;
    double nearLinearSpeed = 0.16;
    double farAngularSpeed = 1.65;
    double nearAngularSpeed = 0.75;
    double positionTolerance = 0.012;
    double orientationTolerance = 0.050;
    double taskStiffness = 58.0;
    double taskWeight = 6600.0;
    double launchTimingTolerance = 0.010;
    double scheduleLatenessTolerance = 0.25;
    double minimumScheduledDuration = 0.25;
    double maximumObjectTranslationDeviation = 0.015;
    double maximumObjectRotationDeviation = 0.12;
  };

  struct MovingReference
  {
    InterceptionPhase phase = InterceptionPhase::Reach;
    double absoluteTime = 0.0;
    double timeToContact = 0.0;
    double phaseProgress = 0.0;
    double gripperClosure = 0.0;

    sva::PTransformd objectPose = sva::PTransformd::Identity();
    Eigen::Vector3d objectLinearVelocityWorld = Eigen::Vector3d::Zero();
    Eigen::Vector3d objectAngularVelocityWorld = Eigen::Vector3d::Zero();
    sva::PTransformd mouthPose = sva::PTransformd::Identity();
    Eigen::Vector3d mouthLinearVelocityWorld = Eigen::Vector3d::Zero();
    Eigen::Vector3d mouthAngularVelocityWorld = Eigen::Vector3d::Zero();
  };

  /**
   * Contact-dwell/confirmation helper retained for internal certification.
   * Runtime capture and load takeover are executed by the single continuous
   * CaptureTransfer state.
   */
  struct ContactConfirmationState
  {
    bool confirming = false;
    bool confirmed = false;
    bool timedOut = false;
    double bilateralStableTime = 0.0;
    double confirmationStableTime = 0.0;
    double confirmationElapsed = 0.0;
  };

  struct ContactConfirmationTransition
  {
    bool enteredConfirming = false;
    bool becameConfirmed = false;
    bool timedOut = false;
  };

  /**
   * Force-aware transfer contract shared by runtime execution and logging.
   *
   * source may be:
   *   - virtual_sensor: deterministic ticker-only contact mechanics and
   *                     human/robot load sharing; simulation evidence only
   *   - synthetic: legacy open-loop signal retained for regression only
   *   - force_sensor: an mc_rtc ForceSensor on sourceRobot
   *   - disabled: measurement unavailable and transfer cannot be certified
   */
  struct ForceTransferPolicy
  {
    std::string source = "virtual_sensor";
    std::string sourceRobot;
    std::string sensorName;
    bool gravityCompensated = true;
    Eigen::Vector3d loadAxisObject = Eigen::Vector3d::UnitZ();

    double objectMass = 0.346;
    double fullSupportForce = 0.346 * 9.81;
    double filterTimeConstant = 0.08;
    double syntheticTransferDuration = 1.20;
    double desiredTransferDuration = 1.20;

    // Simulation-only contact mechanics. The robot-supported load is generated
    // by a virtual spring-damper driven by the actual committed admittance
    // displacement. Human support is the remaining object weight.
    double virtualContactStiffness = 800.0;
    double virtualContactDamping = 25.0;
    double virtualMaximumLoadScale = 1.20;

    double releaseThreshold = 0.85;
    double releaseRateTolerance = 0.08;
    double releaseDwell = 0.30;
    double transferTimeout = 5.00;
    double maximumForce = 80.0;
    double maximumMoment = 8.0;

    double admittanceMass = 1.0;
    double admittanceDamping = 35.0;
    double admittanceStiffness = 80.0;
    double maximumAdmittanceSpeed = 0.030;
    double maximumAdmittanceOffset = 0.015;
  };

  struct ForceTransferMeasurement
  {
    bool valid = false;
    bool synthetic = false;
    bool simulatedPhysics = false;
    Eigen::Vector3d forceWorld = Eigen::Vector3d::Zero();
    Eigen::Vector3d coupleWorld = Eigen::Vector3d::Zero();
    Eigen::Vector3d loadAxisWorld = Eigen::Vector3d::UnitZ();
    double loadForce = 0.0;
    double lateralForce = 0.0;
    double momentNorm = 0.0;
    double transferIndex = 0.0;
    double transferRate = 0.0;
    double humanSupportForce = 0.0;
    double virtualContactDeflection = 0.0;
  };

  HandoverInterceptionController(mc_rbdyn::RobotModulePtr rm,
                                 double dt,
                                 const mc_rtc::Configuration & config);

  bool run() override;
  void reset(const mc_control::ControllerResetData & reset_data) override;

  void activateToolTask();
  void deactivateToolTask();
  void setToolTaskGains(double stiffness, double weight);
  void setGripperJointPriority(bool highPriority);
  void setGripperJointPriority(double priorityBlend);

  /**
   * Hard runtime ownership of finger closure. Every state before
   * CaptureTransfer is open-only. CaptureTransfer, Retreat, Completed and
   * Failure may preserve or command closure. commandGripper() clamps any
   * unauthorized positive command to open.
   */
  void setGripperClosureAuthorized(bool authorized);
  bool gripperClosureAuthorized() const { return gripperClosureAuthorized_; }
  void commandArmPosture(const std::map<std::string, std::vector<double>> & target);
  std::map<std::string, std::vector<double>> interpolateArmPosture(
      const std::map<std::string, std::vector<double>> & from,
      const std::map<std::string, std::vector<double>> & to,
      double alpha) const;
  void commandReadyArmPosture();
  void commandReadyArmPosture(
      const std::map<std::string, std::vector<double>> & target);
  void commandSelectedStandoffPosture();
  void commandSelectedReachPosture(
      double progress,
      const std::map<std::string, std::vector<double>> & startPosture);
  void commandSelectedArmPosture();
  void commandSelectedRetreatPosture();
  const PredictiveReachPolicy & predictiveReachPolicy() const
  {
    return predictiveReachPolicy_;
  }
  std::map<std::string, std::vector<double>> currentArmPosture() const;
  void commandGripper(double close);
  void refreshPhysicalGripperBridge();
  bool physicalGripperBridgeEnabled() const { return physicalGripperBridgeEnabled_; }
  bool physicalGripperFeedbackValid() const { return physicalGripperFeedbackValid_; }
  bool physicalGripperCommandEnabled() const { return physicalGripperCommandEnabled_; }
  double physicalGripperMeasuredPercent() const
  {
    return physicalGripperMeasuredPercent_;
  }
  double physicalGripperMeasuredVelocityPercent() const
  {
    return physicalGripperMeasuredVelocityPercent_;
  }
  double physicalGripperOpenPercent() const
  {
    return physicalGripperOpenPercent_;
  }
  double physicalGripperClosePercent() const
  {
    return physicalGripperClosePercent_;
  }
  double physicalGripperMaxPercent() const
  {
    return physicalGripperMaxPercent_;
  }
  void commandBaseTarget(const sva::PTransformd & W_T_B_cmd);
  void commandMouthTarget(const sva::PTransformd & W_T_M_cmd);
  void commandBaseTargetWithWorldVelocity(
      const sva::PTransformd & W_T_B_cmd,
      const Eigen::Vector3d & linearVelocityWorld,
      const Eigen::Vector3d & angularVelocityWorld);
  void commandMouthTargetWithWorldVelocity(
      const sva::PTransformd & W_T_M_cmd,
      const Eigen::Vector3d & linearVelocityWorld,
      const Eigen::Vector3d & angularVelocityWorld);
  void clearToolTaskReferenceMotion();

  const ForceTransferPolicy & forceTransferPolicy() const
  {
    return forceTransferPolicy_;
  }
  const ForceTransferMeasurement & forceTransferMeasurement() const
  {
    return forceTransferMeasurement_;
  }
  void beginForceTransferBiasCalibration();
  void finishForceTransferBiasCalibration();
  void beginForceTransferExecution();
  void finishForceTransferExecution();
  void setVirtualForceTransferState(double contactDeflection,
                                    double contactVelocity,
                                    bool bilateralContact);
  bool forceTransferExecutionActive() const
  {
    return forceTransferExecutionActive_;
  }
  std::string forceTransferSourceDescription() const;

  void commitAcquiredMouthTarget(const sva::PTransformd & W_T_M_acquired)
  {
    W_T_M_acquired_ = W_T_M_acquired;
  }

  /**
   * Lock the already-selected object-relative terminal grasp to the measured
   * stopped presentation pose. This is a bounded local realization of the
   * immutable candidate, not a candidate change or a replan.
   */
  bool lockStaticPresentationToCurrentObject();

  const std::map<std::string, std::vector<double>> & readyPosture() const
  {
    return readyPosture_;
  }

  double controlDt() const { return controlDt_; }
  double gripperCommand() const { return gripperCommand_; }
  double measuredGripperClosure() const;
  double gripperCommandLead() const { return gripperCommandLead_; }
  double gripperNearContactCommandLead() const { return gripperNearContactCommandLead_; }
  double gripperContactClosureGuard() const { return gripperContactClosureGuard_; }
  double gripperCloseRate() const { return gripperCloseRate_; }
  double gripperTargetGap() const { return gripperTargetGap_; }
  double gripperMaxClosure() const { return gripperMaxClosure_; }
  double gripperMinimumCloseRate() const { return gripperMinimumCloseRate_; }
  double gripperSlowDistance() const { return gripperSlowDistance_; }
  double gripperContactTolerance() const { return gripperContactTolerance_; }
  double gripperPenetrationTolerance() const { return gripperPenetrationTolerance_; }
  double acquisitionCenterTolerance(double minPadClearance) const;
  double acquisitionFarCenterTolerance() const
  {
    return acquisitionFarCenterTolerance_;
  }
  double acquisitionNearCenterTolerance() const
  {
    return acquisitionNearCenterTolerance_;
  }
  double handleDiameter() const { return 2.0 * handleRadius_; }
  bool gripperActuationAvailable() const;
  std::string gripperActuationStatus() const;

  Eigen::Matrix3d worldRotation(const sva::PTransformd & X_0_F) const;
  sva::PTransformd fromWorldPose(const Eigen::Matrix3d & R_W_F,
                                 const Eigen::Vector3d & p_W_F) const;
  Eigen::Vector3d pointToWorld(const sva::PTransformd & W_T_F,
                               const Eigen::Vector3d & p_F) const;
  Eigen::Vector3d pointToLocal(const sva::PTransformd & W_T_F,
                               const Eigen::Vector3d & p_W) const;
  sva::PTransformd compose(const sva::PTransformd & W_T_A,
                           const sva::PTransformd & A_T_B) const;
  sva::PTransformd relativePose(const sva::PTransformd & W_T_A,
                                const sva::PTransformd & W_T_B) const;
  sva::PTransformd interpolatePose(const sva::PTransformd & A,
                                   const sva::PTransformd & B,
                                   double alpha) const;
  sva::PTransformd boundedPoseStep(const sva::PTransformd & current,
                                   const sva::PTransformd & goal,
                                   double maxTranslation,
                                   double maxRotation) const;
  sva::PTransformd advancePoseReference(const sva::PTransformd & reference,
                                        const sva::PTransformd & goal,
                                        double maxLinearSpeed,
                                        double maxAngularSpeed) const;
  void worldPoseTwist(const sva::PTransformd & from,
                      const sva::PTransformd & to,
                      double dt,
                      Eigen::Vector3d & linearVelocityWorld,
                      Eigen::Vector3d & angularVelocityWorld) const;
  double orientationError(const sva::PTransformd & A,
                          const sva::PTransformd & B) const;

  // ---------------------------------------------------------------------------
  // Simulated giver truth model.
  //   committed_plan        V1 history: after commitment the simulated object
  //                         follows the committed presentation time.
  //   independent_scripted  The object follows a robot-independent script
  //                         (IndependentGiverModel.h). No receiver plan, event
  //                         time, grasp or route can reach the simulated truth.
  // ---------------------------------------------------------------------------
  bool independentGiverTruth() const
  {
    return giverTruthModel_ == "independent_scripted";
  }
  const std::string & giverTruthModel() const { return giverTruthModel_; }
  /** Receiver architecture: v1_frozen_prereach (default) or v2_receding. */
  bool receiverArchitectureV2() const
  {
    return receiverArchitecture_ == "v2_receding";
  }
  const std::string & receiverArchitecture() const { return receiverArchitecture_; }
  bool receiverArchitectureConfigurationValid() const
  {
    return receiverArchitectureConfigurationValid_;
  }
  const call_handover::IndependentGiverScript & independentGiverScript() const
  {
    return independentGiverScript_;
  }

  void refreshObjectPose();
  void recordObjectPerceptionTruthSample(const sva::PTransformd & truthPose);
  void selectDelayedObjectMeasurement();
  void applyObjectPerceptionEstimate();
  void resetObjectPerceptionBuffer();
  void refreshSelectedWorldTargets();
  void beginObjectObservation();
  void endObjectObservation(bool freezeSimulatedObject = true);
  bool objectObservationActive() const { return objectObservationActive_; }
  bool simulatedObjectMotionEnabled() const { return simulateMovingObject_; }
  bool objectMotionEstimateValid() const { return objectMotionEstimateValid_; }
  void setObservedObjectMode(ObservedObjectMode mode)
  {
    observedObjectMode_ = mode;
    if(mode == ObservedObjectMode::Static)
    {
      // Commit the stationary model, not residual estimator noise.
      objectLinearVelocityEstimate_.setZero();
      objectAngularVelocityEstimate_.setZero();
      W_T_O_predicted_ = W_T_O_;
    }
  }
  ObservedObjectMode observedObjectMode() const { return observedObjectMode_; }
  bool staticObjectModeSelected() const
  {
    return observedObjectMode_ == ObservedObjectMode::Static;
  }
  bool movingObjectModeSelected() const
  {
    return observedObjectMode_ == ObservedObjectMode::Moving;
  }
  const char * observedObjectModeName() const
  {
    switch(observedObjectMode_)
    {
      case ObservedObjectMode::Static: return "STATIC";
      case ObservedObjectMode::Moving: return "MOVING";
      default: return "UNCLASSIFIED";
    }
  }
  double objectObservationElapsed() const;
  double objectObservationDuration() const { return objectObservationDuration_; }
  double objectPredictionHorizon() const { return objectPredictionHorizon_; }
  bool perceptionLatencyEnabled() const
  {
    return perceptionLatencyEnabled_ && perceptionLatencySeconds_ > 0.0;
  }
  bool perceptionLatencyCompensationEnabled() const
  {
    return perceptionLatencyEnabled()
        && perceptionLatencyCompensationEnabled_;
  }
  double perceptionLatencySeconds() const
  {
    return perceptionLatencyEnabled() ? perceptionLatencySeconds_ : 0.0;
  }
  double objectPerceptionMeasurementAge() const
  {
    return objectPerceptionMeasurementAge_;
  }
  double objectPerceptionRawPositionError() const
  {
    return (W_T_O_perceptionMeasurement_.translation()
          - W_T_O_truth_.translation()).norm();
  }
  double objectPerceptionEstimatePositionError() const
  {
    return (W_T_O_.translation() - W_T_O_truth_.translation()).norm();
  }
  const char * perceptionLatencyModeName() const
  {
    if(!perceptionLatencyEnabled()) { return "IDEAL"; }
    return perceptionLatencyCompensationEnabled_
        ? "DELAYED_COMPENSATED" : "DELAYED_UNCOMPENSATED";
  }
  int objectObservationSamples() const { return objectObservationSamples_; }
  double objectObservationDisplacement() const;
  const Eigen::Vector3d & objectLinearVelocityEstimate() const
  {
    return objectLinearVelocityEstimate_;
  }
  const Eigen::Vector3d & objectAngularVelocityEstimate() const
  {
    return objectAngularVelocityEstimate_;
  }
  const Eigen::Vector3d & simulatedObjectLinearVelocity() const
  {
    return simulatedObjectLinearVelocity_;
  }
  const Eigen::Vector3d & simulatedObjectAngularVelocity() const
  {
    return simulatedObjectAngularVelocity_;
  }
  const sva::PTransformd & predictedObjectPose() const
  {
    return W_T_O_predicted_;
  }
  sva::PTransformd predictObjectPose(double horizon) const;
  sva::PTransformd predictPresentationPose(double horizon) const;
  bool presentationModeEnabled() const { return presentationMode_; }
  double presentationDecelerationDuration() const
  {
    return presentationDecelerationDuration_;
  }
  double presentationAcquisitionWindow() const
  {
    return presentationAcquisitionWindow_;
  }
  double presentationMaximumLinearSpeed() const
  {
    return presentationMaximumLinearSpeed_;
  }
  double presentationMaximumAngularSpeed() const
  {
    return presentationMaximumAngularSpeed_;
  }
  bool presentationCaptureAdmitted(
      double positionError,
      double orientationError,
      const HandoverSafetyReport & report,
      double objectLinearSpeed,
      double objectAngularSpeed) const;
  bool calibrateMouthControlFrame();
  bool refreshGripperGeometry(bool verbose = true);

  sva::PTransformd actualBasePose() const;
  sva::PTransformd actualMouthPose() const;
  sva::PTransformd mouthPoseFromBasePose(const sva::PTransformd & W_T_B) const;
  sva::PTransformd basePoseFromMouthPose(const sva::PTransformd & W_T_M) const;
  /**
   * Pose-dependent half of basePoseFromMouthPose(). B_T_M is a property of the
   * live robot, not of W_T_M, so a caller evaluating many mouth poses against
   * one unchanged robot state may resolve it once and pass it here. The
   * expression below is character-for-character the tail of
   * basePoseFromMouthPose(), so both routes produce identical results.
   */
  sva::PTransformd basePoseFromMouthPoseWith(
      const sva::PTransformd & W_T_M,
      const sva::PTransformd & B_T_M) const;
  /** Resolve the live mouth->base transform used by basePoseFromMouthPose(). */
  sva::PTransformd liveMouthToBaseTransform() const;
  /** Mouth-to-base transform derived from a frozen robot state.
   *
   * The copied-state counterpart of liveMouthToBaseTransform(): identical
   * construction, parameterized on an explicit MultiBodyConfig instead of the
   * live robot. Deriving it from the same frozen state s0 as the rest of the
   * search is what makes every event hypothesis use one common value, instead
   * of resampling the QP solution's cycle-to-cycle jitter per hypothesis.
   */
  bool frozenMouthToBaseTransform(const rbd::MultiBodyConfig & mbc,
                                  sva::PTransformd & B_T_M) const;
  /**
   * Pose-dependent half of interpolatePose(). The two endpoint quaternions and
   * their relative-sign resolution depend only on the endpoints, so a caller
   * sweeping one segment may resolve them once and pass them here.
   */
  sva::PTransformd interpolatePoseWith(
      const Eigen::Vector3d & pA,
      const Eigen::Vector3d & pB,
      const Eigen::Quaterniond & qA,
      const Eigen::Quaterniond & qB,
      double alpha) const;

  double liveMouthGap() const;
  double liveMouthHalfGap() const { return 0.5 * liveMouthGap(); }

  bool prepareCaptureSelection();

  /** The bounded event bank, frozen at the decision epoch.
   *
   * The leads and their presentation poses are generated once, before any
   * geometry runs, and never regenerated. Order is the deterministic
   * enumeration order and is significant. minimumSafeCommitLead is carried here
   * rather than recomputed so the search never needs the deceleration model at
   * step time.
   */
  struct FrozenEventBank
  {
    double searchEpoch = 0.0;
    std::vector<double> leads;
    std::vector<sva::PTransformd> presentationPoses;
    std::size_t configuredHypotheses = 0;
    int maximumHypotheses = 0;
    double maximumSearchWallTime = 0.0;
    double minimumSafeCommitLead = 0.0;
    std::string source = "global_fixed_schedule";
  };

  enum class FiniteSearchStatus
  {
    Running,
    Complete,
    Failed
  };

  /** Progress of one complete finite TRIAD search over the frozen bank.
   *
   * This is the orchestration the SolveInterception state used to carry: which
   * hypothesis is being evaluated, how far the bank has advanced, and the
   * counters the evidence logs report. Holding it here is what makes the whole
   * search callable without the FSM.
   */
  struct FiniteTriadSearchState
  {
    FrozenEventBank bank;
    std::size_t cursor = 0;
    bool hypothesisActive = false;
    int evaluatedHypotheses = 0;
    int feasibleHypotheses = 0;
    int geometryFailures = 0;
    double currentLead = 0.0;
    double currentPresentationTime = 0.0;
    sva::PTransformd currentPresentationPose = sva::PTransformd::Identity();
    std::string failureReason;
    bool active = false;
  };

  /** Complete finite TRIAD search over one frozen event bank.
   *
   * beginFiniteTriadSearch() takes the frozen bank; stepFiniteTriadSearch()
   * advances the search by one bounded unit of work and returns Running until
   * the bank is exhausted. The admission instant is an explicit input rather
   * than a clock read, so the same search can be driven by the control thread
   * against its own clock (reference mode, which reproduces the existing
   * elapsed-time hypothesis pruning exactly) or, later, by a worker with the
   * pruning disabled. Nothing inside reads the controller clock.
   */
  void beginFiniteTriadSearch(const FrozenEventBank & bank);
  FiniteSearchStatus stepFiniteTriadSearch(double admissionNow,
                                           int previewSteps,
                                           int routeWorkUnits);
  const FiniteTriadSearchState & finiteSearchState() const
  {
    return finiteSearch_;
  }
  /** Controller-side preparation that must happen before the frozen search.
   * These are the mutations beginCapturePlanning() used to perform once per
   * hypothesis; none of them belongs inside a relocatable search. */
  void prepareCapturePlanningSession();
  CapturePlanningStatus beginCapturePlanningCore();

  CapturePlanningStatus beginCapturePlanning(bool commitOnSuccess = true);
  CapturePlanningStatus stepCapturePlanning(int maxInternalSteps,
                                            int routeWorkUnits);
  CapturePlanningStatus capturePlanningStatus() const { return capturePlanningStatus_; }

  // V4A.2 candidate/time interception solve. A future object pose is injected
  // only into copied-state rollout; the real robot and visible object continue
  // to evolve independently. The winning complete candidate is committed once
  // only after the contact-time fixed point converges.
  void setPlanningObjectSnapshot(const sva::PTransformd & W_T_O_snapshot);
  void applyPlanningObjectSnapshot();
  void clearPlanningObjectSnapshot();
  bool planningBestCandidateAvailable() const { return plannerContext_.planningFoundFeasible; }
  const std::string & planningBestCandidateName() const
  {
    return plannerContext_.planningBestCandidate.name;
  }
  double planningBestPredictedPresentationTime() const
  {
    return plannerContext_.planningBestCandidate.predictedPresentationTime;
  }
  double planningBestPredictedContactTime() const
  {
    return plannerContext_.planningBestCandidate.predictedContactTime;
  }
  double planningBestPredictedExecutionTime() const
  {
    return plannerContext_.planningBestCandidate.estimatedTime;
  }
  double planningBestClearance() const
  {
    return plannerContext_.planningBestCandidate.minClearance;
  }
  double planningBestScore() const { return plannerContext_.planningBestCandidate.score; }
  /**
   * Finalize the plan selector before a commit. A negative
   * remainingToPresentation disables the moving-event timing filter (used for
   * the measured static event). In binding_cost mode this performs the exact
   * finite-set argmin and fails closed on any invalid complete-plan cost.
   */
  bool selectPlanningBestForCommit(
      double remainingToPresentation = -1.0,
      double minimumReachEntryLead = 0.0,
      double minimumSafeCommitLead = 0.0);
  bool globalTimePlanSelectionEnabled() const
  {
    return completeEventSelectionMode_ == "global_time_plan";
  }
  /** Reset the deferred finite set at the common moving-event search epoch.
   * When globalTimePlanModeActive is true, this also freezes a single copy
   * of the current robot().mbc() (the complete MultiBodyConfig) as the
   * common preview decision-state for every candidate and route evaluated
   * during this search, so results no longer depend on which live robot
   * state happened to exist when a given candidate's turn arrived. */
  void resetGlobalTimePlanSearch(double searchEpoch,
                                  bool globalTimePlanModeActive = false);
  /**
   * Copy every complete hard-feasible plan from the current event into the
   * deferred global set. No robot command is issued by this operation.
   */
  bool captureCurrentEventPlanAlternatives(
      std::size_t hypothesisIndex,
      double eventLeadFromSearchEpoch,
      double eventPresentationTime,
      const sva::PTransformd & W_T_O_presentation);
  /**
   * Exhaustively minimize over every captured (time, grasp, route) record
   * after reapplying the final timing gate at the end of the bounded scan.
   */
  bool selectGlobalTimePlanForCommit(
      double now,
      double minimumReachEntryLead,
      double minimumSafeCommitLead,
      std::size_t evaluatedHypotheses,
      std::size_t configuredHypotheses,
      bool scheduleComplete);
  /** Commit the already-proven global winner exactly once. */
  bool commitGlobalTimePlanSelection();
  double selectedGlobalEventLead() const
  {
    return selectedGlobalEventLead_;
  }
  double selectedGlobalObjectiveCost() const
  {
    return planningSelectedObjectiveCost_;
  }
  double selectedGlobalMotionCost() const
  {
    return selectedGlobalMotionCost_;
  }
  double selectedGlobalScheduleWait() const
  {
    return selectedGlobalScheduleWait_;
  }
  const std::string & planningSelectionReason() const
  {
    return planningCostSelectionReason_;
  }
  bool commitPlanningBestAsInterception(
      double committedPresentationTime,
      double timingResidual,
      const sva::PTransformd & W_T_O_presentation);
  const InterceptionPlan & committedInterceptionPlan() const
  {
    return committedInterceptionPlan_;
  }
  bool committedPlanValid() const
  {
    return interceptionCommitted_ && committedInterceptionPlan_.valid;
  }
  MovingReference interceptionReferenceAt(
      const InterceptionPlan & plan,
      double absoluteTime,
      double sampleDt) const;
  ContactConfirmationTransition advanceContactConfirmation(
      ContactConfirmationState & state,
      bool validContactEvent,
      bool bilateralContactInsideTube,
      double dt,
      double bilateralDwell,
      double confirmationDwell,
      double confirmationTimeout) const;
  MovingReference committedReferenceAt(
      double absoluteTime,
      double sampleDt,
      bool useBoundedLiveCorrection = false,
      double maxTranslationCorrection = 0.0,
      double maxRotationCorrection = 0.0) const;
  bool validateInterceptionPlan(
      const InterceptionPlan & plan,
      std::string * reason = nullptr,
      bool verbose = false) const;
  bool interceptionCommitted() const { return interceptionCommitted_; }
  double controllerTime() const { return controllerTime_; }
  double committedPresentationTime() const
  {
    return committedPlanValid() ? committedInterceptionPlan_.presentationTime : 0.0;
  }
  double committedContactTime() const { return committedContactTime_; }
  double committedAcquisitionDeadline() const
  {
    return committedPlanValid()
        ? committedInterceptionPlan_.acquisitionDeadlineTime : 0.0;
  }
  double timeToCommittedAcquisitionDeadline() const
  {
    return committedPlanValid()
        ? committedInterceptionPlan_.acquisitionDeadlineTime - controllerTime_
        : 0.0;
  }
  double timeToCommittedContact() const
  {
    return interceptionCommitted_ ? committedContactTime_ - controllerTime_ : 0.0;
  }
  double committedTimingResidual() const { return committedTimingResidual_; }
  const sva::PTransformd & committedObjectContactPose() const
  {
    return W_T_O_committedContact_;
  }
  const Eigen::Vector3d & committedObjectLinearVelocity() const
  {
    return committedObjectLinearVelocity_;
  }
  const Eigen::Vector3d & committedObjectAngularVelocity() const
  {
    return committedObjectAngularVelocity_;
  }
  sva::PTransformd committedObjectPoseAt(double absoluteTime) const;
  sva::PTransformd committedObjectPoseWithBoundedCorrection(
      double absoluteTime,
      double maxTranslationCorrection,
      double maxRotationCorrection) const;
  sva::PTransformd committedStandoffTargetAt(double absoluteTime) const;
  sva::PTransformd committedCaptureTargetAt(double absoluteTime) const;
  sva::PTransformd committedApproachTargetAt(double absoluteTime,
                                             double relativeProgress,
                                             bool useBoundedLiveCorrection,
                                             double maxTranslationCorrection,
                                             double maxRotationCorrection) const;
  double committedObjectPositionErrorAt(double absoluteTime) const;
  double committedObjectOrientationErrorAt(double absoluteTime) const;

  bool filterSafeMouthCommand(const sva::PTransformd & W_T_M_current,
                              const sva::PTransformd & W_T_M_proposed,
                              sva::PTransformd & W_T_M_safe,
                              HandoverSafetyReport & report,
                              bool requireCorridor);

  bool evaluateCurrentPoseSafety(HandoverSafetyReport & report,
                                 bool requireCorridor);
  bool evaluateAttachedRetreatSafety(HandoverSafetyReport & report) const;
  bool filterSafeAttachedRetreatCommand(const sva::PTransformd & W_T_M_current,
                                        const sva::PTransformd & W_T_M_proposed,
                                        sva::PTransformd & W_T_M_safe,
                                        HandoverSafetyReport & report) const;
  bool evaluateCurrentClosureSafety(
      HandoverSafetyReport & report,
      bool enforceCentering = true,
      bool allowDesignatedPadContact = false) const;
  bool bilateralPadContactReached(
      HandoverSafetyReport * report = nullptr,
      bool allowDesignatedPadContact = false) const;

  bool hasSelectedCandidate() const { return candidateSelected_; }
  void invalidateSelectedCandidate();

  const sva::PTransformd & currentMouthTransitTarget() const { return W_T_M_transit_; }
  const sva::PTransformd & currentMouthStandoffTarget() const { return W_T_M_standoff_; }
  const sva::PTransformd & currentMouthPregraspTarget() const { return W_T_M_pre_; }
  const sva::PTransformd & currentMouthAcquiredTarget() const { return W_T_M_acquired_; }
  const sva::PTransformd & currentMouthRetreatTarget() const { return W_T_M_retreat_; }
  const sva::PTransformd & committedRelativeStandoffTarget() const { return O_T_M_standoff_; }
  const sva::PTransformd & committedRelativeCaptureTarget() const { return O_T_M_pre_; }
  const sva::PTransformd & objectPose() const { return W_T_O_; }
  const sva::PTransformd & blueHandlePose() const { return W_T_H_; }
  const std::string & toolFrame() const { return toolFrame_; }
  const std::string & selectedCandidateName() const { return selectedCandidateName_; }
  double selectedCandidateClearance() const { return selectedCandidateClearance_; }
  double selectedCandidateScore() const { return selectedCandidateScore_; }
  double selectedCandidatePredictedTime() const { return selectedCandidatePredictedTime_; }
  double selectedCandidatePredictedPresentationTime() const
  {
    return selectedCandidatePredictedPresentationTime_;
  }
  double selectedCandidatePredictedContactTime() const { return selectedCandidatePredictedContactTime_; }
  double selectedCandidatePredictedReachTime() const { return selectedCandidatePredictedReachTime_; }
  double selectedCandidatePredictedApproachTime() const { return selectedCandidatePredictedApproachTime_; }
  double selectedCandidatePredictedAcquireTime() const { return selectedCandidatePredictedAcquireTime_; }
  double selectedCandidatePredictedEffort() const { return selectedCandidatePredictedEffort_; }
  double selectedCandidateContactClosure() const { return selectedCandidateContactClosure_; }

  bool attachObjectToMouth();
  void detachObject();
  bool objectAttached() const { return objectAttached_; }
  bool geometricGripReached() const;

  // Exact controller-time instrumentation used to calibrate interception
  // timing. These measurements do not change the physical controller.
  void resetPhaseTiming();
  void startPhaseTiming(const std::string & phase);
  void finishPhaseTiming(const std::string & phase);
  double phaseDuration(const std::string & phase) const;
  void logPhaseTimingSummary() const;

public:
  // ===========================================================================
  // TRIAD V2: receding complete-action receiver control (src/ReceiverV2.cpp).
  //
  // Before terminal commitment the receiver executes an ACTIVE PROVISIONAL
  // plan while the object moves; the plan is re-certified from the current
  // state and newest prediction, retained while certified and replaced only
  // when it becomes infeasible or invalid. Provisional plans never authorize
  // gripper closure. At the terminal standoff/capture boundary a final
  // current-state check and a fresh terminal certificate promote the active
  // plan to committedInterceptionPlan_ exactly once; afterwards no global
  // reselection is possible and the existing capture/transfer/retreat states
  // run unchanged.
  // ===========================================================================
  enum class ReceiverStepStatusV2
  {
    Running,
    Committed,
    Failed
  };
  bool beginReceiverV2(const mc_rtc::Configuration & stateConfig);
  ReceiverStepStatusV2 stepReceiverV2();
  void endReceiverV2();
  bool receiverV2CommitLatched() const { return v2CommitLatched_; }
  int receiverV2CommitCount() const { return v2CommitCount_; }

public:
  std::shared_ptr<mc_tasks::TransformTask> toolTask_;

  // Integrated Robot-B giver coordination (two-robot setup). Robot A keeps the
  // original observe/evaluate/commit-once/execute-once FSM unchanged.
public:
  bool dualGiverEnabled() const;
  bool dualGiverReady() const;
  bool dualGiverFailed() const;
  bool startDualGiverPresentation();
  void commandDualGiverSafeHold();
  bool dualGiverPresentationScheduleAvailable() const;
  double dualGiverTimeToPresentation() const;
  sva::PTransformd dualGiverPresentationPoseWorld() const;
  std::unique_ptr<DualGiverCoordinator> dualGiver_;


private:
  struct GripperSample
  {
    std::string name;
    std::string frame;
    Eigen::Vector3d p_F = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_B = Eigen::Vector3d::Zero();
    double radius = 0.0;
    bool hardAgainstBlue = true;
  };

  /**
   * One rigid node of the exact clearance-acceleration hierarchy.
   *
   * Every gripper proxy is expressed in the same fixed base frame (p_B), so a
   * node's enclosing sphere is a constant of the proxy set: its radius in the
   * base frame is invariant under the rigid map to world. The node therefore
   * yields a rigorous lower bound on the clearance of every proxy it contains,
   * for one obstacle, from a single transformed centre.
   *
   * The hierarchy is an acceleration structure only. It never produces a
   * reported value; it only proves that a scalar clearance evaluation cannot
   * change the result and may therefore be skipped. Every value written into a
   * HandoverSafetyReport still comes from the original scalar expression.
   */
  struct GripperProxyNode
  {
    Eigen::Vector3d centre_B = Eigen::Vector3d::Zero();
    double radius = 0.0;      // max ||p_B - centre_B|| over contained proxies
    double maxProxyRadius = 0.0;
    int count = 0;
  };

  static constexpr std::size_t maximumProxyNodes_ = 32;

  /**
   * Compact hot-path mirror of one gripper proxy: exactly the fields the inner
   * clearance loop reads, laid out contiguously. GripperSample carries two
   * std::string members and is ~128 bytes, of which the arithmetic needs 32;
   * iterating it streams four times the memory the computation uses and defeats
   * any contiguous access. This mirror is built once, alongside the proxy
   * hierarchy, and is never the source of truth - gripperSamplesB_ remains
   * authoritative and supplies the names once a limiter has been identified.
   */
  struct GripperProxyHot
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double radius = 0.0;
  };

  // Canonical obstacle identifiers for the inner loop. Index order is the
  // frozen evaluation order and must not be reordered; the strings are only
  // materialised once, after the numeric limiter for a query is final.
  enum ClearanceObstacle
  {
    ObstacleGroundPlane = 0,
    ObstacleSensorCore = 1,
    ObstacleHumanHandle = 2,
    ObstacleBlueHandle = 3
  };
  static const char * clearanceObstacleName(int obstacle);

  struct CaptureCandidate
  {
    std::string name;
    std::string transitRouteName = "direct";
    Eigen::Vector3d reachCurveOffsetWorld = Eigen::Vector3d::Zero();
    double transitPathLength = 0.0;
    sva::PTransformd W_T_M_transit = sva::PTransformd::Identity();
    sva::PTransformd W_T_M_standoff = sva::PTransformd::Identity();
    sva::PTransformd W_T_M_pre = sva::PTransformd::Identity();
    sva::PTransformd W_T_M_retreat = sva::PTransformd::Identity();
    double minClearance = -1e9;
    // Phase-specific non-contact reserve during the predictive reach. This is
    // intentionally separated from the terminal closure clearance, where the
    // pads are expected to approach contact.
    double predictiveReachClearance = -1e9;
    double score = 1e9;
    double estimatedTime = 1e9;
    double predictedPresentationTime = 1e9;
    double predictedContactTime = 1e9;
    double predictedReachTime = 1e9;
    double predictedApproachTime = 1e9;
    double predictedAcquireTime = 1e9;
    double rawRolloutTime = 1e9;
    double predictedArmTime = 1e9;
    double predictedClosureTime = 1e9;
    double predictedFixedTime = 1e9;
    double predictedEffort = 1e9;
    double rotation = 1e9;
    double verticalComponent = 0.0;
    double contactClosure = -1.0;

    // Copied terminal timing approximation. It remains outside the hard
    // feasibility contract, but its accepted duration contributes to the cost
    // and can therefore affect selection in binding_cost mode.
    bool terminalTimingAuditRan = false;
    bool terminalTimingAuditSuccess = false;
    // Once the copied terminal controller succeeds, its measured duration is
    // allowed to replace prediction fields. Hard feasibility and the protected
    // heuristic retain their legacy inputs; binding_cost uses the updated
    // prediction through completeCostAudit.
    bool terminalTimingPredictionBound = false;
    double legacyPredictedApproachTime = 1e9;
    double legacyPredictedContactTime = 1e9;
    double legacyEstimatedTime = 1e9;
    double terminalTimingAuditDuration = 1e9;
    double terminalTimingFinalPositionError = 1e9;
    double terminalTimingFinalOrientationError = 1e9;
    double terminalTimingFinalLinearSpeed = 1e9;
    double terminalTimingFinalAngularSpeed = 1e9;
    double terminalTimingFinalStableTime = 0.0;
    std::string terminalTimingAuditReason = "not_run";
    double auditPredictedContactTime = 1e9;
    double auditEstimatedTime = 1e9;

    // Dimensionless complete-action cost. Hard feasibility remains
    // authoritative; these terms are evaluated only for complete routes and
    // become the binding finite-set objective in binding_cost mode.
    // The velocity reserve is a bounded rho^4 soft preference on [0, 1].
    bool completeCostAuditValid = false;
    double predictiveRetreatClearance = -1e9;
    double minimumJointMarginRatio = 1.0;
    double minimumConditionIndex = 1.0;
    double maximumJointVelocityUtilization = 0.0;
    double terminalVelocityUtilization = 1e9;
    double costTime = 0.0;
    double costEffort = 0.0;
    double costPath = 0.0;
    double costRotation = 0.0;
    double costClearance = 0.0;
    double costJointMargin = 0.0;
    double costConditioning = 0.0;
    double costVelocityReserve = 0.0;
    double completeCostAudit = 1e9;

    std::map<std::string, std::vector<double>> plannedTransitArmPosture;
    std::map<std::string, std::vector<double>> plannedStandoffArmPosture;
    std::map<std::string, std::vector<double>> plannedArmPosture;
    std::map<std::string, std::vector<double>> plannedRetreatArmPosture;
    bool previewFeasible = false;
    std::string failureReason = "not_previewed";
  };

  struct GlobalEventPlanAlternative
  {
    CaptureCandidate candidate;
    std::size_t hypothesisIndex = 0;
    double eventLeadFromSearchEpoch = 0.0;
    double eventPresentationTime = 0.0;
    double scheduleWaitBeforeReach = 0.0;
    double predictedSearchToCompletionTime = 0.0;
    double globalObjectiveCost = 1e9;
    sva::PTransformd planningStartMouthPose = sva::PTransformd::Identity();
    sva::PTransformd W_T_O_presentation = sva::PTransformd::Identity();
  };




  /** Deterministic output of one complete frozen finite TRIAD search.
   *
   * This is the exact hand-off the planner produces from one frozen decision
   * epoch: every hard-feasible, cost-valid alternative in source order, with
   * the frozen poses each was certified against. It deliberately carries no
   * receipt-time timing admission - remaining, admissibility and the argmin are
   * applied by the control thread against the clock at result receipt, which is
   * what keeps the decision reproducible from the set alone.
   */
  struct FrozenPlanSet
  {
    double searchEpoch = 0.0;
    std::size_t evaluatedHypotheses = 0;
    std::size_t configuredHypotheses = 0;
    bool scheduleComplete = false;
    // Source order is the deterministic enumeration order and is significant:
    // it is the tie-break of last resort in the finite selector.
    std::vector<GlobalEventPlanAlternative> alternatives;
  };

public:
  /** One background planning job.
   *
   * Exactly one worker, exactly one in-flight request, one result per planning
   * generation. The control thread never blocks on it: it polls an atomic state
   * and, on Ready, reads a result the worker finished writing before it
   * published. A single result buffer is the smallest correct arrangement here,
   * because there is never more than one job in flight and the control thread
   * reads the buffer only after an acquire load paired with the worker's
   * release store.
   */
  enum class PlannerJobState
  {
    Idle = 0,
    Running = 1,
    Ready = 2,
    Failed = 3
  };

  ~HandoverInterceptionController() override;

  void submitFiniteTriadSearch(const FrozenEventBank & bank,
                               double requestControllerTime,
                               int previewSteps,
                               int routeWorkUnits);
  PlannerJobState plannerJobState() const
  {
    return static_cast<PlannerJobState>(
        plannerJobState_.load(std::memory_order_acquire));
  }
  const FrozenPlanSet & plannerResult() const { return plannerResult_; }
  const std::string & plannerFailureReason() const
  {
    return plannerFailureReason_;
  }
  std::uint64_t plannerRequestGeneration() const
  {
    return plannerRequestGeneration_.load(std::memory_order_acquire);
  }
  std::uint64_t plannerResultGeneration() const
  {
    return plannerResultGeneration_.load(std::memory_order_acquire);
  }
  double plannerWorkerWallDuration() const { return plannerWorkerWallDuration_; }
  double plannerRequestControllerTime() const
  {
    return plannerRequestControllerTime_;
  }
  /** Cancel and join. Never called from the control callback. */
  void shutdownPlannerWorker();
  /** Development-only: hold a completed result back for N control cycles, so a
   * late delivery is exercised without touching any timing parameter. */
  void setPlannerPublishDelayCycles(int cycles)
  {
    plannerPublishDelayCycles_ = std::max(0, cycles);
  }
  int plannerPublishDelayCycles() const { return plannerPublishDelayCycles_; }
  int & plannerPublishHeldCycles() { return plannerPublishHeldCycles_; }
  /** Development-only: bump the request generation after submitting, so the
   * worker's result carries a generation the control thread no longer expects.
   * The rejection path is otherwise unreachable, because submitting joins the
   * previous worker before starting a new one. */
  void setPlannerForceStaleGeneration(bool force)
  {
    plannerForceStaleGeneration_ = force;
  }
  /** Verbose per-record planner evidence. Enabled by default; suppressing it
   * removes the high-volume streams (one group of records per complete plan)
   * without touching any computation, selection or commit behaviour. */
  void setPlannerEvidenceLogging(bool enabled)
  {
    plannerEvidenceLogging_ = enabled;
  }
  bool plannerEvidenceLogging() const { return plannerEvidenceLogging_; }
  /** Development-only: make the next worker job throw. */
  void setPlannerInjectFailure(bool inject) { plannerInjectFailure_ = inject; }

private:
  std::thread plannerThread_;
  std::atomic<int> plannerJobState_{0};
  std::atomic<std::uint64_t> plannerRequestGeneration_{0};
  std::atomic<std::uint64_t> plannerResultGeneration_{0};
  std::atomic<bool> plannerCancel_{false};
  bool v2SelectThenCertify_ = false;
  // Exact timing prune: the control thread publishes its clock; the worker uses
  // it as a lower bound on any later selection time.
  bool v2ExactTimingPrune_ = false;
  std::atomic<double> v2ControllerClockForWorker_{-1.0e300};
  long long workUnitsV2() const;
  FrozenPlanSet plannerResult_;
  std::string plannerFailureReason_;
  double plannerRequestControllerTime_ = 0.0;
  double plannerWorkerWallDuration_ = 0.0;
  std::chrono::steady_clock::time_point plannerWorkerStartWall_;
  int plannerPublishDelayCycles_ = 0;
  int plannerPublishHeldCycles_ = 0;
  bool plannerInjectFailure_ = false;
  bool plannerForceStaleGeneration_ = false;
  bool plannerEvidenceLogging_ = true;
  void runPlannerWorker(std::uint64_t generation, int previewSteps,
                        int routeWorkUnits);


  struct PreviewResult
  {
    bool feasible = false;
    double minClearance = 1e9;
    // Raw copied-state rollout time. It is useful for kinematic feasibility,
    // but it is not yet a truthful physical execution time.
    double duration = 0.0;
    double reachStandoffDuration = 0.0;
    double reachCaptureDuration = 0.0;
    double closureDuration = 0.0;
    double retreatDuration = 0.0;
    double effort = 0.0;
    double minimumJointMarginRatio = 1.0;
    double minimumConditionIndex = 1.0;
    double maximumJointVelocityUtilization = 0.0;
    double contactClosure = -1.0;
    std::string reason = "unknown";
    std::string limitingSample = "none";
    std::string limitingObstacle = "none";
  };

  enum class PreviewStepStatus
  {
    Running,
    Succeeded,
    Failed
  };

  // Phases and outcomes of a resumable predictive route rollout. See the
  // rollout state block below for the suspension contract.
  enum class RouteStepPhase
  {
    Idle,
    Reach,
    Approach,
    Dwell,
    Closure,
    Retreat,
    Finalize
  };
  enum class RouteStepOutcome
  {
    Running,
    Feasible,
    Infeasible
  };

  enum class PlanningPhase
  {
    ReachStandoff,
    ReachCapture,
    Closure,
    Retreat
  };

  void loadHandoverConfig(const mc_rtc::Configuration & config);
  void addMethodologyGui();

  Eigen::Matrix3d rpyToWorldRotation(const Eigen::Vector3d & rpy) const;
  sva::PTransformd makePose(const Eigen::Vector3d & translation,
                            const Eigen::Vector3d & rpy) const;

  bool sweptGripperPoseSafe(const sva::PTransformd & W_T_M_from,
                            const sva::PTransformd & W_T_M_to,
                            HandoverSafetyReport & report,
                            bool requireCorridor) const;
  /** Swept clearance on an explicit mouth-to-base transform.
   *
   * The runtime entry point above resolves that transform from the live robot
   * (liveMouthToBaseTransform()). The copied-state planner must not: it passes
   * the transform frozen with the rest of the decision state, so the swept
   * certification depends only on the snapshot. Both entry points evaluate the
   * identical per-pose expressions.
   */
  bool sweptGripperPoseSafeWith(const sva::PTransformd & W_T_M_from,
                                const sva::PTransformd & W_T_M_to,
                                const sva::PTransformd & B_T_M,
                                const sva::PTransformd & W_T_O,
                                const sva::PTransformd & W_T_H,
                                HandoverSafetyReport & report,
                                bool requireCorridor) const;

  bool gripperPoseSafe(const sva::PTransformd & W_T_M,
                       HandoverSafetyReport & report,
                       bool requireCorridor) const;

  bool gripperBasePoseSafe(const sva::PTransformd & W_T_B,
                           const sva::PTransformd & W_T_M,
                           HandoverSafetyReport & report,
                           bool requireCorridor) const;
  /** Clearance against an explicit object/handle world, so the planner never
   * reads the controller's live world members. */
  bool gripperBasePoseSafeWith(const sva::PTransformd & W_T_B,
                               const sva::PTransformd & W_T_M,
                               const sva::PTransformd & W_T_O,
                               const sva::PTransformd & W_T_H,
                               HandoverSafetyReport & report,
                               bool requireCorridor) const;

  bool graspCorridorSafe(const sva::PTransformd & W_T_M,
                         HandoverSafetyReport & report) const;
  bool graspCorridorSafeWith(const sva::PTransformd & W_T_M,
                             const sva::PTransformd & W_T_O,
                             const sva::PTransformd & W_T_H,
                             HandoverSafetyReport & report) const;

  bool wholeRobotGroundSafe(HandoverSafetyReport & report) const;
  bool wholeRobotGroundSafe(const rbd::MultiBodyConfig & mbc,
                            HandoverSafetyReport & report) const;

  bool previewCandidate(CaptureCandidate & candidate,
                        const sva::PTransformd & W_T_M_actual);
  PreviewStepStatus previewReachStep(
      rbd::MultiBodyConfig & mbc,
      const sva::PTransformd & W_T_M_goal,
      bool requireCorridor,
      bool attachedRetreat,
      int & segmentIteration,
      PreviewResult & result,
      const Eigen::Vector3d & linearFeedforwardWorld = Eigen::Vector3d::Zero(),
      const Eigen::Vector3d & angularFeedforwardWorld = Eigen::Vector3d::Zero(),
      bool allowConvergence = true,
      const std::map<std::string, std::vector<double>> * postureTarget = nullptr,
      bool collectDecisionMetrics = false) const;
  bool previewReachSegment(rbd::MultiBodyConfig & mbc,
                           const sva::PTransformd & W_T_M_goal,
                           bool requireCorridor,
                           bool attachedRetreat,
                           PreviewResult & result,
                           bool collectDecisionMetrics = false) const;
  bool previewTerminalCaptureDwellStep(rbd::MultiBodyConfig & mbc,
                                      PreviewResult & result) const;
  bool previewTerminalCaptureDwell(rbd::MultiBodyConfig & mbc,
                                   double duration,
                                   PreviewResult & result) const;
  bool previewVelocityGateParityShadow(
      rbd::MultiBodyConfig mbc,
      const CaptureCandidate & candidate,
      double & duration,
      double & finalPositionError,
      double & finalOrientationError,
      double & finalLinearSpeed,
      double & finalAngularSpeed,
      double & finalStableTime,
      std::string & reason,
      bool verbose = false) const;
  PreviewStepStatus previewClosureStep(rbd::MultiBodyConfig & mbc,
                                       int & closureIndex,
                                       PreviewResult & result) const;
  bool previewClosureSweep(rbd::MultiBodyConfig & mbc,
                           PreviewResult & result) const;
  bool verifyPredictiveStaticCandidate(
      CaptureCandidate & candidate,
      const PreviewResult & staticResult,
      const sva::PTransformd & W_T_O_contact);
  bool verifyPredictiveRouteCandidate(
      CaptureCandidate & candidate,
      const PreviewResult & staticResult,
      const sva::PTransformd & W_T_O_presentation,
      const std::string & routeName,
      const Eigen::Vector3d & curveOffsetWorld);
  RouteStepOutcome beginPredictiveRouteCandidate(
      const CaptureCandidate & candidate,
      const PreviewResult & staticResult,
      const sva::PTransformd & W_T_O_presentation,
      const std::string & routeName,
      const Eigen::Vector3d & curveOffsetWorld);
  RouteStepOutcome stepPredictiveRouteCandidate(int workUnits);
  RouteStepOutcome finalizePredictiveRouteCandidate();
  RouteStepOutcome routeStepFail(const std::string & reason);
  void routeStepRestorePlanningWorld();
  void routeStepSetVirtualObject(const sva::PTransformd & W_T_O_virtual);
  std::vector<std::pair<std::string, Eigen::Vector3d>> transitRouteBank(
      const sva::PTransformd & start,
      const sva::PTransformd & standoff) const;
  sva::PTransformd reachCurvePose(
      const sva::PTransformd & start,
      const sva::PTransformd & standoff,
      const Eigen::Vector3d & curveOffsetWorld,
      double progress) const;
  double reachCurveLength(
      const sva::PTransformd & start,
      const sva::PTransformd & standoff,
      const Eigen::Vector3d & curveOffsetWorld) const;
  InterceptionPlan makeInterceptionPlan(
      const CaptureCandidate & candidate,
      const sva::PTransformd & W_T_O_presentation,
      double presentationTime,
      double reachDuration,
      double approachDuration,
      double acquireDuration,
      double retreatDuration) const;
  sva::PTransformd interceptionObjectPoseAt(
      const InterceptionPlan & plan,
      double absoluteTime) const;
  void interceptionObjectTwistAt(
      const InterceptionPlan & plan,
      double absoluteTime,
      Eigen::Vector3d & linearVelocityWorld,
      Eigen::Vector3d & angularVelocityWorld) const;
  sva::PTransformd interceptionMouthPoseAt(
      const InterceptionPlan & plan,
      double absoluteTime) const;
  sva::PTransformd propagatePoseConstantTwist(
      const sva::PTransformd & referencePose,
      double dt,
      const Eigen::Vector3d & linearVelocityWorld,
      const Eigen::Vector3d & angularVelocityWorld) const;
  bool previewConfigurationSafe(const rbd::MultiBodyConfig & mbc,
                                const sva::PTransformd & W_T_M,
                                bool requireCorridor,
                                HandoverSafetyReport & report) const;
  bool previewJointLimitsSafe(const rbd::MultiBodyConfig & mbc,
                              std::string & reason) const;
  bool previewPadCenters(const rbd::MultiBodyConfig & mbc,
                         Eigen::Vector3d & pL_W,
                         Eigen::Vector3d & pR_W) const;
  /** Copied-state kinematics for the scientific preview.
   *
   * These report failure instead of falling back to live robot state. The
   * planner is a copied-state certification model: if it cannot resolve its
   * own tool body or pad frames from the model and the supplied
   * MultiBodyConfig, the correct behaviour is a deterministic planner failure,
   * not a silent substitution of whatever pose the live robot happens to hold.
   * The removed fallbacks (actualBasePose() in previewBasePose(), and
   * livePadCenters()/actualBasePose()/actualMouthPose() reached through
   * mouthPoseFromBasePose() in previewMouthPose()) were the only live-state
   * reads on the preview path.
   */
  bool previewBasePose(const rbd::MultiBodyConfig & mbc,
                       sva::PTransformd & W_T_B) const;
  bool previewMouthPose(const rbd::MultiBodyConfig & mbc,
                        sva::PTransformd & W_T_M) const;
  bool previewBasePoseFromMouthPose(
      const sva::PTransformd & W_T_M,
      const rbd::MultiBodyConfig & mbc,
      sva::PTransformd & W_T_B) const;
  void setPreviewGripperClosure(rbd::MultiBodyConfig & mbc,
                                double closure) const;
  bool previewDynamicClosureSafety(const rbd::MultiBodyConfig & mbc,
                                   HandoverSafetyReport & report,
                                   bool allowDesignatedPadContact = false) const;
  bool previewAttachedRetreatSafe(const rbd::MultiBodyConfig & mbc,
                                  const sva::PTransformd & W_T_O_carried,
                                  HandoverSafetyReport & report) const;
  bool carriedObjectGroundSafe(const sva::PTransformd & W_T_O_carried,
                               HandoverSafetyReport & report) const;
  bool carriedObjectArmSafe(const rbd::MultiBodyConfig & mbc,
                            const sva::PTransformd & W_T_O_carried,
                            HandoverSafetyReport & report) const;
  bool carriedObjectArmSafe(const sva::PTransformd & W_T_O_carried,
                            HandoverSafetyReport & report) const;
  std::map<std::string, std::vector<double>> armPostureFromMbc(
      const rbd::MultiBodyConfig & mbc) const;
  bool sampleWorldPoint(const GripperSample & sample,
                        const rbd::MultiBodyConfig & mbc,
                        Eigen::Vector3d & pW) const;

  bool startNextPlanningCandidate();
  void updateAttachedObjectPose();
  void updateSimulatedObjectMotion();
  void updateObjectMotionEstimate();
  void updateForceTransferMeasurement();
  // Returns true when transit-route certification is still pending, i.e. the
  // candidate has not been finalised yet and the planner must resume it.
  bool finishCurrentPlanningCandidate(bool feasible);
  CapturePlanningStatus finalizeCapturePlanning();
  double predictedExecutionTime(const PreviewResult & result) const;
  void computeCompletePlanAuditCost(CaptureCandidate & candidate) const;
  double normalizedUpperBarrier(double value, double hard, double soft) const;
  double normalizedLowerBarrier(double value, double soft) const;
  double predictedContactTime(const PreviewResult & result) const;
  void commitCandidate(const CaptureCandidate & best,
                       const sva::PTransformd & W_T_O_reference,
                       bool asInterception,
                       double committedPresentationTime,
                       double timingResidual);

  double pointSegmentDistance(const Eigen::Vector3d & p,
                              const Eigen::Vector3d & a,
                              const Eigen::Vector3d & b) const;

  CaptureCandidate buildCandidate(double angleRad,
                                  double axisSign,
                                  const sva::PTransformd & W_T_M_actual,
                                  const Eigen::Vector3d & baseOutward) const;

  Eigen::Vector3d objectAxis() const;
  /** Object/handle axes taken from an explicit world pose, so the planner can
   * use its own world while the runtime keeps the controller's. */
  Eigen::Vector3d objectAxisFrom(const sva::PTransformd & W_T_O) const;
  Eigen::Vector3d plannerObjectAxis() const
  {
    return objectAxisFrom(plannerContext_.W_T_O);
  }
  Eigen::Vector3d plannerHandleAxis() const { return -plannerObjectAxis(); }
  void applyPlanningObjectSnapshotToPlanner();
  Eigen::Vector3d blueHandleAxis() const;

  void updateWorstClearance(HandoverSafetyReport & report,
                            double clearance,
                            const std::string & sample,
                            const std::string & obstacle) const;

  bool livePadCenters(Eigen::Vector3d & pL_W, Eigen::Vector3d & pR_W) const;
  bool hasJoint(const std::string & jointName) const;
  bool hasActuatedJoint(const std::string & jointName) const;

private:
  double controlDt_ = 0.001;

  std::string toolFrame_ = "gen3_robotiq_85_base_link";
  std::string objectRobotName_ = "call_object";
  std::string objectFrameName_ = "call_object";
  bool toolTaskActive_ = false;

  double taskStiffness_ = 4.0;
  double taskWeight_ = 1800.0;

  std::map<std::string, std::vector<double>> readyPosture_;

  sva::PTransformd W_T_O_config_ = sva::PTransformd::Identity();
  sva::PTransformd W_T_O_ = sva::PTransformd::Identity();
  sva::PTransformd O_T_H_ = sva::PTransformd::Identity();
  sva::PTransformd W_T_H_ = sva::PTransformd::Identity();

  sva::PTransformd B_T_M_config_ = sva::PTransformd::Identity();
  sva::PTransformd B_T_M_control_ = sva::PTransformd::Identity();
  bool mouthCalibrationValid_ = false;

  double sensorRadius_ = 0.017;
  double sensorHalfLength_ = 0.0182;
  double handleRadius_ = 0.01125;
  double handleHalfLength_ = 0.0687;
  double gripperSafetyMargin_ = 0.003;

  bool groundEnabled_ = true;
  double groundZ_ = 0.0;
  double groundSafetyMargin_ = 0.015;
  double armGroundSafetyMargin_ = 0.010;

  int candidateCount_ = 12;
  double candidateStandoffDistance_ = 0.120;
  double candidateRetreatDistance_ = 0.180;
  double candidateClearanceWeight_ = 10.0;
  double candidateTravelWeight_ = 1.0;
  double candidateRotationWeight_ = 0.20;
  double candidateClearanceTieBand_ = 0.002;
  double candidateTransitSpeed_ = 0.30;
  double candidateAngularSpeed_ = 1.00;
  Eigen::Vector3d worldUp_ = Eigen::Vector3d::UnitZ();

  // Internal kinematic-rollout planner parameters.
  double previewDt_ = 0.010;
  int previewMaxIterationsPerSegment_ = 1400;
  double previewLinearGain_ = 3.5;
  double previewAngularGain_ = 3.5;
  double previewDamping_ = 0.04;
  double previewPostureGain_ = 0.35;
  double previewJointLimitWeightGain_ = 0.8;
  double previewJointLimitAvoidanceGain_ = 0.45;
  double previewJointLimitActivation_ = 0.65;
  double previewJointLimitVelocityCap_ = 0.55;
  double previewMaxLinearSpeed_ = 0.40;
  double previewMaxAngularSpeed_ = 1.20;
  double previewPositionTolerance_ = 0.010;
  double previewOrientationTolerance_ = 0.055;
  double previewEffortWeight_ = 0.025;
  double previewRotationWeight_ = 0.08;
  double previewClearanceTieBand_ = 0.003;
  double previewCostTieBand_ = 0.15;

  // Dimensionless complete-action cost. In protected_heuristic mode it is
  // diagnostic; in binding_cost mode its finite-set argmin is the only plan
  // allowed to reach commitCandidate(). The physical references make unlike
  // units comparable and the non-negative weights are normalized to one.
  std::string completePlanSelectionMode_ = "protected_heuristic";
  // V6.6 retains first_admissible_center_out. V6.7 global_time_plan freezes a
  // bounded event set at one epoch and minimizes over time, grasp and route.
  std::string completeEventSelectionMode_ =
      "first_admissible_center_out";
  double decisionCostTieTolerance_ = 1e-9;
  bool decisionBindingPhysicalExecutionAuthorized_ = false;
  bool decisionCostConfigurationValid_ = true;
  double decisionTimeReference_ = 8.0;
  double decisionEffortReference_ = 8.0;
  double decisionPathReference_ = 0.50;
  double decisionCharacteristicLength_ = 0.20;
  double decisionSoftClearance_ = 0.080;
  double decisionSoftJointMargin_ = 0.20;
  double decisionSoftConditionIndex_ = 0.10;
  // Frozen seven-term binding preference objective (T,E,L,C,Q,K,V; sum ~1).
  // decisionRotationWeight_ is retained only for configuration provenance
  // and diagnostics; it defaults to 0.0 so it cannot perturb the shared
  // weight-normalization sum below, and computeCompletePlanAuditCost() no
  // longer references it in the binding cost at all (R is diagnostic-only).
  double decisionTimeWeight_ = 0.4210526;
  double decisionEffortWeight_ = 0.1052632;
  double decisionPathWeight_ = 0.1052632;
  double decisionRotationWeight_ = 0.0;
  double decisionClearanceWeight_ = 0.1578947;
  double decisionJointMarginWeight_ = 0.0842105;
  double decisionConditioningWeight_ = 0.0736842;
  double decisionVelocityReserveWeight_ = 0.0526316;
  int decisionMetricStride_ = 10;

  double previewJointLimitMargin_ = 0.015;
  int previewClosureSamples_ = 100;
  bool previewMovingInterception_ = true;
  // Phase-specific acceptance contracts must match the runtime states.
  double previewMovingReachPositionTolerance_ = 0.012;
  double previewMovingReachOrientationTolerance_ = 0.050;
  // The copied-state predictive reach must certify at least the same
  // non-contact reserve enforced by the runtime reach governor.
  double previewMovingReachMinimumClearance_ = 0.008;
  double previewMovingPositionTolerance_ = 0.004;
  double previewMovingOrientationTolerance_ = 0.030;
  double previewMovingCenteringTolerance_ = 0.00065;
  double previewMovingMaximumRelativeLinearSpeed_ = 0.040;
  double previewMovingMaximumRelativeAngularSpeed_ = 0.30;
  double previewMovingContactTimingTolerance_ = 0.25;
  double previewMovingMaximumContactLateness_ = 0.35;
  int previewMovingMaximumScheduleIterations_ = 3;
  double previewMovingScheduleGrowth_ = 1.20;

  // Symmetric route bank: directions are sampled uniformly around the direct
  // reach chord and are never assigned an above/below/side preference.
  bool transitPlanningEnabled_ = true;
  int transitRouteDirections_ = 8;
  std::vector<double> transitRouteApexOffsets_ = {0.08, 0.14};
  // Phase 5 characterization: direct routes with only the reach duration
  // stretched (no spatial detour). Empty (default) adds nothing.
  std::vector<double> transitDirectTimeStretch_;
  double transitMinimumPredictedClearance_ = 0.020;
  double transitClearancePreferenceBand_ = 0.010;
  double transitMaximumPathStretch_ = 1.90;
  int transitCurveLengthSamples_ = 32;

  PredictiveReachPolicy predictiveReachPolicy_;

  // V4A.0 execution-time model. The arm scale and effective measured
  // gripper rate are calibrated from the V3A.10 static runs. The fixed
  // acquisition overhead mirrors the executed blend/lock/dwell phases.
  double timingArmScale_ = 1.50;
  double timingEffectiveGripperRate_ = 0.120;
  double timingPriorityBlend_ = 0.40;
  double timingCaptureLock_ = 0.25;
  double timingTerminalCaptureDwell_ = 0.08;
  double timingBilateralDwell_ = 0.12;
  double timingConfirmationDwell_ = 0.25;
  double timingConfirmationTimeout_ = 2.0;

  // V4A.1 moving-object observation and constant-twist prediction. The
  // physical robot remains stationary during this phase. In ticker, an
  // optional deterministic object motion generator provides a visible and
  // repeatable observation source. V4A.2 leaves this motion active after
  // observation while candidate-specific future contact events are solved.
  bool simulateMovingObject_ = true;
  // Timestamped perception-delay ablation. This delays only object-pose
  // measurements; it never sleeps or slows the 1 kHz control thread. The
  // candidate/event horizon remains unchanged. Compensation propagates the
  // delayed sample by its measured age before future-event prediction.
  bool perceptionLatencyEnabled_ = false;
  bool perceptionLatencyCompensationEnabled_ = true;
  double perceptionLatencySeconds_ = 0.220;
  double perceptionLatencyBufferDuration_ = 1.0;
  double objectObservationDuration_ = 3.0;
  double objectMinimumObservationTime_ = 1.0;
  int objectMinimumObservationSamples_ = 500;
  double objectPredictionHorizon_ = 1.5;
  double objectVelocityFilterTimeConstant_ = 0.15;
  double objectMaximumLinearSpeed_ = 0.30;
  double objectMaximumAngularSpeed_ = 1.50;
  double objectMaximumSimulatedTravel_ = 0.30;
  Eigen::Vector3d simulatedObjectLinearVelocity_ =
      Eigen::Vector3d(0.0, 0.020, 0.0);
  Eigen::Vector3d simulatedObjectAngularVelocity_ = Eigen::Vector3d::Zero();

  // V5 predictive presentation with proven static acquisition. The giver
  // approaches with the observed twist, smoothly decelerates, and presents
  // the object. The robot reaches the future standoff, waits for a verified
  // stop, then executes the validated open-gripper static insertion/closure.
  bool presentationMode_ = true;
  double presentationDecelerationDuration_ = 1.50;
  double presentationAcquisitionWindow_ = 10.00;
  double presentationCapturePositionTolerance_ = 0.012;
  double presentationCaptureOrientationTolerance_ = 0.030;
  double presentationCaptureCenterTolerance_ = 0.00065;
  double presentationMaximumLinearSpeed_ = 0.004;
  double presentationMaximumAngularSpeed_ = 0.08;

  int sweepSamples_ = 20;
  int backtrackIterations_ = 9;

  double corridorSafetyMargin_ = 0.003;
  double corridorFingerInset_ = 0.010;
  double corridorMaxAngleRad_ = 0.17453292519943295;
  double corridorEntryDepth_ = 0.135;
  double corridorPalmLimit_ = 0.004;
  double corridorAxialTolerance_ = 0.035;
  double calibratedOpenHalfGap_ = 0.067;

  Eigen::Vector3d leftPadPointTip_ = Eigen::Vector3d(-0.0252580, -0.000715, 0.01360);
  Eigen::Vector3d rightPadPointTip_ = Eigen::Vector3d(0.0252580, -0.000035, 0.01360);
  double captureDepth_ = 0.006;

  double gripperOpenQ_ = 0.0;
  double gripperCloseQ_ = 0.8;
  double gripperCloseRate_ = 0.35;
  double gripperTargetGap_ = 0.026;
  double gripperMaxClosure_ = 1.00;
  double gripperCommandLead_ = 0.008;
  double gripperNearContactCommandLead_ = 0.012;
  double gripperContactClosureGuard_ = 0.08;
  double gripperMinimumCloseRate_ = 0.025;
  double gripperSlowDistance_ = 0.012;
  double gripperContactTolerance_ = 0.0015;
  double gripperPenetrationTolerance_ = 0.0005;
  double padCenteringTolerance_ = 0.004;

  // Physical closure authority. False by default and reset on controller
  // reset. This makes early finger closing impossible even if a future state
  // accidentally requests it.
  bool gripperClosureAuthorized_ = false;
  bool gripperAuthorityViolationLogged_ = false;

  // Shared acquisition tube used by both internal closure preview and physical
  // Acquire execution. The allowed bilateral centering error tightens as the
  // pads approach the handle.
  double acquisitionFarCenterTolerance_ = 0.0015;
  double acquisitionNearCenterTolerance_ = 0.00065;
  double acquisitionCenterTightenDistance_ = 0.010;
  double gripperCommand_ = 0.0;

  // V6.4 hardware bridge. Simulation keeps this disabled and uses the same
  // actuated Robotiq model as before. Hardware enables it explicitly; the
  // physical command remains separately gated for staged bring-up.
  bool physicalGripperBridgeEnabled_ = false;
  bool physicalGripperCommandEnabled_ = false;
  bool physicalGripperRequireFeedback_ = true;
  // Calibrated physical feedback endpoints are distinct from the independent
  // command safety cap. The Robotiq reports a small non-zero value when open.
  double physicalGripperOpenPercent_ = 0.87;
  double physicalGripperClosePercent_ = 35.0;
  double physicalGripperMaxPercent_ = 35.0;
  bool physicalGripperFeedbackValid_ = false;
  double physicalGripperMeasuredPercent_ = 0.0;
  double physicalGripperMeasuredVelocityPercent_ = 0.0;
  uint64_t physicalGripperFeedbackSequence_ = 0;
  bool physicalGripperFeedbackWarningLogged_ = false;

  double gripperJointWeightNormal_ = 25.0;
  double gripperJointWeightHigh_ = 1000.0;
  double gripperPostureStiffnessNormal_ = 0.5;
  double gripperPostureStiffnessHigh_ = 6.0;
  double gripperPostureWeight_ = 2.0;

  double readyPostureStiffness_ = 0.8;
  double readyPostureWeight_ = 800.0;

  std::vector<GripperSample> gripperSamplesB_;
  // Exact clearance-acceleration hierarchy over gripperSamplesB_. Rebuilt only
  // by refreshGripperGeometry(); never reconstructed inside a query.
  std::vector<GripperProxyNode> gripperProxyNodes_;
  std::vector<int> gripperSampleNode_;   // sample index -> node index
  // Contiguous hot-path mirrors of gripperSamplesB_, same indexing, rebuilt
  // only by refreshGripperGeometry() via rebuildGripperProxyHierarchy().
  std::vector<GripperProxyHot> gripperProxyHot_;
  std::vector<unsigned char> gripperProxyHardBlue_;
  void rebuildGripperProxyHierarchy();

  // Resumable transit-route certification.
  //
  // The 17-route bank used to be certified as one atomic burst inside a single
  // planner step, so one control cycle was charged the whole bank. The bank is
  // now advanced one route per planner step. Route generation, route order,
  // per-route certification, the audit-record insertion order, the best-route
  // fold and its comparison arithmetic are unchanged; only the point at which
  // the fold may be suspended is new.
  //
  // Suspension is safe at a route boundary because verifyPredictiveRouteCandidate
  // is world-state neutral across a complete call: every return path either
  // precedes the first world mutation or restores W_T_O_, W_T_H_ and
  // planningM_T_O_ before returning.
  //
  // W_T_O_ advances between control cycles, so the presentation anchor the
  // original code captured once per bank is snapshotted here to keep every
  // route in a bank evaluated against the identical object pose.
  /** Immutable planner configuration, mirrored from the controller.
   *
   * Every configuration value, calibrated geometry constant and fixed threshold
   * the scientific search reads. It is a mirror rather than the primary copy
   * because some of it is calibrated at runtime - the gripper samples, the
   * collision proxy hierarchy and the mouth control frame - and must track those
   * updates. refreshPlannerConfig() is called wherever a source changes and
   * again at every search freeze, so the planner and the runtime always observe
   * identical values. A worker snapshots this once and never reads the
   * controller again while it runs.
   */
  struct PlannerConfig
  {
    double acquisitionCenterTightenDistance = 0.010;
    double acquisitionFarCenterTolerance = 0.0015;
    double acquisitionNearCenterTolerance = 0.00065;
    double armGroundSafetyMargin = 0.010;
    int candidateCount = 12;
    double candidateRetreatDistance = 0.180;
    double candidateStandoffDistance = 0.120;
    double captureDepth = 0.006;
    std::string completeEventSelectionMode = "first_admissible_center_out";
    std::string completePlanSelectionMode = "protected_heuristic";
    double corridorAxialTolerance = 0.035;
    double corridorEntryDepth = 0.135;
    double corridorFingerInset = 0.010;
    double corridorMaxAngleRad = 0.17453292519943295;
    double corridorPalmLimit = 0.004;
    double corridorSafetyMargin = 0.003;
    double decisionCharacteristicLength = 0.20;
    double decisionClearanceWeight = 0.1578947;
    double decisionConditioningWeight = 0.0736842;
    bool decisionCostConfigurationValid = true;
    double decisionCostTieTolerance = 1e-9;
    double decisionEffortReference = 8.0;
    double decisionEffortWeight = 0.1052632;
    double decisionJointMarginWeight = 0.0842105;
    int decisionMetricStride = 10;
    double decisionPathReference = 0.50;
    double decisionPathWeight = 0.1052632;
    double decisionSoftClearance = 0.080;
    double decisionSoftConditionIndex = 0.10;
    double decisionSoftJointMargin = 0.20;
    double decisionTimeReference = 8.0;
    double decisionTimeWeight = 0.4210526;
    double decisionVelocityReserveWeight = 0.0526316;
    ForceTransferPolicy forceTransferPolicy;
    double gripperCloseQ = 0.8;
    double gripperCloseRate = 0.35;
    double gripperContactTolerance = 0.0015;
    bool gripperGeometryValid = false;
    double gripperMaxClosure = 1.00;
    double gripperOpenQ = 0.0;
    double gripperPenetrationTolerance = 0.0005;
    std::vector<unsigned char> gripperProxyHardBlue;
    std::vector<GripperProxyHot> gripperProxyHot;
    std::vector<GripperProxyNode> gripperProxyNodes;
    double gripperSafetyMargin = 0.003;
    std::vector<int> gripperSampleNode;
    std::vector<GripperSample> gripperSamplesB;
    bool groundEnabled = true;
    double groundSafetyMargin = 0.015;
    double groundZ = 0.0;
    double handleHalfLength = 0.0687;
    double handleRadius = 0.01125;
    Eigen::Vector3d leftPadPointTip = Eigen::Vector3d(-0.0252580, -0.000715, 0.01360);
    bool mouthCalibrationValid = false;
    std::string objectFrameName = "call_object";
    std::string objectRobotName = "call_object";
    double padCenteringTolerance = 0.004;
    double perceptionLatencyBufferDuration = 1.0;
    double perceptionLatencySeconds = 0.220;
    bool physicalGripperCommandEnabled = false;
    PredictiveReachPolicy predictiveReachPolicy;
    double presentationAcquisitionWindow = 10.00;
    double presentationDecelerationDuration = 1.50;
    double previewAngularGain = 3.5;
    int previewClosureSamples = 100;
    double previewDamping = 0.04;
    double previewDt = 0.010;
    double previewEffortWeight = 0.025;
    double previewJointLimitActivation = 0.65;
    double previewJointLimitAvoidanceGain = 0.45;
    double previewJointLimitMargin = 0.015;
    double previewJointLimitVelocityCap = 0.55;
    double previewJointLimitWeightGain = 0.8;
    double previewLinearGain = 3.5;
    double previewMaxAngularSpeed = 1.20;
    int previewMaxIterationsPerSegment = 1400;
    double previewMaxLinearSpeed = 0.40;
    double previewOrientationTolerance = 0.055;
    double previewPositionTolerance = 0.010;
    double previewPostureGain = 0.35;
    double previewRotationWeight = 0.08;
    std::map<std::string, std::vector<double>> readyPosture;
    Eigen::Vector3d rightPadPointTip = Eigen::Vector3d(0.0252580, -0.000035, 0.01360);
    double sensorHalfLength = 0.0182;
    double sensorRadius = 0.017;
    bool simulateMovingObject = true;
    int sweepSamples = 20;
    double timingArmScale = 1.50;
    double timingBilateralDwell = 0.12;
    double timingCaptureLock = 0.25;
    double timingConfirmationDwell = 0.25;
    double timingConfirmationTimeout = 2.0;
    double timingEffectiveGripperRate = 0.120;
    double timingPriorityBlend = 0.40;
    double timingTerminalCaptureDwell = 0.08;
    std::string toolFrame = "gen3_robotiq_85_base_link";
    double transitClearancePreferenceBand = 0.010;
    int transitCurveLengthSamples = 32;
    double transitMaximumPathStretch = 1.90;
    double transitMinimumPredictedClearance = 0.020;
    bool transitPlanningEnabled = true;
    std::vector<double> transitRouteApexOffsets = {0.08, 0.14};
    std::vector<double> transitDirectTimeStretch;
    int transitRouteDirections = 8;
    Eigen::Vector3d worldUp = Eigen::Vector3d::UnitZ();

    // Worker snapshot of values that are otherwise read from the live robot or
    // the live estimator. They are copied on the control thread at every
    // refresh (freeze) and read by planner code only while it executes on the
    // background worker thread (see plannerWorkerThreadActive()). Control-
    // thread callers keep their original live reads, so V1 behaviour is
    // unchanged whenever the live value equals the frozen one.
    double frozenMouthHalfGap = 0.0;
    std::vector<std::vector<double>> jointPositionLower;
    std::vector<std::vector<double>> jointPositionUpper;
    std::vector<std::vector<double>> jointVelocityLower;
    std::vector<std::vector<double>> jointVelocityUpper;
    ObservedObjectMode observedObjectMode = ObservedObjectMode::Unclassified;
    Eigen::Vector3d objectLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d objectAngularVelocity = Eigen::Vector3d::Zero();
    bool objectMotionEstimateValid = false;
    bool presentationMode = true;
    bool previewMovingInterception = true;
    bool conditionalPresentationV2 = false;
    // TRIAD V2 exact timing prune of route certification (FULL_SEARCH only).
    bool v2ExactTimingPrune = false;
    double v2PruneCommitLead = 0.0;
    double v2PruneEntryLead = 0.0;
  };

  // ---------------------------- TRIAD V2 state ------------------------------
  enum class ReceiverJobTypeV2
  {
    None,
    FullSearch,
    RecertifyActive,
    TerminalCertify,
    CertifySelected,
    // TRIAD-lite (supervisorMode: control_aware): evaluate the receiving grasp
    // family from the frozen current state and the current object estimate.
    ControlAwareSelect
  };
  static const char * receiverJobTypeNameV2(ReceiverJobTypeV2 type);

  enum class ReceiverPhaseV2
  {
    Idle,
    SearchHeld,
    ProvisionalReach,
    TerminalTrack,
    Committed,
    Failed,
    // TRIAD-lite: continuously track the selected object-relative standoff;
    // acquisition entry is a measured gate, not a searched event time.
    ControlAwareTrack
  };
  static const char * receiverPhaseNameV2(ReceiverPhaseV2 phase);

  /** Abstract object-prediction record consumed by V2 planning. The default
   * source is constant-twist extrapolation of the estimator, with no stop
   * assumption; any predictor producing poseAt() can replace it. */
  struct ObjectPredictionRecordV2
  {
    bool valid = false;
    std::uint64_t generation = 0;
    double stamp = 0.0;
    double measurementAge = 0.0;
    sva::PTransformd pose = sva::PTransformd::Identity();
    Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
  };
  sva::PTransformd predictionPoseAtV2(const ObjectPredictionRecordV2 & record,
                                      double absoluteTime) const;
  ObjectPredictionRecordV2 currentObjectPredictionV2() const;

  /** Provisional receiver plan: may drive the robot, may be updated or
   * replaced before the terminal boundary, never authorizes closure and is
   * never a commitment. Owned by the control thread. */
  struct ProvisionalReceiverPlanV2
  {
    bool valid = false;
    std::uint64_t planId = 0;
    std::uint64_t sourcePlanningGeneration = 0;
    std::uint64_t adoptedStateGeneration = 0;
    CaptureCandidate candidate;
    InterceptionPlan plan;
    std::size_t hypothesisIndex = 0;
    double eventLead = 0.0;
    double globalCost = 1e9;
    double adoptedTime = 0.0;
    int certifications = 0;
    std::map<std::string, std::vector<double>> holdPosture;
  };

  struct ReceiverJobRequestV2
  {
    ReceiverJobTypeV2 type = ReceiverJobTypeV2::None;
    std::uint64_t planningGeneration = 0;
    std::uint64_t stateGeneration = 0;
    std::uint64_t planId = 0;
    double snapshotTime = 0.0;
    ObjectPredictionRecordV2 prediction;
    CaptureCandidate candidate;
    InterceptionPlan plan;
    sva::PTransformd snapshotMouthPose = sva::PTransformd::Identity();
    sva::PTransformd terminalObjectPose = sva::PTransformd::Identity();
    // Phase E receding interception: incumbent grasp (always resolved) and the
    // executed command/reference state AT snapshotTime (not latency shifted).
    int incumbentGraspId = -1;
    bool interceptionStartValid = false;
    sva::PTransformd interceptionStartPose = sva::PTransformd::Identity();
    Eigen::Vector3d interceptionStartLinearVelocity = Eigen::Vector3d::Zero();
    double interceptionStartClearanceScale = 1.0;
    // Work units per cancellation check in the worker rollout. The rollout is
    // invariant to this suspension granularity (see stepPredictiveRouteCandidate).
    int routeWorkUnits = 128;
  };

  // Phase 2B preview/runtime parity instrumentation (logging only). One sample
  // of the copied-state rollout of a certification job: phase 0 reach, 1
  // insertion, 3 closure, 4 carried retreat; t is plan time for the reach and
  // cumulative preview duration of the phase result otherwise.
  struct ParitySampleV2
  {
    int phase = 0;
    double t = 0.0;
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    double clearance = std::numeric_limits<double>::quiet_NaN();
    // reach phase only: timed interception reference, rate-limited reference
    // and bounded command positions used by this step
    Eigen::Vector3d reference = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d rateLimited = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d command = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    double clearanceScale = std::numeric_limits<double>::quiet_NaN();
    double jointMarginRatio = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> jointQ;
    std::vector<double> postureTarget;
  };

  /** TRIAD-lite: layered evaluation of one receiving grasp (worker output). */
  struct ControlAwareCandidateEvalV2
  {
    call_handover::GraspCandidateRecord record;
    CaptureCandidate candidate;
    sva::PTransformd objectPose = sva::PTransformd::Identity();
    double reachStandoffDuration = 0.0;
    double reachCaptureDuration = 0.0;
    double closureDuration = 0.0;
    double retreatDuration = 0.0;
    std::vector<call_handover::AuthorityEvaluation> standoffAuthority;
    std::vector<call_handover::AuthorityEvaluation> captureAuthority;
    std::vector<int> standoffDamper;
    std::vector<int> captureDamper;
    // Phase B front end (graspFamily: receiving); legacy_ring leaves these unset.
    std::string family = "legacy_ring";
    double theta = std::numeric_limits<double>::quiet_NaN();
    double axialOffset = 0.0;
    double reachabilityScore = std::numeric_limits<double>::quiet_NaN();
    bool shortlisted = true;
  };

  /** One grasp hypothesis before exact evaluation (worker thread). */
  struct ControlAwareHypothesisV2
  {
    int surrogateEvent = 0;  ///< Phase C: first event with surrogate reachability (0 at rest)
    double tauSurrogate = 0.0;
    call_handover::GraspParameters grasp;
    CaptureCandidate candidate;
    std::string family = "legacy_ring";
    double theta = std::numeric_limits<double>::quiet_NaN();
    double axialOffset = 0.0;
    double reachabilityScore = std::numeric_limits<double>::quiet_NaN();
    bool mechanicalPassed = true;
    std::string mechanicalReason = "not_applicable";
    bool shortlisted = true;
    bool evaluate = true;
  };

  struct ControlAwareFunnelStatsV2
  {
    int generated = 0;
    int mechanical = 0;
    int reachable = 0;
    int shortlisted = 0;
    int evaluated = 0;
    double frontEndWall = 0.0;
    double exactWall = 0.0;
  };

  /** Phase C: timed rollout of the velocity-matched rendezvous reference. */
  struct InterceptionRolloutV2
  {
    bool feasible = false;
    std::string reason = "not_run";
    int steps = 0;
    double duration = 0.0;
    double minClearance = std::numeric_limits<double>::infinity();
    double finalPositionError = std::numeric_limits<double>::quiet_NaN();
    double finalOrientationError = std::numeric_limits<double>::quiet_NaN();
    double finalRelativeLinearSpeed = std::numeric_limits<double>::quiet_NaN();
    double finalRelativeAngularSpeed = std::numeric_limits<double>::quiet_NaN();
    double conditionIndexAtRendezvous = std::numeric_limits<double>::quiet_NaN();
    // Phase D: authority of the commanded interception reference (tool-body
    // twist of each commanded step, sampled every authorityStride steps) and of
    // the synchronization twist at the rendezvous configuration.
    double pathReserve = std::numeric_limits<double>::quiet_NaN();
    double pathResidual = 0.0;
    int pathSamples = 0;
    double pathLimitingTime = std::numeric_limits<double>::quiet_NaN();
    double pathPeakLinearSpeed = 0.0;
    double syncReserve = std::numeric_limits<double>::quiet_NaN();
    double syncResidual = 0.0;
    bool insideSecurity = false;
    std::map<std::string, std::vector<double>> rendezvousArmPosture;
    rbd::MultiBodyConfig rendezvousState;
  };

  /** Phase C: one (g, tau) attempt of the ascending event sweep (log record). */
  struct InterceptionAttemptV2
  {
    int graspId = -1;
    double tau = 0.0;
    std::string stage;   ///< pursuit | surrogate | mechanical | exact | timing | budget | rollout | feasible
    std::string reason;
    double surrogate = std::numeric_limits<double>::quiet_NaN();
    double requiredTime = std::numeric_limits<double>::quiet_NaN();
    double wall = 0.0;
  };

  /** Phase C: earliest feasible encounter of one grasp. */
  struct InterceptionCandidateV2
  {
    int hypothesisIndex = -1;
    int graspId = -1;
    std::string status = "not_evaluated";  ///< feasible | infeasible_within_horizon | dominated | budget
    double tauStar = std::numeric_limits<double>::infinity();
    double tauSurrogate = std::numeric_limits<double>::infinity();  ///< surrogate lower bound on tau*
    int surrogateEvent = -1;
    int attempts = 0;
    int exactEvaluations = 0;
    int rollouts = 0;
    ControlAwareCandidateEvalV2 eval;  ///< exact layers at tau*
    InterceptionRolloutV2 rollout;
    sva::PTransformd objectPoseAtRendezvous = sva::PTransformd::Identity();
    /** Phase D: kappa(g, tau*) = min over interception-path, synchronization
     * and insertion demands (authority never decides F_I; it filters F_C). */
    double kappa = std::numeric_limits<double>::quiet_NaN();
    std::string kappaLimitingPhase = "none";
    int authorityRejectedEvents = 0;
    double tauStarInterception = std::numeric_limits<double>::infinity();  ///< earliest tau in F_I
  };

  struct InterceptionJobStatsV2
  {
    bool active = false;
    double step = std::numeric_limits<double>::quiet_NaN();
    double tieBand = 0.0;
    double horizon = 0.0;
    double anchor = 0.0;
    double pointSpeed = 0.0;
    int events = 0;
    int exactEvaluations = 0;
    int rollouts = 0;
    bool budgetExhausted = false;
    double tauBest = std::numeric_limits<double>::infinity();
    double sweepWall = 0.0;
    double rolloutWall = 0.0;
    double exactWall = 0.0;
    double frontEndWall = 0.0;
    ControlAwareFunnelStatsV2 funnel;
  };

  struct ReceiverJobResultV2
  {
    std::vector<ControlAwareCandidateEvalV2> controlAwareCandidates;
    ControlAwareFunnelStatsV2 controlAwareFunnel;
    std::vector<InterceptionCandidateV2> interceptionCandidates;
    std::vector<InterceptionAttemptV2> interceptionAttempts;
    InterceptionJobStatsV2 interception;
    double controlAwareFrameConsistency = std::numeric_limits<double>::quiet_NaN();
    std::vector<ParitySampleV2> parityTrace;
    bool parityFromReachStart = false;
    ReceiverJobTypeV2 type = ReceiverJobTypeV2::None;
    std::uint64_t planningGeneration = 0;
    std::uint64_t stateGeneration = 0;
    std::uint64_t planId = 0;
    double snapshotTime = 0.0;
    bool success = false;
    std::string reason = "not_run";
    double wallDuration = 0.0;
    ObjectPredictionRecordV2 prediction;
    CaptureCandidate candidate;
    InterceptionPlan plan;
    sva::PTransformd snapshotMouthPose = sva::PTransformd::Identity();
    sva::PTransformd objectPose = sva::PTransformd::Identity();
  };

  struct PendingJobV2
  {
    bool active = false;
    ReceiverJobTypeV2 type = ReceiverJobTypeV2::None;
    std::uint64_t planningGeneration = 0;
    std::uint64_t stateGeneration = 0;
    std::uint64_t planId = 0;
    double submitTime = 0.0;
    // Snapshot the job was planned from (for supersession and drift audit).
    ObjectPredictionRecordV2 prediction;
    sva::PTransformd snapshotMouthPose = sva::PTransformd::Identity();
    std::vector<double> bankEventTimes;
    std::vector<sva::PTransformd> bankPoses;
    // Non-blocking cancellation of a superseded generation.
    bool cancelRequested = false;
    std::string cancelReason;
    double cancelRequestTime = 0.0;
  };

  struct ReceiverV2Parameters
  {
    int planningStepsPerCycle = 96;
    int routeWorkUnitsPerCycle = 128;
    double minimumCommitRemainingTime = 1.6;
    double minimumReachEntryLead = 0.050;
    double maximumEventSearchWallTime = 7.0;
    double holdTaskStiffness = 18.0;
    double holdTaskWeight = 4200.0;
    double terminalLinearTrackingLead = 0.030;
    double terminalAngularTrackingLead = 0.10;
    double terminalPositionTolerance = 0.012;
    double terminalOrientationTolerance = 0.050;
    double terminalMaximumOpenClosure = 0.050;
    double terminalStableDwell = 0.10;
    double terminalTaskStiffness = 40.0;
    double terminalTaskWeight = 5400.0;
    double settleLinearSpeedTolerance = 0.0025;
    double settleAngularSpeedTolerance = 0.015;
    int logEvery = 50;
    bool injectStaleTerminalResultOnce = false;
    bool injectStaleRecertifyResultOnce = false;
    // Development-only: advance the state generation once while a FULL_SEARCH
    // is in flight, to exercise supersession cancellation.
    bool injectSupersedeFullSearchOnce = false;
    // Characterization only: run one FULL_SEARCH at the first (moving) epoch
    // and one after the object has been quasi-static for
    // characterizationRestDwell, log every record, adopt nothing, then fail.
    bool characterizeOnly = false;
    double characterizationRestDwell = 1.0;
  };

  bool submitReceiverFullSearchV2(double now);
  bool submitReceiverCertificationV2(ReceiverJobTypeV2 type, double now,
                                     const CaptureCandidate * candidate = nullptr,
                                     const InterceptionPlan * plan = nullptr);
  void runReceiverWorkerJobV2(std::uint64_t generation);
  void runRecertifyActiveRolloutV2(ReceiverJobResultV2 & result);
  void runTerminalCertificationV2(ReceiverJobResultV2 & result);
  void terminalFromStateV2(ReceiverJobResultV2 & result, const ReceiverJobRequestV2 & request,
                           const rbd::MultiBodyConfig & startState);
  RouteStepOutcome runRouteStepToCompletionV2();
  void processReceiverJobResultV2(double now);
  bool adoptFullSearchResultV2(double now);
  void adoptProvisionalPlanV2(const GlobalEventPlanAlternative & alternative,
                              const CaptureCandidate & candidate, const InterceptionPlan & plan,
                              double now, const char * source, std::uint64_t sourceGeneration,
                              std::uint64_t certificateGeneration);
  void handleSelectedCertificationV2(const PendingJobV2 & pending, double now);
  // Select-then-certify state: the FULL_SEARCH result whose selected record is
  // being re-certified at the newest prediction, and records already refused.
  bool v2SelectedCertificationPending_ = false;
  std::size_t v2SelectedRecord_ = 0;
  std::uint64_t v2SelectedSearchGeneration_ = 0;
  std::vector<char> v2ExcludedRecords_;
  long long v2SelectedCertifications_ = 0;
  long long v2SelectedCertificationFailures_ = 0;
  void invalidateProvisionalPlanV2(const std::string & reason, double now);
  bool executeProvisionalReachV2(double now);
  bool executeTerminalTrackV2(double now, bool & gateSatisfied);
  bool commitProvisionalReceiverPlanV2(const ReceiverJobResultV2 & certificate, double now);
  void logReceiverMotionV2(double now, bool force);
  void checkSupersessionV2(double now);
  void logSnapshotAuditV2(const PendingJobV2 & pending, double now, const char * outcome);
  void logJobProfileV2(const PendingJobV2 & pending, double now) const;
  void filterHypothesisFreshnessV2(const PendingJobV2 & pending, double now);
  double independentGiverSpeedForLogV2(double now) const;

  // ---------------------- TRIAD-lite control-aware supervisor ---------------
  // Feature-flagged by ReceiverV2 `supervisorMode: control_aware` (default
  // `bank_search` keeps TRIAD V2 unchanged). See
  // research/triad_lite/TRIAD_CONTROL_AWARE_IMPLEMENTATION.md.
  struct ControlAwareParametersV2
  {
    int graspsPerSign = 32;
    double clearanceFloor = 0.025;
    bool capabilityDiagnostics = false; ///< Sampled logging only; never alters commands.
    bool matchedExperiment = false;
    std::vector<int> matchedCandidateIds;
    double lookaheadSeconds = 0.20;
    bool diagnoseContinuation = false; ///< Diagnostic only; changes compute work, never admissibility.
    double kappaMin = 1.0;
    double kappaMaximum = 8.0;
    double residualTolerance = 1.0e-4;
    double angularCharacteristicLength = 0.20;
    double insertionSpeed = 0.38;
    double disturbanceSpeed = 0.0;
    double switchDwell = 0.30;
    bool requireObjectStopped = true;
    bool admissionRequiresCurrentAuthority = true;
    bool reselectWhileTracking = true;
    bool trustIncumbentReevaluation = true;
    call_handover::GraspSelectionTolerances selection;
    // Phase B receiving-grasp front end (default legacy_ring = 04e9efc behaviour).
    std::string graspFamily = "legacy_ring";
    call_handover::ReceivingGraspFamilySpec receivingSpec;
    double receivingAxialMax = -1.0;  ///< < 0: derived receivingAxialLimit
    double fingerHalfWidth = 0.0125;
    double axialMargin = 0.003;
    bool funnelEnabled = true;
    int funnelShortlistSize = 40;  ///< Phase B characterization (recall and best-clearance retention)
    double funnelPruneTolerance = 0.02;
    double funnelThetaBinWidth = 0.2374;
    bool funnelCharacterizeAll = false;
    // Phase C predictive interception solver (TRIAD_PREDICTIVE_INTERCEPTION_AUDIT.md sec. 3).
    std::string interceptionMode = "disabled";     ///< disabled | characterize | characterize_hold (no adoption)
    double interceptionHorizon = 8.0;              ///< numerical (s)
    double interceptionMinimumStep = 0.02;         ///< numerical floor on the event spacing (s)
    double interceptionComputationLatency = 0.80;  ///< L_calc (s): p90 moving-job wall, Phase C characterization
    double interceptionEntryLead = 0.05;           ///< L_entry = ReceiverV2 minimumReachEntryLead
    double interceptionEpsPosition = 0.015;        ///< predictiveReachPolicy.maximumObjectTranslationDeviation
    double interceptionEpsRotation = 0.12;         ///< predictiveReachPolicy.maximumObjectRotationDeviation
    double interceptionTerminalLinearSpeed = 0.04;   ///< MovePregrasp terminalLinearSpeedTolerance
    double interceptionTerminalAngularSpeed = 0.08;  ///< MovePregrasp terminalAngularSpeedTolerance
    bool interceptionTimingSkip = true;            ///< model-based skip of timing-infeasible events
    int interceptionMaximumExactEvaluations = 450; ///< numerical: covers all but one (near-ground, 1078) observed first-feasible counts (Phase C/D)
    int interceptionMaximumRollouts = 40;          ///< numerical: >= max 35 per job observed (Phase C/D)
    bool interceptionLogAttempts = true;
    // Phase D controller-authority demands (sec. 3.7).
    int authorityStride = 1;                       ///< numerical: every commanded step (stride 5 missed peaks, Phase D)
    bool authorityFilter = false;                  ///< F_C = {F_I : kappa >= kappaMin} (FULL variant)
    // Phase E execution variant: reactive (B0, 04e9efc tracker) | predictive (B1)
    // | predictive_capability (B2) | full (FULL = B1 + F_C + authority tie-break).
    std::string variant = "reactive";
    call_handover::InterceptionTieBreak tieBreak = call_handover::InterceptionTieBreak::Clearance;
  };
  bool v2ControlAware_ = false;
  ControlAwareParametersV2 v2CaParams_;
  call_handover::QpKinematicsLimits v2QpLimits_;
  call_handover::GraspSelectorState v2CaSelector_;
  std::unique_ptr<rbd::Jacobian> v2CaRuntimeJacobian_;
  int v2CaSelectionsSubmitted_ = 0;
  int v2CaSelectionsProcessed_ = 0;
  int v2CaSwitches_ = 0;
  int v2CaAborts_ = 0;
  int v2CaAdmitsDeferred_ = 0;
  bool loadControlAwareConfigV2(const mc_rtc::Configuration & stateConfig);
  void runControlAwareSelectionV2(ReceiverJobResultV2 & result);
  std::vector<ControlAwareHypothesisV2> controlAwareHypothesesV2(
      const ReceiverJobRequestV2 & request, const sva::PTransformd & objectPose, const Eigen::Vector3d & handleAxis,
      const Eigen::Vector3d & outward, ControlAwareFunnelStatsV2 & stats,
      const std::vector<std::pair<double, sva::PTransformd>> * events = nullptr) const;
  void handleControlAwareSelectionV2(const PendingJobV2 & pending, double now);
  /** Exact controller layers (standoff, corridor capture, closure, carried
   * retreat, security distance, clearance floor, authority) for one grasp at
   * one object pose. Shared by the at-rest selection and the interception sweep. */
  void evaluateControlAwareExactLayersV2(ControlAwareCandidateEvalV2 & eval, CaptureCandidate c,
                                         const sva::PTransformd & objectPose,
                                         const sva::PTransformd & startMouthPose,
                                         const Eigen::Vector3d & objectLinearVelocity,
                                         const Eigen::Vector3d & objectAngularVelocity);
  /** Phase C: earliest feasible encounter per shortlisted grasp (worker thread). */
  void solveInterceptionV2(ReceiverJobResultV2 & result, const std::vector<ControlAwareHypothesisV2> & hypotheses,
                           const std::vector<std::pair<double, sva::PTransformd>> & events,
                           const sva::PTransformd & snapshotObjectPose);
  /** Pursuit necessary condition: the rate-limited reference cannot cover the
   * pose change to W_T_M within (tau - L_calc). */
  bool interceptionPursuitPossibleV2(const sva::PTransformd & startMouth, const sva::PTransformd & W_T_M,
                                     double tau) const;
  /** Phase C: timed rollout from the frozen state tracking the Hermite
   * rendezvous reference toward compose(T_O(t), O_T_M_standoff), ending at tRendezvous. */
  /** Phase D insertion demand at the acquisition interface (object at rest):
   * MovePregrasp far-speed insertion along -y_M of the grasp. */
  std::vector<call_handover::AuthorityDemand> insertionDemandV2(const sva::PTransformd & W_T_M_goal) const;
  void rolloutInterceptionV2(const ObjectPredictionRecordV2 & prediction, double tStart, double tRendezvous,
                             const sva::PTransformd & O_T_M_standoff,
                             const std::map<std::string, std::vector<double>> & postureTarget,
                             InterceptionRolloutV2 & out, const sva::PTransformd * startReference = nullptr,
                             const Eigen::Vector3d & startLinearVelocity = Eigen::Vector3d::Zero());
  sva::PTransformd controlAwareGraspPoseAtV2(const ObjectPredictionRecordV2 & prediction, double absoluteTime,
                                            const sva::PTransformd & O_T_M) const;
  void logInterceptionResultV2(const PendingJobV2 & pending, const ReceiverJobResultV2 & result, double now) const;
  // Phase E: receding predictive interception execution (control thread).
  struct InterceptionExecutionV2
  {
    bool valid = false;
    int graspId = -1;
    sva::PTransformd O_T_M_standoff = sva::PTransformd::Identity();
    sva::PTransformd meetingPose = sva::PTransformd::Identity();
    double t0 = 0.0;
    double tRendezvous = 0.0;
    Eigen::Vector3d p0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d v0 = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R0 = Eigen::Matrix3d::Identity();
    Eigen::Vector3d pG0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d vG0 = Eigen::Vector3d::Zero();
    Eigen::Matrix3d RG0 = Eigen::Matrix3d::Identity();
    bool synchronizationLogged = false;
  };
  InterceptionExecutionV2 v2Icpt_;
  Eigen::Vector3d v2CaCommandLinearVelocity_ = Eigen::Vector3d::Zero();
  double v2IcptLastLog_ = -1.0;
  sva::PTransformd v2RepairLastActualPose_ = sva::PTransformd::Identity();
  double v2RepairLastActualTime_ = -1.0;
  int v2IcptRetained_ = 0;
  int v2IcptPatched_ = 0;
  int v2IcptLatencyRefused_ = 0;
  bool v2IcptReplanClosed_ = false;
  int v2IcptInconclusive_ = 0;
  bool predictiveVariantV2() const { return v2ControlAware_ && v2CaParams_.variant != "reactive" && v2CaParams_.variant != "lookahead"; }
  call_handover::RendezvousReferenceState interceptionExecutionReferenceV2(
      double t, const ObjectPredictionRecordV2 & prediction) const;
  bool matchesActiveGraspGeometryV2(const InterceptionCandidateV2 & candidate) const;
  void setInterceptionExecutionV2(const InterceptionCandidateV2 & candidate, double tRendezvous, double now);
  void handlePredictiveSelectionV2(const PendingJobV2 & pending, const ReceiverJobResultV2 & result, double now);
  bool adoptControlAwareCandidateV2(const ControlAwareCandidateEvalV2 & eval, double now, const std::string & event);
  /** 0 running, 1 committed, -1 failed. */
  int stepControlAwareTrackV2(double now, bool workerIdle);
  std::vector<call_handover::AuthorityDemand> controlAwareDemandsV2(
      const Eigen::Vector3d & objectLinearVelocity, const Eigen::Vector3d & objectAngularVelocity,
      const Eigen::Vector3d & objectPosition, const sva::PTransformd & W_T_M_goal,
      const sva::PTransformd & W_T_B_eval) const;
  /** Directional authority at a planner-model configuration (worker thread). */
  std::vector<call_handover::AuthorityEvaluation> controlAwareAuthorityAtPreviewV2(
      const rbd::MultiBodyConfig & mbc, const std::vector<call_handover::AuthorityDemand> & demands,
      std::vector<int> & damper, bool & insideSecurity) const;
  /** Directional authority at the live robot configuration (control thread). */
  std::vector<call_handover::AuthorityEvaluation> controlAwareAuthorityAtRuntimeV2(
      const std::vector<call_handover::AuthorityDemand> & demands, bool & insideSecurity);
  static std::string classifyControlAwareRejectionV2(const std::string & reason);

  ReceiverV2Parameters v2Params_;
  bool v2Active_ = false;
  bool v2EstimationActive_ = false;
  ReceiverPhaseV2 v2Phase_ = ReceiverPhaseV2::Idle;
  std::uint64_t v2StateGeneration_ = 0;
  std::uint64_t v2PredictionGeneration_ = 0;
  std::uint64_t v2PlanIdCounter_ = 0;
  ProvisionalReceiverPlanV2 provisionalReceiverPlan_;
  PendingJobV2 v2Pending_;
  ReceiverJobRequestV2 v2Request_;
  ReceiverJobResultV2 v2Result_;
  ReceiverJobResultV2 v2LatestTerminalCertificate_;
  bool v2LatestTerminalCertificateValid_ = false;
  // Phase 2B parity instrumentation (never read by a decision).
  bool v2ParityTrace_ = false;
  std::vector<ParitySampleV2> v2ParityReachTrace_;
  std::uint64_t v2ParityReachTracePlanId_ = 0;
  std::uint64_t v2ParityReachTraceGeneration_ = 0;
  std::uint64_t v2ParityReachLoggedPlanId_ = 0;
  std::vector<ParitySampleV2> v2ParityLastResumeTrace_;
  std::uint64_t v2ParityLastResumeGeneration_ = 0;
  double v2ParityLastRuntimeLog_ = -1.0;
  bool parityRuntimeTraceActive_ = false;
  void logParityTraceV2(const char * tag, std::uint64_t planId,
                        const std::vector<ParitySampleV2> & trace) const;
  void logParityRuntimeV2();
  void parityRecordPreviewV2(int phase, double t, double clearance);
  bool v2CommitLatched_ = false;
  int v2CommitCount_ = 0;
  bool v2ReselectionLocked_ = false;
  std::vector<double> v2Leads_;
  double v2SearchBudgetStart_ = 0.0;
  double v2ObjectQuasiStaticSince_ = -1.0;
  double v2GateStableSince_ = -1.0;
  double v2PhaseEntryTime_ = 0.0;
  sva::PTransformd v2HoldPose_ = sva::PTransformd::Identity();
  sva::PTransformd v2ReferencePose_ = sva::PTransformd::Identity();
  sva::PTransformd v2PreviousMouthPose_ = sva::PTransformd::Identity();
  double v2PreviousMouthTime_ = -1.0;
  double v2MouthLinearSpeed_ = 0.0;
  double v2MouthAngularSpeed_ = 0.0;
  double v2ClearanceScale_ = 1.0;
  double v2LastMotionLogTime_ = -1.0;
  std::string v2ReplacementReason_ = "initial";
  std::map<std::string, std::vector<double>> v2HoldArmPosture_;
  struct ReceiverV2Counters
  {
    int fullSearchSubmitted = 0;
    int recertifySubmitted = 0;
    int terminalSubmitted = 0;
    int generationsWhileRobotMoving = 0;
    int generationsWhileObjectMoving = 0;
    int staleRejected = 0;
    int retained = 0;
    int updated = 0;
    int replacements = 0;
    int adoptions = 0;
    int searchFailures = 0;
    int cancelRequested = 0;
    int cancelled = 0;
    double minimumRuntimeClearance = 1e9;
  } v2Counters_;
  bool v2StaleTerminalInjected_ = false;
  bool v2StaleRecertifyInjected_ = false;
  bool v2SupersedeInjected_ = false;
  int v2CharacterizationStage_ = 0;
  void logCharacterizationSearchV2(const PendingJobV2 & pending, double now);
  double v2FirstMotionTime_ = -1.0;

  PlannerConfig plannerConfig_;
  void refreshPlannerConfig();
  /** True only on the background planner worker thread. Planner functions
   * that are shared with control-thread callers use this to read the frozen
   * snapshot instead of live robot or estimator state. */
  static bool plannerWorkerThreadActive();
  static void setPlannerWorkerThreadFlag(bool active);

  /** TRIAD V2 characterization timer: adds the scope's wall time and one count
   * to a worker stage bucket. Disabled (no clock read) unless the caller is the
   * worker thread of a V2 run, so V1 and control-thread callers are unaffected.
   * Never read by any decision. */
  struct StageTimerV2
  {
    StageTimerV2(double * wall, long long * count) : wall_(wall), count_(count)
    {
      if(wall_) { start_ = std::chrono::steady_clock::now(); }
    }
    ~StageTimerV2()
    {
      if(!wall_) { return; }
      *wall_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
      ++*count_;
    }
    StageTimerV2(const StageTimerV2 &) = delete;
    StageTimerV2 & operator=(const StageTimerV2 &) = delete;
  private:
    double * wall_;
    long long * count_;
    std::chrono::steady_clock::time_point start_;
  };
  bool stageProfilingActiveV2() const
  {
    return plannerWorkerThreadActive() && plannerConfig_.conditionalPresentationV2;
  }
#define TRIAD_V2_STAGE_TIMER(bucket) \
  StageTimerV2 triadV2StageTimer(stageProfilingActiveV2() \
      ? &plannerContext_.stageWall[PlannerContext::bucket] : nullptr, \
      &plannerContext_.stageCount[PlannerContext::bucket])
  void resetStageProfileV2(std::uint64_t generation, const char * jobType) const;
  void logCertStageV2(const char * path, const CaptureCandidate & candidate, bool feasible,
                      const char * deepest, double staticReach, double routeReachDuration,
                      const std::string & reason) const;
  const char * routeDeepestStageV2(bool feasible) const;
  // Phase D search-policy replay logging (V2 worker only, no decision reads it).
  void logMemoRecordsV2(const std::vector<CaptureCandidate> & completePlans) const;
  bool exactTimingPruneRoutesV2(const CaptureCandidate & staticCandidate) const;
  std::string stageSnapshotV2() const;
  const char * staticDeepestStageV2(bool feasible) const;

  /** Mutable state owned by one finite TRIAD search.
   *
   * Everything here is scratch for a single search from a single frozen epoch:
   * the copied rollout configurations, the preview kinematic caches (rbd::Jacobian
   * is stateful and must never be shared), the resumable route-step machine, the
   * route-certification fold and the record accumulators. None of it is
   * controller, execution, commit or FSM state, and none of it outlives the
   * search. Isolating it here is what makes the planner relocatable: a worker
   * would own one of these outright instead of sharing controller members.
   */
  struct PlannerContext
  {
    std::vector<GlobalEventPlanAlternative> globalEventPlanAlternatives;
    CaptureCandidate planningBestCandidate;
    int planningCandidateCount = 0;
    int planningCandidateIndex = 0;
    int planningClosureIndex = 0;
    std::vector<CaptureCandidate> planningCompletePlanAuditCandidates;
    std::size_t planningCompletePlanCount = 0;
    std::size_t planningCostValidCount = 0;
    CaptureCandidate planningCurrentCandidate;
    bool planningFoundFeasible = false;
    std::unique_ptr<rbd::MultiBodyConfig> planningMbc;
    double planningPhaseStartDuration = 0.0;
    PlanningPhase planningPhase = PlanningPhase::ReachStandoff;
    PreviewResult planningResult;
    int planningSegmentIteration = 0;
    // The planner's own world. The search used to hijack the controller's
    // W_T_O_/W_T_H_/planningM_T_O_ and restore them; those members are read by
    // control-thread logging every cycle, so a search running off-thread would
    // race them. The search now mutates only these.
    // True while a hypothesis presentation world is established in this
    // context. Replaces the controller's planningObjectSnapshotActive_ flag
    // for the search, which no longer routes through that snapshot.
    bool plannerWorldActive = false;
    sva::PTransformd W_T_O = sva::PTransformd::Identity();
    sva::PTransformd W_T_H = sva::PTransformd::Identity();
    sva::PTransformd planningM_T_O = sva::PTransformd::Identity();
    sva::PTransformd planningStartMouthPose = sva::PTransformd::Identity();
    bool planningStartMouthPoseValid = false;
    std::size_t planningTimingAdmissibleCount = 0;
    std::vector<unsigned char> previewGripperJoint;  // joint -> skip flag
    Eigen::VectorXd previewJointVelocityLower;
    Eigen::VectorXd previewJointVelocityUpper;
    bool previewKinematicCacheValid = false;
    int previewToolBodyIndex = -1;
    std::unique_ptr<rbd::Jacobian> previewToolJacobian;
    bool routeCertificationActive = false;
    std::vector<std::pair<std::string, Eigen::Vector3d>> routeCertificationBank;
    CaptureCandidate routeCertificationBest;
    bool routeCertificationFound = false;
    std::size_t routeCertificationIndex = 0;
    std::string routeCertificationLastFailure;
    bool routeCertificationMovingRequested = false;
    sva::PTransformd routeCertificationPresentation = sva::PTransformd::Identity();
    CaptureCandidate routeStepCandidate;
    double routeStepClearanceScale = 1.0;
    int routeStepClosureIndex = 0;
    sva::PTransformd routeStepCommandReference = sva::PTransformd::Identity();
    int routeStepDwellIndex = 0;
    int routeStepDwellSteps = 0;
    rbd::MultiBodyConfig routeStepMbc;
    double routeStepPhaseStart = 0.0;
    RouteStepPhase routeStepPhase = RouteStepPhase::Idle;
    InterceptionPlan routeStepPlan;
    sva::PTransformd routeStepPresentationAnchor = sva::PTransformd::Identity();
    int routeStepReachIndex = 0;
    int routeStepReachIteration = 0;
    double routeStepReachOrientationError = 0.0;
    double routeStepReachPositionError = 0.0;
    PreviewResult routeStepReachResult;
    int routeStepReachSteps = 0;
    sva::PTransformd routeStepRetreatAttachment = sva::PTransformd::Identity();
    int routeStepRetreatIteration = 0;
    PreviewResult routeStepRetreatResult;
    sva::PTransformd routeStepSavedHandle = sva::PTransformd::Identity();
    sva::PTransformd routeStepSavedObject = sva::PTransformd::Identity();
    sva::PTransformd routeStepSavedPlanningAttachment = sva::PTransformd::Identity();
    int routeStepSegmentIteration = 0;
    PreviewResult routeStepTerminalResult;
    rbd::MultiBodyConfig routeStepTimingAuditStartMbc;
    bool routeStepTransitPostureSaved = false;

    // TRIAD V2 characterization only (never read by any decision; written only
    // when stageProfilingActiveV2()). Stage buckets partition the worker's time
    // except the Nested* buckets, which are nested inside the phase buckets.
    enum StageBucket : int
    {
      StageStaticReachStandoff = 0,
      StageStaticReachCapture,
      StageStaticClosure,
      StageStaticRetreat,
      StageRouteSetup, // RouteStepPhase::Idle
      StageRouteReach,
      StageRouteApproach,
      StageRouteDwell,
      StageRouteClosure,
      StageRouteRetreat,
      StageRouteFinalize,
      StageHypothesisSetup,
      StageTerminalStandoff,
      StageNestedIkStep,
      StageNestedSweptQuery,
      StageNestedConfigurationSafety,
      StageNestedClosureSafety,
      StageBucketCount
    };
    std::array<double, StageBucketCount> stageWall{};
    std::array<long long, StageBucketCount> stageCount{};
    std::uint64_t certJobGeneration = 0;
    std::chrono::steady_clock::time_point certJobStart;
    std::string certJobType;
    double certJobWall = 0.0;
    long long certStaticRecords = 0;
    long long certRouteRecords = 0;
    long long certMemoReuses = 0;
    long long certTimingPruned = 0;
    bool parityTraceActive = false;
    std::vector<ParitySampleV2> parityTrace;
    double certStaticReachClearance = std::numeric_limits<double>::quiet_NaN();
    double certRouteReachDuration = std::numeric_limits<double>::quiet_NaN();
    RouteStepPhase routeStepFailedPhase = RouteStepPhase::Idle;

    // TRIAD V2 exact memoization inside one frozen search: a hypothesis whose
    // predicted presentation pose is bitwise identical to an already evaluated
    // hypothesis has an identical copied-state certification (rollouts use
    // times relative to the presentation). Only lead-dependent fields are
    // recomputed by captureCurrentEventPlanAlternatives(). Never used in V1.
    struct HypothesisCertificationMemo
    {
      sva::PTransformd pose = sva::PTransformd::Identity();
      std::size_t hypothesis = 0;
      bool foundFeasible = false;
      std::vector<CaptureCandidate> completePlans;
    };
    std::vector<HypothesisCertificationMemo> v2HypothesisMemo;
  };

  /** Immutable inputs frozen at the decision epoch.
   *
   * These are the values the copied-state rollout must read instead of the live
   * controller, because they are exactly the ones that change while a search is
   * running. Immutable configuration (preview dt, banks, thresholds, weights,
   * geometry, tie tolerances) is category-A shareable read-only data and still
   * lives on the controller; moving it here is mechanical and carries no
   * semantic content.
   */
  struct PlanningSnapshot
  {
    bool frozenRobotStateValid = false;
    rbd::MultiBodyConfig frozenRobotState;
    double searchEpoch = 0.0;
    bool mouthToBaseTransformValid = false;
    sva::PTransformd mouthToBaseTransform = sva::PTransformd::Identity();
    // Planner-owned copy of the multibody. It is immutable for the controller's
    // lifetime, but mc_rtc owns its instance and mutates the Robot around it
    // every cycle, and RBDyn makes no thread-safety guarantee, so the planner
    // works from its own copy rather than sharing that instance.
    bool modelValid = false;
    rbd::MultiBody model;
  };

  /** The multibody the copied-state planner evaluates against. */
  const rbd::MultiBody & plannerModel() const
  {
    return planningSnapshot_.modelValid ? planningSnapshot_.model : robot().mb();
  }
  void refreshPlannerModel();

  FiniteTriadSearchState finiteSearch_;
  mutable PlannerContext plannerContext_;
  PlanningSnapshot planningSnapshot_;


  // Mouth-to-base transform frozen with the rest of the decision state. The
  // swept clearance certification used to read this from the live robot on
  // every sweep; the planner now uses this frozen copy.
  bool stepTransitRouteCertification(int routeWorkUnits);
  void routeCertificationAcceptRouteResult(bool feasible);
  void completeCurrentPlanningCandidate(bool feasible,
                                        bool movingVerificationRequested);

  // Sub-route resumability.
  //
  // One predictive route rollout used to run to completion inside a single
  // planner step, so a control cycle was charged a whole route. The rollout is
  // now advanced in bounded work units. The arithmetic, the operation order,
  // the collision sample order, the early-failure points, the failure reasons
  // and the candidate fields are unchanged; only the points at which the
  // rollout may be suspended are new.
  //
  // A suspension is only ever taken at a boundary that carries no partial
  // arithmetic: after a complete reach rollout iteration, a complete terminal
  // approach segment step, a complete capture dwell step, a complete closure
  // sweep step or a complete retreat step. Never inside previewReachStep(),
  // previewClosureStep() or a swept gripper clearance query.
  //
  // At a suspension the controller world transforms are restored to the saved
  // controller state, so a cycle that observes them between work units sees
  // what it would have seen before the route started. That is sound because
  // the simulated object pose is recomputed closed-form each cycle rather than
  // integrated. On resume the virtual world is re-established from the stored
  // frozen presentation anchor; the reach phase needs no re-establishment
  // because every reach iteration sets its own virtual object before the first
  // world-dependent read and interceptionReferenceAt() is a pure function of
  // the frozen plan.
  //
  // The copied multibody state is carried across suspensions and is never
  // re-seeded from the live robot().mbc().

  // Preview-loop invariants. The multibody, the tool frame and the joint
  // velocity limits do not change while a plan is being searched, so the
  // quantities derived from them are built once by refreshPreviewKinematicCache()
  // instead of being rebuilt on every IK iteration. Nothing here participates in
  // the numerics: the cache supplies exactly the values the per-call code
  // constructed, so the arithmetic and its order are untouched.
  // Built eagerly by refreshGripperGeometry() and, defensively, on first use.
  // rbd::Jacobian::jacobian() writes into the object's own storage, so the
  // cache is mutable and is only ever touched by the serial preview.
  void refreshPreviewKinematicCache() const;
  // Tool body index is a model constant; resolving it once here removes the
  // per-call lookup that previously carried a live-robot fallback.
  static bool collisionOracleCheckEnabled();
  bool gripperGeometryValid_ = false;

  CapturePlanningStatus capturePlanningStatus_ = CapturePlanningStatus::Idle;
  Eigen::Vector3d planningBaseOutward_ = Eigen::Vector3d::UnitY();

  /** Build the frozen plan set from the completed search, and emit it at full
   * double precision for equivalence checking. Called once, at selector entry,
   * after the search has finished and after the selector's now has been taken,
   * so it cannot perturb the planner's own elapsed time or the commit clock. */

  FrozenPlanSet buildFrozenPlanSet(std::size_t evaluatedHypotheses,
                                   std::size_t configuredHypotheses,
                                   bool scheduleComplete) const;
  void logFrozenPlanSet(const FrozenPlanSet & planSet) const;

  bool planningCostSelectionValid_ = false;
  bool planningCostSelectionCommitAdmissible_ = false;
  bool planningGlobalSelectionActive_ = false;
  std::string planningCostSelectionReason_ = "not_run";
  double planningMinimumAdmissibleCost_ = 1e9;
  double planningSelectedObjectiveCost_ = 1e9;
  double selectedGlobalMotionCost_ = 1e9;
  double selectedGlobalScheduleWait_ = 0.0;
  double selectedGlobalEventLead_ = 0.0;
  double selectedGlobalEventPresentationTime_ = 0.0;
  double selectedGlobalMinimumReachEntryLead_ = 0.0;
  double selectedGlobalMinimumSafeCommitLead_ = 0.0;
  /** Common frozen preview decision-state for the current global-time-plan
   * search: a single robot().mbc() snapshot captured once at
   * resetGlobalTimePlanSearch(), reused as the seed for every candidate
   * (startNextPlanningCandidate()) and every route
   * (verifyPredictiveRouteCandidate()) preview rollout in that search. Valid
   * only while planningSnapshot_.frozenRobotStateValid is true; invalidated and
   * recaptured on the next resetGlobalTimePlanSearch() call. */
  std::size_t selectedGlobalHypothesisIndex_ = 0;
  std::size_t globalEvaluatedHypotheses_ = 0;
  std::size_t globalConfiguredHypotheses_ = 0;
  bool globalScheduleComplete_ = false;
  sva::PTransformd selectedGlobalObjectPresentationPose_ =
      sva::PTransformd::Identity();
  sva::PTransformd selectedGlobalPlanningStartMouthPose_ =
      sva::PTransformd::Identity();
  bool capturePlanningCommitOnSuccess_ = true;
  bool planningObjectSnapshotActive_ = false;
  sva::PTransformd W_T_O_planningSnapshot_ = sva::PTransformd::Identity();

  bool candidateSelected_ = false;
  std::string selectedCandidateName_ = "none";
  double selectedCandidateClearance_ = -1e9;
  double selectedCandidateScore_ = 1e9;
  double selectedCandidatePredictedTime_ = 1e9;
  double selectedCandidatePredictedPresentationTime_ = 1e9;
  double selectedCandidatePredictedContactTime_ = 1e9;
  double selectedCandidatePredictedReachTime_ = 1e9;
  double selectedCandidatePredictedApproachTime_ = 1e9;
  double selectedCandidatePredictedAcquireTime_ = 1e9;
  double selectedCandidatePredictedEffort_ = 1e9;
  double selectedCandidateContactClosure_ = -1.0;
  std::map<std::string, std::vector<double>> selectedTransitArmPosture_;
  std::map<std::string, std::vector<double>> selectedStandoffArmPosture_;
  std::map<std::string, std::vector<double>> selectedArmPosture_;
  std::map<std::string, std::vector<double>> selectedRetreatArmPosture_;

  sva::PTransformd O_T_M_transit_ = sva::PTransformd::Identity();
  sva::PTransformd O_T_M_standoff_ = sva::PTransformd::Identity();
  sva::PTransformd O_T_M_pre_ = sva::PTransformd::Identity();
  sva::PTransformd O_T_M_retreat_ = sva::PTransformd::Identity();

  sva::PTransformd W_T_M_transit_ = sva::PTransformd::Identity();
  sva::PTransformd W_T_M_standoff_ = sva::PTransformd::Identity();
  sva::PTransformd W_T_M_pre_ = sva::PTransformd::Identity();
  sva::PTransformd W_T_M_acquired_ = sva::PTransformd::Identity();
  sva::PTransformd W_T_M_retreat_ = sva::PTransformd::Identity();

  sva::PTransformd planningM_T_O_ = sva::PTransformd::Identity();

  bool interceptionCommitted_ = false;
  InterceptionPlan committedInterceptionPlan_;
  // Simulation truth follows the same committed timing/profile, but is
  // anchored to the actual current object pose rather than a delayed
  // perception estimate. This keeps latency ablations scientifically valid.
  InterceptionPlan simulatedTruthInterceptionPlan_;
  bool simulatedTruthInterceptionPlanValid_ = false;
  double committedContactTime_ = 0.0;
  double committedTimingResidual_ = 1e9;
  sva::PTransformd W_T_O_committedContact_ = sva::PTransformd::Identity();
  Eigen::Vector3d committedObjectLinearVelocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d committedObjectAngularVelocity_ = Eigen::Vector3d::Zero();
  sva::PTransformd M_T_O_attached_ = sva::PTransformd::Identity();
  bool objectAttached_ = false;
  bool simulateKinematicAttachment_ = true;

  struct TimedObjectPose
  {
    double time = 0.0;
    sva::PTransformd pose = sva::PTransformd::Identity();
  };

  std::string giverTruthModel_ = "committed_plan";
  std::string receiverArchitecture_ = "v1_frozen_prereach";
  bool receiverArchitectureConfigurationValid_ = true;
  call_handover::IndependentGiverScript independentGiverScript_;
  bool independentGiverScriptArmed_ = false;
  double independentGiverStartTime_ = 0.0;
  double lastGiverTruthSampleLogTime_ = -1.0;

  bool objectObservationActive_ = false;
  bool simulatedObjectMotionActive_ = false;
  bool simulatedObjectPoseFrozen_ = false;
  bool havePreviousObjectObservation_ = false;
  bool objectMotionEstimateValid_ = false;
  ObservedObjectMode observedObjectMode_ = ObservedObjectMode::Unclassified;
  int objectObservationSamples_ = 0;
  double objectObservationStartTime_ = 0.0;
  double previousObjectObservationTime_ = 0.0;
  sva::PTransformd W_T_O_observationStart_ = sva::PTransformd::Identity();
  sva::PTransformd W_T_O_simulated_ = sva::PTransformd::Identity();
  sva::PTransformd W_T_O_truth_ = sva::PTransformd::Identity();
  sva::PTransformd W_T_O_perceptionMeasurement_ = sva::PTransformd::Identity();
  std::deque<TimedObjectPose> objectPerceptionBuffer_;
  double objectPerceptionMeasurementTime_ = 0.0;
  double objectPerceptionMeasurementAge_ = 0.0;
  double lastObjectPerceptionTruthSampleTime_ = -1.0;
  bool objectPerceptionMeasurementValid_ = false;
  sva::PTransformd W_T_O_previousObservation_ = sva::PTransformd::Identity();
  sva::PTransformd W_T_O_predicted_ = sva::PTransformd::Identity();
  Eigen::Vector3d objectLinearVelocityEstimate_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d objectAngularVelocityEstimate_ = Eigen::Vector3d::Zero();

  // V6.1 force-aware transfer. Motion/grasp feasibility remains geometric and
  // hard; these signals certify physical load takeover after the committed
  // bilateral grasp. The default synthetic source is explicitly ticker-only.
  ForceTransferPolicy forceTransferPolicy_;
  ForceTransferMeasurement forceTransferMeasurement_;
  bool forceTransferBiasCalibrationActive_ = false;
  int forceTransferBiasSamples_ = 0;
  Eigen::Vector3d forceTransferBiasForceSum_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d forceTransferBiasCoupleSum_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d forceTransferBiasForceWorld_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d forceTransferBiasCoupleWorld_ = Eigen::Vector3d::Zero();
  bool forceTransferFilterInitialized_ = false;
  Eigen::Vector3d forceTransferFilteredForceWorld_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d forceTransferFilteredCoupleWorld_ = Eigen::Vector3d::Zero();
  bool forceTransferExecutionActive_ = false;
  double forceTransferExecutionStartTime_ = 0.0;
  double forceTransferPreviousIndex_ = 0.0;
  double virtualForceTransferDeflection_ = 0.0;
  double virtualForceTransferVelocity_ = 0.0;
  bool virtualForceTransferBilateralContact_ = false;

  // Controller-time phase measurements. A single active phase is expected.
  double controllerTime_ = 0.0;
  std::map<std::string, double> phaseStartTimes_;
  std::map<std::string, double> phaseDurations_;
};
