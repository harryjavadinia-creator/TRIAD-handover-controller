#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/TransformTask.h>

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
#include <string>
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

  double liveMouthGap() const;
  double liveMouthHalfGap() const { return 0.5 * liveMouthGap(); }

  bool prepareCaptureSelection();
  CapturePlanningStatus beginCapturePlanning(bool commitOnSuccess = true);
  CapturePlanningStatus stepCapturePlanning(int maxInternalSteps);
  CapturePlanningStatus capturePlanningStatus() const { return capturePlanningStatus_; }

  // V4A.2 candidate/time interception solve. A future object pose is injected
  // only into copied-state rollout; the real robot and visible object continue
  // to evolve independently. The winning complete candidate is committed once
  // only after the contact-time fixed point converges.
  void setPlanningObjectSnapshot(const sva::PTransformd & W_T_O_snapshot);
  void applyPlanningObjectSnapshot();
  void clearPlanningObjectSnapshot();
  bool planningBestCandidateAvailable() const { return planningFoundFeasible_; }
  const std::string & planningBestCandidateName() const
  {
    return planningBestCandidate_.name;
  }
  double planningBestPredictedPresentationTime() const
  {
    return planningBestCandidate_.predictedPresentationTime;
  }
  double planningBestPredictedContactTime() const
  {
    return planningBestCandidate_.predictedContactTime;
  }
  double planningBestPredictedExecutionTime() const
  {
    return planningBestCandidate_.estimatedTime;
  }
  double planningBestClearance() const
  {
    return planningBestCandidate_.minClearance;
  }
  double planningBestScore() const { return planningBestCandidate_.score; }
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
  std::shared_ptr<mc_tasks::TransformTask> toolTask_;

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

  bool gripperPoseSafe(const sva::PTransformd & W_T_M,
                       HandoverSafetyReport & report,
                       bool requireCorridor) const;

  bool gripperBasePoseSafe(const sva::PTransformd & W_T_B,
                           const sva::PTransformd & W_T_M,
                           HandoverSafetyReport & report,
                           bool requireCorridor) const;

  bool graspCorridorSafe(const sva::PTransformd & W_T_M,
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
  sva::PTransformd previewBasePose(const rbd::MultiBodyConfig & mbc) const;
  sva::PTransformd previewMouthPose(const rbd::MultiBodyConfig & mbc) const;
  sva::PTransformd previewBasePoseFromMouthPose(
      const sva::PTransformd & W_T_M,
      const rbd::MultiBodyConfig & mbc) const;
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
  Eigen::Vector3d sampleWorldPoint(const GripperSample & sample,
                                   const rbd::MultiBodyConfig & mbc) const;

  bool startNextPlanningCandidate();
  void updateAttachedObjectPose();
  void updateSimulatedObjectMotion();
  void updateObjectMotionEstimate();
  void updateForceTransferMeasurement();
  void finishCurrentPlanningCandidate(bool feasible);
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
  bool gripperGeometryValid_ = false;

  CapturePlanningStatus capturePlanningStatus_ = CapturePlanningStatus::Idle;
  PlanningPhase planningPhase_ = PlanningPhase::ReachStandoff;
  int planningCandidateIndex_ = 0;
  int planningCandidateCount_ = 0;
  int planningSegmentIteration_ = 0;
  int planningClosureIndex_ = 0;
  Eigen::Vector3d planningBaseOutward_ = Eigen::Vector3d::UnitY();
  sva::PTransformd planningStartMouthPose_ = sva::PTransformd::Identity();
  std::unique_ptr<rbd::MultiBodyConfig> planningMbc_;
  CaptureCandidate planningCurrentCandidate_;
  CaptureCandidate planningBestCandidate_;
  std::vector<CaptureCandidate> planningCompletePlanAuditCandidates_;
  std::vector<GlobalEventPlanAlternative> globalEventPlanAlternatives_;
  bool planningCostSelectionValid_ = false;
  bool planningCostSelectionCommitAdmissible_ = false;
  bool planningGlobalSelectionActive_ = false;
  std::string planningCostSelectionReason_ = "not_run";
  std::size_t planningCompletePlanCount_ = 0;
  std::size_t planningCostValidCount_ = 0;
  std::size_t planningTimingAdmissibleCount_ = 0;
  double planningMinimumAdmissibleCost_ = 1e9;
  double planningSelectedObjectiveCost_ = 1e9;
  double selectedGlobalMotionCost_ = 1e9;
  double selectedGlobalScheduleWait_ = 0.0;
  double selectedGlobalEventLead_ = 0.0;
  double selectedGlobalEventPresentationTime_ = 0.0;
  double selectedGlobalMinimumReachEntryLead_ = 0.0;
  double selectedGlobalMinimumSafeCommitLead_ = 0.0;
  double globalTimePlanSearchEpoch_ = 0.0;
  /** Common frozen preview decision-state for the current global-time-plan
   * search: a single robot().mbc() snapshot captured once at
   * resetGlobalTimePlanSearch(), reused as the seed for every candidate
   * (startNextPlanningCandidate()) and every route
   * (verifyPredictiveRouteCandidate()) preview rollout in that search. Valid
   * only while globalTimePlanFrozenRobotStateValid_ is true; invalidated and
   * recaptured on the next resetGlobalTimePlanSearch() call. */
  rbd::MultiBodyConfig globalTimePlanFrozenRobotState_;
  bool globalTimePlanFrozenRobotStateValid_ = false;
  std::size_t selectedGlobalHypothesisIndex_ = 0;
  std::size_t globalEvaluatedHypotheses_ = 0;
  std::size_t globalConfiguredHypotheses_ = 0;
  bool globalScheduleComplete_ = false;
  sva::PTransformd selectedGlobalObjectPresentationPose_ =
      sva::PTransformd::Identity();
  sva::PTransformd selectedGlobalPlanningStartMouthPose_ =
      sva::PTransformd::Identity();
  PreviewResult planningResult_;
  double planningPhaseStartDuration_ = 0.0;
  bool planningFoundFeasible_ = false;
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
