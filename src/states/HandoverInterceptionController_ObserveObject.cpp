#include "HandoverInterceptionController_ObserveObject.h"
#include "../HandoverInterceptionController.h"

#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>

void HandoverInterceptionController_ObserveObject::configure(
    const mc_rtc::Configuration & config)
{
  if(config.has("minimumObservedDisplacement"))
  {
    config("minimumObservedDisplacement", minimumObservedDisplacement_);
  }
  if(config.has("stationaryLinearSpeedTolerance"))
  {
    config("stationaryLinearSpeedTolerance",
           stationaryLinearSpeedTolerance_);
  }
  if(config.has("stationaryAngularSpeedTolerance"))
  {
    config("stationaryAngularSpeedTolerance",
           stationaryAngularSpeedTolerance_);
  }
  if(config.has("maximumRobotTranslation"))
  {
    config("maximumRobotTranslation", maximumRobotTranslation_);
  }
  if(config.has("maximumRobotRotation"))
  {
    config("maximumRobotRotation", maximumRobotRotation_);
  }
  if(config.has("settleDwell")) { config("settleDwell", settleDwell_); }
  if(config.has("settleTimeout")) { config("settleTimeout", settleTimeout_); }
  if(config.has("settleLinearSpeedTolerance"))
  {
    config("settleLinearSpeedTolerance", settleLinearSpeedTolerance_);
  }
  if(config.has("settleAngularSpeedTolerance"))
  {
    config("settleAngularSpeedTolerance", settleAngularSpeedTolerance_);
  }
  if(config.has("maximumOpenClosure"))
  {
    config("maximumOpenClosure", maximumOpenClosure_);
  }
  if(config.has("gripperClosureRateTolerance"))
  {
    config("gripperClosureRateTolerance", gripperClosureRateTolerance_);
  }
  if(config.has("gripperClosureDriftTolerance"))
  {
    config("gripperClosureDriftTolerance", gripperClosureDriftTolerance_);
  }
  if(config.has("holdTaskStiffness"))
  {
    config("holdTaskStiffness", holdTaskStiffness_);
  }
  if(config.has("holdTaskWeight"))
  {
    config("holdTaskWeight", holdTaskWeight_);
  }
  if(config.has("logEvery")) { config("logEvery", logEvery_); }
  if(config.has("maxIter")) { config("maxIter", maxIter_); }

  minimumObservedDisplacement_ = std::max(0.0, minimumObservedDisplacement_);
  stationaryLinearSpeedTolerance_ = std::max(
      0.0, stationaryLinearSpeedTolerance_);
  stationaryAngularSpeedTolerance_ = std::max(
      0.0, stationaryAngularSpeedTolerance_);
  maximumRobotTranslation_ = std::max(0.0, maximumRobotTranslation_);
  maximumRobotRotation_ = std::max(0.0, maximumRobotRotation_);
  settleDwell_ = std::max(0.0, settleDwell_);
  settleTimeout_ = std::max(settleDwell_, settleTimeout_);
  settleLinearSpeedTolerance_ = std::max(0.0, settleLinearSpeedTolerance_);
  settleAngularSpeedTolerance_ = std::max(0.0, settleAngularSpeedTolerance_);
  maximumOpenClosure_ = std::max(0.0, maximumOpenClosure_);
  gripperClosureRateTolerance_ = std::max(0.0, gripperClosureRateTolerance_);
  gripperClosureDriftTolerance_ = std::max(0.0, gripperClosureDriftTolerance_);
  holdTaskStiffness_ = std::max(0.0, holdTaskStiffness_);
  holdTaskWeight_ = std::max(0.0, holdTaskWeight_);
  logEvery_ = std::max<uint64_t>(1, logEvery_);
  maxIter_ = std::max<uint64_t>(1, maxIter_);
}

void HandoverInterceptionController_ObserveObject::start(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  iter_ = 0;
  completed_ = false;
  phase_ = Phase::Settling;
  settleElapsed_ = 0.0;
  settleStableTime_ = 0.0;
  maxRobotTranslationObserved_ = 0.0;
  maxRobotRotationObserved_ = 0.0;
  maxMouthTranslationObserved_ = 0.0;
  maxGripperClosureDriftObserved_ = 0.0;

  holdBasePose_ = ctl.actualBasePose();
  previousBasePose_ = holdBasePose_;
  holdMouthPose_ = ctl.actualMouthPose();
  previousClosure_ = ctl.measuredGripperClosure();
  openClosureReference_ = previousClosure_;
  holdArmPosture_ = ctl.currentArmPosture();

  ctl.setGripperClosureAuthorized(false);
  ctl.activateToolTask();
  ctl.setToolTaskGains(holdTaskStiffness_, holdTaskWeight_);
  ctl.commandBaseTarget(holdBasePose_);
  ctl.commandReadyArmPosture(holdArmPosture_);
  ctl.setGripperJointPriority(false);
  ctl.commandGripper(0.0);
  ctl.beginForceTransferBiasCalibration();

  mc_rtc::log::warning(
      "[ObserveObject] settling the controlled gripper-base frame and open aperture before observation dwell={:.2f}s timeout={:.2f}s",
      settleDwell_, settleTimeout_);
  const Eigen::Vector3d vSim = ctl.simulatedObjectLinearVelocity();
  mc_rtc::log::success(
      "[VelocityGatedTerminalCapture] V5.2.5 observation={:.2f}s objectSpeed={:.4f}m/s deceleration={:.2f}s acquisitionWindow={:.2f}s openThroughInsertion=true",
      ctl.objectObservationDuration(), vSim.norm(),
      ctl.presentationDecelerationDuration(),
      ctl.presentationAcquisitionWindow());
}

bool HandoverInterceptionController_ObserveObject::run(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  ++iter_;

  ctl.setGripperClosureAuthorized(false);

  // The observation controller and stationarity gate now use the same frame.
  // No mouth-frame inverse is used here, so Robotiq aperture kinematics cannot
  // create a fictitious arm-base command.
  ctl.activateToolTask();
  ctl.setToolTaskGains(holdTaskStiffness_, holdTaskWeight_);
  ctl.commandBaseTarget(holdBasePose_);
  ctl.commandReadyArmPosture(holdArmPosture_);
  ctl.setGripperJointPriority(false);
  ctl.commandGripper(0.0);

  const double dt = std::max(1e-9, ctl.controlDt());
  const sva::PTransformd currentBase = ctl.actualBasePose();
  const sva::PTransformd currentMouth = ctl.actualMouthPose();
  const double measuredClosure = ctl.measuredGripperClosure();
  const double baseLinearSpeed =
      (currentBase.translation() - previousBasePose_.translation()).norm() / dt;
  const double baseAngularSpeed =
      ctl.orientationError(currentBase, previousBasePose_) / dt;
  const double closureRate = (measuredClosure - previousClosure_) / dt;
  previousBasePose_ = currentBase;
  previousClosure_ = measuredClosure;

  if(phase_ == Phase::Settling)
  {
    settleElapsed_ += dt;

    const bool armSettled =
        baseLinearSpeed <= settleLinearSpeedTolerance_
        && baseAngularSpeed <= settleAngularSpeedTolerance_;
    const bool gripperOpenEnough = measuredClosure <= maximumOpenClosure_;
    const bool gripperSettled =
        gripperOpenEnough
        && std::abs(closureRate) <= gripperClosureRateTolerance_;
    if(armSettled && gripperSettled) { settleStableTime_ += dt; }
    else { settleStableTime_ = 0.0; }

    if(iter_ % logEvery_ == 1)
    {
      mc_rtc::log::info(
          "[ObserveSettle] t={:.2f}/{:.2f}s stable={:.3f}/{:.3f}s vBase={:.5f}m/s wBase={:.5f}rad/s closure={:.5f} closureRate={:.5f}/s openEnough={}",
          settleElapsed_, settleTimeout_, settleStableTime_, settleDwell_,
          baseLinearSpeed, baseAngularSpeed, measuredClosure, closureRate,
          gripperOpenEnough);
    }

    if(settleStableTime_ >= settleDwell_)
    {
      openClosureReference_ = measuredClosure;
      if(openClosureReference_ > maximumOpenClosure_)
      {
        mc_rtc::log::error(
            "[ObserveSettle] settled gripper is not sufficiently open reference={:.5f} maximum={:.5f}; object never moved",
            openClosureReference_, maximumOpenClosure_);
        output("FAIL");
        completed_ = true;
        return true;
      }

      holdBasePose_ = currentBase;
      previousBasePose_ = currentBase;
      holdMouthPose_ = currentMouth;
      previousClosure_ = measuredClosure;
      holdArmPosture_ = ctl.currentArmPosture();
      ctl.commandBaseTarget(holdBasePose_);
      ctl.commandReadyArmPosture(holdArmPosture_);
      maxRobotTranslationObserved_ = 0.0;
      maxRobotRotationObserved_ = 0.0;
      maxMouthTranslationObserved_ = 0.0;
      maxGripperClosureDriftObserved_ = 0.0;
      phase_ = Phase::Observing;
      iter_ = 0;
      ctl.beginObjectObservation();

      const Eigen::Vector3d p = ctl.objectPose().translation();
      mc_rtc::log::success(
          "[ObserveSettle] controlled base and gripper settled; stationary baseline armed vBase={:.5f}m/s wBase={:.5f}rad/s openReference={:.5f}",
          baseLinearSpeed, baseAngularSpeed, openClosureReference_);
      mc_rtc::log::warning(
          "[ObserveObject] unified static/moving observation started with robot stationary p0=[{:.3f},{:.3f},{:.3f}] duration={:.2f}s horizon={:.2f}s",
          p.x(), p.y(), p.z(), ctl.objectObservationDuration(),
          ctl.objectPredictionHorizon());
      return false;
    }

    if(settleElapsed_ >= settleTimeout_)
    {
      mc_rtc::log::error(
          "[ObserveSettle] controlled base or gripper did not settle before timeout={:.2f}s vBase={:.5f} wBase={:.5f} closure={:.5f} closureRate={:.5f}/s openLimit={:.5f}; object never moved",
          settleTimeout_, baseLinearSpeed, baseAngularSpeed,
          measuredClosure, closureRate, maximumOpenClosure_);
      output("FAIL");
      completed_ = true;
      return true;
    }
    return false;
  }

  const Eigen::Vector3d p = ctl.objectPose().translation();
  const Eigen::Vector3d v = ctl.objectLinearVelocityEstimate();
  const Eigen::Vector3d w = ctl.objectAngularVelocityEstimate();
  const Eigen::Vector3d pp = ctl.predictedObjectPose().translation();
  const double robotTranslation =
      (currentBase.translation() - holdBasePose_.translation()).norm();
  const double robotRotation = ctl.orientationError(currentBase, holdBasePose_);
  const double mouthTranslation =
      (currentMouth.translation() - holdMouthPose_.translation()).norm();
  const double closureDrift =
      std::abs(measuredClosure - openClosureReference_);
  maxRobotTranslationObserved_ = std::max(
      maxRobotTranslationObserved_, robotTranslation);
  maxRobotRotationObserved_ = std::max(
      maxRobotRotationObserved_, robotRotation);
  maxMouthTranslationObserved_ = std::max(
      maxMouthTranslationObserved_, mouthTranslation);
  maxGripperClosureDriftObserved_ = std::max(
      maxGripperClosureDriftObserved_, closureDrift);

  if(robotTranslation > maximumRobotTranslation_
     || robotRotation > maximumRobotRotation_)
  {
    ctl.endObjectObservation(true);
    mc_rtc::log::error(
        "[ObserveObject] controlled base left the stationary tube translation={:.4f} rotation={:.4f} mouthTranslation={:.4f}; failing before planning",
        robotTranslation, robotRotation, mouthTranslation);
    output("FAIL");
    completed_ = true;
    return true;
  }

  if(closureDrift > gripperClosureDriftTolerance_
     || std::abs(closureRate) > gripperClosureRateTolerance_)
  {
    ctl.endObjectObservation(true);
    mc_rtc::log::error(
        "[ObserveObject] gripper aperture changed during observation reference={:.5f} measured={:.5f} drift={:.5f} rate={:.5f}/s limits=[{:.5f},{:.5f}]; failing before planning",
        openClosureReference_, measuredClosure, closureDrift, closureRate,
        gripperClosureDriftTolerance_, gripperClosureRateTolerance_);
    output("FAIL");
    completed_ = true;
    return true;
  }

  if(iter_ % logEvery_ == 1)
  {
    mc_rtc::log::info(
        "[ObserveObject] t={:.2f}/{:.2f}s samples={} valid={} moved={:.4f} p=[{:.3f},{:.3f},{:.3f}] v=[{:.4f},{:.4f},{:.4f}] w=[{:.4f},{:.4f},{:.4f}] predicted=[{:.3f},{:.3f},{:.3f}] latency=[mode:{} age:{:.3f}s rawErr:{:.4f} estErr:{:.4f}] baseMoved={:.4f} mouthMoved={:.4f} closureDrift={:.5f}",
        ctl.objectObservationElapsed(), ctl.objectObservationDuration(),
        ctl.objectObservationSamples(), ctl.objectMotionEstimateValid(),
        ctl.objectObservationDisplacement(), p.x(), p.y(), p.z(),
        v.x(), v.y(), v.z(), w.x(), w.y(), w.z(),
        pp.x(), pp.y(), pp.z(), ctl.perceptionLatencyModeName(),
        ctl.objectPerceptionMeasurementAge(),
        ctl.objectPerceptionRawPositionError(),
        ctl.objectPerceptionEstimatePositionError(),
        robotTranslation, mouthTranslation, closureDrift);
  }

  if(ctl.objectObservationElapsed() >= ctl.objectObservationDuration())
  {
    const bool estimateOK = ctl.objectMotionEstimateValid();
    const double moved = ctl.objectObservationDisplacement();
    const Eigen::Vector3d finalV = ctl.objectLinearVelocityEstimate();
    const Eigen::Vector3d finalW = ctl.objectAngularVelocityEstimate();
    const Eigen::Vector3d truthV = ctl.simulatedObjectLinearVelocity();
    const Eigen::Vector3d truthW = ctl.simulatedObjectAngularVelocity();

    if(!estimateOK)
    {
      ctl.endObjectObservation(true);
      mc_rtc::log::error(
          "[ObserveObject] observation rejected valid=false samples={} moved={:.4f}; robot never leaves readiness",
          ctl.objectObservationSamples(), moved);
      output("FAIL");
      completed_ = true;
      return true;
    }

    const double linearSpeed = finalV.norm();
    const double angularSpeed = finalW.norm();
    // Final estimated twist decides whether the object is presently static.
    // Displacement remains the evidence gate for a genuinely moving mode.
    // Thus an object that was carried into place and stopped during the
    // observation is accepted as a static presentation at its measured pose.
    const bool stationary = linearSpeed <= stationaryLinearSpeedTolerance_
        && angularSpeed <= stationaryAngularSpeedTolerance_;
    const bool moving = !stationary
        && moved >= minimumObservedDisplacement_;

    if(!stationary && !moving)
    {
      ctl.endObjectObservation(true);
      mc_rtc::log::error(
          "[PresentationMode] rejected=AMBIGUOUS moved={:.4f}/{:.4f}m speed=[v:{:.4f}/{:.4f},w:{:.4f}/{:.4f}]; no physical motion",
          moved, minimumObservedDisplacement_, linearSpeed,
          stationaryLinearSpeedTolerance_, angularSpeed,
          stationaryAngularSpeedTolerance_);
      output("FAIL");
      completed_ = true;
      return true;
    }

    const auto mode = stationary
        ? HandoverInterceptionController::ObservedObjectMode::Static
        : HandoverInterceptionController::ObservedObjectMode::Moving;
    ctl.setObservedObjectMode(mode);
    ctl.endObjectObservation(stationary);
    const Eigen::Vector3d current = ctl.objectPose().translation();
    const Eigen::Vector3d predicted = ctl.predictedObjectPose().translation();

    mc_rtc::log::success(
        "[ObjectEstimator] estimate valid samples={} moved={:.4f} v=[{:.4f},{:.4f},{:.4f}]m/s w=[{:.4f},{:.4f},{:.4f}]rad/s",
        ctl.objectObservationSamples(), moved,
        finalV.x(), finalV.y(), finalV.z(),
        finalW.x(), finalW.y(), finalW.z());
    mc_rtc::log::success(
        "[PerceptionLatencySummary] mode={} configuredDelay={:.3f}s measurementAge={:.3f}s rawPositionError={:.4f}m compensatedPositionError={:.4f}m predictionHorizon={:.3f}s horizonRetuned=false controlThreadSleep=false",
        ctl.perceptionLatencyModeName(), ctl.perceptionLatencySeconds(),
        ctl.objectPerceptionMeasurementAge(),
        ctl.objectPerceptionRawPositionError(),
        ctl.objectPerceptionEstimatePositionError(),
        ctl.objectPredictionHorizon());
    mc_rtc::log::success(
        "[PresentationMode] selected={} moved={:.4f}m speed=[v:{:.4f},w:{:.4f}] thresholds=[movingDisplacement:{:.4f},staticV:{:.4f},staticW:{:.4f}] measuredClassification=true beforePlanning=true",
        ctl.observedObjectModeName(), moved, linearSpeed, angularSpeed,
        minimumObservedDisplacement_, stationaryLinearSpeedTolerance_,
        stationaryAngularSpeedTolerance_);
    if(ctl.simulatedObjectMotionEnabled())
    {
      mc_rtc::log::success(
          "[ObjectEstimator] simulation truth vError={:.5f}m/s wError={:.5f}rad/s",
          (finalV - truthV).norm(), (finalW - truthW).norm());
    }
    if(stationary)
    {
      mc_rtc::log::success(
          "[ObserveObject] observation complete; stationary object frozen for complete current-pose planning current=[{:.3f},{:.3f},{:.3f}]",
          current.x(), current.y(), current.z());
    }
    else
    {
      mc_rtc::log::success(
          "[ObserveObject] observation complete; object remains moving for interception solve current=[{:.3f},{:.3f},{:.3f}] predicted@{:.2f}s=[{:.3f},{:.3f},{:.3f}]",
          current.x(), current.y(), current.z(), ctl.objectPredictionHorizon(),
          predicted.x(), predicted.y(), predicted.z());
    }
    mc_rtc::log::success(
        "[ObserveObject] robot-stationary gate maxTranslation={:.4f} maxRotation={:.4f} maxMouthTranslation={:.4f} maxClosureDrift={:.5f} openReference={:.5f}",
        maxRobotTranslationObserved_, maxRobotRotationObserved_,
        maxMouthTranslationObserved_, maxGripperClosureDriftObserved_,
        openClosureReference_);
    if(stationary)
    {
      mc_rtc::log::warning(
          "[ObserveObject] static presentation will evaluate one complete current-pose grasp-route-acquire-transfer-retreat action set; infeasible geometry is rejected without physical trial");
    }
    else
    {
      mc_rtc::log::warning(
          "[ObserveObject] V5.2.5 velocity-gated terminal-capture profile keeps the faster object approaching during planning; one presentation event and one bounded post-stop acquisition window will be committed while the robot remains stationary");
    }
    ctl.finishForceTransferBiasCalibration();

    output("OK");
    completed_ = true;
    return true;
  }

  if(iter_ >= maxIter_)
  {
    ctl.endObjectObservation(true);
    mc_rtc::log::error(
        "[ObserveObject] observation watchdog expired; object frozen and robot held at readiness");
    output("FAIL");
    completed_ = true;
    return true;
  }

  return false;
}

void HandoverInterceptionController_ObserveObject::teardown(
    mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HandoverInterceptionController &>(ctl_);
  if(ctl.objectObservationActive()) { ctl.endObjectObservation(true); }
  ctl.finishForceTransferBiasCalibration();
  mc_rtc::log::info(
      "[ObserveObject] teardown completed={}", completed_);
}

EXPORT_SINGLE_STATE("HandoverInterceptionController_ObserveObject",
                    HandoverInterceptionController_ObserveObject)
