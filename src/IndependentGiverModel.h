#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace call_handover
{

/**
 * Robot-independent simulated giver (TRIAD V2).
 *
 * The giver carries the object from its observation-start pose along its
 * initial twist, travels a fixed distance and comes to rest with a C2-smooth
 * quintic stop of fixed duration, then presents the object at rest.
 *
 * The state is a pure function of this script and the elapsed time since the
 * giver started moving. Nothing here can receive a receiver plan, an
 * interception time, a grasp or a route: that is the independence property the
 * V2 evidence checks rely on.
 *
 * Both parameters are pre-existing controller configuration values, not V2
 * tuning: travelDistance is movingObject.maximumSimulatedTravel and
 * stopDuration is movingObject.presentationDecelerationDuration. Before the
 * stop begins, the motion is identical to V1's pre-commit constant-twist
 * simulation.
 */
struct IndependentGiverScript
{
  Eigen::Vector3d startPosition = Eigen::Vector3d::Zero();
  Eigen::Matrix3d startRotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
  double travelDistance = 0.40;
  double stopDuration = 0.85;
};

struct IndependentGiverState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
  double speedScale = 0.0;  // 1 while cruising, 0 at rest
  bool atRest = false;
};

namespace giver_detail
{
inline double clamp01(double x) { return std::max(0.0, std::min(1.0, x)); }
inline double quinticStep(double x)
{
  x = clamp01(x);
  return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}
// Integral of (1 - quinticStep) from 0 to x; equals 0.5 at x = 1.
inline double quinticStopIntegral(double x)
{
  x = clamp01(x);
  const double x2 = x * x;
  const double x4 = x2 * x2;
  return x - 2.5 * x4 + 3.0 * x4 * x - x4 * x2;
}
} // namespace giver_detail

/** Cruise duration before the stop begins, and the effective stop duration. */
inline void independentGiverSchedule(const IndependentGiverScript & script,
                                     double & cruiseDuration,
                                     double & effectiveStopDuration)
{
  const double speed = script.linearVelocity.norm();
  const double distance = std::max(0.0, script.travelDistance);
  double stop = std::max(0.0, script.stopDuration);
  if(speed <= 1e-12)
  {
    cruiseDuration = 0.0;
    effectiveStopDuration = 0.0;
    return;
  }
  // A stop of duration D from speed s covers 0.5 s D. If the distance is too
  // short for the configured stop, the stop is shortened to end at the
  // distance.
  if(0.5 * speed * stop > distance) { stop = 2.0 * distance / speed; }
  cruiseDuration = std::max(0.0, (distance - 0.5 * speed * stop) / speed);
  effectiveStopDuration = stop;
}

inline IndependentGiverState independentGiverStateAt(
    const IndependentGiverScript & script,
    double elapsed)
{
  IndependentGiverState state;
  const double t = std::max(0.0, elapsed);
  double cruise = 0.0;
  double stop = 0.0;
  independentGiverSchedule(script, cruise, stop);

  double effectiveTravelTime = 0.0;  // time at initial twist giving the same displacement
  double scale = 0.0;
  if(script.linearVelocity.norm() <= 1e-12)
  {
    // A giver with no linear motion is static; no rotation is invented.
    effectiveTravelTime = 0.0;
    scale = 0.0;
    state.atRest = true;
  }
  else if(t <= cruise)
  {
    effectiveTravelTime = t;
    scale = 1.0;
  }
  else if(stop > 1e-12 && t < cruise + stop)
  {
    const double u = (t - cruise) / stop;
    effectiveTravelTime = cruise + stop * giver_detail::quinticStopIntegral(u);
    scale = 1.0 - giver_detail::quinticStep(u);
  }
  else
  {
    effectiveTravelTime = cruise + 0.5 * stop;
    scale = 0.0;
    state.atRest = true;
  }

  state.position = script.startPosition + effectiveTravelTime * script.linearVelocity;
  state.rotation = script.startRotation;
  const double omega = script.angularVelocity.norm();
  if(omega > 1e-9 && effectiveTravelTime > 0.0)
  {
    state.rotation = Eigen::AngleAxisd(
        omega * effectiveTravelTime,
        script.angularVelocity / omega).toRotationMatrix() * script.startRotation;
  }
  state.linearVelocity = scale * script.linearVelocity;
  state.angularVelocity = scale * script.angularVelocity;
  state.speedScale = scale;
  return state;
}

} // namespace call_handover
