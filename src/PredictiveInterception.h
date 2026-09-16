#pragma once

// TRIAD-lite predictive interception: pure math (Eigen + std only).
//
// Formulation: research/triad_lite/TRIAD_PREDICTIVE_INTERCEPTION_AUDIT.md sec. 3.
// Literature: earliest feasible rendezvous on the predicted object trajectory with
// the robot travel time bounded by the target arrival time (Croft, Fenton,
// Benhabib, IEEE TSMC 1998; Hujic et al., IEEE/ASME T-Mech 1998), replanned with
// patches that keep motion continuity. Because the exact feasibility predicate
// (IK convergence, collision, joint limits) is not monotone in tau, the event is
// found by an ascending first-feasible scan at the derived pose-change resolution
// (TRIAD Phase 3) instead of the secant intersection of Hujic et al.

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "ControlAwareGraspSupervisor.h"

namespace call_handover
{

// ---------------------------------------------------------------------------
// Necessary lower bound on the interception time
// ---------------------------------------------------------------------------

/** Smallest tau >= 0 with |d + v tau| <= speed * tau, d = target - pursuer
 * (constant-velocity target, pursuer speed bound). Exact for straight-line
 * pursuit; a necessary condition for any pursuer whose speed is bounded by
 * `speed`. Returns +inf when no such tau exists. */
inline double interceptionPositionLowerBound(const Eigen::Vector3d & d, const Eigen::Vector3d & v, double speed)
{
  const double c = d.squaredNorm();
  if(c <= 0.0) { return 0.0; }
  if(!(speed > 0.0)) { return std::numeric_limits<double>::infinity(); }
  const double a = v.squaredNorm() - speed * speed;
  const double b = 2.0 * d.dot(v);
  // f(tau) = a tau^2 + b tau + c <= 0 required.
  if(std::abs(a) < 1e-15)
  {
    if(b >= 0.0) { return std::numeric_limits<double>::infinity(); }
    return -c / b;
  }
  const double disc = b * b - 4.0 * a * c;
  if(disc < 0.0) { return std::numeric_limits<double>::infinity(); }
  const double sq = std::sqrt(disc);
  const double r1 = (-b - sq) / (2.0 * a);
  const double r2 = (-b + sq) / (2.0 * a);
  const double lo = std::min(r1, r2);
  const double hi = std::max(r1, r2);
  if(a < 0.0)
  {
    // f <= 0 outside (lo, hi); c > 0 means tau = 0 is inside, so the first
    // feasible tau is hi (lo < 0 < hi because lo * hi = c / a < 0).
    return std::max(0.0, hi);
  }
  // a > 0 (target faster than the pursuer): f <= 0 only on [lo, hi].
  if(hi < 0.0) { return std::numeric_limits<double>::infinity(); }
  return std::max(0.0, lo);
}

/** Orientation necessary bound: the relative angle can shrink at most at
 * (angularSpeed + |targetAngularSpeed|): tau >= angle / (angularSpeed + |w|). */
inline double interceptionOrientationLowerBound(double angle, double targetAngularSpeed, double angularSpeed)
{
  if(angle <= 0.0) { return 0.0; }
  const double rate = angularSpeed + std::abs(targetAngularSpeed);
  return rate > 0.0 ? angle / rate : std::numeric_limits<double>::infinity();
}

// ---------------------------------------------------------------------------
// Event schedule (numerical scan of a continuous event)
// ---------------------------------------------------------------------------

struct InterceptionScheduleSpec
{
  double lowerBound = 0.0;        ///< necessary bound (s from now)
  double latencyAnchor = 0.0;     ///< L_calc + L_entry (s)
  double linearSpeed = 0.0;       ///< |v_O| (m/s)
  double angularSpeed = 0.0;      ///< |w_O| (rad/s)
  double epsPosition = 0.015;     ///< pose-change tolerance (m)
  double epsRotation = 0.12;      ///< pose-change tolerance (rad)
  double minimumStep = 0.02;      ///< numerical floor on the spacing (s)
  double horizon = 8.0;           ///< numerical horizon (s)
  double restLinearSpeed = 0.004; ///< below both rest thresholds the event collapses
  double restAngularSpeed = 0.08;
};

/** Ascending events tau_j = tau_0 + j * dtau, tau_0 = max(lowerBound, latencyAnchor),
 * dtau = max(minimumStep, min(epsP/|v|, epsR/|w|)); a single event at rest (TRIAD
 * Phase 3 rest collapse). Empty when tau_0 exceeds the horizon. */
inline std::vector<double> interceptionEventSchedule(const InterceptionScheduleSpec & s, double * stepOut = nullptr)
{
  std::vector<double> out;
  const double tau0 = std::max(s.lowerBound, s.latencyAnchor);
  if(!std::isfinite(tau0) || tau0 > s.horizon) { return out; }
  const bool rest = s.linearSpeed <= s.restLinearSpeed && s.angularSpeed <= s.restAngularSpeed;
  if(rest)
  {
    if(stepOut) { *stepOut = std::numeric_limits<double>::infinity(); }
    out.push_back(tau0);
    return out;
  }
  const double stepP = s.linearSpeed > 0.0 ? s.epsPosition / s.linearSpeed : std::numeric_limits<double>::infinity();
  const double stepR = s.angularSpeed > 0.0 ? s.epsRotation / s.angularSpeed : std::numeric_limits<double>::infinity();
  const double step = std::max(s.minimumStep, std::min(stepP, stepR));
  if(stepOut) { *stepOut = step; }
  for(double tau = tau0; tau <= s.horizon + 1e-12; tau += step) { out.push_back(tau); }
  return out;
}

// ---------------------------------------------------------------------------
// Velocity-matched rendezvous reference (cubic Hermite error decay)
// ---------------------------------------------------------------------------

struct RendezvousReferenceState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
};

inline Eigen::Vector3d rotationLog(const Eigen::Matrix3d & R)
{
  const Eigen::AngleAxisd aa(R);
  if(!std::isfinite(aa.angle()) || std::abs(aa.angle()) < 1e-12) { return Eigen::Vector3d::Zero(); }
  return aa.angle() * aa.axis();
}

inline Eigen::Matrix3d rotationExp(const Eigen::Vector3d & w)
{
  const double n = w.norm();
  if(n < 1e-12) { return Eigen::Matrix3d::Identity(); }
  return Eigen::AngleAxisd(n, w / n).toRotationMatrix();
}

/** Reference that starts at (p0, v0, R0) at t0 and meets the moving target
 * (pG, vG, RG, wG evaluated at t) at t0 + T with matched position and velocity:
 *   e(u) = h00(u) e0 + T h10(u) edot0,  e0 = p0 - pG(t0), edot0 = v0 - vG(t0),
 *   p(t) = pG(t) + e(u),  u = clamp((t - t0) / T, 0, 1);
 *   R(t) = RG(t) Exp(h00(u) phi0),  phi0 = Log(RG(t0)^T R0).
 * Position and linear velocity are continuous at t0 and matched at t0 + T;
 * orientation matches at both ends, angular-rate continuity at t0 is not
 * enforced (stated approximation). */
inline RendezvousReferenceState rendezvousReference(double t, double t0, double T,
                                                    const Eigen::Vector3d & p0, const Eigen::Vector3d & v0,
                                                    const Eigen::Matrix3d & R0,
                                                    const Eigen::Vector3d & pG0, const Eigen::Vector3d & vG0,
                                                    const Eigen::Matrix3d & RG0,
                                                    const Eigen::Vector3d & pG, const Eigen::Vector3d & vG,
                                                    const Eigen::Matrix3d & RG, const Eigen::Vector3d & wG)
{
  RendezvousReferenceState r;
  const double D = std::max(1e-9, T);
  const double u = std::min(1.0, std::max(0.0, (t - t0) / D));
  const double inside = (t - t0) / D >= 0.0 && (t - t0) / D <= 1.0 ? 1.0 : 0.0;
  const double h00 = 2.0 * u * u * u - 3.0 * u * u + 1.0;
  const double h10 = u * u * u - 2.0 * u * u + u;
  const double dh00 = 6.0 * u * u - 6.0 * u;
  const double dh10 = 3.0 * u * u - 4.0 * u + 1.0;
  const Eigen::Vector3d e0 = p0 - pG0;
  const Eigen::Vector3d ed0 = v0 - vG0;
  r.position = pG + h00 * e0 + D * h10 * ed0;
  r.linearVelocity = vG + inside * ((dh00 / D) * e0 + dh10 * ed0);
  const Eigen::Vector3d phi0 = rotationLog(RG0.transpose() * R0);
  r.rotation = RG * rotationExp(h00 * phi0);
  r.angularVelocity = wG + inside * (RG * ((dh00 / D) * phi0));
  return r;
}

// ---------------------------------------------------------------------------
// Earliest-encounter selection (no weights)
// ---------------------------------------------------------------------------

enum class InterceptionTieBreak
{
  Clearance,          ///< B1: clearance
  Capability,         ///< B2: generic capability (condition index), then clearance
  AuthorityReserve    ///< FULL: min(kappa, saturation), then clearance
};

struct InterceptionRecord
{
  GraspCandidateRecord base;   ///< admissible, clearance, reserve, grasp id
  double tau = std::numeric_limits<double>::infinity();
  double capability = 0.0;     ///< generic capability measure (B2)
};

struct InterceptionSelectionTolerances
{
  double tauTieBand = 0.19;         ///< numerically indistinguishable encounters (s): the scan step
  double clearanceTieBand = 0.005;
  double reserveTieBand = 0.10;
  double reserveSaturation = 2.0;
  double capabilityTieBand = 0.01;
};

/** S1 admissible; S2 tau within tauTieBand of the earliest; then the tie-break
 * cascade of the declared mode; final order by tau then grasp id. The returned
 * outcome is compatible with updateGraspSelector (inFinalBand = survived all bands). */
inline GraspSelectionOutcome selectEarliestInterception(const std::vector<InterceptionRecord> & records,
                                                        const InterceptionSelectionTolerances & tol,
                                                        InterceptionTieBreak mode)
{
  GraspSelectionOutcome out;
  out.inFinalBand.assign(records.size(), false);
  const std::size_t n = records.size();
  std::vector<char> alive(n, 0);
  double tauMin = std::numeric_limits<double>::infinity();
  for(std::size_t i = 0; i < n; ++i)
  {
    if(records[i].base.admissible && std::isfinite(records[i].tau))
    {
      alive[i] = 1;
      tauMin = std::min(tauMin, records[i].tau);
    }
  }
  if(!std::isfinite(tauMin)) { return out; }
  for(std::size_t i = 0; i < n; ++i)
  {
    if(alive[i] && records[i].tau > tauMin + tol.tauTieBand + 1e-12) { alive[i] = 0; }
  }
  auto band = [&](double (*value)(const InterceptionRecord &, const InterceptionSelectionTolerances &), double width)
  {
    double best = -std::numeric_limits<double>::infinity();
    for(std::size_t i = 0; i < n; ++i)
    {
      if(alive[i]) { best = std::max(best, value(records[i], tol)); }
    }
    for(std::size_t i = 0; i < n; ++i)
    {
      if(alive[i] && value(records[i], tol) < best - width - 1e-12) { alive[i] = 0; }
    }
    return best;
  };
  auto clearance = [](const InterceptionRecord & r, const InterceptionSelectionTolerances &) { return r.base.clearance; };
  auto reserve = [](const InterceptionRecord & r, const InterceptionSelectionTolerances & t)
  { return std::min(r.base.reserve, t.reserveSaturation); };
  auto capability = [](const InterceptionRecord & r, const InterceptionSelectionTolerances &) { return r.capability; };
  if(mode == InterceptionTieBreak::AuthorityReserve) { out.bestReserve = band(reserve, tol.reserveTieBand); }
  if(mode == InterceptionTieBreak::Capability) { band(capability, tol.capabilityTieBand); }
  out.bestClearance = band(clearance, tol.clearanceTieBand);
  for(std::size_t i = 0; i < n; ++i)
  {
    if(!alive[i]) { continue; }
    out.inFinalBand[i] = true;
    if(out.bestIndex < 0) { out.bestIndex = static_cast<int>(i); continue; }
    const auto & b = records[static_cast<std::size_t>(out.bestIndex)];
    if(records[i].tau < b.tau || (records[i].tau == b.tau && records[i].base.grasp.id < b.base.grasp.id))
    {
      out.bestIndex = static_cast<int>(i);
    }
  }
  return out;
}

} // namespace call_handover
