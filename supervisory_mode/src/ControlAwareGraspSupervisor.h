#pragma once

// TRIAD supervisory mode control-aware grasp supervisor: pure, deterministic math.
//
// This header deliberately depends only on Eigen and the standard library so
// that every decision rule can be unit-tested offline. It contains no
// controller state and reads no robot. The controller integration lives in
// ReceiverV2.cpp behind ReceiverV2 `supervisorMode: control_aware`.
//
// Scope (see supervisory_mode/supervisor_design_review.md):
//  * the decision variable is a receiving grasp g = (sign, phi) about the
//    handle axis, no time bank, no route bank, no weighted objective;
//  * robot authority is evaluated at velocity level against the hard bounds of
//    the mc_rtc Tasks-backend KinematicsConstraint
//    (tasks::qp::DamperJointLimitsConstr): velocityPercent * [vl, vu]
//    intersected with the position damper. Acceleration and jerk bounds are
//    infinite for the Gen3 module and task weights are soft, so neither
//    restricts the admissible joint-velocity set;
//  * the damper gain xi is history-dependent in Tasks; the damper offset is
//    its lower bound, so the box built here is an inner (conservative)
//    approximation of the admissible set.

#include <Eigen/Core>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace call_handover
{

// ---------------------------------------------------------------------------
// Grasp family
// ---------------------------------------------------------------------------

/** Receiving grasp about a cylindrical handle: the tool z axis is sign * handle
 * axis, the approach (outward) direction is rotated by phi about it. */
struct GraspParameters
{
  int id = -1;
  int sign = 1;
  double phi = 0.0;
};

/** 2 * perSign grasps, ordered sign +1 then -1, phi = 2 pi k / perSign.
 * Ordering and angles are identical to the V2 static-screen enumeration. */
inline std::vector<GraspParameters> generateGraspFamily(int perSign)
{
  const int n = std::max(4, perSign);
  std::vector<GraspParameters> family;
  family.reserve(static_cast<std::size_t>(2 * n));
  for(int s = 0; s < 2; ++s)
  {
    for(int k = 0; k < n; ++k)
    {
      GraspParameters g;
      g.id = static_cast<int>(family.size());
      g.sign = s == 0 ? 1 : -1;
      g.phi = 2.0 * M_PI * static_cast<double>(k) / static_cast<double>(n);
      family.push_back(g);
    }
  }
  return family;
}

/** World rotation of the grasp ("mouth") frame, same construction as
 * HandoverInterceptionController::buildCandidate: z = sign * handleAxis,
 * y = Rot(z, phi) * outward projected orthogonally to z, x = y x z. */
inline Eigen::Matrix3d graspFrameRotation(const Eigen::Vector3d & handleAxis,
                                          const Eigen::Vector3d & outward,
                                          const GraspParameters & g)
{
  const Eigen::Vector3d zM = (g.sign >= 0 ? 1.0 : -1.0) * handleAxis.normalized();
  Eigen::Vector3d yM = Eigen::AngleAxisd(g.phi, zM) * outward;
  yM -= zM * zM.dot(yM);
  if(yM.norm() < 1e-9) { yM = Eigen::Vector3d::UnitY(); }
  yM.normalize();
  Eigen::Vector3d xM = yM.cross(zM);
  if(xM.norm() < 1e-9) { xM = Eigen::Vector3d::UnitZ(); }
  xM.normalize();
  yM = zM.cross(xM).normalized();
  Eigen::Matrix3d R;
  R.col(0) = xM;
  R.col(1) = yM;
  R.col(2) = zM;
  return R;
}

// ---------------------------------------------------------------------------
// Admissible joint velocities of the mc_rtc kinematics constraint
// ---------------------------------------------------------------------------

struct QpKinematicsLimits
{
  double velocityPercent = 0.95; ///< KinematicsConstraint velocityPercent
  double interPercent = 0.10;    ///< damper[0]: iDist = interPercent * range
  double securityPercent = 0.01; ///< damper[1]: sDist = securityPercent * range
  double damperOffset = 0.50;    ///< damper[2]: lower bound of the damping gain xi
};

struct JointVelocityBox
{
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;
  /** -1 lower damper active, +1 upper damper active, 0 free. */
  std::vector<int> damper;
  /** Some limited joint is within the security distance of a limit. The QP
   * then forces it away; such a configuration is not an admissible grasp. */
  bool insideSecurity = false;
  /** lower <= upper for every joint. */
  bool consistent = true;
};

/** Per-joint bounds on the next joint velocity, exactly as
 * tasks::qp::DamperJointLimitsConstr::update builds them, with the damping gain
 * replaced by its history-free lower bound xi = damperOffset. Joints with
 * non-finite position limits (continuous joints) have no damper. */
inline JointVelocityBox qpJointVelocityBox(const Eigen::VectorXd & q,
                                           const Eigen::VectorXd & qMin,
                                           const Eigen::VectorXd & qMax,
                                           const Eigen::VectorXd & vMin,
                                           const Eigen::VectorXd & vMax,
                                           const QpKinematicsLimits & limits)
{
  const Eigen::Index n = q.size();
  JointVelocityBox box;
  box.lower = limits.velocityPercent * vMin;
  box.upper = limits.velocityPercent * vMax;
  box.damper.assign(static_cast<std::size_t>(n), 0);
  for(Eigen::Index i = 0; i < n; ++i)
  {
    if(!std::isfinite(qMin[i]) || !std::isfinite(qMax[i]) || qMax[i] <= qMin[i]) { continue; }
    const double range = qMax[i] - qMin[i];
    const double iDist = limits.interPercent * range;
    const double sDist = limits.securityPercent * range;
    const double ld = q[i] - qMin[i];
    const double ud = qMax[i] - q[i];
    const double span = std::max(1e-12, iDist - sDist);
    if(ld < iDist)
    {
      box.damper[static_cast<std::size_t>(i)] = -1;
      box.lower[i] = std::max(box.lower[i], -limits.damperOffset * (ld - sDist) / span);
      if(ld <= sDist) { box.insideSecurity = true; }
    }
    else if(ud < iDist)
    {
      box.damper[static_cast<std::size_t>(i)] = 1;
      box.upper[i] = std::min(box.upper[i], limits.damperOffset * (ud - sDist) / span);
      if(ud <= sDist) { box.insideSecurity = true; }
    }
    if(box.lower[i] > box.upper[i]) { box.consistent = false; }
  }
  return box;
}

// ---------------------------------------------------------------------------
// Box-constrained least squares (primal active set)
// ---------------------------------------------------------------------------

struct BoxLeastSquaresResult
{
  Eigen::VectorXd x;
  double residual = std::numeric_limits<double>::infinity();
  int iterations = 0;
  bool converged = false;
};

/** min 1/2 ||A x - b||^2 + 1/2 rho ||x||^2  s.t. lower <= x <= upper.
 *
 * Primal active-set method for a strictly convex bound-constrained QP
 * (rho > 0 makes A^T A + rho I positive definite when A is rank deficient).
 * Finite termination for strictly convex problems; maxIterations is a guard.
 * The reported residual is ||A x - b|| without the regularisation term. */
inline BoxLeastSquaresResult boxConstrainedLeastSquares(const Eigen::MatrixXd & A,
                                                        const Eigen::VectorXd & b,
                                                        const Eigen::VectorXd & lower,
                                                        const Eigen::VectorXd & upper,
                                                        double rho = 1e-10,
                                                        int maxIterations = 500)
{
  const Eigen::Index n = A.cols();
  BoxLeastSquaresResult out;
  out.x = Eigen::VectorXd::Zero(n);
  for(Eigen::Index i = 0; i < n; ++i)
  {
    if(lower[i] > upper[i]) { return out; }
    out.x[i] = std::min(upper[i], std::max(lower[i], 0.0));
  }
  const Eigen::MatrixXd H = A.transpose() * A + rho * Eigen::MatrixXd::Identity(n, n);
  const Eigen::VectorXd g = -A.transpose() * b;
  // 0 free, -1 held at lower, +1 held at upper, 2 fixed (lower == upper).
  std::vector<int> state(static_cast<std::size_t>(n), 0);
  for(Eigen::Index i = 0; i < n; ++i)
  {
    if(lower[i] == upper[i]) { state[static_cast<std::size_t>(i)] = 2; }
  }
  const double scale = 1.0 + H.cwiseAbs().maxCoeff() + g.cwiseAbs().maxCoeff();
  for(int it = 0; it < maxIterations; ++it)
  {
    out.iterations = it + 1;
    std::vector<Eigen::Index> F;
    for(Eigen::Index i = 0; i < n; ++i)
    {
      if(state[static_cast<std::size_t>(i)] == 0) { F.push_back(i); }
    }
    Eigen::VectorXd p = Eigen::VectorXd::Zero(n);
    if(!F.empty())
    {
      const Eigen::Index m = static_cast<Eigen::Index>(F.size());
      Eigen::MatrixXd HFF(m, m);
      Eigen::VectorXd rhs(m);
      for(Eigen::Index r = 0; r < m; ++r)
      {
        double s = g[F[static_cast<std::size_t>(r)]];
        for(Eigen::Index c = 0; c < n; ++c)
        {
          if(state[static_cast<std::size_t>(c)] != 0) { s += H(F[static_cast<std::size_t>(r)], c) * out.x[c]; }
        }
        rhs[r] = -s;
        for(Eigen::Index c = 0; c < m; ++c)
        {
          HFF(r, c) = H(F[static_cast<std::size_t>(r)], F[static_cast<std::size_t>(c)]);
        }
      }
      const Eigen::VectorXd xF = HFF.ldlt().solve(rhs);
      for(Eigen::Index r = 0; r < m; ++r)
      {
        p[F[static_cast<std::size_t>(r)]] = xF[r] - out.x[F[static_cast<std::size_t>(r)]];
      }
    }
    if(p.norm() <= 1e-13 * (1.0 + out.x.norm()))
    {
      const Eigen::VectorXd grad = H * out.x + g;
      Eigen::Index release = -1;
      double worst = 1e-12 * scale;
      for(Eigen::Index i = 0; i < n; ++i)
      {
        const int st = state[static_cast<std::size_t>(i)];
        double violation = 0.0;
        if(st == -1) { violation = -grad[i]; }      // at lower: optimal iff grad >= 0
        else if(st == 1) { violation = grad[i]; }   // at upper: optimal iff grad <= 0
        if(violation > worst)
        {
          worst = violation;
          release = i;
        }
      }
      if(release < 0)
      {
        out.converged = true;
        break;
      }
      state[static_cast<std::size_t>(release)] = 0;
      continue;
    }
    double alpha = 1.0;
    Eigen::Index block = -1;
    int blockState = 0;
    for(Eigen::Index i : F)
    {
      if(p[i] < 0.0)
      {
        const double a = (lower[i] - out.x[i]) / p[i];
        if(a < alpha)
        {
          alpha = a;
          block = i;
          blockState = -1;
        }
      }
      else if(p[i] > 0.0)
      {
        const double a = (upper[i] - out.x[i]) / p[i];
        if(a < alpha)
        {
          alpha = a;
          block = i;
          blockState = 1;
        }
      }
    }
    alpha = std::max(0.0, alpha);
    out.x += alpha * p;
    if(block >= 0)
    {
      out.x[block] = blockState < 0 ? lower[block] : upper[block];
      state[static_cast<std::size_t>(block)] = blockState;
    }
  }
  for(Eigen::Index i = 0; i < n; ++i) { out.x[i] = std::min(upper[i], std::max(lower[i], out.x[i])); }
  out.residual = (A * out.x - b).norm();
  return out;
}

// ---------------------------------------------------------------------------
// Directional authority
// ---------------------------------------------------------------------------

/** Largest kappa in [0, kappaMax] such that kappa * y is realizable within the
 * box up to tol: max { kappa : min_{lower<=x<=upper} ||A x - kappa y|| <= tol }.
 *
 * r(kappa) = min_x ||A x - kappa y|| is convex in kappa (partial minimisation
 * of a jointly convex function over a convex set), so {kappa : r <= tol} is an
 * interval; when r(0) <= tol it starts at 0 and bisection is exact up to the
 * bisection resolution. Returns 0 when even kappa = 0 is not realizable. */
inline double directionalReserve(const Eigen::MatrixXd & A,
                                 const Eigen::VectorXd & y,
                                 const Eigen::VectorXd & lower,
                                 const Eigen::VectorXd & upper,
                                 double tol,
                                 double kappaMax,
                                 int bisectionIterations = 40)
{
  auto feasible = [&](double kappa)
  {
    return boxConstrainedLeastSquares(A, kappa * y, lower, upper).residual <= tol;
  };
  if(!feasible(0.0)) { return 0.0; }
  if(feasible(kappaMax)) { return kappaMax; }
  double lo = 0.0;
  double hi = kappaMax;
  for(int i = 0; i < bisectionIterations; ++i)
  {
    const double mid = 0.5 * (lo + hi);
    if(feasible(mid)) { lo = mid; }
    else { hi = mid; }
  }
  return lo;
}

/** A declared tool-frame twist the receiver must be able to realize.
 * Row order follows RBDyn Jacobians: [angular; linear], world frame. */
struct AuthorityDemand
{
  std::string label;
  Eigen::Matrix<double, 6, 1> twist = Eigen::Matrix<double, 6, 1>::Zero();
};

struct AuthorityEvaluation
{
  std::string label;
  double residual = std::numeric_limits<double>::infinity(); ///< weighted, m/s
  double reserve = 0.0;                                      ///< kappa*
  bool realizable = false;                                   ///< residual <= tol
};

/** Angular rows are scaled by a characteristic length (m) so the residual is
 * expressed in m/s; this changes the metric of the residual, not which twists
 * are exactly realizable. */
inline AuthorityEvaluation evaluateAuthorityDemand(const Eigen::MatrixXd & J,
                                                   const JointVelocityBox & box,
                                                   const AuthorityDemand & demand,
                                                   double angularLength,
                                                   double tol,
                                                   double kappaMax)
{
  AuthorityEvaluation e;
  e.label = demand.label;
  if(!box.consistent) { return e; }
  Eigen::Matrix<double, 6, 1> w;
  w << angularLength, angularLength, angularLength, 1.0, 1.0, 1.0;
  const Eigen::MatrixXd A = w.asDiagonal() * J;
  const Eigen::VectorXd y = w.asDiagonal() * demand.twist;
  e.residual = boxConstrainedLeastSquares(A, y, box.lower, box.upper).residual;
  e.realizable = e.residual <= tol;
  e.reserve = directionalReserve(A, y, box.lower, box.upper, tol, kappaMax);
  return e;
}

struct CapabilityMetrics
{
  double sigmaMin = 0.0;
  double conditionIndex = 0.0;
  double manipulability = 0.0;
};

/** Same task-row scaling for reserve and generic capability. No selection use. */
inline CapabilityMetrics scaledCapability(const Eigen::MatrixXd & J, double angularLength)
{
  CapabilityMetrics m;
  if(J.rows() != 6 || !J.allFinite()) { return m; }
  Eigen::MatrixXd A = J;
  A.topRows(3) *= angularLength;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> e(A * A.transpose());
  if(e.info() != Eigen::Success) { return m; }
  const auto lambda = e.eigenvalues().cwiseMax(0.0).eval();
  m.sigmaMin = std::sqrt(lambda.minCoeff());
  m.conditionIndex = m.sigmaMin / std::sqrt(std::max(1e-12, lambda.maxCoeff()));
  m.manipulability = std::sqrt(lambda.prod());
  return m;
}

// ---------------------------------------------------------------------------
// Candidate records, lexicographic selection and hysteresis
// ---------------------------------------------------------------------------

struct GraspCandidateRecord
{
  GraspParameters grasp;
  std::string name;
  bool geometryFeasible = false;
  bool robotFeasible = false;
  bool clearanceFeasible = false;
  bool authorityFeasible = false;
  bool admissible = false;
  /** none | geometry | ik | joint_limits | collision | security_distance |
   * clearance_floor | authority | cancelled */
  std::string rejectionLayer = "not_evaluated";
  std::string rejectionReason = "not_evaluated";
  double clearance = -std::numeric_limits<double>::infinity();
  double reserve = 0.0;   ///< min kappa* over demands and evaluated configurations
  double residual = std::numeric_limits<double>::infinity(); ///< max over demands
  double reachDistance = std::numeric_limits<double>::infinity();
};

struct GraspSelectionTolerances
{
  double clearanceTieBand = 0.005; ///< m
  double reserveTieBand = 0.10;    ///< kappa
  double reserveSaturation = 2.0;  ///< kappa above which extra reserve is not preferred
};

struct GraspSelectionOutcome
{
  int bestIndex = -1;
  std::vector<bool> inFinalBand; ///< survived the clearance and reserve bands
  double bestClearance = 0.0;
  double bestReserve = 0.0;
};

/** Filter cascade (transitive, no weights):
 *   S1 = admissible;
 *   S2 = S1 within clearanceTieBand of the best clearance in S1;
 *   S3 = S2 within reserveTieBand of the best min(reserve, saturation) in S2;
 *   pick the shortest reach distance in S3, ties by lowest grasp id. */
inline GraspSelectionOutcome selectGraspLexicographic(const std::vector<GraspCandidateRecord> & records,
                                                      const GraspSelectionTolerances & tol)
{
  GraspSelectionOutcome out;
  out.inFinalBand.assign(records.size(), false);
  double cBest = -std::numeric_limits<double>::infinity();
  for(const auto & r : records)
  {
    if(r.admissible) { cBest = std::max(cBest, r.clearance); }
  }
  if(!std::isfinite(cBest)) { return out; }
  auto sat = [&](double k) { return std::min(k, tol.reserveSaturation); };
  double kBest = -std::numeric_limits<double>::infinity();
  for(const auto & r : records)
  {
    if(r.admissible && r.clearance >= cBest - tol.clearanceTieBand) { kBest = std::max(kBest, sat(r.reserve)); }
  }
  for(std::size_t i = 0; i < records.size(); ++i)
  {
    const auto & r = records[i];
    if(!(r.admissible && r.clearance >= cBest - tol.clearanceTieBand && sat(r.reserve) >= kBest - tol.reserveTieBand))
    {
      continue;
    }
    out.inFinalBand[i] = true;
    if(out.bestIndex < 0) { out.bestIndex = static_cast<int>(i); continue; }
    const auto & b = records[static_cast<std::size_t>(out.bestIndex)];
    if(r.reachDistance < b.reachDistance
       || (r.reachDistance == b.reachDistance && r.grasp.id < b.grasp.id))
    {
      out.bestIndex = static_cast<int>(i);
    }
  }
  out.bestClearance = cBest;
  out.bestReserve = kBest;
  return out;
}

struct GraspSelectorState
{
  int incumbentId = -1;
  int challengerId = -1;
  double challengerSince = 0.0;
  bool frozen = false;
};

struct GraspSelectorDecision
{
  int selectedId = -1;
  /** frozen | hold_no_admissible | abort_to_hold | initial | keep |
   * keep_within_bands | switch_inadmissible | challenger_pending |
   * switch_dominated */
  std::string event;
  bool changed = false;
};

/** Hysteresis: the incumbent is kept while it remains admissible and survives
 * the clearance/reserve bands. A dominated incumbent is replaced only after the
 * same challenger stays best for dwell seconds. An inadmissible incumbent is
 * replaced immediately. A frozen selection never changes. */
/** trustIncumbentReevaluation = false: a re-evaluation of the incumbent from a
 * state the robot reached while executing it (e.g. a moving arm) is not allowed
 * to remove it; only a challenger that dominates beyond the bands for dwell
 * seconds replaces it. Removal of an executing incumbent is then left to the
 * fresh terminal certificate and runtime safety. This avoids treating
 * resumed-evaluation disagreement (TRIAD V2 Phase 2 section 2.4) as
 * infeasibility. */
inline GraspSelectorDecision updateGraspSelector(GraspSelectorState & state,
                                                 const std::vector<GraspCandidateRecord> & records,
                                                 const GraspSelectionOutcome & outcome,
                                                 double now,
                                                 double dwell,
                                                 bool trustIncumbentReevaluation = true)
{
  GraspSelectorDecision d;
  if(state.frozen)
  {
    d.selectedId = state.incumbentId;
    d.event = "frozen";
    return d;
  }
  if(outcome.bestIndex < 0 && state.incumbentId >= 0 && !trustIncumbentReevaluation)
  {
    d.selectedId = state.incumbentId;
    d.event = "keep_untrusted_reevaluation";
    return d;
  }
  if(outcome.bestIndex < 0)
  {
    d.event = state.incumbentId >= 0 ? "abort_to_hold" : "hold_no_admissible";
    d.changed = state.incumbentId >= 0;
    state.incumbentId = -1;
    state.challengerId = -1;
    return d;
  }
  const int bestId = records[static_cast<std::size_t>(outcome.bestIndex)].grasp.id;
  int incumbentIndex = -1;
  for(std::size_t i = 0; i < records.size(); ++i)
  {
    if(records[i].grasp.id == state.incumbentId) { incumbentIndex = static_cast<int>(i); }
  }
  if(state.incumbentId < 0)
  {
    state.incumbentId = bestId;
    state.challengerId = -1;
    d.selectedId = bestId;
    d.event = "initial";
    d.changed = true;
    return d;
  }
  const bool incumbentRejected = incumbentIndex < 0 || !records[static_cast<std::size_t>(incumbentIndex)].admissible;
  if(incumbentRejected && trustIncumbentReevaluation)
  {
    state.incumbentId = bestId;
    state.challengerId = -1;
    d.selectedId = bestId;
    d.event = "switch_inadmissible";
    d.changed = true;
    return d;
  }
  d.selectedId = state.incumbentId;
  if(bestId == state.incumbentId)
  {
    state.challengerId = -1;
    d.event = "keep";
    return d;
  }
  if(!incumbentRejected && outcome.inFinalBand[static_cast<std::size_t>(incumbentIndex)])
  {
    state.challengerId = -1;
    d.event = "keep_within_bands";
    return d;
  }
  if(state.challengerId != bestId)
  {
    state.challengerId = bestId;
    state.challengerSince = now;
    d.event = "challenger_pending";
    return d;
  }
  if(now - state.challengerSince + 1e-12 >= dwell)
  {
    state.incumbentId = bestId;
    state.challengerId = -1;
    d.selectedId = bestId;
    d.event = "switch_dominated";
    d.changed = true;
    return d;
  }
  d.event = "challenger_pending";
  return d;
}

} // namespace call_handover
