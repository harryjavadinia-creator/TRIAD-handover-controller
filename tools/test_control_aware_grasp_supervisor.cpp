// Deterministic unit tests for src/ControlAwareGraspSupervisor.h.
// Build: tools/run_control_aware_supervisor_unit_tests.sh

#include "../src/ControlAwareGraspSupervisor.h"

#include <cstdio>
#include <cstdlib>
#include <random>
#include <set>

namespace
{
int failures = 0;
#define CHECK(cond)                                                                   \
  do                                                                                  \
  {                                                                                   \
    if(!(cond))                                                                       \
    {                                                                                 \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);            \
      ++failures;                                                                     \
    }                                                                                 \
  } while(0)

using namespace call_handover;

bool near(double a, double b, double tol) { return std::abs(a - b) <= tol; }

// The reserve is the boundary of {kappa : residual(kappa y) <= tol}; it exceeds
// the exact supremum by at most tol / ||W y|| (residual grows at most linearly).
bool reserveNear(double reserve, double exact, double tol, double weightedDemandNorm)
{
  return reserve >= exact - 1e-9 && reserve <= exact + tol / weightedDemandNorm + 1e-9;
}

// Brute-force reference: enumerate every free/lower/upper assignment.
double bruteForceBoxLs(const Eigen::MatrixXd & A, const Eigen::VectorXd & b, const Eigen::VectorXd & lo,
                       const Eigen::VectorXd & hi, double rho)
{
  const int n = static_cast<int>(A.cols());
  const Eigen::MatrixXd H = A.transpose() * A + rho * Eigen::MatrixXd::Identity(n, n);
  const Eigen::VectorXd g = -A.transpose() * b;
  double best = std::numeric_limits<double>::infinity();
  int total = 1;
  for(int i = 0; i < n; ++i) { total *= 3; }
  for(int code = 0; code < total; ++code)
  {
    std::vector<int> st(static_cast<std::size_t>(n));
    int c = code;
    for(int i = 0; i < n; ++i) { st[static_cast<std::size_t>(i)] = c % 3; c /= 3; }
    Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
    std::vector<int> F;
    for(int i = 0; i < n; ++i)
    {
      if(st[static_cast<std::size_t>(i)] == 1) { x[i] = lo[i]; }
      else if(st[static_cast<std::size_t>(i)] == 2) { x[i] = hi[i]; }
      else { F.push_back(i); }
    }
    if(!F.empty())
    {
      const int m = static_cast<int>(F.size());
      Eigen::MatrixXd HFF(m, m);
      Eigen::VectorXd rhs(m);
      for(int r = 0; r < m; ++r)
      {
        double s = g[F[static_cast<std::size_t>(r)]];
        for(int k = 0; k < n; ++k)
        {
          if(st[static_cast<std::size_t>(k)] != 0) { s += H(F[static_cast<std::size_t>(r)], k) * x[k]; }
        }
        rhs[r] = -s;
        for(int k = 0; k < m; ++k) { HFF(r, k) = H(F[static_cast<std::size_t>(r)], F[static_cast<std::size_t>(k)]); }
      }
      const Eigen::VectorXd xF = HFF.ldlt().solve(rhs);
      for(int r = 0; r < m; ++r) { x[F[static_cast<std::size_t>(r)]] = xF[r]; }
    }
    bool ok = true;
    for(int i = 0; i < n; ++i) { ok = ok && x[i] >= lo[i] - 1e-9 && x[i] <= hi[i] + 1e-9; }
    if(!ok) { continue; }
    const double f = 0.5 * (A * x - b).squaredNorm() + 0.5 * rho * x.squaredNorm();
    best = std::min(best, f);
  }
  return best;
}

void testGraspFamily()
{
  const auto family = generateGraspFamily(32);
  CHECK(family.size() == 64u);
  std::set<std::pair<int, long>> keys;
  for(std::size_t i = 0; i < family.size(); ++i)
  {
    CHECK(family[i].id == static_cast<int>(i));
    CHECK(family[i].sign == (i < 32 ? 1 : -1));
    keys.insert({family[i].sign, std::lround(family[i].phi * 1e6)});
  }
  CHECK(keys.size() == 64u);
  CHECK(near(family[1].phi - family[0].phi, 2.0 * M_PI / 32.0, 1e-12));
  CHECK(generateGraspFamily(2).size() == 8u); // minimum 4 per sign, as in V2

  // Frame geometry: orthonormal, right-handed, z along +/- handle axis, y orthogonal to it.
  const Eigen::Vector3d axis = Eigen::Vector3d(0.3, -0.2, 0.9).normalized();
  const Eigen::Vector3d outward(1.0, 0.4, 0.1);
  for(const auto & g : family)
  {
    const Eigen::Matrix3d R = graspFrameRotation(axis, outward, g);
    CHECK((R.transpose() * R - Eigen::Matrix3d::Identity()).norm() < 1e-9);
    CHECK(near(R.determinant(), 1.0, 1e-9));
    CHECK(near(R.col(2).dot(axis), static_cast<double>(g.sign), 1e-9));
    CHECK(std::abs(R.col(1).dot(axis)) < 1e-9);
  }
  // phi = 0 recovers the projected outward direction.
  Eigen::Vector3d proj = outward - axis * axis.dot(outward);
  proj.normalize();
  CHECK((graspFrameRotation(axis, outward, family[0]).col(1) - proj).norm() < 1e-9);
}

void testVelocityBox()
{
  QpKinematicsLimits lim; // 0.95, 0.1, 0.01, 0.5
  Eigen::VectorXd qMin(3), qMax(3), vMin(3), vMax(3), q(3);
  const double inf = std::numeric_limits<double>::infinity();
  qMin << -2.0, -inf, -2.0;
  qMax << 2.0, inf, 2.0;
  vMin << -1.0, -1.2, -1.0;
  vMax << 1.0, 1.2, 1.0;
  // joint 0 free, joint 1 continuous, joint 2 inside the lower damper zone.
  q << 0.0, 100.0, -2.0 + 0.2;
  const auto box = qpJointVelocityBox(q, qMin, qMax, vMin, vMax, lim);
  CHECK(near(box.lower[0], -0.95, 1e-12) && near(box.upper[0], 0.95, 1e-12));
  CHECK(near(box.lower[1], -1.14, 1e-12) && near(box.upper[1], 1.14, 1e-12));
  CHECK(box.damper[0] == 0 && box.damper[1] == 0 && box.damper[2] == -1);
  // range 4: iDist 0.4, sDist 0.04, d = 0.2 -> lower = -0.5 (0.16/0.36)
  CHECK(near(box.lower[2], -0.5 * (0.2 - 0.04) / (0.4 - 0.04), 1e-12));
  CHECK(near(box.upper[2], 0.95, 1e-12));
  CHECK(!box.insideSecurity && box.consistent);

  q << 1.99, 0.0, 0.0; // within sDist of the upper limit: forced away
  const auto sec = qpJointVelocityBox(q, qMin, qMax, vMin, vMax, lim);
  CHECK(sec.insideSecurity);
  CHECK(sec.damper[0] == 1 && sec.upper[0] < 0.0);
}

void testBoxLeastSquares()
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  const double rho = 1e-10;
  for(int trial = 0; trial < 60; ++trial)
  {
    const int rows = trial % 2 == 0 ? 6 : 3;
    const int cols = trial % 3 == 0 ? 7 : 5;
    Eigen::MatrixXd A(rows, cols);
    Eigen::VectorXd b(rows), lo(cols), hi(cols);
    for(int i = 0; i < rows; ++i)
    {
      b[i] = 3.0 * u(rng);
      for(int j = 0; j < cols; ++j) { A(i, j) = u(rng); }
    }
    for(int j = 0; j < cols; ++j)
    {
      const double a = 0.6 * u(rng), c = 0.6 * u(rng);
      lo[j] = std::min(a, c) - 0.05;
      hi[j] = std::max(a, c) + 0.05;
    }
    const auto r = boxConstrainedLeastSquares(A, b, lo, hi, rho);
    CHECK(r.converged);
    const double f = 0.5 * (A * r.x - b).squaredNorm() + 0.5 * rho * r.x.squaredNorm();
    CHECK(near(f, bruteForceBoxLs(A, b, lo, hi, rho), 1e-7));
    for(int j = 0; j < cols; ++j) { CHECK(r.x[j] >= lo[j] - 1e-12 && r.x[j] <= hi[j] + 1e-12); }
  }
}

void testReserveWellConditionedVersusConstrained()
{
  // 6 x 7 "Jacobian": identity on the first 6 joints, the 7th duplicates joint 0.
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 7);
  for(int i = 0; i < 6; ++i) { J(i, i) = 1.0; }
  J(0, 6) = 1.0;
  JointVelocityBox box;
  box.lower = -Eigen::VectorXd::Ones(7);
  box.upper = Eigen::VectorXd::Ones(7);
  box.damper.assign(7, 0);

  AuthorityDemand linear;
  linear.twist << 0, 0, 0, 0.5, 0, 0; // linear x (row 3) at 0.5
  const auto free = evaluateAuthorityDemand(J, box, linear, 0.2, 1e-6, 8.0);
  CHECK(free.realizable);
  CHECK(reserveNear(free.reserve, 2.0, 1e-6, 0.5));

  AuthorityDemand angular;
  angular.twist << 0.5, 0, 0, 0, 0, 0; // row 0 driven by joints 0 and 6
  const auto redundant = evaluateAuthorityDemand(J, box, angular, 0.2, 1e-6, 8.0);
  CHECK(reserveNear(redundant.reserve, 4.0, 1e-6, 0.2 * 0.5));

  // Constrain the direction: joint 3 may only move at 0.1 in the positive sense (damper).
  JointVelocityBox limited = box;
  limited.upper[3] = 0.1;
  limited.damper[3] = 1;
  const auto constrained = evaluateAuthorityDemand(J, limited, linear, 0.2, 1e-6, 8.0);
  CHECK(!constrained.realizable);
  CHECK(reserveNear(constrained.reserve, 0.2, 1e-6, 0.5));
  // The opposite direction is unaffected by that upper bound.
  AuthorityDemand opposite = linear;
  opposite.twist[3] = -0.5;
  CHECK(reserveNear(evaluateAuthorityDemand(J, limited, opposite, 0.2, 1e-6, 8.0).reserve, 2.0, 1e-6, 0.5));

  // Rank deficiency: a direction outside the column space is never realizable.
  Eigen::MatrixXd Jd = J;
  Jd.row(5).setZero();
  AuthorityDemand missing;
  missing.twist << 0, 0, 0, 0, 0, 0.3;
  const auto none = evaluateAuthorityDemand(Jd, box, missing, 0.2, 1e-6, 8.0);
  CHECK(!none.realizable && reserveNear(none.reserve, 0.0, 1e-6, 0.3));

  // Inconsistent box -> not realizable.
  JointVelocityBox broken = box;
  broken.consistent = false;
  CHECK(!evaluateAuthorityDemand(J, broken, linear, 0.2, 1e-6, 8.0).realizable);
}

void testObviousJointLimitCase()
{
  // Planar 2R-like 3x2 map where joint 0 sits in its lower damper zone.
  QpKinematicsLimits lim;
  Eigen::VectorXd qMin(2), qMax(2), vMin(2), vMax(2), q(2);
  qMin << -1.0, -1.0;
  qMax << 1.0, 1.0;
  vMin << -1.0, -1.0;
  vMax << 1.0, 1.0;
  q << -1.0 + 0.05, 0.0; // d = 0.05, iDist 0.2, sDist 0.02
  const auto box = qpJointVelocityBox(q, qMin, qMax, vMin, vMax, lim);
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 2);
  J(3, 0) = 1.0; // linear x from joint 0 only
  J(4, 1) = 1.0;
  AuthorityDemand towardLimit;
  towardLimit.twist << 0, 0, 0, -0.3, 0, 0;
  AuthorityDemand awayFromLimit;
  awayFromLimit.twist << 0, 0, 0, 0.3, 0, 0;
  const auto toward = evaluateAuthorityDemand(J, box, towardLimit, 0.2, 1e-6, 8.0);
  const auto away = evaluateAuthorityDemand(J, box, awayFromLimit, 0.2, 1e-6, 8.0);
  const double damperBound = 0.5 * (0.05 - 0.02) / (0.2 - 0.02);
  CHECK(reserveNear(toward.reserve, damperBound / 0.3, 1e-6, 0.3));
  CHECK(!toward.realizable);
  CHECK(away.realizable && reserveNear(away.reserve, 0.95 / 0.3, 1e-6, 0.3));
}

GraspCandidateRecord rec(int id, bool admissible, double clearance, double reserve, double reach)
{
  GraspCandidateRecord r;
  r.grasp.id = id;
  r.admissible = admissible;
  r.clearance = clearance;
  r.reserve = reserve;
  r.reachDistance = reach;
  return r;
}

void testSelection()
{
  GraspSelectionTolerances tol; // 5 mm, 0.1, sat 2
  std::vector<GraspCandidateRecord> rs = {
      rec(0, false, 0.20, 5.0, 0.1),  // inadmissible, ignored
      rec(1, true, 0.080, 1.2, 0.50), // clearance best band
      rec(2, true, 0.077, 1.9, 0.60), // within 5 mm; best reserve band
      rec(3, true, 0.060, 3.0, 0.10), // clearance outside band
      rec(4, true, 0.078, 1.85, 0.40) // within bands, shortest reach among S3
  };
  auto out = selectGraspLexicographic(rs, tol);
  CHECK(out.bestIndex == 4);
  CHECK(!out.inFinalBand[0] && !out.inFinalBand[1] && out.inFinalBand[2] && !out.inFinalBand[3] && out.inFinalBand[4]);
  // Saturation: reserves 2.5 and 6.0 are equal preference.
  std::vector<GraspCandidateRecord> sat = {rec(0, true, 0.08, 6.0, 0.9), rec(1, true, 0.08, 2.5, 0.3)};
  CHECK(selectGraspLexicographic(sat, tol).bestIndex == 1);
  // Deterministic tie by id.
  std::vector<GraspCandidateRecord> tie = {rec(5, true, 0.08, 1.0, 0.3), rec(2, true, 0.08, 1.0, 0.3)};
  CHECK(tie[static_cast<std::size_t>(selectGraspLexicographic(tie, tol).bestIndex)].grasp.id == 2);
  std::vector<GraspCandidateRecord> empty = {rec(0, false, 0.1, 1.0, 0.1)};
  CHECK(selectGraspLexicographic(empty, tol).bestIndex < 0);
}

void testHysteresisAndFreeze()
{
  GraspSelectionTolerances tol;
  GraspSelectorState st;
  const double dwell = 0.3;

  // Two candidates alternating as "best" only by reach distance: no chatter.
  std::vector<GraspCandidateRecord> a = {rec(1, true, 0.08, 1.5, 0.30), rec(2, true, 0.08, 1.5, 0.31)};
  std::vector<GraspCandidateRecord> b = {rec(1, true, 0.08, 1.5, 0.31), rec(2, true, 0.08, 1.5, 0.30)};
  auto d = updateGraspSelector(st, a, selectGraspLexicographic(a, tol), 0.0, dwell);
  CHECK(d.event == "initial" && d.selectedId == 1);
  int switches = 0;
  for(int k = 1; k <= 40; ++k)
  {
    const auto & rs = (k % 2) ? b : a;
    d = updateGraspSelector(st, rs, selectGraspLexicographic(rs, tol), 0.05 * k, dwell);
    if(d.changed) { ++switches; }
    CHECK(d.selectedId == 1);
  }
  CHECK(switches == 0);

  // Dominated incumbent: switch only after the challenger persists for dwell.
  std::vector<GraspCandidateRecord> dom = {rec(1, true, 0.050, 1.5, 0.3), rec(2, true, 0.080, 1.5, 0.3)};
  d = updateGraspSelector(st, dom, selectGraspLexicographic(dom, tol), 3.00, dwell);
  CHECK(d.event == "challenger_pending" && d.selectedId == 1);
  d = updateGraspSelector(st, dom, selectGraspLexicographic(dom, tol), 3.20, dwell);
  CHECK(d.event == "challenger_pending" && d.selectedId == 1);
  d = updateGraspSelector(st, dom, selectGraspLexicographic(dom, tol), 3.31, dwell);
  CHECK(d.event == "switch_dominated" && d.selectedId == 2 && d.changed);

  // Inadmissible incumbent: immediate switch.
  std::vector<GraspCandidateRecord> bad = {rec(1, true, 0.08, 1.5, 0.3), rec(2, false, 0.08, 1.5, 0.3)};
  d = updateGraspSelector(st, bad, selectGraspLexicographic(bad, tol), 3.32, dwell);
  CHECK(d.event == "switch_inadmissible" && d.selectedId == 1);

  // Freeze: nothing changes afterwards, even when the frozen grasp becomes inadmissible.
  st.frozen = true;
  std::vector<GraspCandidateRecord> later = {rec(1, false, 0.08, 1.5, 0.3), rec(2, true, 0.2, 5.0, 0.1)};
  d = updateGraspSelector(st, later, selectGraspLexicographic(later, tol), 10.0, dwell);
  CHECK(d.event == "frozen" && d.selectedId == 1 && !d.changed);

  // No admissible grasp: an unfrozen incumbent is released (hold).
  GraspSelectorState st2;
  st2.incumbentId = 4;
  std::vector<GraspCandidateRecord> none = {rec(4, false, 0.08, 1.5, 0.3)};
  d = updateGraspSelector(st2, none, selectGraspLexicographic(none, tol), 0.0, dwell);
  CHECK(d.event == "abort_to_hold" && d.selectedId < 0 && st2.incumbentId < 0);

  // Untrusted re-evaluation (executing incumbent): no abort, no immediate switch;
  // a challenger still replaces it after the dwell.
  GraspSelectorState st3;
  st3.incumbentId = 7;
  std::vector<GraspCandidateRecord> reeval = {rec(7, false, 0.08, 1.5, 0.3)};
  d = updateGraspSelector(st3, reeval, selectGraspLexicographic(reeval, tol), 0.0, dwell, false);
  CHECK(d.event == "keep_untrusted_reevaluation" && d.selectedId == 7 && st3.incumbentId == 7 && !d.changed);
  std::vector<GraspCandidateRecord> challenger = {rec(7, false, 0.08, 1.5, 0.3), rec(8, true, 0.08, 1.5, 0.3)};
  d = updateGraspSelector(st3, challenger, selectGraspLexicographic(challenger, tol), 1.0, dwell, false);
  CHECK(d.event == "challenger_pending" && d.selectedId == 7);
  d = updateGraspSelector(st3, challenger, selectGraspLexicographic(challenger, tol), 1.31, dwell, false);
  CHECK(d.event == "switch_dominated" && d.selectedId == 8 && d.changed);
}
} // namespace

int main()
{
  testGraspFamily();
  testVelocityBox();
  testBoxLeastSquares();
  testReserveWellConditionedVersusConstrained();
  testObviousJointLimitCase();
  testSelection();
  testHysteresisAndFreeze();
  if(failures != 0)
  {
    std::fprintf(stderr, "%d check(s) failed\n", failures);
    return EXIT_FAILURE;
  }
  std::printf("test_control_aware_grasp_supervisor: all checks passed\n");
  return EXIT_SUCCESS;
}
