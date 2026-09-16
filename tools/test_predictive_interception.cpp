// Deterministic unit tests for src/PredictiveInterception.h (TRIAD-lite Phase C).
// Build: tools/run_control_aware_supervisor_unit_tests.sh

#include "../src/PredictiveInterception.h"

#include <cstdio>
#include <cstdlib>

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

double bruteForceLowerBound(const Eigen::Vector3d & d, const Eigen::Vector3d & v, double speed)
{
  for(double tau = 0.0; tau <= 50.0; tau += 1e-4)
  {
    if((d + v * tau).norm() <= speed * tau + 1e-9) { return tau; }
  }
  return std::numeric_limits<double>::infinity();
}

void testLowerBound()
{
  // stationary target
  CHECK(near(interceptionPositionLowerBound(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d::Zero(), 0.5), 2.0, 1e-12));
  // approaching target: closing speed 1
  CHECK(near(interceptionPositionLowerBound(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(-0.5, 0, 0), 0.5), 1.0, 1e-9));
  // target faster than the pursuer, moving away: impossible
  CHECK(std::isinf(interceptionPositionLowerBound(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(1, 0, 0), 0.5)));
  // target faster but approaching: first feasible time exists
  CHECK(near(interceptionPositionLowerBound(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(-1, 0, 0), 0.5), 2.0 / 3.0, 1e-9));
  // coincident
  CHECK(interceptionPositionLowerBound(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.1, 0, 0), 0.3) == 0.0);
  // random cases against brute force
  const Eigen::Vector3d ds[] = {Eigen::Vector3d(0.4, -0.2, 0.1), Eigen::Vector3d(-0.3, 0.5, 0.2), Eigen::Vector3d(0.1, 0.1, -0.6)};
  const Eigen::Vector3d vs[] = {Eigen::Vector3d(0.08, 0, 0), Eigen::Vector3d(0, -0.2, 0.05), Eigen::Vector3d(0.3, 0.1, 0.0)};
  for(const auto & d : ds)
  {
    for(const auto & v : vs)
    {
      for(double speed : {0.1, 0.38})
      {
        const double exact = interceptionPositionLowerBound(d, v, speed);
        const double brute = bruteForceLowerBound(d, v, speed);
        CHECK((std::isinf(exact) && std::isinf(brute)) || near(exact, brute, 2e-4));
      }
    }
  }
  CHECK(near(interceptionOrientationLowerBound(1.0, 0.5, 1.5), 0.5, 1e-12));
  CHECK(interceptionOrientationLowerBound(0.0, 0.5, 1.5) == 0.0);
}

void testSchedule()
{
  InterceptionScheduleSpec s;
  s.lowerBound = 0.3;
  s.latencyAnchor = 0.6;
  s.linearSpeed = 0.0;  // rest: single event
  double step = 0.0;
  auto e = interceptionEventSchedule(s, &step);
  CHECK(e.size() == 1u && near(e[0], 0.6, 1e-12) && std::isinf(step));
  s.linearSpeed = 0.08;  // dtau = 0.015 / 0.08 = 0.1875
  s.horizon = 2.0;
  e = interceptionEventSchedule(s, &step);
  CHECK(near(step, 0.1875, 1e-12));
  CHECK(near(e.front(), 0.6, 1e-12));
  for(std::size_t i = 1; i < e.size(); ++i) { CHECK(near(e[i] - e[i - 1], 0.1875, 1e-9)); }
  CHECK(e.back() <= 2.0 + 1e-12);
  s.minimumStep = 0.5;  // numerical floor dominates
  e = interceptionEventSchedule(s, &step);
  CHECK(near(step, 0.5, 1e-12));
  s.lowerBound = 5.0;  // beyond the horizon: no event
  CHECK(interceptionEventSchedule(s).empty());
}

// Analytic moving-target oracle: feasible iff the pursuer (speed bound) reaches the
// target position AND the meeting point lies inside a workspace ball.
struct Oracle
{
  Eigen::Vector3d p0, v, pursuer;
  double speed, radius;
  bool feasible(double tau) const
  {
    const Eigen::Vector3d target = p0 + v * tau;
    return (target - pursuer).norm() <= speed * tau + 1e-12 && target.norm() <= radius;
  }
};

double firstFeasible(const Oracle & o, const InterceptionScheduleSpec & s)
{
  for(double tau : interceptionEventSchedule(s))
  {
    if(o.feasible(tau)) { return tau; }
  }
  return std::numeric_limits<double>::infinity();
}

void testEarliestAndImpossibleInterception()
{
  Oracle o{Eigen::Vector3d(0.6, -0.4, 0.3), Eigen::Vector3d(0, 0.08, 0), Eigen::Vector3d(0.3, 0.0, 0.4), 0.38, 0.85};
  InterceptionScheduleSpec s;
  s.lowerBound = interceptionPositionLowerBound(o.p0 - o.pursuer, o.v, o.speed);
  s.latencyAnchor = 0.2;
  s.linearSpeed = o.v.norm();
  s.horizon = 8.0;
  double step = 0.0;
  interceptionEventSchedule(s, &step);
  const double tau = firstFeasible(o, s);
  const double exact = std::max(s.lowerBound, s.latencyAnchor);
  CHECK(std::isfinite(tau));
  CHECK(tau >= exact - 1e-12 && tau <= exact + step + 1e-12);  // earliest within one resolution step
  // the meeting pose at the returned event differs from the exact first-crossing pose by <= eps_p
  CHECK((o.v * (tau - exact)).norm() <= s.epsPosition + 1e-12);

  // impossible: target leaves the workspace before the pursuer can reach it
  Oracle away{Eigen::Vector3d(0.8, 0.0, 0.0), Eigen::Vector3d(0.3, 0, 0), Eigen::Vector3d(-0.3, 0.0, 0.0), 0.38, 0.85};
  InterceptionScheduleSpec sa;
  sa.lowerBound = interceptionPositionLowerBound(away.p0 - away.pursuer, away.v, away.speed);
  sa.linearSpeed = away.v.norm();
  CHECK(std::isinf(firstFeasible(away, sa)));

  // changed object motion changes the solution (replanning input)
  Oracle faster = o;
  faster.v = Eigen::Vector3d(0, 0.2, 0);
  InterceptionScheduleSpec sf = s;
  sf.lowerBound = interceptionPositionLowerBound(faster.p0 - faster.pursuer, faster.v, faster.speed);
  sf.linearSpeed = faster.v.norm();
  CHECK(!near(sf.lowerBound, s.lowerBound, 1e-6));
  CHECK(std::isfinite(firstFeasible(faster, sf)) != std::isfinite(firstFeasible(o, s))
        || !near(firstFeasible(faster, sf), tau, 1e-9));
}

void testRendezvousReference()
{
  const double t0 = 1.0;
  const double T = 1.5;
  const Eigen::Vector3d p0(0.2, 0.1, 0.5);
  const Eigen::Vector3d v0(0.05, -0.02, 0.1);
  const Eigen::Matrix3d R0 = Eigen::AngleAxisd(0.6, Eigen::Vector3d(0.2, 0.9, 0.1).normalized()).toRotationMatrix();
  const Eigen::Vector3d pGstart(0.7, -0.3, 0.4);
  const Eigen::Vector3d vG(0.0, 0.08, 0.0);
  const Eigen::Vector3d wG(0.0, 0.0, 0.2);
  const Eigen::Matrix3d RGstart = Eigen::AngleAxisd(-0.4, Eigen::Vector3d::UnitX()).toRotationMatrix();
  auto target = [&](double t, Eigen::Vector3d & p, Eigen::Matrix3d & R)
  {
    p = pGstart + vG * (t - t0);
    R = rotationExp(wG * (t - t0)) * RGstart;
  };
  auto ref = [&](double t)
  {
    Eigen::Vector3d pG;
    Eigen::Matrix3d RG;
    target(t, pG, RG);
    return rendezvousReference(t, t0, T, p0, v0, R0, pGstart, vG, RGstart, pG, vG, RG, wG);
  };
  const auto a = ref(t0);
  CHECK((a.position - p0).norm() < 1e-12);
  CHECK((a.linearVelocity - v0).norm() < 1e-12);  // patch continuity
  CHECK((a.rotation - R0).norm() < 1e-9);
  const auto b = ref(t0 + T);
  Eigen::Vector3d pG;
  Eigen::Matrix3d RG;
  target(t0 + T, pG, RG);
  CHECK((b.position - pG).norm() < 1e-12);         // rendezvous position
  CHECK((b.linearVelocity - vG).norm() < 1e-12);   // velocity matching
  CHECK((b.rotation - RG).norm() < 1e-9);
  const auto after = ref(t0 + T + 0.7);            // after the rendezvous the reference follows the target
  target(t0 + T + 0.7, pG, RG);
  CHECK((after.position - pG).norm() < 1e-12 && (after.linearVelocity - vG).norm() < 1e-12);
  // analytic linear velocity equals the finite difference inside the patch
  const double tm = t0 + 0.37 * T;
  const double h = 1e-6;
  const Eigen::Vector3d fd = (ref(tm + h).position - ref(tm - h).position) / (2 * h);
  CHECK((fd - ref(tm).linearVelocity).norm() < 1e-6);
}

InterceptionRecord irec(int id, double tau, double clearance, double reserve, double capability, bool ok = true)
{
  InterceptionRecord r;
  r.base.grasp.id = id;
  r.base.admissible = ok;
  r.base.clearance = clearance;
  r.base.reserve = reserve;
  r.tau = tau;
  r.capability = capability;
  return r;
}

void testSelectionAndHysteresis()
{
  InterceptionSelectionTolerances tol;  // tau band 0.19 s
  std::vector<InterceptionRecord> rs = {
      irec(0, 1.00, 0.050, 1.1, 0.10),
      irec(1, 1.10, 0.080, 1.9, 0.05),  // same encounter band, more clearance and reserve
      irec(2, 1.15, 0.060, 1.2, 0.30),  // same band, best capability
      irec(3, 0.70, 0.090, 3.0, 0.40, false),  // earliest but inadmissible
      irec(4, 2.00, 0.200, 5.0, 0.90)   // later encounter never preferred
  };
  const auto b1 = selectEarliestInterception(rs, tol, InterceptionTieBreak::Clearance);
  CHECK(b1.bestIndex == 1);
  const auto full = selectEarliestInterception(rs, tol, InterceptionTieBreak::AuthorityReserve);
  CHECK(full.bestIndex == 1);
  const auto b2 = selectEarliestInterception(rs, tol, InterceptionTieBreak::Capability);
  CHECK(b2.bestIndex == 2);
  CHECK(!b1.inFinalBand[4] && !b1.inFinalBand[3]);
  // earliest dominates when outside the band
  std::vector<InterceptionRecord> early = {irec(0, 1.00, 0.030, 1.0, 0.0), irec(1, 1.50, 0.200, 5.0, 1.0)};
  CHECK(selectEarliestInterception(early, tol, InterceptionTieBreak::AuthorityReserve).bestIndex == 0);
  // no admissible
  std::vector<InterceptionRecord> none = {irec(0, 1.0, 0.05, 1.0, 0.1, false)};
  CHECK(selectEarliestInterception(none, tol, InterceptionTieBreak::Clearance).bestIndex < 0);

  // Hysteresis with the existing selector: a challenger inside the band never
  // replaces the incumbent; an infeasible incumbent is replaced immediately.
  GraspSelectorState st;
  auto baseRecords = [](const std::vector<InterceptionRecord> & v)
  {
    std::vector<GraspCandidateRecord> out;
    for(const auto & r : v) { out.push_back(r.base); }
    return out;
  };
  std::vector<InterceptionRecord> a = {irec(5, 1.00, 0.08, 1.5, 0.1), irec(6, 1.05, 0.08, 1.5, 0.1)};
  std::vector<InterceptionRecord> b = {irec(5, 1.06, 0.08, 1.5, 0.1), irec(6, 1.00, 0.08, 1.5, 0.1)};
  auto d = updateGraspSelector(st, baseRecords(a), selectEarliestInterception(a, tol, InterceptionTieBreak::Clearance), 0.0, 0.3);
  CHECK(d.selectedId == 5);
  int switches = 0;
  for(int k = 1; k <= 30; ++k)
  {
    const auto & r = (k % 2) ? b : a;
    d = updateGraspSelector(st, baseRecords(r), selectEarliestInterception(r, tol, InterceptionTieBreak::Clearance),
                            0.05 * k, 0.3);
    if(d.changed) { ++switches; }
  }
  CHECK(switches == 0);
  std::vector<InterceptionRecord> lost = {irec(5, 1.0, 0.08, 1.5, 0.1, false), irec(6, 1.2, 0.08, 1.5, 0.1)};
  d = updateGraspSelector(st, baseRecords(lost), selectEarliestInterception(lost, tol, InterceptionTieBreak::Clearance), 2.0, 0.3);
  CHECK(d.event == "switch_inadmissible" && d.selectedId == 6);
}
} // namespace

int main()
{
  testLowerBound();
  testSchedule();
  testEarliestAndImpossibleInterception();
  testRendezvousReference();
  testSelectionAndHysteresis();
  if(failures != 0)
  {
    std::fprintf(stderr, "%d check(s) failed\n", failures);
    return EXIT_FAILURE;
  }
  std::printf("test_predictive_interception: all checks passed\n");
  return EXIT_SUCCESS;
}
