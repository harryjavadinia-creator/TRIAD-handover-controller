// Dependency-light unit test for src/IndependentGiverModel.h (Eigen only).
//   g++ -std=c++14 -I/usr/include/eigen3 -Isrc tools/test_independent_giver_model.cpp -o /tmp/t && /tmp/t
#include "IndependentGiverModel.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

using call_handover::IndependentGiverScript;
using call_handover::independentGiverStateAt;
using call_handover::independentGiverSchedule;

static int failures = 0;
static void check(bool ok, const char * what)
{
  std::printf("%s %s\n", ok ? "PASS" : "FAIL", what);
  if(!ok) { ++failures; }
}

int main()
{
  IndependentGiverScript s;
  s.startPosition = Eigen::Vector3d(0.55, -0.56, 0.15);
  s.linearVelocity = Eigen::Vector3d(0.0, 0.08, 0.0);
  s.travelDistance = 0.40;
  s.stopDuration = 0.85;

  double cruise = 0.0, stop = 0.0;
  independentGiverSchedule(s, cruise, stop);
  check(std::abs(cruise - (0.40 - 0.5 * 0.08 * 0.85) / 0.08) < 1e-12, "cruise duration");
  check(std::abs(stop - 0.85) < 1e-12, "stop duration");

  // Constant twist before the stop: identical to V1's pre-commit simulation.
  for(double t : {0.0, 0.9, 2.5, cruise})
  {
    const auto st = independentGiverStateAt(s, t);
    check((st.position - (s.startPosition + t * s.linearVelocity)).norm() < 1e-12,
          "cruise position equals constant twist");
  }
  // Rest exactly after the configured travel distance.
  const auto rest = independentGiverStateAt(s, cruise + stop + 3.0);
  check(std::abs((rest.position - s.startPosition).norm() - 0.40) < 1e-12, "rest after travel distance");
  check(rest.atRest && rest.linearVelocity.norm() == 0.0, "zero velocity at rest");
  // Velocity continuity at both ends of the stop.
  const auto a = independentGiverStateAt(s, cruise + 1e-6);
  check((a.linearVelocity - s.linearVelocity).norm() < 1e-6, "velocity continuous at stop start");
  const auto b = independentGiverStateAt(s, cruise + stop - 1e-6);
  check(b.linearVelocity.norm() < 1e-6, "velocity continuous at stop end");
  // Monotone progress along the direction of motion.
  double prev = -1.0; bool monotone = true;
  for(int k = 0; k <= 800; ++k)
  {
    const double y = independentGiverStateAt(s, k * 0.01).position.y();
    if(y + 1e-15 < prev) { monotone = false; }
    prev = y;
  }
  check(monotone, "monotone progress");
  // Purity: the same script and time always give the same state.
  check((independentGiverStateAt(s, 3.7).position
         - independentGiverStateAt(s, 3.7).position).norm() == 0.0, "deterministic");
  // Short travel shortens the stop instead of overshooting.
  IndependentGiverScript shortTravel = s; shortTravel.travelDistance = 0.01;
  const auto sr = independentGiverStateAt(shortTravel, 10.0);
  check(std::abs((sr.position - s.startPosition).norm() - 0.01) < 1e-12, "short travel ends at distance");
  // Static giver stays at its start pose.
  IndependentGiverScript still = s; still.linearVelocity.setZero();
  check((independentGiverStateAt(still, 5.0).position - s.startPosition).norm() == 0.0, "static giver");

  std::printf("%s (%d failure(s))\n", failures ? "FAIL" : "PASS", failures);
  return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
