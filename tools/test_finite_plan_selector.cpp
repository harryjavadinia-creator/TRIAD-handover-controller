#include "../src/FinitePlanSelector.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

namespace
{
call_handover::FinitePlanRecord plan(std::size_t index,
                                     const std::string & candidate,
                                     const std::string & route,
                                     double cost,
                                     double reachTime,
                                     double clearance,
                                     bool valid = true)
{
  call_handover::FinitePlanRecord result;
  result.sourceIndex = index;
  result.candidateName = candidate;
  result.routeName = route;
  result.cost = cost;
  result.predictedPresentationTime = reachTime;
  result.clearance = clearance;
  result.costValid = valid;
  return result;
}
}

int main()
{
  constexpr double tolerance = 1e-9;

  {
    const std::vector<call_handover::FinitePlanRecord> plans{
        plan(0, "axisP", "direct", 1.42, 1.0, 0.05),
        plan(1, "axisP", "ring", 1.08, 1.2, 0.04),
        plan(2, "axisN", "direct", 1.31, 0.9, 0.06)};
    const auto result = call_handover::selectFinitePlan(
        plans, tolerance, false, 0.0, 0.0, 0.0);
    assert(result.success);
    assert(result.commitAdmissible);
    assert(result.selectedRecord == 1);
    assert(std::abs(result.minimumAdmissibleCost - 1.08) <= tolerance);
  }

  {
    const std::vector<call_handover::FinitePlanRecord> plans{
        plan(0, "axisP", "direct", 1.0, 1.0, 0.05),
        plan(1, "axisN", "direct", 0.5, 0.9, 0.06, false)};
    const auto result = call_handover::selectFinitePlan(
        plans, tolerance, false, 0.0, 0.0, 0.0);
    assert(!result.success);
    assert(!result.allCostsValid);
    assert(result.reason == "incomplete_or_nonfinite_cost_set");
  }

  {
    // The lower-cost plan cannot enter the reach with the required reserve;
    // constrained minimization must therefore select the faster admissible
    // plan, not fail after selecting the unconstrained minimum.
    const std::vector<call_handover::FinitePlanRecord> plans{
        plan(0, "slow", "ring", 0.50, 2.80, 0.08),
        plan(1, "fast", "direct", 0.80, 1.50, 0.05)};
    const auto result = call_handover::selectFinitePlan(
        plans, tolerance, true, 2.00, 0.25, 0.50);
    assert(result.success);
    assert(result.commitAdmissible);
    assert(result.timingAdmissibleCount == 1);
    assert(result.selectedRecord == 1);
    assert(std::abs(result.minimumAdmissibleCost - 0.80) <= tolerance);
  }

  {
    // When every plan is too slow, the fastest one may only drive the existing
    // event-time refinement. The commit guard must remain false.
    const std::vector<call_handover::FinitePlanRecord> plans{
        plan(0, "slower", "ring", 0.40, 2.80, 0.08),
        plan(1, "fastest", "direct", 0.90, 2.10, 0.05)};
    const auto result = call_handover::selectFinitePlan(
        plans, tolerance, true, 2.00, 0.25, 0.50);
    assert(result.success);
    assert(!result.commitAdmissible);
    assert(result.timingAdmissibleCount == 0);
    assert(result.selectedRecord == 1);
    assert(result.reason == "timing_refinement_required");
  }

  {
    // Exact/near ties are reproducible: reach time, clearance, candidate name,
    // route name and source index form the deterministic secondary order.
    const std::vector<call_handover::FinitePlanRecord> plans{
        plan(0, "beta", "direct", 1.0, 1.0, 0.05),
        plan(1, "alpha", "direct", 1.0 + 0.5e-9, 1.0, 0.05)};
    const auto result = call_handover::selectFinitePlan(
        plans, tolerance, false, 0.0, 0.0, 0.0);
    assert(result.success);
    assert(result.selectedRecord == 1);
    assert(std::abs(result.minimumAdmissibleCost - 1.0) <= 1e-12);
    assert(std::abs(plans[result.selectedRecord].cost
                    - result.minimumAdmissibleCost) <= tolerance);
  }

  {
    // A chain of pairwise near-ties must never drift beyond the tolerance from
    // the exact minimum.
    const std::vector<call_handover::FinitePlanRecord> plans{
        plan(0, "zeta", "direct", 1.0, 1.0, 0.05),
        plan(1, "beta", "direct", 1.0 + 0.75e-9, 1.0, 0.05),
        plan(2, "alpha", "direct", 1.0 + 1.50e-9, 1.0, 0.05)};
    const auto result = call_handover::selectFinitePlan(
        plans, tolerance, false, 0.0, 0.0, 0.0);
    assert(result.success);
    assert(result.selectedRecord == 1);
    assert(std::abs(plans[result.selectedRecord].cost
                    - result.minimumAdmissibleCost) <= tolerance);
  }

  std::cout << "finite-plan selector tests: PASS\n";
  return 0;
}
