#include "../src/FiniteEventPlanSelector.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

namespace
{

call_handover::FiniteEventPlanRecord record(
    std::size_t source,
    std::size_t hypothesis,
    double eventLead,
    double absoluteEvent,
    double reach,
    double execution,
    double motionCost,
    double globalCost,
    const std::string & candidate,
    const std::string & route,
    bool costValid = true)
{
  call_handover::FiniteEventPlanRecord out;
  out.sourceIndex = source;
  out.hypothesisIndex = hypothesis;
  out.costValid = costValid;
  out.motionCost = motionCost;
  out.globalCost = globalCost;
  out.eventLead = eventLead;
  out.eventPresentationTime = absoluteEvent;
  out.predictedPresentationDuration = reach;
  out.predictedExecutionDuration = execution;
  out.clearance = 0.08;
  out.candidateName = candidate;
  out.routeName = route;
  return out;
}

} // namespace

int main()
{
  using call_handover::selectFiniteEventPlan;

  {
    const double global = call_handover::extendMotionCostToSearchEpoch(
        0.50, 0.40, 8.0, 5.0, 2.0);
    assert(std::abs(global - 0.65) < 1e-12);
    assert(!std::isfinite(call_handover::extendMotionCostToSearchEpoch(
        0.50, 0.40, 0.0, 5.0, 2.0)));
  }

  {
    // A later event is allowed to win when its complete global objective is
    // lower; this is the distinction from earliest-feasible selection.
    std::vector<call_handover::FiniteEventPlanRecord> records = {
        record(0, 1, 4.0, 14.0, 2.0, 7.0, 0.60, 0.70,
               "grasp_a", "direct"),
        record(1, 2, 6.0, 16.0, 2.2, 6.0, 0.40, 0.55,
               "grasp_b", "ring")};
    const auto selected = selectFiniteEventPlan(
        records, 11.0, 0.05, 1.6, 1e-9);
    assert(selected.success);
    assert(selected.commitAdmissible);
    assert(selected.selectedRecord == 1);
    assert(std::abs(selected.minimumAdmissibleGlobalCost - 0.55) < 1e-12);
  }

  {
    // Final readmission removes an event that expired while the finite set was
    // being evaluated, even if it has the lowest numerical cost.
    std::vector<call_handover::FiniteEventPlanRecord> records = {
        record(0, 1, 3.0, 12.0, 2.0, 6.0, 0.30, 0.35,
               "expired", "direct"),
        record(1, 2, 7.0, 17.0, 2.0, 6.0, 0.50, 0.60,
               "live", "ring")};
    const auto selected = selectFiniteEventPlan(
        records, 10.5, 0.05, 1.6, 1e-9);
    assert(selected.success);
    assert(selected.timingAdmissibleCount == 1);
    assert(selected.selectedRecord == 1);
  }

  {
    // A single non-finite complete-plan objective is excluded on its own; it
    // must not remove an otherwise cost-valid sibling from the pool.
    std::vector<call_handover::FiniteEventPlanRecord> records = {
        record(0, 1, 5.0, 15.0, 2.0, 6.0, 0.40, 0.50,
               "valid", "direct"),
        record(1, 1, 5.0, 15.0, 2.0, 6.0, 0.45,
               std::numeric_limits<double>::infinity(),
               "invalid", "ring")};
    const auto selected = selectFiniteEventPlan(
        records, 11.0, 0.05, 1.6, 1e-9);
    assert(selected.success);
    assert(selected.costValidCount == 1);
    assert(!selected.allCostsValid);
    assert(selected.selectedRecord == 0);
    assert(std::abs(selected.minimumAdmissibleGlobalCost - 0.50) < 1e-12);
  }

  {
    // GROUND_NEAR hypothesis-10 shape: 9 complete plans, 8 individually
    // cost-valid, 1 cost-invalid. The invalid record is given the
    // numerically lowest cost on purpose: if per-candidate exclusion were
    // broken, it would wrongly win the argmin.
    std::vector<call_handover::FiniteEventPlanRecord> records;
    for(int i = 0; i < 8; ++i)
    {
      records.push_back(record(
          static_cast<std::size_t>(i), 10, 5.5, 15.5, 2.0, 6.0,
          0.55 + 0.02 * i, 0.60 + 0.02 * i,
          "axisP_side_45deg", "route_" + std::to_string(i)));
    }
    records.push_back(record(
        8, 10, 5.5, 15.5, 2.0, 6.0, 0.05, 0.05,
        "axisP_side_337deg", "ring140mm_1of8", false));
    const auto selected = selectFiniteEventPlan(
        records, 11.0, 0.05, 1.6, 1e-9);
    assert(selected.success);
    assert(selected.completePlanCount == 9);
    assert(selected.costValidCount == 8);
    assert(!selected.allCostsValid);
    assert(selected.selectedRecord != 8);
    assert(std::abs(selected.minimumAdmissibleGlobalCost - 0.60) < 1e-9);
  }

  {
    // If every candidate at the only hypothesis fails the cost audit, the
    // search fails safely rather than selecting from an empty valid set.
    std::vector<call_handover::FiniteEventPlanRecord> records = {
        record(0, 14, 8.0, 18.0, 2.0, 6.0, 0.0, 0.0,
               "a", "direct", false),
        record(1, 14, 8.0, 18.0, 2.0, 6.0, 0.0, 0.0,
               "b", "ring", false)};
    const auto selected = selectFiniteEventPlan(
        records, 11.0, 0.05, 1.6, 1e-9);
    assert(!selected.success);
    assert(!selected.commitAdmissible);
    assert(selected.costValidCount == 0);
    assert(selected.reason == "incomplete_or_nonfinite_global_cost_set");
  }

  {
    // Non-finite records (NaN, +Inf, -Inf) must never be selectable, even
    // mixed with a single valid finite alternative.
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double posInf = std::numeric_limits<double>::infinity();
    const double negInf = -std::numeric_limits<double>::infinity();
    std::vector<call_handover::FiniteEventPlanRecord> records = {
        record(0, 5, 5.0, 15.0, 2.0, 6.0, nan, nan,
               "nanCandidate", "direct"),
        record(1, 5, 5.0, 15.0, 2.0, 6.0, posInf, posInf,
               "infCandidate", "ring"),
        record(2, 5, 5.0, 15.0, 2.0, 6.0, negInf, negInf,
               "neginfCandidate", "direct"),
        record(3, 5, 5.0, 15.0, 2.0, 6.0, 0.42, 0.47,
               "validCandidate", "ring")};
    const auto selected = selectFiniteEventPlan(
        records, 11.0, 0.05, 1.6, 1e-9);
    assert(selected.success);
    assert(selected.costValidCount == 1);
    assert(selected.selectedRecord == 3);
    assert(std::abs(selected.minimumAdmissibleGlobalCost - 0.47) < 1e-12);
  }

  {
    // The selector's reported minimum must equal an independently computed
    // minimum over just the valid, timing-admissible records.
    std::vector<call_handover::FiniteEventPlanRecord> records = {
        record(0, 3, 4.0, 14.0, 2.0, 6.0, 0.50, 0.58, "a", "direct"),
        record(1, 3, 4.0, 14.0, 2.0, 6.0, 0.20, 0.30, "b", "ring", false),
        record(2, 3, 4.0, 14.0, 2.0, 6.0, 0.45, 0.52, "c", "direct"),
        record(3, 3, 4.0, 14.0, 2.0, 6.0, 0.60, 0.66, "d", "ring")};
    const auto selected = selectFiniteEventPlan(
        records, 11.0, 0.05, 1.6, 1e-9);
    double independentMinimum = std::numeric_limits<double>::infinity();
    for(std::size_t i = 0; i < records.size(); ++i)
    {
      if(i == 1) { continue; } // the only cost-invalid record
      independentMinimum = std::min(independentMinimum, records[i].globalCost);
    }
    assert(selected.success);
    assert(std::abs(selected.minimumAdmissibleGlobalCost - independentMinimum)
           < 1e-12);
    assert(std::abs(records[selected.selectedRecord].globalCost
                    - independentMinimum) < 1e-12);
  }

  {
    // If all alternatives are too late to start safely, no fallback is
    // returned for commitment.
    std::vector<call_handover::FiniteEventPlanRecord> records = {
        record(0, 1, 3.0, 12.0, 2.0, 6.0, 0.40, 0.50,
               "a", "direct")};
    const auto selected = selectFiniteEventPlan(
        records, 11.0, 0.05, 1.6, 1e-9);
    assert(!selected.success);
    assert(!selected.commitAdmissible);
    assert(selected.reason == "no_final_timing_admissible_time_plan");
  }

  {
    // A coherent tie set uses earlier predicted completion as the first
    // deterministic secondary key.
    std::vector<call_handover::FiniteEventPlanRecord> records = {
        record(0, 2, 6.0, 16.0, 2.0, 7.0, 0.50, 0.6000000004,
               "later_completion", "direct"),
        record(1, 1, 5.0, 15.0, 2.0, 6.0, 0.50, 0.6000000000,
               "earlier_completion", "ring")};
    const auto selected = selectFiniteEventPlan(
        records, 11.0, 0.05, 1.6, 1e-9);
    assert(selected.success);
    assert(selected.selectedRecord == 1);
  }

  {
    const auto r = record(0, 1, 5.0, 15.0, 2.0, 7.0, 0.5, 0.6,
                          "a", "direct");
    assert(std::abs(call_handover::searchToCompletionDuration(r) - 10.0)
           < 1e-12);
  }

  {
    // The optional timing-diagnostic output must reflect exactly the
    // predicate the selector itself used, not a separate recomputation:
    // one diagnostic per cost-valid record, each with the true admissibility
    // outcome for that record, in agreement with timingAdmissibleCount.
    std::vector<call_handover::FiniteEventPlanRecord> records = {
        record(0, 1, 3.0, 12.0, 2.0, 6.0, 0.30, 0.35,
               "expired", "direct"),
        record(1, 2, 7.0, 17.0, 2.0, 6.0, 0.50, 0.60,
               "live", "ring")};
    std::vector<call_handover::FiniteEventPlanTimingDiagnostic> diagnostics;
    const auto selected = selectFiniteEventPlan(
        records, 10.5, 0.05, 1.6, 1e-9, &diagnostics);
    assert(selected.success);
    assert(selected.timingAdmissibleCount == 1);
    assert(diagnostics.size() == records.size());

    const auto expired = std::find_if(
        diagnostics.begin(), diagnostics.end(),
        [](const auto & d) { return d.candidateName == "expired"; });
    const auto live = std::find_if(
        diagnostics.begin(), diagnostics.end(),
        [](const auto & d) { return d.candidateName == "live"; });
    assert(expired != diagnostics.end() && live != diagnostics.end());
    assert(expired->routeName == "direct" && expired->hypothesisIndex == 1);
    assert(expired->costValid);
    assert(!expired->timingAdmissible);
    assert(!expired->eventWindowAdmissible);
    assert(std::abs(expired->remaining - 1.5) < 1e-12);
    assert(live->routeName == "ring" && live->hypothesisIndex == 2);
    assert(live->costValid);
    assert(live->timingAdmissible);
    assert(live->eventWindowAdmissible);
    assert(std::abs(live->remaining - 6.5) < 1e-12);
  }

  std::cout << "finite event-time plan selector tests: PASS\n";
  return 0;
}
