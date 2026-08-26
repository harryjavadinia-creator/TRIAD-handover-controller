#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace call_handover
{

/**
 * Solver-facing record for one already-certified complete plan.
 *
 * The selector never creates plans and never softens feasibility. It receives
 * only complete plans that survived the copied-state hard checks, verifies
 * that every one has a finite cost, applies the event-time admission gate,
 * and returns the exact finite-set argmin.
 */
struct FinitePlanRecord
{
  std::size_t sourceIndex = 0;
  bool costValid = false;
  double cost = std::numeric_limits<double>::infinity();
  double predictedPresentationTime = std::numeric_limits<double>::infinity();
  double clearance = -std::numeric_limits<double>::infinity();
  std::string candidateName;
  std::string routeName;
};

struct FinitePlanSelection
{
  bool success = false;
  bool allCostsValid = false;
  bool commitAdmissible = false;
  std::size_t selectedRecord = std::numeric_limits<std::size_t>::max();
  std::size_t completePlanCount = 0;
  std::size_t costValidCount = 0;
  std::size_t timingAdmissibleCount = 0;
  double minimumAdmissibleCost = std::numeric_limits<double>::infinity();
  std::string reason = "not_run";
};

inline bool deterministicSecondaryOrder(const FinitePlanRecord & lhs,
                                        const FinitePlanRecord & rhs)
{
  constexpr double secondaryTolerance = 1e-12;
  if(lhs.predictedPresentationTime
     < rhs.predictedPresentationTime - secondaryTolerance)
  {
    return true;
  }
  if(rhs.predictedPresentationTime
     < lhs.predictedPresentationTime - secondaryTolerance)
  {
    return false;
  }
  if(lhs.clearance > rhs.clearance + secondaryTolerance) { return true; }
  if(rhs.clearance > lhs.clearance + secondaryTolerance) { return false; }
  if(lhs.candidateName != rhs.candidateName)
  {
    return lhs.candidateName < rhs.candidateName;
  }
  if(lhs.routeName != rhs.routeName) { return lhs.routeName < rhs.routeName; }
  return lhs.sourceIndex < rhs.sourceIndex;
}

inline bool lowerCost(const FinitePlanRecord & lhs,
                      const FinitePlanRecord & rhs,
                      double costTieTolerance)
{
  if(lhs.cost < rhs.cost - costTieTolerance) { return true; }
  if(rhs.cost < lhs.cost - costTieTolerance) { return false; }
  return deterministicSecondaryOrder(lhs, rhs);
}

inline bool fasterForTimingRefinement(const FinitePlanRecord & lhs,
                                      const FinitePlanRecord & rhs,
                                      double costTieTolerance)
{
  constexpr double timeTolerance = 1e-12;
  if(lhs.predictedPresentationTime
     < rhs.predictedPresentationTime - timeTolerance)
  {
    return true;
  }
  if(rhs.predictedPresentationTime
     < lhs.predictedPresentationTime - timeTolerance)
  {
    return false;
  }
  return lowerCost(lhs, rhs, costTieTolerance);
}

/**
 * Exact exhaustive minimization over a finite set of complete plans.
 *
 * When timing is enforced, a candidate is commit-admissible only when both
 * the event-level commit lead and its candidate-specific reach-entry lead are
 * safe. If none is currently admissible, the fastest cost-valid plan is
 * returned only to drive the existing event-time refinement; the result is
 * explicitly marked commitAdmissible=false and must never be committed.
 */
inline FinitePlanSelection selectFinitePlan(
    const std::vector<FinitePlanRecord> & records,
    double costTieTolerance,
    bool enforceTiming,
    double remainingToPresentation,
    double minimumReachEntryLead,
    double minimumSafeCommitLead)
{
  FinitePlanSelection result;
  result.completePlanCount = records.size();
  costTieTolerance = std::max(0.0, costTieTolerance);

  if(records.empty())
  {
    result.reason = "no_complete_plans";
    return result;
  }

  for(const auto & record : records)
  {
    const bool valid = record.costValid && std::isfinite(record.cost)
        && std::isfinite(record.predictedPresentationTime)
        && std::isfinite(record.clearance);
    if(valid) { ++result.costValidCount; }
  }
  result.allCostsValid = result.costValidCount == records.size();
  if(!result.allCostsValid)
  {
    result.reason = "incomplete_or_nonfinite_cost_set";
    return result;
  }

  const bool eventWindowAdmissible = !enforceTiming
      || (std::isfinite(remainingToPresentation)
          && std::isfinite(minimumReachEntryLead)
          && std::isfinite(minimumSafeCommitLead)
          && remainingToPresentation + 1e-12 >= minimumSafeCommitLead);

  std::size_t fastest = std::numeric_limits<std::size_t>::max();
  double exactMinimumAdmissibleCost =
      std::numeric_limits<double>::infinity();
  std::vector<std::size_t> admissible;
  for(std::size_t i = 0; i < records.size(); ++i)
  {
    const auto & record = records[i];
    if(fastest == std::numeric_limits<std::size_t>::max()
       || fasterForTimingRefinement(
           record, records[fastest], costTieTolerance))
    {
      fastest = i;
    }

    const bool reachEntryAdmissible = !enforceTiming
        || (eventWindowAdmissible
            && record.predictedPresentationTime + minimumReachEntryLead
                   <= remainingToPresentation + 1e-12);
    if(!reachEntryAdmissible) { continue; }

    ++result.timingAdmissibleCount;
    admissible.push_back(i);
    exactMinimumAdmissibleCost = std::min(
        exactMinimumAdmissibleCost, record.cost);
  }

  if(!admissible.empty())
  {
    // Define one coherent numerical tie set around the exact minimum. This
    // avoids pairwise-tolerance drift where a chain of near-equal candidates
    // could otherwise move the selected value beyond the configured bound.
    std::size_t best = std::numeric_limits<std::size_t>::max();
    for(const std::size_t index : admissible)
    {
      if(records[index].cost
         > exactMinimumAdmissibleCost + costTieTolerance)
      {
        continue;
      }
      if(best == std::numeric_limits<std::size_t>::max()
         || deterministicSecondaryOrder(records[index], records[best]))
      {
        best = index;
      }
    }

    result.success = true;
    result.commitAdmissible = true;
    result.selectedRecord = best;
    result.minimumAdmissibleCost = exactMinimumAdmissibleCost;
    result.reason = "minimum_cost_admissible_plan";
    return result;
  }

  if(enforceTiming && fastest != std::numeric_limits<std::size_t>::max())
  {
    result.success = true;
    result.commitAdmissible = false;
    result.selectedRecord = fastest;
    result.reason = "timing_refinement_required";
    return result;
  }

  result.reason = "no_admissible_plan";
  return result;
}

} // namespace call_handover
