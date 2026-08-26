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
 * One complete hard-feasible (event time, grasp, route) alternative.
 *
 * motionCost is the seven-term complete-plan binding cost (T,E,L,C,Q,K,V);
 * the orientation quantity R is diagnostic-only and excluded from that sum.
 * globalCost uses the same normalized time weight, but extends its time
 * horizon backwards to the common search epoch:
 *
 *   globalCost = motionCost
 *              + w_T * (eventLead - reachDuration) / timeReference.
 *
 * Thus the time contribution represents predicted search-to-completion time;
 * no additional cost term or weight is introduced.
 */
struct FiniteEventPlanRecord
{
  std::size_t sourceIndex = 0;
  std::size_t hypothesisIndex = 0;
  bool costValid = false;
  double motionCost = std::numeric_limits<double>::infinity();
  double globalCost = std::numeric_limits<double>::infinity();
  double eventLead = std::numeric_limits<double>::infinity();
  double eventPresentationTime = std::numeric_limits<double>::infinity();
  double predictedPresentationDuration =
      std::numeric_limits<double>::infinity();
  double predictedExecutionDuration =
      std::numeric_limits<double>::infinity();
  double clearance = -std::numeric_limits<double>::infinity();
  std::string candidateName;
  std::string routeName;
};

struct FiniteEventPlanSelection
{
  bool success = false;
  bool allCostsValid = false;
  bool commitAdmissible = false;
  std::size_t selectedRecord = std::numeric_limits<std::size_t>::max();
  std::size_t completePlanCount = 0;
  std::size_t costValidCount = 0;
  std::size_t timingAdmissibleCount = 0;
  double minimumAdmissibleGlobalCost =
      std::numeric_limits<double>::infinity();
  std::string reason = "not_run";
};

/**
 * Optional, read-only record of the exact final timing gate as actually
 * evaluated for one cost-valid alternative inside selectFiniteEventPlan().
 * This is captured, not recomputed: every field below is the same local
 * value the selector already computes for its own admissibility decision.
 */
struct FiniteEventPlanTimingDiagnostic
{
  std::size_t sourceIndex = 0;
  std::size_t hypothesisIndex = 0;
  std::string candidateName;
  std::string routeName;
  bool costValid = false;
  double globalCost = std::numeric_limits<double>::infinity();
  double remaining = std::numeric_limits<double>::quiet_NaN();
  double predictedPresentationDuration =
      std::numeric_limits<double>::quiet_NaN();
  double minimumReachEntryLead = std::numeric_limits<double>::quiet_NaN();
  double minimumSafeCommitLead = std::numeric_limits<double>::quiet_NaN();
  bool eventWindowAdmissible = false;
  bool timingAdmissible = false;
};

inline double extendMotionCostToSearchEpoch(
    double motionCost,
    double normalizedTimeWeight,
    double timeReference,
    double eventLead,
    double predictedPresentationDuration)
{
  if(!std::isfinite(motionCost) || !std::isfinite(normalizedTimeWeight)
     || !std::isfinite(timeReference) || timeReference <= 0.0
     || !std::isfinite(eventLead)
     || !std::isfinite(predictedPresentationDuration))
  {
    return std::numeric_limits<double>::infinity();
  }
  const double waitBeforeReach = eventLead - predictedPresentationDuration;
  return motionCost
       + normalizedTimeWeight * waitBeforeReach / timeReference;
}

inline double searchToCompletionDuration(
    const FiniteEventPlanRecord & record)
{
  return record.eventLead - record.predictedPresentationDuration
       + record.predictedExecutionDuration;
}

inline bool deterministicEventSecondaryOrder(
    const FiniteEventPlanRecord & lhs,
    const FiniteEventPlanRecord & rhs)
{
  constexpr double tolerance = 1e-12;
  const double lhsCompletion = searchToCompletionDuration(lhs);
  const double rhsCompletion = searchToCompletionDuration(rhs);
  if(lhsCompletion < rhsCompletion - tolerance) { return true; }
  if(rhsCompletion < lhsCompletion - tolerance) { return false; }
  if(lhs.eventPresentationTime
     < rhs.eventPresentationTime - tolerance)
  {
    return true;
  }
  if(rhs.eventPresentationTime
     < lhs.eventPresentationTime - tolerance)
  {
    return false;
  }
  if(lhs.predictedPresentationDuration
     < rhs.predictedPresentationDuration - tolerance)
  {
    return true;
  }
  if(rhs.predictedPresentationDuration
     < lhs.predictedPresentationDuration - tolerance)
  {
    return false;
  }
  if(lhs.clearance > rhs.clearance + tolerance) { return true; }
  if(rhs.clearance > lhs.clearance + tolerance) { return false; }
  if(lhs.candidateName != rhs.candidateName)
  {
    return lhs.candidateName < rhs.candidateName;
  }
  if(lhs.routeName != rhs.routeName) { return lhs.routeName < rhs.routeName; }
  if(lhs.hypothesisIndex != rhs.hypothesisIndex)
  {
    return lhs.hypothesisIndex < rhs.hypothesisIndex;
  }
  return lhs.sourceIndex < rhs.sourceIndex;
}

/**
 * Exhaustive argmin over the complete finite time-grasp-route set.
 *
 * The final timing gate is evaluated at selection time, after the entire
 * bounded schedule has been inspected. This prevents an alternative that
 * expired during planning from reaching commit. Invalid or non-finite
 * records are excluded individually and never compared against the valid
 * ones; the search fails closed only when no valid admissible record
 * remains.
 */
inline FiniteEventPlanSelection selectFiniteEventPlan(
    const std::vector<FiniteEventPlanRecord> & records,
    double now,
    double minimumReachEntryLead,
    double minimumSafeCommitLead,
    double costTieTolerance,
    std::vector<FiniteEventPlanTimingDiagnostic> * timingDiagnostics = nullptr)
{
  if(timingDiagnostics != nullptr) { timingDiagnostics->clear(); }
  FiniteEventPlanSelection result;
  result.completePlanCount = records.size();
  costTieTolerance = std::max(0.0, costTieTolerance);

  if(records.empty())
  {
    result.reason = "no_complete_time_plan_alternatives";
    return result;
  }
  if(!std::isfinite(now) || !std::isfinite(minimumReachEntryLead)
     || !std::isfinite(minimumSafeCommitLead)
     || minimumReachEntryLead < 0.0 || minimumSafeCommitLead < 0.0)
  {
    result.reason = "invalid_global_timing_gate";
    return result;
  }

  std::vector<std::size_t> costValid;
  for(std::size_t i = 0; i < records.size(); ++i)
  {
    const auto & record = records[i];
    const bool valid = record.costValid
        && std::isfinite(record.motionCost)
        && std::isfinite(record.globalCost)
        && std::isfinite(record.eventLead)
        && std::isfinite(record.eventPresentationTime)
        && std::isfinite(record.predictedPresentationDuration)
        && std::isfinite(record.predictedExecutionDuration)
        && std::isfinite(record.clearance);
    if(valid) { costValid.push_back(i); }
  }
  result.costValidCount = costValid.size();
  result.allCostsValid = result.costValidCount == records.size();
  if(costValid.empty())
  {
    result.reason = "incomplete_or_nonfinite_global_cost_set";
    return result;
  }

  std::vector<std::size_t> admissible;
  double exactMinimum = std::numeric_limits<double>::infinity();
  for(const std::size_t i : costValid)
  {
    const auto & record = records[i];
    const double remaining = record.eventPresentationTime - now;
    const bool eventWindowAdmissible =
        remaining + 1e-12 >= minimumSafeCommitLead;
    const bool reachEntryAdmissible = eventWindowAdmissible
        && record.predictedPresentationDuration + minimumReachEntryLead
               <= remaining + 1e-12;

    if(timingDiagnostics != nullptr)
    {
      FiniteEventPlanTimingDiagnostic diagnostic;
      diagnostic.sourceIndex = record.sourceIndex;
      diagnostic.hypothesisIndex = record.hypothesisIndex;
      diagnostic.candidateName = record.candidateName;
      diagnostic.routeName = record.routeName;
      diagnostic.costValid = record.costValid;
      diagnostic.globalCost = record.globalCost;
      diagnostic.remaining = remaining;
      diagnostic.predictedPresentationDuration =
          record.predictedPresentationDuration;
      diagnostic.minimumReachEntryLead = minimumReachEntryLead;
      diagnostic.minimumSafeCommitLead = minimumSafeCommitLead;
      diagnostic.eventWindowAdmissible = eventWindowAdmissible;
      diagnostic.timingAdmissible = reachEntryAdmissible;
      timingDiagnostics->push_back(diagnostic);
    }

    if(!reachEntryAdmissible) { continue; }

    ++result.timingAdmissibleCount;
    admissible.push_back(i);
    exactMinimum = std::min(exactMinimum, record.globalCost);
  }

  if(admissible.empty())
  {
    result.reason = "no_final_timing_admissible_time_plan";
    return result;
  }

  std::size_t best = std::numeric_limits<std::size_t>::max();
  for(const std::size_t index : admissible)
  {
    if(records[index].globalCost > exactMinimum + costTieTolerance) { continue; }
    if(best == std::numeric_limits<std::size_t>::max()
       || deterministicEventSecondaryOrder(records[index], records[best]))
    {
      best = index;
    }
  }

  result.success = best != std::numeric_limits<std::size_t>::max();
  result.commitAdmissible = result.success;
  result.selectedRecord = best;
  result.minimumAdmissibleGlobalCost = exactMinimum;
  result.reason = result.success
      ? "minimum_global_time_grasp_route_cost"
      : "global_tie_set_selection_failed";
  return result;
}

} // namespace call_handover
