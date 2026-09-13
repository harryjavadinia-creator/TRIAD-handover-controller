#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace call_handover
{

/**
 * Bounded event-lead schedule, restated from
 * HandoverInterceptionController_SolveInterception::buildBoundedEventLeadSchedule()
 * (V1, global_time_plan mode) so TRIAD V2 uses exactly the same temporal bank.
 * tools/test_bounded_event_lead_schedule.cpp checks equality with the
 * reported V1 bank. The V1 state keeps its own copy unchanged.
 */
struct BoundedEventLeadScheduleConfig
{
  bool boundedEventSearchEnabled = true;
  int maximumEventHypotheses = 15;
  double eventSearchLeadStep = 0.45;
  double attemptedLeadTolerance = 0.02;
  double initialPresentationLead = 3.25;
  double minimumPresentationLead = 1.8;
  double maximumPresentationLead = 8.0;
};

inline std::vector<double> buildBoundedEventLeadSchedule(
    BoundedEventLeadScheduleConfig c)
{
  // Same clamping as the V1 state's configure().
  c.maximumEventHypotheses = std::max(1, c.maximumEventHypotheses);
  c.eventSearchLeadStep = std::max(0.05, c.eventSearchLeadStep);
  c.attemptedLeadTolerance = std::max(1e-4, c.attemptedLeadTolerance);
  c.minimumPresentationLead = std::max(0.1, c.minimumPresentationLead);
  c.maximumPresentationLead = std::max(c.minimumPresentationLead, c.maximumPresentationLead);
  c.initialPresentationLead = std::max(
      c.minimumPresentationLead, std::min(c.maximumPresentationLead, c.initialPresentationLead));

  std::vector<double> leads;
  auto addLead = [&](double lead)
  {
    lead = std::max(c.minimumPresentationLead, std::min(c.maximumPresentationLead, lead));
    for(const double existing : leads)
    {
      if(std::abs(existing - lead) <= c.attemptedLeadTolerance) { return; }
    }
    if(static_cast<int>(leads.size()) < c.maximumEventHypotheses) { leads.push_back(lead); }
  };

  addLead(c.initialPresentationLead);
  if(c.boundedEventSearchEnabled && c.maximumEventHypotheses > 1)
  {
    const int endpointReserve = c.maximumEventHypotheses >= 5 ? 2 : 0;
    const int interiorLimit = c.maximumEventHypotheses - endpointReserve;
    for(int ring = 1; static_cast<int>(leads.size()) < interiorLimit; ++ring)
    {
      const double lower = c.initialPresentationLead - static_cast<double>(ring) * c.eventSearchLeadStep;
      const double upper = c.initialPresentationLead + static_cast<double>(ring) * c.eventSearchLeadStep;
      addLead(lower);
      if(static_cast<int>(leads.size()) >= interiorLimit) { break; }
      addLead(upper);
      if(lower <= c.minimumPresentationLead && upper >= c.maximumPresentationLead) { break; }
    }
    if(endpointReserve > 0)
    {
      addLead(c.minimumPresentationLead);
      addLead(c.maximumPresentationLead);
    }
  }
  std::sort(leads.begin(), leads.end());
  return leads;
}

} // namespace call_handover
