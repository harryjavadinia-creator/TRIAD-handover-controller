// g++ -std=c++14 -Isrc tools/test_bounded_event_lead_schedule.cpp -o /tmp/t && /tmp/t
#include "BoundedEventLeadSchedule.h"

#include <cmath>
#include <cstdio>

int main()
{
  // The reported V1 bank (docs/mathematics.md, logged [FrozenPlanRecord] leads).
  const double expected[] = {1.80, 1.90, 2.35, 2.80, 3.25, 3.70, 4.15,
                             4.60, 5.05, 5.50, 5.95, 6.40, 6.85, 8.00};
  const auto leads = call_handover::buildBoundedEventLeadSchedule({});
  bool ok = leads.size() == 14;
  for(std::size_t i = 0; ok && i < leads.size(); ++i)
  {
    ok = std::abs(leads[i] - expected[i]) < 1e-12;
  }
  std::printf("%s bounded event lead schedule reproduces the 14-lead V1 bank (%zu leads)\n",
              ok ? "PASS" : "FAIL", leads.size());
  return ok ? 0 : 1;
}
