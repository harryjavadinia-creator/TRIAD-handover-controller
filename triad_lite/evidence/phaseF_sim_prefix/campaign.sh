#!/usr/bin/env bash
# Phase F: B0/B1/B2/FULL in situ, identical perception / grasp family / funnel / low-level law.
# Interleaved (repeat -> scenario -> variant) so machine drift spreads over variants.
E="$1"
for r in 1 2 3; do
  for s in longitudinal near-ground lateral-low diagonal; do
    for v in reactive predictive predictive_capability full; do
      "$E/run_one.sh" "$E" "r$r/$v" "$s" "$v"
    done
  done
done
for s in longitudinal near-ground lateral-low diagonal; do "$E/run_one.sh" "$E" "regression/bank_search" "$s" bank_search; done
echo CAMPAIGN_DONE
