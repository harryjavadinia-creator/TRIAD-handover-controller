#!/usr/bin/env bash
# Phase F (after the rollout command-start fix): predictive variants re-run; B0 reactive and the
# bank_search regression are unaffected by the fix and are taken from ../phaseF_sim_prefix.
E="$1"
for r in 1 2 3; do
  for s in longitudinal near-ground lateral-low diagonal; do
    for v in predictive predictive_capability full; do
      "$E/run_one.sh" "$E" "r$r/$v" "$s" "$v"
    done
  done
done
echo CAMPAIGN_DONE
