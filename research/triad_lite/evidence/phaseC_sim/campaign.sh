#!/usr/bin/env bash
# Phase C: interception solver characterization from the held arm (no adoption),
# uncensored budgets, timing skip on/off.
cd ~/TRIAD_SCIENTIFIC_AUDIT
export PATH=$HOME/mc_rtc_ws/install/bin:$PATH
E="$1"
run() { local tag=$1 s=$2 ov=$3; rm -rf $E/logs/$tag/$s; mkdir -p $E/logs/$tag; TRIAD_RECEIVER_MODE=v2 TRIAD_MAX_WAIT=240 TRIAD_EXTRA_OVERRIDE=$E/overrides/$ov.yaml scripts/run_scenario.sh $s $E/logs/$tag/$s > $E/$tag.$s.out 2>&1; echo "$tag $s exit=$? completed=$(grep -c '\[Completed\] full plan-once' $E/logs/$tag/$s/$s.log 2>/dev/null)"; }
for s in longitudinal near-ground lateral-low diagonal; do run hold_skip $s interception_hold_skip_true; done
for s in longitudinal near-ground lateral-low diagonal; do run hold_noskip $s interception_hold_skip_false; done
echo CAMPAIGN_DONE
