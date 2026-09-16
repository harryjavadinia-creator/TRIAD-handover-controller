#!/usr/bin/env bash
# Phase D: authority demands on the interception solver (held arm, no adoption).
cd ~/TRIAD_SCIENTIFIC_AUDIT
export PATH=$HOME/mc_rtc_ws/install/bin:$PATH
E="$1"
run() { local tag=$1 s=$2 ov=$3; rm -rf $E/logs/$tag/$s; mkdir -p $E/logs/$tag; TRIAD_RECEIVER_MODE=v2 TRIAD_MAX_WAIT=240 TRIAD_EXTRA_OVERRIDE=$E/overrides/$ov.yaml scripts/run_scenario.sh $s $E/logs/$tag/$s > $E/$tag.$s.out 2>&1; echo "$tag $s exit=$?"; }
for cfg in log filter stride1; do for s in longitudinal near-ground lateral-low diagonal; do run $cfg $s $cfg; done; done
echo CAMPAIGN_DONE
