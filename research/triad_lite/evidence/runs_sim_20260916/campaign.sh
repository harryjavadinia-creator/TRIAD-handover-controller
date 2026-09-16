#!/usr/bin/env bash
# TRIAD-lite sanity campaign: V2 bank_search (default) vs V2 supervisorMode=control_aware.
cd ~/TRIAD_SCIENTIFIC_AUDIT
export PATH=$HOME/mc_rtc_ws/install/bin:$PATH
E="$1"
run() { local tag=$1 s=$2 ov=$3; rm -rf $E/logs/$tag/$s; mkdir -p $E/logs/$tag; if [[ -n "$ov" ]]; then TRIAD_RECEIVER_MODE=v2 TRIAD_MAX_WAIT=200 TRIAD_EXTRA_OVERRIDE=$E/overrides/$ov.yaml scripts/run_scenario.sh $s $E/logs/$tag/$s > $E/$tag.$s.out 2>&1; else TRIAD_RECEIVER_MODE=v2 TRIAD_MAX_WAIT=200 scripts/run_scenario.sh $s $E/logs/$tag/$s > $E/$tag.$s.out 2>&1; fi; echo "$tag $s exit=$? completed=$(grep -c '\[Completed\] full plan-once' $E/logs/$tag/$s/$s.log 2>/dev/null)"; }
for s in longitudinal near-ground lateral-low diagonal; do run bank $s ""; run lite $s control_aware; done
echo CAMPAIGN_DONE
