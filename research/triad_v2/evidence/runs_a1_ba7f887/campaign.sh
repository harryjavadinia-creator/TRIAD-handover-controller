#!/usr/bin/env bash
cd ~/TRIAD_SCIENTIFIC_AUDIT
export PATH=$HOME/mc_rtc_ws/install/bin:$PATH
E="$1"
run() { local tag=$1 mode=$2 s=$3 ov=$4; if grep -q "=== run result ===" $E/$tag.$s.out 2>/dev/null; then echo "$tag $s skipped(done)"; return; fi; rm -rf $E/$tag/$s; if [[ -n "$ov" ]]; then TRIAD_RECEIVER_MODE=$mode TRIAD_EXTRA_OVERRIDE=$E/overrides/$ov.yaml scripts/run_scenario.sh $s $E/$tag/$s > $E/$tag.$s.out 2>&1; else TRIAD_RECEIVER_MODE=$mode scripts/run_scenario.sh $s $E/$tag/$s > $E/$tag.$s.out 2>&1; fi; echo "$tag $s exit=$? $(grep -c '\[Completed\] full plan-once' $E/$tag/$s/$s.log 2>/dev/null)"; }
SC="longitudinal near-ground lateral-low diagonal"
run char v2 near-ground char_default
for s in $SC; do run v1 v1 $s ""; done
for s in $SC; do run char v2 $s char_default; done
for s in $SC; do run insitu v2 $s ""; done
echo CAMPAIGN_DONE
