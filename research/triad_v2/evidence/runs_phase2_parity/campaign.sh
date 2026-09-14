#!/usr/bin/env bash
cd ~/TRIAD_SCIENTIFIC_AUDIT
export PATH=$HOME/mc_rtc_ws/install/bin:$PATH
E="$1"
run() { local tag=$1 mode=$2 s=$3 ov=$4; if grep -q "=== run result ===" $E/$tag.$s.out 2>/dev/null; then echo "$tag $s skipped(done)"; return; fi; rm -rf $E/logs/$tag/$s; mkdir -p $E/logs/$tag; if [[ -n "$ov" ]]; then TRIAD_RECEIVER_MODE=$mode TRIAD_EXTRA_OVERRIDE=$E/overrides/$ov.yaml scripts/run_scenario.sh $s $E/logs/$tag/$s > $E/$tag.$s.out 2>&1; else TRIAD_RECEIVER_MODE=$mode scripts/run_scenario.sh $s $E/logs/$tag/$s > $E/$tag.$s.out 2>&1; fi; echo "$tag $s exit=$? completed=$(grep -c '\[Completed\] full plan-once' $E/logs/$tag/$s/$s.log 2>/dev/null)"; }
SC="longitudinal near-ground lateral-low diagonal"
for s in $SC; do run v1 v1 $s ""; done
for r in a b c; do for s in $SC; do run parity_$r v2 $s parity; done; done
for ov in speed004 speed016 stop0425 lat022c lat022u; do for s in longitudinal near-ground; do run $ov v2 $s $ov; done; done
echo CAMPAIGN_DONE
