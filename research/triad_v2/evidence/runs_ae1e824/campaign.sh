#!/usr/bin/env bash
cd ~/TRIAD_SCIENTIFIC_AUDIT
export PATH=$HOME/mc_rtc_ws/install/bin:$PATH
E="$1"
run() { local tag=$1 mode=$2 s=$3 ov=$4; if [[ -n "$ov" ]]; then TRIAD_MAX_WAIT=900 TRIAD_RECEIVER_MODE=$mode TRIAD_EXTRA_OVERRIDE=$E/overrides/$ov.yaml scripts/run_scenario.sh $s $E/$tag/$s > $E/$tag.$s.out 2>&1; else TRIAD_RECEIVER_MODE=$mode scripts/run_scenario.sh $s $E/$tag/$s > $E/$tag.$s.out 2>&1; fi; echo "$tag $s exit=$? $(grep -c '\[Completed\] full plan-once' $E/$tag/$s/$s.log 2>/dev/null)"; }
SC="longitudinal near-ground lateral-low diagonal"
for s in $SC; do run v1 v1 $s ""; done
for s in $SC; do run v1_independent v1-independent $s ""; done
for s in $SC; do run v2 v2 $s ""; done
for s in $SC; do run v2_repeat v2 $s ""; done
run v2_inject_stale v2 diagonal inject_stale
run v2_inject_supersede v2 longitudinal inject_supersede
for c in char_default char_T char_G char_R; do for s in $SC; do run $c v2 $s $c; done; done
echo CAMPAIGN_DONE
