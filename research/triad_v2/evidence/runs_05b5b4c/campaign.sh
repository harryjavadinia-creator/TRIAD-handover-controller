#!/usr/bin/env bash
cd ~/TRIAD_SCIENTIFIC_AUDIT
export PATH=$HOME/mc_rtc_ws/install/bin:$PATH
E="$1"
run() { local tag=$1 mode=$2 s=$3 ov=$4; if grep -q "=== run result ===" $E/$tag.$s.out 2>/dev/null; then echo "$tag $s skipped(done)"; return; fi; rm -rf $E/$tag/$s; if [[ -n "$ov" ]]; then TRIAD_RECEIVER_MODE=$mode TRIAD_EXTRA_OVERRIDE=$E/overrides/$ov.yaml scripts/run_scenario.sh $s $E/$tag/$s > $E/$tag.$s.out 2>&1; else TRIAD_RECEIVER_MODE=$mode scripts/run_scenario.sh $s $E/$tag/$s > $E/$tag.$s.out 2>&1; fi; echo "$tag $s exit=$? $(grep -c '\[Completed\] full plan-once' $E/$tag/$s/$s.log 2>/dev/null)"; }
SC="longitudinal near-ground lateral-low diagonal"
for s in $SC; do run v1 v1 $s ""; done
for rep in a b; do
  for s in $SC; do run select_$rep v2 $s ""; done
  for s in $SC; do run restart_$rep v2 $s cancel_and_restart; done
done
run select_inject_stale v2 diagonal inject_stale
run select_inject_supersede v2 longitudinal inject_supersede
echo CAMPAIGN_DONE
