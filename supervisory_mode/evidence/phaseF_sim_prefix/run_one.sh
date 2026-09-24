#!/usr/bin/env bash
# usage: run_one.sh EVIDENCE_DIR TAG SCENARIO VARIANT
cd ~/TRIAD_SCIENTIFIC_AUDIT
export PATH=$HOME/mc_rtc_ws/install/bin:$PATH
E="$1"; tag=$2; s=$3; v=$4
rm -rf $E/logs/$tag/$s; mkdir -p $E/logs/$tag
TRIAD_RECEIVER_MODE=v2 TRIAD_MAX_WAIT=240 TRIAD_EXTRA_OVERRIDE=$E/overrides/$v.yaml scripts/run_scenario.sh $s $E/logs/$tag/$s > $E/logs/$tag/$s.out 2>&1
echo "$tag $s variant=$v exit=$? completed=$(grep -c '\[Completed\] full plan-once' $E/logs/$tag/$s/$s.log 2>/dev/null)"
