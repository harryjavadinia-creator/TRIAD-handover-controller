#!/usr/bin/env bash
# Run one of the four TRIAD scenarios on the physical Robot A alone, against the virtual object, with the
# configuration written by prepare_hardware_config.sh --single: set the scenario's object start and velocity
# in the override, send the arm to its Home action, start mc_kortex, wait for the terminal outcome, stop the
# driver, print a summary. Retries once when the plan search misses its timing (no admissible plan).
#
#   two_robot/run_single_robot_scenario.sh <near-ground|longitudinal|lateral-low|diagonal> [--no-home]
#
# Environment: TRIAD_BUILD_DIR, MC_RTC_INSTALL (as in the README); TRIAD_HARDWARE_LOG_DIR (default
# ~/TRIAD_hardware_logs); ROBOT_A_IP (default 192.168.1.10); KORTEX_HOME_BIN (default
# $TRIAD_BUILD_DIR/kortex_home, built by tools/build_kortex_home.sh). The Kortex credentials are read from
# ~/.config/mc_rtc/mc_rtc.yaml (gen3_2f85 block) and passed to the homing tool through its environment only.
#
# Every run ends in the fail-safe hold at the closure check (there is no object between the fingers) and
# the driver then ignores SIGINT, so the stop escalates to SIGKILL; the robot holds its pose when the
# session drops and the next Home action recovers it.
set -uo pipefail
S="${1:-}"; NOHOME=false; [[ "${2:-}" == "--no-home" ]] && NOHOME=true
case "$S" in
  near-ground)  T="[0.25, 0.62, 0.15]"; V="[0.0, -0.08, 0.0]";;
  longitudinal) T="[0.92, 0.0, 0.55]";  V="[-0.08, 0.0, 0.0]";;
  lateral-low)  T="[0.55, -0.56, 0.15]"; V="[0.0, 0.08, 0.0]";;
  diagonal)     T="[0.90, 0.0, 0.30]";  V="[-0.0565685, 0.0, 0.0565685]";;
  *) echo "usage: $0 <near-ground|longitudinal|lateral-low|diagonal> [--no-home]" >&2; exit 2;;
esac
: "${TRIAD_BUILD_DIR:?set TRIAD_BUILD_DIR to the CMake build directory of this repository}"
MC_RTC_INSTALL="${MC_RTC_INSTALL:-$HOME/mc_rtc_ws/install}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG="$HOME/.config/mc_rtc/mc_rtc.yaml"; OVR="$HOME/.config/mc_rtc/controllers/HandoverInterceptionController.yaml"
LOGDIR="${TRIAD_HARDWARE_LOG_DIR:-$HOME/TRIAD_hardware_logs}"; mkdir -p "$LOGDIR"
KORTEX="${MC_RTC_INSTALL}/bin/mc_kortex"; [[ -x "$KORTEX" ]] || KORTEX="$(command -v mc_kortex)"
HOME_BIN="${KORTEX_HOME_BIN:-$TRIAD_BUILD_DIR/kortex_home}"
export PATH="$MC_RTC_INSTALL/bin:$PATH" LD_LIBRARY_PATH="$TRIAD_BUILD_DIR/src:$MC_RTC_INSTALL/lib:${LD_LIBRARY_PATH:-}"
grep -q 'on_startup: false' "$CFG" || { echo "Kortex.init_posture.on_startup must be false in $CFG (the robot rejects it and the driver crashes)" >&2; exit 3; }
grep -q 'enabled: false' <(grep -A1 hardwareGripperCommissioning "$OVR") || { echo "gripper commissioning switch is on; switch it off first" >&2; exit 3; }
for kv in "robots.call_object.init_pos.translation=$T" "object.translation=$T" "movingObject.simulatedLinearVelocity=$V"; do
  python3 "$SCRIPT_DIR/tools/set_override_key.py" "${kv%%=*}" "${kv#*=}" "$OVR" > /dev/null || exit 3
done
echo "scenario=$S object=$T velocity=$V"
for attempt in 1 2; do
  pgrep -x mc_kortex > /dev/null && { echo "mc_kortex is already running" >&2; exit 4; }
  if ! $NOHOME; then
    [[ -x "$HOME_BIN" ]] || { echo "homing tool not found at $HOME_BIN (bash two_robot/tools/build_kortex_home.sh)" >&2; exit 5; }
    export KORTEX_IP="${ROBOT_A_IP:-192.168.1.10}"
    export KORTEX_USER="$(awk '/gen3_2f85:/{f=1} f&&/username:/{print $2; exit}' "$CFG")"
    export KORTEX_PASS="$(awk '/gen3_2f85:/{f=1} f&&/password:/{print $2; exit}' "$CFG")"
    HOME_OUT="$(timeout 60 "$HOME_BIN" Home 2>&1)"; HRC=$?; unset KORTEX_USER KORTEX_PASS
    echo "home: rc=$HRC $(echo "$HOME_OUT" | grep -E 'action event|after joints' | tr '\n' ' ')"
    [[ $HRC -eq 0 ]] || { echo "$HOME_OUT" | tail -3 >&2; exit 5; }
    sleep 3
  fi
  LOG="$LOGDIR/single_robot_${S}_$(date +%Y%m%d_%H%M%S).log"
  "$KORTEX" > "$LOG" 2>&1 & PID=$!
  TERM_RE='\[Completed\] full plan-once handover completed|Starting state HandoverInterceptionController_Failure|fail-safe hold'
  for _ in $(seq 1 150); do sleep 2; if grep -qE "$TERM_RE" "$LOG" || ! kill -0 $PID 2>/dev/null; then break; fi; done
  sleep 4; kill -INT $PID 2>/dev/null; for _ in $(seq 1 10); do kill -0 $PID 2>/dev/null || break; sleep 1; done
  kill -0 $PID 2>/dev/null && { kill -TERM $PID 2>/dev/null; sleep 5; }
  kill -0 $PID 2>/dev/null && { echo "  driver ignored INT/TERM, hard kill"; kill -KILL $PID 2>/dev/null; sleep 2; }
  wait $PID 2>/dev/null; RC=$?
  python3 "$SCRIPT_DIR/tools/set_override_key.py" dualHandover.motionEnabled false "$OVR" > /dev/null || true
  SEQ=$(grep -oE 'Starting state HandoverInterceptionController_[A-Za-z]+' "$LOG" | uniq | cut -d_ -f2 | tr '\n' ' ')
  SEARCH=$(grep -E 'GlobalTimePlanSearchSummary' "$LOG" | head -1 | grep -oE 'committed=[a-z]+|reason=[a-z_]+|feasibleHypotheses=[0-9]+|elapsed=[0-9.]+s' | tr '\n' ' ')
  echo "attempt $attempt: driver rc=$RC log=$LOG"
  echo "  states: $SEQ"
  echo "  search: $SEARCH"
  echo "  commit: $(grep -E 'PresentationCommit\] COMMITTED' "$LOG" | head -1 | grep -oE 'candidate=[^ ]+|route=[^ ]+|presentationTime=[0-9.]+s' | tr '\n' ' ')"
  echo "  reach:  $(grep -E 'PredictiveReach\] committed state entered' "$LOG" | grep -oE 'reach to \[[^]]*\]') $(grep -E 'PredictiveReachGovernor\] completed' "$LOG" | grep -oE 'minimumClearance=[0-9.]+')"
  echo "  end:    $(grep -E 'Acquire\] hard closure|\[Completed\]|ForceTransferGate|GlobalTimePlanSelection\] success=false' "$LOG" | head -1 | cut -c1-160)"
  echo "  gripper: $(grep 'gripper BRIDGE' "$LOG" | tail -1 | grep -oE 'measured=[0-9.]+%')"
  if echo "$SEARCH" | grep -q 'no_final_timing_admissible'; then echo "  timing miss (search too slow for the approaching object): retrying"; continue; fi
  break
done
