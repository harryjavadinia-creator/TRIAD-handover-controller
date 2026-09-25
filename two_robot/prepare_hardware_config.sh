#!/usr/bin/env bash
# Write the mc_rtc configuration for the physical arms from the repository files:
#   $HOME/.config/mc_rtc/mc_rtc.yaml                                   (global profile, two Kortex arms)
#   $HOME/.config/mc_rtc/controllers/HandoverInterceptionController.yaml (two-robot overlay + receiver hardware overlay)
# Existing files are backed up next to themselves with a timestamp. The controller is loaded from the
# build tree (TRIAD_BUILD_DIR), nothing is installed into mc_rtc.
#
#   TRIAD_BUILD_DIR=... MAIN_ROBOT_MODULE_PATH=... MC_RTC_INSTALL=... bash two_robot/prepare_hardware_config.sh [--single]
#
# --single: Robot A only; Robot B is removed from the profile and the giver disabled. This is the configuration of
#           the gripper smoke test and of the four scenarios on the physical arm against the virtual object
#           (run_single_robot_scenario.sh). A handover from a human hand also needs an object-pose source that
#           this repository does not provide.
# Afterwards put your Kortex username/password into mc_rtc.yaml (placeholders <kortex-username>/<kortex-password>)
# and check the IPs. Never commit that file.
set -euo pipefail
: "${TRIAD_BUILD_DIR:?set TRIAD_BUILD_DIR to the CMake build directory of this repository}"
: "${MAIN_ROBOT_MODULE_PATH:?set MAIN_ROBOT_MODULE_PATH to your gen3_2f85 robot-module directory}"
MC_RTC_INSTALL="${MC_RTC_INSTALL:-$HOME/mc_rtc_ws/install}"
SINGLE=false; [[ "${1:-}" == "--single" ]] && SINGLE=true
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG="$HOME/.config/mc_rtc"; mkdir -p "$CFG/controllers"
STAMP="$(date +%Y%m%d_%H%M%S)"
for f in "$CFG/mc_rtc.yaml" "$CFG/controllers/HandoverInterceptionController.yaml"; do
  [[ -f "$f" ]] && cp "$f" "$f.bak_$STAMP" && echo "backed up $f -> $f.bak_$STAMP"
done

MODS="${TRIAD_BUILD_DIR}/hardware_mods"; mkdir -p "${MODS}/etc"
ln -sfn "${TRIAD_BUILD_DIR}/src/HandoverInterceptionController_controller.so" "${MODS}/HandoverInterceptionController_controller.so"
ln -sfn "${TRIAD_BUILD_DIR}/etc/HandoverInterceptionController.yaml" "${MODS}/etc/HandoverInterceptionController.yaml"

LOGDIR="${TRIAD_HARDWARE_LOG_DIR:-$HOME/TRIAD_hardware_logs}"; mkdir -p "$LOGDIR"
sed -e "s#<gen3_2f85-module-directory>#${MAIN_ROBOT_MODULE_PATH}#" \
    -e "s#<controller-modules-directory>#${MODS}#" \
    -e "s#<log-directory>#${LOGDIR}#" \
    "${SCRIPT_DIR}/mc_rtc.two_kortex.yaml" > "$CFG/mc_rtc.yaml"
if $SINGLE; then
  python3 - "$CFG/mc_rtc.yaml" <<'PY'
import re, sys
p = sys.argv[1]; s = open(p).read()
s = re.sub(r"\n  kinova:\n(?:    .*\n)+", "\n", s)   # drop Robot B from the Kortex block
open(p, "w").write(s)
PY
fi

{
  echo "StatesLibraries:"
  echo "- \"${MC_RTC_INSTALL}/lib/mc_controller/fsm/states\""
  echo "- \"${TRIAD_BUILD_DIR}/src/states\""
  echo "StatesFiles:"
  echo "- \"${MC_RTC_INSTALL}/lib/mc_controller/fsm/states/data\""
  if $SINGLE; then
    # keep the object and the receiver settings of the two-robot overlay, drop Robot B and the giver
    python3 - "${SCRIPT_DIR}/HandoverInterceptionController.two_robot.yaml" <<'PY'
import re, sys
s = open(sys.argv[1]).read()
s = re.sub(r"  kinova:\n(?:    .*\n)+", "", s)
s = s.replace("dualHandover:\n  enabled: true\n  motionEnabled: true", "dualHandover:\n  enabled: false\n  motionEnabled: false")
s = s.replace("  simulateMotion: false\n", "")
print(s)
PY
  else
    cat "${SCRIPT_DIR}/HandoverInterceptionController.two_robot.yaml"
  fi
  echo
  cat "${SCRIPT_DIR}/HandoverInterceptionController.hardware_receiver.yaml"
} > "$CFG/controllers/HandoverInterceptionController.yaml"
# motion stays off until you switch it on for a step
python3 "${SCRIPT_DIR}/tools/set_override_key.py" dualHandover.motionEnabled false "$CFG/controllers/HandoverInterceptionController.yaml" > /dev/null || true

echo "wrote $CFG/mc_rtc.yaml  (mode: $($SINGLE && echo 'Robot A only' || echo 'two robots'))"
echo "wrote $CFG/controllers/HandoverInterceptionController.yaml"
echo "controller:  ${MODS} (build tree)"
echo "logs:        ${LOGDIR}"
grep -n "kortex-username\|kortex-password" "$CFG/mc_rtc.yaml" >/dev/null && echo "NOW EDIT $CFG/mc_rtc.yaml: replace <kortex-username> and <kortex-password>, check the IPs."
