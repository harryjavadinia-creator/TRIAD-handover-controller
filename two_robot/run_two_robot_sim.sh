#!/usr/bin/env bash
# Two-robot TRIAD handover in simulation (mc_rtc_ticker), Robot A = receiver, Robot B = giver.
#
# Runs straight from a build tree, without installing anything into the shared mc_rtc install:
#   TRIAD_BUILD_DIR=/path/to/build  MAIN_ROBOT_MODULE_PATH=/path/to/kinova_gen3_2f85_mcdesc \
#   MC_RTC_INSTALL=$HOME/mc_rtc_ws/install  bash two_robot/run_two_robot_sim.sh [seconds] [out_dir]
#
# TRIAD_SYNC_RATIO=<sim/real> (e.g. 0.5) slows the simulation when a viewer is attached; --run-for
# counts simulated seconds.
#
# Requirements: the Kinova robot module ("Kinova", from mc_kinova) installed in MC_RTC_INSTALL,
# and a Gen3 + 2F-85 module directory named gen3_2f85 (see docs/robot_module.md).
set -euo pipefail
RUN_FOR="${1:-60}"
OUT_DIR="${2:-two_robot/results/$(date +%Y%m%d_%H%M%S)_two_robot_sim}"
: "${TRIAD_BUILD_DIR:?set TRIAD_BUILD_DIR to the CMake build directory of this repository}"
: "${MAIN_ROBOT_MODULE_PATH:?set MAIN_ROBOT_MODULE_PATH to your gen3_2f85 robot-module directory}"
MC_RTC_INSTALL="${MC_RTC_INSTALL:-$HOME/mc_rtc_ws/install}"
TICKER="${MC_RTC_INSTALL}/bin/mc_rtc_ticker"
[[ -x "$TICKER" ]] || TICKER="$(command -v mc_rtc_ticker)"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_HOME="$(mktemp -d)"
trap 'rm -rf "${RUN_HOME}"' EXIT

# Giver scenario: TRIAD_GIVER_SCENARIO=<name from dualHandover.scenarios> (default pure_x).
# The object's initial pose is moved to that scenario's objectStartWorld.
GIVER_SCENARIO="${TRIAD_GIVER_SCENARIO:-pure_x}"
OVERLAY="${SCRIPT_DIR}/HandoverInterceptionController.two_robot.yaml"
if [[ "${GIVER_SCENARIO}" != "pure_x" ]]; then
  START="$(awk -v s="${GIVER_SCENARIO}:" '$1==s{f=1;next} f&&/objectStartWorld:/{sub(/.*objectStartWorld: */,"");print;exit}' "${OVERLAY}")"
  [[ -n "${START}" ]] || { echo "unknown giver scenario: ${GIVER_SCENARIO}" >&2; exit 2; }
  sed -e "s/^  scenario: pure_x/  scenario: ${GIVER_SCENARIO}/" \
      -e "s/translation: \[0.92, 0.0, 0.55\]/translation: ${START}/" \
      "${OVERLAY}" > "${RUN_HOME}/overlay.yaml"
  OVERLAY="${RUN_HOME}/overlay.yaml"
fi

# A scratch controller-module directory that points at the build tree.
MODS="${RUN_HOME}/mods"; mkdir -p "${MODS}/etc"
ln -s "${TRIAD_BUILD_DIR}/src/HandoverInterceptionController_controller.so" "${MODS}/"
ln -s "${TRIAD_BUILD_DIR}/etc/HandoverInterceptionController.yaml" "${MODS}/etc/"

mkdir -p "${RUN_HOME}/.config/mc_rtc/controllers"
{
  echo "StatesLibraries:"
  echo "- \"${MC_RTC_INSTALL}/lib/mc_controller/fsm/states\""
  echo "- \"${TRIAD_BUILD_DIR}/src/states\""
  echo "StatesFiles:"
  echo "- \"${MC_RTC_INSTALL}/lib/mc_controller/fsm/states/data\""
  cat "${OVERLAY}"
} > "${RUN_HOME}/.config/mc_rtc/controllers/HandoverInterceptionController.yaml"

cat > "${RUN_HOME}/mc_rtc.yaml" <<EOF
MainRobot: [env, "${MAIN_ROBOT_MODULE_PATH}", gen3_2f85]
Enabled: [HandoverInterceptionController]
ControllerModulePaths: ["${MODS}"]
Timestep: 0.001
Log: true
LogDirectory: "${RUN_HOME}"
# mc_rtc_ticker dereferences the GUI server unconditionally, so it must stay enabled.
GUIServer:
  Enable: true
  Timestep: 0.05
  IPC:
    Socket: "${RUN_HOME}/mc_rtc"
  TCP:
    Host: "127.0.0.1"
    Ports: [4242, 4343]
EOF

mkdir -p "${OUT_DIR}"
cp "${RUN_HOME}/.config/mc_rtc/controllers/HandoverInterceptionController.yaml" "${OUT_DIR}/controller_override.yaml"
LOG="${OUT_DIR}/two_robot_sim.log"
echo "Running ${TICKER} for ${RUN_FOR} s, giver scenario ${GIVER_SCENARIO} (log: ${LOG})"
HOME="${RUN_HOME}" LD_LIBRARY_PATH="${TRIAD_BUILD_DIR}/src:${MC_RTC_INSTALL}/lib:${LD_LIBRARY_PATH:-}" \
  "${TICKER}" -f "${RUN_HOME}/mc_rtc.yaml" --run-for "${RUN_FOR}" ${TRIAD_SYNC_RATIO:+--sync-ratio "${TRIAD_SYNC_RATIO}"} > "${LOG}" 2>&1 || true
cp "${RUN_HOME}"/*.bin "${OUT_DIR}/" 2>/dev/null || true

echo "--- Robot B (giver) milestones:"
grep -E "\[DualGiver" "${LOG}" | head -20 || true
echo "--- Robot A (receiver) states:"
grep -E "Starting state|\[Completed\]|\[Failure" "${LOG}" | head -20 || true
if grep -q "\[Completed\] full plan-once handover completed" "${LOG}"; then echo "RESULT: COMPLETED"; else echo "RESULT: NOT COMPLETED (see log)"; fi
