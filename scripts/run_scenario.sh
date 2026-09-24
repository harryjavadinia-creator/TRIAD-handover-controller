#!/usr/bin/env bash
# Run one of the four reported TRIAD scenarios in simulation and capture its
# log, without modifying any tracked file or the researcher's persistent
# mc_rtc configuration.
#
# Mechanism: mc_rtc merges a per-controller user configuration fragment from
# $HOME/.config/mc_rtc/controllers/<ControllerName>.yaml on top of the
# installed default (mc_control::MCController::robot_config /
# MCGlobalController::GlobalConfiguration::load_controllers_configs). This
# script points HOME at a fresh temporary directory for the duration of the
# run and writes only the scenario-varying fields there, so the merge is a
# deep merge: everything not written in the fragment keeps the installed
# controller's default value. Nothing under the real $HOME is read or
# written.
#
# Usage:
#   scripts/run_scenario.sh <near-ground|longitudinal|lateral-low|diagonal> [output-dir]
#
# Optional environment:
#   TRIAD_RECEIVER_MODE=finite-plan (default)        giver follows the commit (alias v1)
#   TRIAD_RECEIVER_MODE=finite-plan-independent      against the robot-independent giver (alias v1-independent)
#   TRIAD_RECEIVER_MODE=receding                     receding mode, independent giver (alias v2)
#   TRIAD_EXTRA_OVERRIDE=<file>          YAML appended to the override (fault injection)
#   TRIAD_MAX_WAIT=<seconds>             wall-clock bound for the ticker (default 180)
#   TRIAD_SYNC_RATIO=<sim/real>          run the simulation slower than real time (0.5 = half
#                                        speed); use it when a viewer is attached, see docs/quickstart.md
#   TRIAD_BUILD_DIR=<dir>                run the controller straight from this CMake build tree
#                                        (no install into mc_rtc needed); MC_RTC_INSTALL locates
#                                        mc_rtc's own FSM states (default $HOME/mc_rtc_ws/install)
#
# Requires:
#   - the controller built (docs/quickstart.md); either installed into mc_rtc or
#     run from the build tree with TRIAD_BUILD_DIR
#   - MAIN_ROBOT_MODULE_PATH set to a local Kinova Gen3 + Robotiq 2F-85
#     mc_rtc robot-module directory (see configs/mc_rtc.yaml.example)
#   - mc_rtc_ticker on PATH

set -euo pipefail

usage() {
  echo "usage: $0 <near-ground|longitudinal|lateral-low|diagonal> [output-dir]" >&2
  exit 2
}

# Runs mc_rtc_ticker in the background, watches its log for a terminal
# outcome, waits a small fixed grace interval, then terminates it -- so
# callers never need to intervene manually. Sets TICKER_STOP_REASON,
# TICKER_TERM_SENT, TICKER_KILL_SENT and TICKER_AUTONOMOUS_STATUS. A
# non-zero exit status caused by this script's own SIGTERM/SIGKILL is
# expected behavior, not a crash -- TICKER_STOP_REASON distinguishes that
# (WRAPPER_TERMINATED) from the process having already exited on its own
# before this script attempted to stop it (SPONTANEOUS_EXIT_*), which is the
# separate, unresolved issue documented in docs/troubleshooting.md.
run_ticker_autonomous() {
  local home_dir="$1" global_config="$2" log_file="$3"
  local terminal_re='\[Completed\] full plan-once handover completed|Starting state HandoverInterceptionController_Failure'
  local grace_seconds=2
  local max_wait_seconds="${TRIAD_MAX_WAIT:-180}"

  # TRIAD_SYNC_RATIO=<sim/real> slows the simulation (e.g. 0.5 = half speed) so that a
  # viewer attached to the controller does not eat into the planner's real-time budget.
  HOME="${home_dir}" mc_rtc_ticker -f "${global_config}" ${TRIAD_SYNC_RATIO:+--sync-ratio "${TRIAD_SYNC_RATIO}"} > "${log_file}" 2>&1 &
  local ticker_pid=$!

  local waited=0
  local terminal_seen=false
  while kill -0 "${ticker_pid}" 2>/dev/null; do
    if grep -qE "${terminal_re}" "${log_file}" 2>/dev/null; then
      terminal_seen=true
      break
    fi
    sleep 1
    waited=$((waited + 1))
    if [[ "${waited}" -ge "${max_wait_seconds}" ]]; then
      echo "WARNING: no terminal outcome detected within ${max_wait_seconds}s; terminating anyway" >&2
      break
    fi
  done

  if [[ "${terminal_seen}" == true ]]; then
    sleep "${grace_seconds}"
  fi

  TICKER_TERM_SENT=false
  TICKER_KILL_SENT=false

  if kill -0 "${ticker_pid}" 2>/dev/null; then
    kill -TERM "${ticker_pid}" 2>/dev/null || true
    TICKER_TERM_SENT=true
    local waited_term=0
    while kill -0 "${ticker_pid}" 2>/dev/null && [[ "${waited_term}" -lt 5 ]]; do
      sleep 1
      waited_term=$((waited_term + 1))
    done
    if kill -0 "${ticker_pid}" 2>/dev/null; then
      kill -KILL "${ticker_pid}" 2>/dev/null || true
      TICKER_KILL_SENT=true
    fi
    if [[ "${terminal_seen}" == true ]]; then
      TICKER_STOP_REASON="WRAPPER_TERMINATED"
    else
      TICKER_STOP_REASON="WRAPPER_TERMINATED_AFTER_TIMEOUT"
    fi
  else
    if [[ "${terminal_seen}" == true ]]; then
      TICKER_STOP_REASON="SPONTANEOUS_EXIT_AFTER_TERMINAL_OUTCOME"
    else
      TICKER_STOP_REASON="SPONTANEOUS_EXIT_BEFORE_TERMINAL_OUTCOME"
    fi
  fi

  TICKER_AUTONOMOUS_STATUS=0
  wait "${ticker_pid}" 2>/dev/null || TICKER_AUTONOMOUS_STATUS=$?
}

[[ $# -ge 1 ]] || usage
SCENARIO="$1"
OUT_DIR="${2:-results/$(date +%Y%m%d_%H%M%S)_${SCENARIO}}"

: "${MAIN_ROBOT_MODULE_PATH:?set MAIN_ROBOT_MODULE_PATH to your gen3_2f85 robot-module directory}"
command -v mc_rtc_ticker >/dev/null || { echo "mc_rtc_ticker not found on PATH" >&2; exit 1; }

case "$SCENARIO" in
  near-ground)
    TRANSLATION="[0.25, 0.62, 0.15]"
    VELOCITY="[0.0, -0.08, 0.0]"
    ;;
  longitudinal)
    TRANSLATION="[0.92, 0.00, 0.55]"
    VELOCITY="[-0.08, 0.0, 0.0]"
    ;;
  lateral-low)
    TRANSLATION="[0.55, -0.56, 0.15]"
    VELOCITY="[0.0, 0.08, 0.0]"
    ;;
  diagonal)
    TRANSLATION="[0.90, 0.00, 0.30]"
    VELOCITY="[-0.0565685, 0.0, 0.0565685]"
    ;;
  *)
    usage
    ;;
esac

echo "Selected scenario: ${SCENARIO}"
echo "  object translation: ${TRANSLATION}"
echo "  object velocity:    ${VELOCITY}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

RUN_HOME="$(mktemp -d)"
CLEANUP_DONE=false
cleanup() {
  if [[ "${CLEANUP_DONE}" == true ]]; then
    return
  fi
  CLEANUP_DONE=true
  rm -rf "${RUN_HOME}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

mkdir -p "${RUN_HOME}/.config/mc_rtc/controllers"
cat > "${RUN_HOME}/.config/mc_rtc/controllers/HandoverInterceptionController.yaml" <<EOF
robots:
  call_object:
    init_pos:
      translation: ${TRANSLATION}
object:
  translation: ${TRANSLATION}
movingObject:
  simulatedLinearVelocity: ${VELOCITY}
EOF

RECEIVER_MODE="${TRIAD_RECEIVER_MODE:-v1}"
# mode names as in the documentation map to the configuration keys
case "$RECEIVER_MODE" in
  finite-plan) RECEIVER_MODE=v1 ;;
  finite-plan-independent) RECEIVER_MODE=v1-independent ;;
  receding) RECEIVER_MODE=v2 ;;
esac
case "$RECEIVER_MODE" in
  v1) ;;
  v1-independent)
    echo "  giverTruthModel: independent_scripted" >> "${RUN_HOME}/.config/mc_rtc/controllers/HandoverInterceptionController.yaml"
    ;;
  v2)
    echo "  giverTruthModel: independent_scripted" >> "${RUN_HOME}/.config/mc_rtc/controllers/HandoverInterceptionController.yaml"
    echo "receiverArchitecture: v2_receding" >> "${RUN_HOME}/.config/mc_rtc/controllers/HandoverInterceptionController.yaml"
    ;;
  *)
    echo "unknown TRIAD_RECEIVER_MODE=${RECEIVER_MODE}" >&2
    exit 2
    ;;
esac
if [[ -n "${TRIAD_EXTRA_OVERRIDE:-}" ]]; then
  cat "${TRIAD_EXTRA_OVERRIDE}" >> "${RUN_HOME}/.config/mc_rtc/controllers/HandoverInterceptionController.yaml"
fi
echo "  receiver mode:      ${RECEIVER_MODE}"

GLOBAL_CONFIG="${RUN_HOME}/mc_rtc.yaml"
sed "s#\${MAIN_ROBOT_MODULE_PATH}#${MAIN_ROBOT_MODULE_PATH}#" \
  "${REPO_ROOT}/configs/mc_rtc.yaml.example" > "${GLOBAL_CONFIG}"

# Build-tree mode: point mc_rtc at the controller library, its configuration and
# its FSM states inside the build directory instead of the mc_rtc install.
if [[ -n "${TRIAD_BUILD_DIR:-}" ]]; then
  MC_RTC_INSTALL="${MC_RTC_INSTALL:-$HOME/mc_rtc_ws/install}"
  MODS="${RUN_HOME}/mods"; mkdir -p "${MODS}/etc"
  ln -s "${TRIAD_BUILD_DIR}/src/HandoverInterceptionController_controller.so" "${MODS}/"
  ln -s "${TRIAD_BUILD_DIR}/etc/HandoverInterceptionController.yaml" "${MODS}/etc/"
  printf 'ControllerModulePaths: ["%s"]\n' "${MODS}" >> "${GLOBAL_CONFIG}"
  {
    echo "StatesLibraries:"
    echo "- \"${MC_RTC_INSTALL}/lib/mc_controller/fsm/states\""
    echo "- \"${TRIAD_BUILD_DIR}/src/states\""
    echo "StatesFiles:"
    echo "- \"${MC_RTC_INSTALL}/lib/mc_controller/fsm/states/data\""
    cat "${RUN_HOME}/.config/mc_rtc/controllers/HandoverInterceptionController.yaml"
  } > "${RUN_HOME}/override.tmp"
  mv "${RUN_HOME}/override.tmp" "${RUN_HOME}/.config/mc_rtc/controllers/HandoverInterceptionController.yaml"
  export LD_LIBRARY_PATH="${TRIAD_BUILD_DIR}/src:${MC_RTC_INSTALL}/lib:${LD_LIBRARY_PATH:-}"
  echo "  controller:         build tree ${TRIAD_BUILD_DIR}"
fi

mkdir -p "${OUT_DIR}"
LOG_FILE="${OUT_DIR}/${SCENARIO}.log"
cp "${RUN_HOME}/.config/mc_rtc/controllers/HandoverInterceptionController.yaml" \
   "${OUT_DIR}/scenario_override.yaml"

echo "Running mc_rtc_ticker (log: ${LOG_FILE}) ..."
# mc_rtc_ticker does not exit on its own after a scenario reaches a terminal
# outcome. run_ticker_autonomous watches the log for a terminal marker
# (completed handover or FSM failure state), waits a small fixed grace
# interval for trailing log lines, then terminates the ticker itself -- no
# manual intervention required. Process termination and scientific runtime
# verification are two independent results and must never be collapsed into
# one another: TICKER_EXIT_STATUS is reported separately from
# HANDOVER_COMPLETED/RUNTIME_CHECKER_RESULT below and is never treated as a
# pass/fail signal by itself. See docs/troubleshooting.md.
run_ticker_autonomous "${RUN_HOME}" "${GLOBAL_CONFIG}" "${LOG_FILE}"
TICKER_STATUS="${TICKER_AUTONOMOUS_STATUS}"

HANDOVER_COMPLETED=false
if grep -q "\[Completed\] full plan-once handover completed" "${LOG_FILE}"; then
  HANDOVER_COMPLETED=true
fi

RUNTIME_CHECKER_RESULT=SKIPPED
RUNTIME_CHECKER="${REPO_ROOT}/tools/check_global_time_plan_log.py"
if [[ "${RECEIVER_MODE}" == v2 ]]; then
  RUNTIME_CHECKER="${REPO_ROOT}/tools/check_v2_run_log.py"
fi
if [[ "${HANDOVER_COMPLETED}" == true ]]; then
  if python3 "${RUNTIME_CHECKER}" "${LOG_FILE}" > "${OUT_DIR}/checker_output.txt" 2>&1; then
    RUNTIME_CHECKER_RESULT=PASS
  else
    RUNTIME_CHECKER_RESULT=FAIL
  fi
fi

SCENARIO_IDENTITY_RESULT=SKIPPED
if python3 "${REPO_ROOT}/tools/verify_scenario_identity.py" "${LOG_FILE}" \
    --expect-scenario "${SCENARIO}" > "${OUT_DIR}/scenario_identity_output.txt" 2>&1; then
  SCENARIO_IDENTITY_RESULT=PASS
else
  SCENARIO_IDENTITY_RESULT=FAIL
fi

echo ""
echo "=== run result ==="
echo "TICKER_STOP_REASON=${TICKER_STOP_REASON}"
echo "TICKER_TERM_SENT=${TICKER_TERM_SENT}"
echo "TICKER_KILL_SENT=${TICKER_KILL_SENT}"
echo "TICKER_EXIT_STATUS=${TICKER_STATUS}"
echo "HANDOVER_COMPLETED=${HANDOVER_COMPLETED}"
echo "RUNTIME_CHECKER_RESULT=${RUNTIME_CHECKER_RESULT}"
echo "SCENARIO_IDENTITY_RESULT=${SCENARIO_IDENTITY_RESULT}"
if [[ "${TICKER_STOP_REASON}" == SPONTANEOUS_EXIT_* ]]; then
  echo "NOTE: mc_rtc_ticker exited on its own (${TICKER_STOP_REASON}, status" >&2
  echo "${TICKER_STATUS}) before this script attempted to terminate it -- see" >&2
  echo "docs/troubleshooting.md. This is reported separately from" >&2
  echo "HANDOVER_COMPLETED/RUNTIME_CHECKER_RESULT above; it is not evidence of" >&2
  echo "either scientific success or scientific failure by itself." >&2
fi
echo "Log, scenario override and checker output preserved in: ${OUT_DIR}"

if [[ "${HANDOVER_COMPLETED}" != true || "${RUNTIME_CHECKER_RESULT}" != PASS || "${SCENARIO_IDENTITY_RESULT}" != PASS ]]; then
  exit 1
fi
