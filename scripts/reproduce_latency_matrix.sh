#!/usr/bin/env bash
# Reproduce one (or all) of the July-19 perception-latency matrix runs
# (5 scenarios x 3 conditions -- see docs/experiments.md, "Dataset A: July-19
# latency matrix") at its exact historical source commit, without touching
# the scientific-baseline scientific baseline this release branch is built from.
#
# dataset-a-baseline is an ancestor of scientific-baseline (verified: git merge-base --is-ancestor
# dataset-a-baseline scientific-baseline => exit 0), so a full-history clone of this repository can
# always create this historical checkout locally -- no separate remote or
# branch is required.
#
# Mechanism: creates an isolated Git worktree at the historical commit and
# configures it (no build, no install yet). mc_rtc controllers always
# install into one shared location regardless of source commit, so building
# the historical commit will overwrite the shared installation
# scripts/run_scenario.sh depends on. Before that happens, this script
# derives the exact set of install destination paths from the historical
# build's generated cmake_install.cmake files (tools/derive_cmake_install_paths.py
# -- read-only, no install performed) and snapshots the pre-existing state
# of every one of those paths, whatever that state is (present with some
# content, or absent entirely -- this works identically on a repository that
# has never been built before). Only then does it build and install the
# historical source, run the requested scenario/condition combination(s),
# and afterward restore every snapshotted path exactly: present paths are
# restored byte-for-byte and hash-verified, paths that were absent before
# are removed. This is file-level exact restoration, not a rebuild of any
# particular commit -- if the historical install created a new parent
# directory that did not previously exist (e.g. on a machine where this
# controller had never been installed before), that now-empty directory can
# remain after restoration even though every file underneath it is restored
# or removed exactly; this has no effect on any file content or on any
# scientific result, since CMake recreates parent directories as needed on
# the next install regardless. The scientific baseline source itself (this
# repository's tracked files) is never modified. Only individual FILE
# install entries are handled; a DIRECTORY-type CMake install entry would
# make this snapshot incomplete, so the script derives its path set via
# tools/derive_cmake_install_paths.py, which fails closed (nonzero exit) if
# it finds one, rather than silently proceeding.
#
# For each run, reproduction is verified at two independent levels:
# tools/check_latency_log.py's raw controller metrics (LATENCY_METRICS_PASS),
# and tools/verify_latency_matrix_cell.py's comparison of the observed
# completion/failure class against the historically expected class for that
# scenario/condition cell (REPRODUCTION_RESULT) -- an expected FAILURE is a
# successful reproduction of that cell.
#
# mc_rtc_ticker is terminated by this script itself once a terminal outcome
# is logged, after a small fixed grace interval -- no manual intervention
# required. Whether the wrapper had to terminate the process, or the process
# had already exited on its own, is recorded separately (TICKER_STOP_REASON);
# see docs/troubleshooting.md for a separate, unresolved spontaneous-exit
# issue this distinguishes from intentional termination.
#
# Usage:
#   scripts/reproduce_latency_matrix.sh --scenario <name> --condition <name> [--dry-run] --mc-rtc-prefix <path>
#   scripts/reproduce_latency_matrix.sh --all [--dry-run] --mc-rtc-prefix <path>
#
# scenarios:  static_nominal | canonical_yz | pure_x | diagonal_xz | ground_near
# conditions: ideal | compensated220 | uncompensated220
#
# --mc-rtc-prefix <path> (or the MC_RTC_PREFIX environment variable) must
# point at your mc_rtc installation prefix (the same one CMAKE_PREFIX_PATH
# would use to build this repository -- see README.md's Build section).
# There is no private-workspace default; the script fails clearly if
# neither is given rather than guessing a local layout.
#
# Requires the same environment as scripts/run_scenario.sh (see
# docs/simulation.md): MAIN_ROBOT_MODULE_PATH set, mc_rtc_ticker on PATH.

set -euo pipefail

HISTORICAL_REF="dataset-a-baseline"

usage() {
  cat >&2 <<'USAGE'
usage:
  scripts/reproduce_latency_matrix.sh --scenario <name> --condition <name> [--dry-run] --mc-rtc-prefix <path>
  scripts/reproduce_latency_matrix.sh --all [--dry-run] --mc-rtc-prefix <path>

scenarios:  static_nominal | canonical_yz | pure_x | diagonal_xz | ground_near
conditions: ideal | compensated220 | uncompensated220

--mc-rtc-prefix <path> (or MC_RTC_PREFIX env var) is required for any
non-dry-run invocation: the mc_rtc installation prefix to configure the
historical commit against. There is no private-workspace default.
USAGE
  exit 2
}

DRY_RUN=false
DO_ALL=false
ONLY_SCENARIO=""
ONLY_CONDITION=""
MC_RTC_PREFIX="${MC_RTC_PREFIX:-}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --scenario) ONLY_SCENARIO="$2"; shift 2 ;;
    --condition) ONLY_CONDITION="$2"; shift 2 ;;
    --all) DO_ALL=true; shift ;;
    --dry-run) DRY_RUN=true; shift ;;
    --mc-rtc-prefix) MC_RTC_PREFIX="$2"; shift 2 ;;
    *) usage ;;
  esac
done

if [[ "${DO_ALL}" != true ]]; then
  [[ -n "${ONLY_SCENARIO}" && -n "${ONLY_CONDITION}" ]] || usage
fi

if [[ "${DRY_RUN}" != true ]]; then
  if [[ -z "${MC_RTC_PREFIX}" ]]; then
    echo "ERROR: --mc-rtc-prefix <path> or MC_RTC_PREFIX must be set (your mc_rtc install prefix)." >&2
    echo "There is no private-workspace default; see README.md's Build section." >&2
    exit 2
  fi
  if [[ ! -d "${MC_RTC_PREFIX}" ]]; then
    echo "ERROR: --mc-rtc-prefix is not a directory: ${MC_RTC_PREFIX}" >&2
    exit 2
  fi
fi

SCENARIOS=(static_nominal canonical_yz pure_x diagonal_xz ground_near)
CONDITIONS=(ideal compensated220 uncompensated220)

# Recovered exactly from the 15 historical per-run YAML/metadata snapshots
# (docs/experiments.md); not inferred or retuned.
scenario_translation() {
  case "$1" in
    static_nominal)  echo "[0.55, 0, 0.55]" ;;
    canonical_yz)    echo "[0.55, -0.56, 0.15]" ;;
    pure_x)          echo "[0.92, 0, 0.55]" ;;
    diagonal_xz)     echo "[0.9, 0, 0.3]" ;;
    ground_near)     echo "[0.25, 0.62, 0.15]" ;;
    *) echo "unknown scenario: $1" >&2; exit 2 ;;
  esac
}
scenario_velocity() {
  case "$1" in
    static_nominal)  echo "[0, 0, 0]" ;;
    canonical_yz)    echo "[0, 0.08, 0]" ;;
    pure_x)          echo "[-0.08, 0, 0]" ;;
    diagonal_xz)     echo "[-0.0565685, 0, 0.0565685]" ;;
    ground_near)     echo "[0, -0.08, 0]" ;;
    *) echo "unknown scenario: $1" >&2; exit 2 ;;
  esac
}
condition_fields() {
  # prints: enabled compensate
  case "$1" in
    ideal)             echo "false false" ;;
    compensated220)    echo "true true" ;;
    uncompensated220)  echo "true false" ;;
    *) echo "unknown condition: $1" >&2; exit 2 ;;
  esac
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

HISTORICAL_COMMIT="$(
  git -C "${REPO_ROOT}" rev-parse "${HISTORICAL_REF}^{commit}"
)"

WORKTREE_DIR="${REPO_ROOT}/.historical-worktrees/dataset-a-baseline"

ensure_worktree() {
  if [[ ! -d "${WORKTREE_DIR}" ]]; then
    echo "Creating isolated worktree at ${HISTORICAL_COMMIT} (${WORKTREE_DIR}) ..."
    mkdir -p "$(dirname "${WORKTREE_DIR}")"
    git -C "${REPO_ROOT}" worktree add --detach "${WORKTREE_DIR}" "${HISTORICAL_COMMIT}"
  fi
  local head
  head="$(git -C "${WORKTREE_DIR}" rev-parse HEAD)"
  if [[ "${head}" != "${HISTORICAL_COMMIT}" ]]; then
    echo "ERROR: ${WORKTREE_DIR} is at ${head}, expected ${HISTORICAL_COMMIT}" >&2
    exit 1
  fi
}

configure_only() {
  local dir="$1"
  env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u ROS_PACKAGE_PATH \
    CMAKE_PREFIX_PATH="${MC_RTC_PREFIX}" \
    cmake -S "${dir}" -B "${dir}/build" \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_DISABLE_FIND_PACKAGE_rclcpp=ON >/dev/null
}

build_and_install() {
  local dir="$1" label="$2"
  echo "Building and installing ${label} (${dir}) ..."
  configure_only "${dir}"
  cmake --build "${dir}/build" -j"$(nproc)" >/dev/null
  cmake --install "${dir}/build" >/dev/null
}

# --- Exact pre-install-state snapshot/restore ---------------------------
#
# The set of paths to snapshot is derived from the HISTORICAL build's own
# generated cmake_install.cmake files (via tools/derive_cmake_install_paths.py),
# not from any prior release build -- so this works correctly even when this
# repository has never been built or installed before. Each such path's
# state (present with specific bytes, or absent) is recorded before the
# historical install runs, and restored exactly afterward.

SNAPSHOT_DIR=""
SNAPSHOT_INDEX=""
CLEANUP_DONE=false

snapshot_pre_install_state() {
  ensure_worktree
  echo "Configuring the historical worktree (no build, no install) to derive its install paths ..."
  configure_only "${WORKTREE_DIR}"

  local paths
  paths="$(python3 "${REPO_ROOT}/tools/derive_cmake_install_paths.py" "${WORKTREE_DIR}/build")"
  if [[ -z "${paths}" ]]; then
    echo "ERROR: derived zero install paths from the historical build; refusing to proceed." >&2
    exit 1
  fi

  SNAPSHOT_DIR="$(mktemp -d)"
  SNAPSHOT_INDEX="${SNAPSHOT_DIR}/index.tsv"
  : > "${SNAPSHOT_INDEX}"

  local n=0 path hash
  while IFS= read -r path; do
    [[ -n "${path}" ]] || continue
    n=$((n + 1))
    if [[ -e "${path}" ]]; then
      hash="$(sha256sum "${path}" | awk '{print $1}')"
      cp -a "${path}" "${SNAPSHOT_DIR}/${n}.bin"
      printf '%s\tPRESENT\t%s\t%s\n' "${n}" "${hash}" "${path}" >> "${SNAPSHOT_INDEX}"
    else
      printf '%s\tABSENT\t-\t%s\n' "${n}" "${path}" >> "${SNAPSHOT_INDEX}"
    fi
  done <<< "${paths}"

  echo "Snapshotted pre-install state of ${n} path(s), derived from the historical build's own install plan."
}

restore_pre_install_state() {
  if [[ -z "${SNAPSHOT_INDEX}" || ! -f "${SNAPSHOT_INDEX}" ]]; then
    return
  fi

  echo "Restoring the exact pre-run state (byte-for-byte, verified by hash) ..."
  local restored=0 removed=0 mismatches=0
  local n status hash path
  while IFS=$'\t' read -r n status hash path; do
    if [[ "${status}" == "PRESENT" ]]; then
      cp -a "${SNAPSHOT_DIR}/${n}.bin" "${path}"
      local after
      after="$(sha256sum "${path}" | awk '{print $1}')"
      if [[ "${after}" != "${hash}" ]]; then
        echo "ERROR: restored file does not match its pre-run hash: ${path}" >&2
        mismatches=$((mismatches + 1))
      else
        restored=$((restored + 1))
      fi
    else
      if [[ -e "${path}" ]]; then
        rm -f "${path}"
        removed=$((removed + 1))
      fi
    fi
  done < "${SNAPSHOT_INDEX}"

  echo "Restore verification: ${restored} file(s) restored and hash-verified, ${removed} file(s) removed (absent before this run), ${mismatches} hash mismatch(es)."
  rm -rf "${SNAPSHOT_DIR}"
  SNAPSHOT_INDEX=""
  if [[ "${mismatches}" -ne 0 ]]; then
    echo "ERROR: install state was NOT restored exactly; see above." >&2
    return 1
  fi
}

cleanup() {
  # Idempotent: safe to call more than once (e.g. once from the normal exit
  # path and once from a signal handler racing it).
  if [[ "${CLEANUP_DONE}" == true ]]; then
    return
  fi
  CLEANUP_DONE=true
  restore_pre_install_state
}

# --- Autonomous ticker termination, with stop-reason reporting ----------
#
# Sets TICKER_STOP_REASON, TICKER_TERM_SENT, TICKER_KILL_SENT and
# TICKER_AUTONOMOUS_STATUS. A non-zero exit status caused by this script's
# own SIGTERM/SIGKILL is expected behavior, not a crash -- TICKER_STOP_REASON
# distinguishes that case (WRAPPER_TERMINATED) from the process having
# already exited on its own before this script attempted to stop it
# (SPONTANEOUS_EXIT_*), which is the separate, unresolved issue documented
# in docs/troubleshooting.md.
run_ticker_autonomous() {
  local home_dir="$1" global_config="$2" log_file="$3"
  local terminal_re='\[Completed\] full plan-once handover completed|Starting state HandoverInterceptionController_Failure'
  local grace_seconds=2
  local max_wait_seconds=180

  HOME="${home_dir}" mc_rtc_ticker -f "${global_config}" > "${log_file}" 2>&1 &
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

run_one() {
  local scenario="$1" condition="$2"
  local translation velocity enabled compensate

  translation="$(scenario_translation "${scenario}")"
  velocity="$(scenario_velocity "${scenario}")"
  read -r enabled compensate <<<"$(condition_fields "${condition}")"

  echo "Selected historical run: scenario=${scenario} condition=${condition}"
  echo "  translation=${translation} velocity=${velocity}"
  echo "  perceptionLatency.enabled=${enabled} compensate=${compensate} delay=0.220"

  local out_dir="results/latency_matrix_dataset_a/${scenario}_${condition}_$(date +%Y%m%d_%H%M%S)"
  local override_yaml
  override_yaml="$(cat <<EOF
robots:
  call_object:
    init_pos:
      translation: ${translation}
object:
  translation: ${translation}
movingObject:
  simulatedLinearVelocity: ${velocity}
  perceptionLatency:
    enabled: ${enabled}
    delay: 0.220
    compensate: ${compensate}
    bufferDuration: 1.0
gripper:
  physicalBridge:
    enabled: false
    commandEnabled: false
    requireFeedback: false
EOF
)"

  if [[ "${DRY_RUN}" == true ]]; then
    mkdir -p "${out_dir}"
    printf '%s\n' "${override_yaml}" > "${out_dir}/generated_override.yaml"
    echo "Dry run: wrote ${out_dir}/generated_override.yaml (no build, no simulation run)"
    return 0
  fi

  : "${MAIN_ROBOT_MODULE_PATH:?set MAIN_ROBOT_MODULE_PATH to your gen3_2f85 robot-module directory}"
  command -v mc_rtc_ticker >/dev/null || { echo "mc_rtc_ticker not found on PATH" >&2; exit 1; }

  local run_home
  run_home="$(mktemp -d)"

  mkdir -p "${run_home}/.config/mc_rtc/controllers"
  printf '%s\n' "${override_yaml}" > "${run_home}/.config/mc_rtc/controllers/HandoverInterceptionController.yaml"

  local global_config="${run_home}/mc_rtc.yaml"
  sed "s#\${MAIN_ROBOT_MODULE_PATH}#${MAIN_ROBOT_MODULE_PATH}#" \
    "${REPO_ROOT}/configs/mc_rtc.yaml.example" > "${global_config}"

  mkdir -p "${out_dir}"
  local log_file="${out_dir}/${scenario}_${condition}.log"
  cp "${run_home}/.config/mc_rtc/controllers/HandoverInterceptionController.yaml" \
     "${out_dir}/scenario_override.yaml"

  local started
  started="$(date --iso-8601=seconds)"
  echo "Running mc_rtc_ticker (log: ${log_file}) ..."
  run_ticker_autonomous "${run_home}" "${global_config}" "${log_file}"

  {
    echo "scenario=${scenario}"
    echo "condition=${condition}"
    echo "started=${started}"
    echo "finished=$(date --iso-8601=seconds)"
    echo "controller_head=$(git -C "${WORKTREE_DIR}" rev-parse HEAD)"
    echo "controller_branch=historical-detached-worktree"
    echo "ticker_stop_reason=${TICKER_STOP_REASON}"
    echo "ticker_term_sent=${TICKER_TERM_SENT}"
    echo "ticker_kill_sent=${TICKER_KILL_SENT}"
    echo "ticker_exit_status=${TICKER_AUTONOMOUS_STATUS}"
  } > "${out_dir}/RUN_METADATA.txt"

  set +e
  python3 "${REPO_ROOT}/tools/verify_latency_matrix_cell.py" "${log_file}" \
    --scenario "${scenario}" --condition "${condition}" \
    > "${out_dir}/reproduction_verdict.json"
  local reproduction_status=$?
  set -e
  local reproduction_result
  reproduction_result="$(python3 -c "import json,sys; print(json.load(open('${out_dir}/reproduction_verdict.json'))['reproduction_result'])" 2>/dev/null || echo UNKNOWN)"
  local expected_outcome observed_outcome latency_metrics_pass
  expected_outcome="$(python3 -c "import json; print(json.load(open('${out_dir}/reproduction_verdict.json'))['expected_outcome'])" 2>/dev/null || echo UNKNOWN)"
  observed_outcome="$(python3 -c "import json; print(json.load(open('${out_dir}/reproduction_verdict.json'))['observed_outcome'])" 2>/dev/null || echo UNKNOWN)"
  latency_metrics_pass="$(python3 -c "import json; print(json.load(open('${out_dir}/reproduction_verdict.json'))['latency_metrics_pass'])" 2>/dev/null || echo UNKNOWN)"

  echo ""
  echo "=== historical run result ==="
  echo "TICKER_STOP_REASON=${TICKER_STOP_REASON}"
  echo "TICKER_TERM_SENT=${TICKER_TERM_SENT}"
  echo "TICKER_KILL_SENT=${TICKER_KILL_SENT}"
  echo "TICKER_EXIT_STATUS=${TICKER_AUTONOMOUS_STATUS}"
  echo "EXPECTED_OUTCOME=${expected_outcome}"
  echo "OBSERVED_OUTCOME=${observed_outcome}"
  echo "LATENCY_METRICS_PASS=${latency_metrics_pass}"
  echo "REPRODUCTION_RESULT=${reproduction_result}"
  echo "Log, override, metadata and reproduction verdict preserved in: ${out_dir}"

  rm -rf "${run_home}"

  if [[ "${reproduction_status}" -ne 0 ]]; then
    OVERALL_STATUS=1
  fi
}

OVERALL_STATUS=0
NEEDS_BUILD=true
[[ "${DRY_RUN}" == true ]] && NEEDS_BUILD=false

if [[ "${NEEDS_BUILD}" == true ]]; then
  snapshot_pre_install_state
  trap cleanup EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM

  echo "WARNING: installing historical commit ${HISTORICAL_COMMIT} will" >&2
  echo "temporarily overwrite the shared mc_rtc installation used by" >&2
  echo "scripts/run_scenario.sh; it will be restored exactly on exit." >&2
  build_and_install "${WORKTREE_DIR}" "historical commit ${HISTORICAL_COMMIT}"
fi

if [[ "${DO_ALL}" == true ]]; then
  for scenario in "${SCENARIOS[@]}"; do
    for condition in "${CONDITIONS[@]}"; do
      run_one "${scenario}" "${condition}"
    done
  done
else
  run_one "${ONLY_SCENARIO}" "${ONLY_CONDITION}"
fi

exit "${OVERALL_STATUS}"
