#!/usr/bin/env bash
# Regression gate for the exact clearance-acceleration hierarchy.
#
# The hierarchy in gripperBasePoseSafe() is an acceleration structure only: it
# may skip a scalar clearance evaluation solely when a rigorous bound proves the
# evaluation cannot change the report. This script runs a frozen reproduction
# scenario with TRIAD_COLLISION_ORACLE_CHECK=1, which makes the controller
# evaluate the original brute-force loop alongside the accelerated one and
# compare the complete report contract on every single query:
#
#   report.safe, report.minClearance, report.groundClearance,
#   report.sample, report.obstacle
#
# Any disagreement is a correctness regression, not a performance regression.
# The run must report zero mismatches on all five fields.
#
# Usage:
#   tools/check_collision_hierarchy_oracle.sh [scenario] [output-dir]
#
# Requires the same environment as scripts/run_scenario.sh (built and installed
# controller, MAIN_ROBOT_MODULE_PATH, mc_rtc_ticker on PATH).
#
# Note: oracle-check mode runs both evaluators and is therefore much slower than
# normal. Never use a run made under this mode for performance measurement.

set -euo pipefail

SCENARIO="${1:-longitudinal}"
OUT_DIR="${2:-results/$(date +%Y%m%d_%H%M%S)_oracle_${SCENARIO}}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

export TRIAD_COLLISION_ORACLE_CHECK=1

echo "Running ${SCENARIO} with collision oracle comparison enabled ..."
SCENARIO_STATUS=0
"${REPO_ROOT}/scripts/run_scenario.sh" "${SCENARIO}" "${OUT_DIR}" || SCENARIO_STATUS=$?

LOG_FILE="${OUT_DIR}/${SCENARIO}.log"
if [[ ! -f "${LOG_FILE}" ]]; then
  echo "ORACLE_EQUIVALENCE_RESULT=FAIL (no log at ${LOG_FILE})" >&2
  exit 1
fi

SUMMARY="$(grep -o 'CollisionOracleSummary\].*' "${LOG_FILE}" | tail -1 || true)"
MISMATCH_LINES="$(grep -c 'CollisionOracleMismatch' "${LOG_FILE}" || true)"

echo ""
echo "=== collision oracle equivalence ==="
if [[ -z "${SUMMARY}" ]]; then
  echo "ORACLE_EQUIVALENCE_RESULT=FAIL"
  echo "reason: no [CollisionOracleSummary] line found. Either the controller"
  echo "        predates the oracle-check mode, or planning never reached the"
  echo "        global selection stage."
  exit 1
fi
echo "${SUMMARY}"
echo "per-query mismatch reports: ${MISMATCH_LINES}"

RESULT=FAIL
if [[ "${SUMMARY}" == *"result=PASS"* && "${MISMATCH_LINES}" -eq 0 ]]; then
  RESULT=PASS
fi

echo "ORACLE_EQUIVALENCE_RESULT=${RESULT}"
echo "SCENARIO_RUN_EXIT_STATUS=${SCENARIO_STATUS}"
echo "Log preserved in: ${OUT_DIR}"

if [[ "${RESULT}" != PASS ]]; then
  echo "" >&2
  echo "The accelerated evaluator disagreed with the frozen brute-force oracle." >&2
  echo "This is a hard correctness failure: the hierarchy must reproduce the" >&2
  echo "oracle exactly, including the strict (clearance < minClearance) update" >&2
  echo "semantics and the resulting limiting sample/obstacle." >&2
  exit 1
fi

# The scenario's own scientific gates must also hold; oracle equivalence alone
# is not sufficient evidence that the run was valid.
if [[ "${SCENARIO_STATUS}" -ne 0 ]]; then
  echo "" >&2
  echo "Oracle equivalence passed but the scenario itself did not: see" >&2
  echo "HANDOVER_COMPLETED / RUNTIME_CHECKER_RESULT / SCENARIO_IDENTITY_RESULT" >&2
  echo "above. Reported separately and never collapsed into one another." >&2
  exit "${SCENARIO_STATUS}"
fi
