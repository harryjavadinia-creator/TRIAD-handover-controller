#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

"${script_dir}/run_binding_cost_unit_tests.sh"
python3 "${script_dir}/check_binding_cost_source.py"
python3 "${script_dir}/test_check_binding_cost_log.py"
python3 "${script_dir}/test_check_global_time_plan_log.py"
python3 -m py_compile "${script_dir}/check_binding_cost_source.py"
python3 -m py_compile \
  "${script_dir}/check_binding_cost_log.py" \
  "${script_dir}/test_check_binding_cost_log.py" \
  "${script_dir}/check_global_time_plan_log.py" \
  "${script_dir}/test_check_global_time_plan_log.py"

echo "global time-plan binding-cost development checks: PASS"
