#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "${build_dir}"' EXIT

"${CXX:-c++}" \
  -std=c++14 -Wall -Wextra -Werror -pedantic \
  "${script_dir}/test_finite_plan_selector.cpp" \
  -o "${build_dir}/test_finite_plan_selector"

"${build_dir}/test_finite_plan_selector"

"${CXX:-c++}" \
  -std=c++14 -Wall -Wextra -Werror -pedantic \
  "${script_dir}/test_finite_event_plan_selector.cpp" \
  -o "${build_dir}/test_finite_event_plan_selector"

"${build_dir}/test_finite_event_plan_selector"
