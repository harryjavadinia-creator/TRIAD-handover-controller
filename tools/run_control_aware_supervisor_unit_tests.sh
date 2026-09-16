#!/usr/bin/env bash
# Offline unit tests for the TRIAD-lite control-aware grasp supervisor math.
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "${build_dir}"' EXIT
eigen_include="${EIGEN3_INCLUDE_DIR:-/usr/include/eigen3}"

"${CXX:-c++}" \
  -std=c++14 -Wall -Wextra -Werror -pedantic -O1 \
  -I"${eigen_include}" \
  "${script_dir}/test_control_aware_grasp_supervisor.cpp" \
  -o "${build_dir}/test_control_aware_grasp_supervisor"

"${build_dir}/test_control_aware_grasp_supervisor"
