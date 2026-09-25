#!/usr/bin/env bash
# Build two_robot/tools/kortex_home.cpp against the Kortex API that mc_kortex downloads at configure time.
#   KORTEX_ROOT_DIR=<kortex_api/2.6.0> TRIAD_BUILD_DIR=<build> bash two_robot/tools/build_kortex_home.sh
# Output: ${TRIAD_BUILD_DIR}/kortex_home
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KORTEX_ROOT_DIR="${KORTEX_ROOT_DIR:-$HOME/mc_rtc_ws/build/mc_kortex/kortex_api/2.6.0}"
OUT_DIR="${TRIAD_BUILD_DIR:-$SCRIPT_DIR/../../build}"
[[ -f "$KORTEX_ROOT_DIR/lib/release/libKortexApiCpp.a" ]] || { echo "Kortex API not found at $KORTEX_ROOT_DIR (set KORTEX_ROOT_DIR)" >&2; exit 1; }
mkdir -p "$OUT_DIR"
g++ -std=c++17 -O1 -D_OS_UNIX -Wno-deprecated-declarations \
  -I"$KORTEX_ROOT_DIR/include" -I"$KORTEX_ROOT_DIR/include/client" -I"$KORTEX_ROOT_DIR/include/common" \
  -I"$KORTEX_ROOT_DIR/include/messages" -I"$KORTEX_ROOT_DIR/include/client_stubs" \
  "$SCRIPT_DIR/kortex_home.cpp" "$KORTEX_ROOT_DIR/lib/release/libKortexApiCpp.a" -lpthread -o "$OUT_DIR/kortex_home"
echo "built $OUT_DIR/kortex_home"
