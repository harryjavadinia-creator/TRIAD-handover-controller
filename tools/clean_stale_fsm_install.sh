#!/usr/bin/env bash
set -euo pipefail

# Remove only stale data-defined copies of the compiled HandoverInterception
# states. These files cause "no base is specified" startup errors.
#
# Usage: tools/clean_stale_fsm_install.sh --mc-rtc-prefix /path/to/mc_rtc/install
#
# --mc-rtc-prefix (or the MC_RTC_PREFIX environment variable) must point at
# your mc_rtc installation prefix -- the same one this repository was built
# against (see README.md's Build section). There is no private-workspace
# default.

MC_RTC_PREFIX="${MC_RTC_PREFIX:-}"

usage() {
  echo "usage: $0 --mc-rtc-prefix /path/to/mc_rtc/install" >&2
  exit 2
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mc-rtc-prefix) MC_RTC_PREFIX="$2"; shift 2 ;;
    *) usage ;;
  esac
done

if [[ -z "${MC_RTC_PREFIX}" ]]; then
  echo "ERROR: --mc-rtc-prefix <path> or MC_RTC_PREFIX must be set (your mc_rtc install prefix)." >&2
  echo "There is no private-workspace default; see README.md's Build section." >&2
  exit 2
fi
if [[ ! -d "${MC_RTC_PREFIX}" ]]; then
  echo "ERROR: --mc-rtc-prefix is not a directory: ${MC_RTC_PREFIX}" >&2
  exit 2
fi

roots=(
  "${MC_RTC_PREFIX}/lib/mc_controller/fsm/states/data"
  "${MC_RTC_PREFIX}/lib/mc_controller/HandoverInterceptionController/states/data"
)

found=0
for root in "${roots[@]}"; do
  [[ -d "$root" ]] || continue
  while IFS= read -r -d '' f; do
    echo "Removing stale FSM state data: $f"
    rm -f "$f"
    found=1
  done < <(find "$root" -maxdepth 1 \( -type f -o -type l \) \
    -name 'HandoverInterceptionController_*' -print0)
done

if [[ "$found" -eq 0 ]]; then
  echo "No stale HandoverInterceptionController state-data files found."
fi
