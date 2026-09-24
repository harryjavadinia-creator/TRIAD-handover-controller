#!/usr/bin/env bash
# Verify that the frozen V1 background-planning sources pinned by
# docs/source_sync_f56add3.sha256 are still byte-identical at the commit that
# preserved them (the last V1-only main, ebaedfa). The current tree has since
# gained the V2, TRIAD-lite and two-robot code, so the manifest is checked
# against history, not against the working tree. Needs full git history.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
REF="${FROZEN_V1_COMMIT:-ebaedfa44821f8bc0be1b4ffa53cdf0af4a0b204}"
status=0
while read -r sha path; do
  [[ -z "${sha}" || "${sha}" == \#* ]] && continue
  actual="$(git show "${REF}:${path}" | sha256sum | cut -d' ' -f1)"
  if [[ "${actual}" == "${sha}" ]]; then echo "${path}: OK at ${REF:0:7}"; else echo "${path}: FAILED at ${REF:0:7}"; status=1; fi
done < docs/source_sync_f56add3.sha256
exit "${status}"
