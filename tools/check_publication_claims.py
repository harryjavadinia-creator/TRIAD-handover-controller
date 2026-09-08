#!/usr/bin/env python3
"""Fail closed if known publication-claim regressions reappear.

This checker deliberately scans only publication-facing interpretation files.
Frozen source comments and byte-preserved archived reports are excluded because
known historical wording is superseded downstream rather than rewritten.
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]

FILES = [
    "README.md",
    "docs/architecture.md",
    "docs/mathematics.md",
    "docs/performance.md",
    "docs/provenance.md",
    "docs/release_validation.md",
    "docs/reproducibility.md",
    "docs/corrections_of_record.md",
    "evidence/README.md",
]

texts = {}
for rel in FILES:
    path = ROOT / rel
    if not path.is_file():
        print(f"FAIL missing publication file: {rel}")
        sys.exit(1)
    texts[rel] = path.read_text(encoding="utf-8")

joined = "\n".join(texts.values())
failures = []
checks = 0


def check(label, ok):
    global checks
    checks += 1
    if ok:
        print(f"  PASS  {label}")
    else:
        print(f"  FAIL  {label}")
        failures.append(label)


# Exact/near-exact regressions from the final independent cross-audit.
forbidden = {
    "absolute callback-no-join assurance": "no mutex, condition variable, future or join is reachable",
    "absolute copied-state no-live-pose assurance": "reads no live robot **pose or configuration** after",
    "near-ground raw byte-identity claim": "all 229 control-thread records are byte-identical",
    "H002 universal-failure claim": "no tested perturbation moves it off that boundary",
    "feasible-set lower-bound interpretation": "coverage figures derived from it are lower bounds only",
    "async clean-machine reproduction overclaim": "after\" column is independently reproduced by",
}
for label, phrase in forbidden.items():
    check(label + " absent", phrase.lower() not in joined.lower())

# Required corrected claims.
required = {
    "finite bank is explicit": "14 × 32 × 17 = 7616",
    "no weight sensitivity is explicit": "no weight-sensitivity result is reported",
    "simulation-only scope is explicit": "no validated end-to-end physical",
    "join qualification is explicit": "teardown can reach worker cancellation and `join()`",
    "live fingertip qualification is explicit": "residual live fingertip-frame reads",
    "near-ground sourceIndex normalization is explicit": "sourceIndex",
    "near-ground raw hashes are not identical": "full raw records and full set hashes are **not** identical",
    "H002 post-hoc qualification is explicit": "secondary, post-hoc diagnostic",
    "H002 3/8 split is explicit": "3 complete and 8 fail",
    "0.60 ambiguous observation is explicit": "classification is `AMBIGUOUS`",
    "0.60 displacement is explicit": "0.0241 m",
    "0.60 speed is explicit": "0.0760 m/s",
    "coverage interpretation is rejected": "not feasible-space coverage",
    "WCET is rejected": "not WCET",
    "historical/current validation is separated": "historical exact-serial runtime revalidation",
}
for label, phrase in required.items():
    check(label, phrase.lower() in joined.lower())

# File-specific guards to avoid a correction existing only somewhere unrelated.
check(
    "README qualifies callback lifecycle",
    "FSM teardown can reach worker cancellation and `join()`" in texts["README.md"],
)
check(
    "architecture qualifies live aperture",
    "Residual live\nfingertip-frame reads" in texts["docs/architecture.md"]
    or "Residual live fingertip-frame reads" in texts["docs/architecture.md"],
)
check(
    "evidence README makes full 66-case result primary",
    "The left-hand column is the primary result" in texts["evidence/README.md"],
)
check(
    "release validation denies current-head runtime relabeling",
    "not current asynchronous-head\nruntime validation" in texts["docs/release_validation.md"]
    or "not current asynchronous-head runtime validation" in texts["docs/release_validation.md"],
)
check(
    "performance scopes <2 ms to in-planning ControllerRun",
    "applies only to the in-planning `ControllerRun` sample" in texts["docs/performance.md"],
)

print()
if failures:
    print(f"PUBLICATION CLAIM CHECK: FAIL ({len(failures)} of {checks} checks failed)")
    for item in failures:
        print(f"  - {item}")
    sys.exit(1)

print(f"PUBLICATION CLAIM CHECK: PASS ({checks} checks)")
