#!/usr/bin/env python3
"""Fail closed if known publication-claim regressions reappear."""
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
    "docs/quickstart.md",
    "docs/results.md",
    "docs/related_work.md",
    "docs/corrections_of_record.md",
    "docs/experiments.md",
    "evidence/README.md",
]

texts = {}
for rel in FILES:
    path = ROOT / rel
    if not path.is_file():
        print(f"FAIL missing publication file: {rel}")
        sys.exit(1)
    texts[rel] = path.read_text(encoding="utf-8")


def norm(text):
    return " ".join(text.casefold().split())


joined = norm("\n".join(texts.values()))
norm_files = {rel: norm(text) for rel, text in texts.items()}
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


forbidden = {
    "absolute callback-no-join assurance": "no mutex, condition variable, future or join is reachable",
    "absolute copied-state no-live-pose assurance": "reads no live robot **pose or configuration** after",
    "near-ground raw byte-identity claim": "all 229 control-thread records are byte-identical",
}
for label, phrase in forbidden.items():
    check(label + " absent", norm(phrase) not in joined)

required = {
    "finite bank is explicit": "14 × 32 × 17 = 7616",
    "no weight sensitivity is explicit": "no weight-sensitivity result is reported",
    "simulation-only scope is explicit": "no validated end-to-end physical",
    "join qualification is explicit": "teardown can reach worker cancellation and `join()`",
    "live fingertip qualification is explicit": "residual live fingertip-frame reads",
    "near-ground sourceIndex normalization is explicit": "sourceIndex",
    "near-ground raw hashes are not identical": "full raw records and full set hashes are **not** identical",
    "0.60 ambiguous observation is explicit": "`ambiguous`",
    "0.60 displacement is explicit": "0.0241 m",
    "0.60 speed is explicit": "0.0760 m/s",
    "WCET is rejected": "not WCET",
    "historical/current validation is separated": "historical exact-serial runtime revalidation",
}
for label, phrase in required.items():
    check(label, norm(phrase) in joined)

release_surface = norm("\n".join(
    texts[p] for p in [
        "README.md", "docs/results.md", "docs/provenance.md",
        "docs/reproducibility.md", "docs/experiments.md",
        "docs/corrections_of_record.md", "evidence/README.md"
    ]
))
for label, phrase in {
    "held-out campaign": "held-out scenarios",
    "local perturbation campaign": "local perturbations",
    "H002 campaign diagnostic": "h002",
}.items():
    check(label + " absent from current release surface", norm(phrase) not in release_surface)

check(
    "README qualifies callback lifecycle",
    norm("FSM teardown can reach worker cancellation and `join()`")
    in norm_files["README.md"],
)
architecture = norm_files["docs/architecture.md"]
check(
    "architecture qualifies live aperture",
    "residual live" in architecture
    and "fingertip-frame reads" in architecture
    and "aperture" in architecture,
)
check(
    "release validation denies current-head runtime relabeling",
    norm("not current asynchronous-head runtime validation")
    in norm_files["docs/release_validation.md"],
)
check(
    "performance scopes <2 ms to in-planning ControllerRun",
    norm("applies only to the in-planning `ControllerRun` sample")
    in norm_files["docs/performance.md"],
)

print()
if failures:
    print(f"PUBLICATION CLAIM CHECK: FAIL ({len(failures)} of {checks} checks failed)")
    for item in failures:
        print(f"  - {item}")
    sys.exit(1)

print(f"PUBLICATION CLAIM CHECK: PASS ({checks} checks)")
