#!/usr/bin/env python3
"""Fail closed if known scientific-documentation claim regressions reappear."""
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
    "docs/global_time_plan.md",
    "docs/robot_module.md",
    "docs/real_robot.md",
    "docs/simulation.md",
    "docs/timing_frontiers.md",
    "docs/troubleshooting.md",
    "call_object_description/README.md",
    "evidence/README.md",
    ".github/workflows/source-checks.yml",
]

texts = {}
for rel in FILES:
    path = ROOT / rel
    if not path.is_file():
        print(f"FAIL missing documentation file: {rel}")
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
    "audience-specific supervisor-facing wording": "supervisor-facing",
    "audience-specific supervisor-package wording": "supervisor package",
    "internal release-facing wording": "release-facing",
    "presentation-specific interpretation wording": "for presentation purposes",
    "internal publication-head wording": "publication-head",
    "internal current-story wording": "main current experiment story",
    "internal current-publication-interpretation wording": "current publication interpretation",
    "internal present-discussion wording": "present controller/method discussion",
    "publication-preparation process wording": "during publication preparation",
}
for label, phrase in forbidden.items():
    check(label + " absent", norm(phrase) not in joined)

required = {
    "finite bank is explicit": "14 × 32 × 17 = 7616",
    "no weight sensitivity is explicit": "no weight-sensitivity result is reported",
    "simulation-only scope is explicit": "no validated end-to-end physical",
    "live fingertip qualification is explicit": "residual live fingertip-frame reads",
    "0.60 ambiguous observation is explicit": "`ambiguous`",
    "0.60 displacement is explicit": "0.0241 m",
    "0.60 speed is explicit": "0.0760 m/s",
    "WCET limitation is explicit": "wcet",
    "historical exact-serial validation is separated": "historical exact-serial runtime revalidation",
}
for label, phrase in required.items():
    check(label, norm(phrase) in joined)

scientific_surface = norm("\n".join(
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
    check(label + " absent from scientific documentation surface",
          norm(phrase) not in scientific_surface)

readme = norm_files["README.md"]
check(
    "README qualifies background-worker lifecycle",
    "background worker" in readme
    and "shutdown/reset" in readme
    and "wcet" in readme,
)

architecture = norm_files["docs/architecture.md"]
check(
    "architecture qualifies live aperture",
    "fingertip-frame" in architecture
    and "aperture" in architecture
    and "race freedom" in architecture,
)

validation = norm_files["docs/release_validation.md"]
check(
    "validation keeps historical and later source states separate",
    "historical exact-serial runtime revalidation" in validation
    and "not a runtime rerun" in validation
    and "f56add3" in validation,
)

performance = norm_files["docs/performance.md"]
check(
    "performance page is scoped to historical exact-serial evidence",
    "earlier exact-serial" in performance
    and "background-planning implementation" in performance
    and "8,168,732" in performance,
)

print()
if failures:
    print(f"DOCUMENTATION CLAIM CHECK: FAIL ({len(failures)} of {checks} checks failed)")
    for item in failures:
        print(f"  - {item}")
    sys.exit(1)

print(f"DOCUMENTATION CLAIM CHECK: PASS ({checks} checks)")
