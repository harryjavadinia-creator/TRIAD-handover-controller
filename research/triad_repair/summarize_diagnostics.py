#!/usr/bin/env python3
"""Describe diagnostic evidence, not statistical H1/H2 inference."""
import collections
import hashlib
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent

def fields(line):
    return dict(re.findall(r'(\w+)=(\[[^\]]*\]|[^ ]+)', line))

def main():
    summary = {}
    for directory in sorted((ROOT / 'evidence').iterdir()):
        log = directory / 'run.log'
        if not log.exists():
            continue
        lines = log.read_text().splitlines()
        pairs = [fields(l) for l in lines if '[TriadRepairContinuation]' in l]
        cancelled = [p for p in pairs if 'cancelled' in p.get('snapshotReason', '') or 'cancelled' in p.get('rendezvousReason', '')]
        pairs = [p for p in pairs if p not in cancelled]
        metrics = [fields(l) for l in lines if '[TriadRepairCapability]' in l]
        entry = {
            'completed_event': any('[Completed] full plan-once handover completed' in l for l in lines),
            'commit_events': sum('[V2TerminalCommit]' in l and 'committed=true' in l for l in lines),
            'paired_terminal_checks': len(pairs),
            'cancelled_terminal_checks': len(cancelled),
            'pair_counts': dict(collections.Counter((p.get('snapshotTerminal', '?') + '/' + p.get('rendezvousTerminal', '?')) for p in pairs)),
            'capability_samples': len(metrics),
            'interpretation': 'implementation diagnostic only; not independent outcome evidence',
        }
        if metrics:
            entry['same_sample_minima'] = {k: min(float(m[k]) for m in metrics) for k in ['kappa','sigmaMin','conditionIndex','manipulability','velocityWeightedSigma']}
            entry['kappa_below_one_samples'] = sum(float(m['kappa']) < 1 for m in metrics)
        summary[directory.name] = entry
        native = []
        for f in sorted(directory.glob('*.bin')):
            h = hashlib.sha256()
            with f.open('rb') as stream:
                for block in iter(lambda: stream.read(1024 * 1024), b''):
                    h.update(block)
            native.append({'name': f.name, 'bytes': f.stat().st_size, 'sha256': h.hexdigest(), 'storage': 'retained locally, not committed'})
        (directory / 'native_log_inventory.json').write_text(json.dumps(native, indent=2) + '\n')
        effective = directory / 'modules/etc/HandoverInterceptionController.yaml'
        if effective.exists():
            (directory / 'effective_controller.yaml').write_text(effective.read_text())
    (ROOT / 'DIAGNOSTIC_SUMMARY.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))

if __name__ == '__main__':
    main()
