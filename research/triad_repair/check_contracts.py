#!/usr/bin/env python3
"""Integration wiring checks, supplementary to numerical/runtime tests.
These do not establish preview/QP fidelity or physical feasibility.
"""
from pathlib import Path
root=Path(__file__).resolve().parents[2]
s=(root/'src/ReceiverV2.cpp').read_text()
def body(name):
    start=s.index('HandoverInterceptionController::'+name+'(')
    start=s.index('{',start); depth=1; i=start+1
    while depth:
        depth+=(s[i]=='{')-(s[i]=='}'); i+=1
    return s[start:i]
f=body('controlAwareHypothesesV2')
assert 'objectFixedReceivingGraspPoses(' in f
assert 'receivingGraspPoses(handle, gi, outward' not in f
f=body('submitReceiverCertificationV2')
assert 'interceptionStartPose = v2ReferencePose_' in f
assert 'interceptionExecutionReferenceV2(now +' not in f
f=body('rolloutInterceptionV2')
assert 'mbc = planningSnapshot_.frozenRobotState' in f
assert 'std::fill(a.begin()' not in f
assert 'commandReference = referenceStart' in f
f=body('solveInterceptionV2')
assert 'rolloutInterceptionV2(prediction, request.snapshotTime,' in f
assert 'terminalFromStateV2(fromRendezvous, terminal, rollout.rendezvousState)' in f
f=body('setInterceptionExecutionV2')
assert f.index('matchesActiveGraspGeometryV2') < f.index('v2Icpt_ = x')
assert 'x.O_T_M_standoff = provisionalReceiverPlan_.plan.O_T_M_standoff' in f
f=body('handlePredictiveSelectionV2')
assert f.index('matchesActiveGraspGeometryV2') < f.index('updateGraspSelector')
for name in ['predictionPoseAtV2','rolloutInterceptionV2','solveInterceptionV2','controlAwareHypothesesV2']:
    f=body(name)
    assert not any(x in f for x in ['independentGiverScript_', 'independentGiverStateAt(', 'presentationStopTime_'])
f=body('controlAwareHypothesesV2')
assert 'v2CaParams_.matchedCandidateIds' in f
f=body('evaluateControlAwareExactLayersV2')
assert '!eval.record.authorityFeasible && !v2CaParams_.matchedExperiment' in f
f=body('handleControlAwareSelectionV2')
assert 'records.back().reserve = 0.0' in f
f=body('stepControlAwareTrackV2')
assert 'referenceStart = compose(objectNext, relativePose(objectNow, v2ReferencePose_))' in f
print('PASS matched explicit pool, diagnostic authority, observed-motion feedforward')
print('PASS identity, immutable geometry, snapshot epoch, linked terminal state, no truth-stop reads in decision functions')
print('LIMIT: source wiring checks; not a physical execution certificate')
