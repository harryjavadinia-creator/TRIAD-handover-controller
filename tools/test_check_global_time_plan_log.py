#!/usr/bin/env python3
"""Regression tests for the global time-grasp-route runtime result checker."""

from check_global_time_plan_log import (
    verify_failsafe_text,
    verify_frozen_objective,
    verify_text,
)


VALID = """
[warning] [PlanSelectionConfiguration] mode=binding_cost costConfigurationValid=true tieTolerance=1.000e-09 allowPhysicalExecution=false weightSumBeforeNormalization=1.000000 eventTimePolicy=global_time_plan timeTerm=search_to_completion
[success] [GlobalTimePlanSearchConfiguration] enabled=true fixedSearchEpoch=10.000s fixedAbsoluteEvents=true configuredHypotheses=2 leadRange=[3.000,6.000] objective=min_time_grasp_route timeTerm=search_to_completion scheduleMustComplete=true finalTimingReadmission=true robotStationary=true
[success] [GlobalPlanCost] hypothesis=1 eventLead=3.000s presentationTime=13.000s candidate=a route=direct valid=true motionJ=0.500000000 scheduleWait=+1.000s searchToCompletion=7.000s globalJ=0.550000000 timeTerm=search_to_completion hardFeasibilityUnchanged=true
[success] [GlobalTimePlanCapture] success=true hypothesis=1 eventLead=3.000s completePlansAdded=1 cumulativeCompletePlans=1 deferredCommit=true robotMotion=false
[success] [GlobalPlanCost] hypothesis=2 eventLead=6.000s presentationTime=16.000s candidate=b route=ring valid=true motionJ=0.400000000 scheduleWait=+3.500s searchToCompletion=9.500s globalJ=0.540000000 timeTerm=search_to_completion hardFeasibilityUnchanged=true
[success] [GlobalTimePlanCapture] success=true hypothesis=2 eventLead=6.000s completePlansAdded=1 cumulativeCompletePlans=2 deferredCommit=true robotMotion=false
[success] [GlobalTimePlanSelection] success=true scheduleComplete=true evaluatedHypotheses=2 configuredHypotheses=2 completePlans=2 costValidPlans=2 timingAdmissiblePlans=2 hypothesis=2 eventLead=6.000s presentationTime=16.000s candidate=b route=ring motionJ=0.400000000 scheduleWait=+3.500s selectedGlobalJ=0.540000000 minimumGlobalJ=0.540000000 selectedWithinMinimumTolerance=true tieTolerance=1.000e-09 tieBreak=[completion,event,candidateReach,clearance,candidate,route,hypothesis,index]
[success] [GlobalTimePlanCommitProof] committed=true scheduleComplete=true evaluatedHypotheses=2 configuredHypotheses=2 completePlans=2 costValidPlans=2 timingAdmissiblePlans=2 hypothesis=2 eventLead=6.000s presentationTime=16.000s candidate=b route=ring motionJ=0.400000000 scheduleWait=+3.500s selectedGlobalJ=0.540000000 minimumGlobalJ=0.540000000 selectedWithinMinimumTolerance=true tieTolerance=1.000e-09 objective=time_grasp_route noRetiming=true noReplanning=true
[success] [GlobalTimePlanSearchSummary] committed=true scheduleComplete=true evaluatedHypotheses=2 configuredHypotheses=2 feasibleHypotheses=2 geometryFailures=0 selectedLead=6.000s selectedMotionJ=0.400000000 selectedGlobalJ=0.540000000 scheduleWait=+3.500s elapsed=1.000s robotMoved=[0.0000,0.0000] oneCommit=true globalArgmin=true
[success] [Completed] full plan-once handover completed: grasp confirmed and carried-object retreat finished mode=MOVING
"""

# A: a successful run whose pool legitimately contains both valid=true and
# valid=false GlobalPlanCost records (hypothesis 1 has one of each; the
# invalid one is correctly excluded from the pool and never compared).
PARTIAL_INVALID_HYPOTHESIS = """
[warning] [PlanSelectionConfiguration] mode=binding_cost costConfigurationValid=true tieTolerance=1.000e-09 allowPhysicalExecution=false weightSumBeforeNormalization=1.000000 eventTimePolicy=global_time_plan timeTerm=search_to_completion
[success] [GlobalTimePlanSearchConfiguration] enabled=true fixedSearchEpoch=10.000s fixedAbsoluteEvents=true configuredHypotheses=2 leadRange=[3.000,6.000] objective=min_time_grasp_route timeTerm=search_to_completion scheduleMustComplete=true finalTimingReadmission=true robotStationary=true
[success] [GlobalPlanCost] hypothesis=1 eventLead=3.000s presentationTime=13.000s candidate=a route=direct valid=true motionJ=0.500000000 scheduleWait=+1.000s searchToCompletion=7.000s globalJ=0.550000000 timeTerm=search_to_completion hardFeasibilityUnchanged=true
[success] [GlobalPlanCost] hypothesis=1 eventLead=3.000s presentationTime=13.000s candidate=z route=ring valid=false motionJ=1000000000.000000 scheduleWait=+1.000s searchToCompletion=1000000001.000s globalJ=1000000000.550000000 timeTerm=search_to_completion hardFeasibilityUnchanged=true
[success] [GlobalTimePlanCapture] success=true hypothesis=1 eventLead=3.000s completePlansAdded=1 cumulativeCompletePlans=1 deferredCommit=true robotMotion=false
[success] [GlobalPlanCost] hypothesis=2 eventLead=6.000s presentationTime=16.000s candidate=b route=ring valid=true motionJ=0.400000000 scheduleWait=+3.500s searchToCompletion=9.500s globalJ=0.540000000 timeTerm=search_to_completion hardFeasibilityUnchanged=true
[success] [GlobalTimePlanCapture] success=true hypothesis=2 eventLead=6.000s completePlansAdded=1 cumulativeCompletePlans=2 deferredCommit=true robotMotion=false
[success] [GlobalTimePlanSelection] success=true scheduleComplete=true evaluatedHypotheses=2 configuredHypotheses=2 completePlans=2 costValidPlans=2 timingAdmissiblePlans=2 hypothesis=2 eventLead=6.000s presentationTime=16.000s candidate=b route=ring motionJ=0.400000000 scheduleWait=+3.500s selectedGlobalJ=0.540000000 minimumGlobalJ=0.540000000 selectedWithinMinimumTolerance=true tieTolerance=1.000e-09 tieBreak=[completion,event,candidateReach,clearance,candidate,route,hypothesis,index]
[success] [GlobalTimePlanCommitProof] committed=true scheduleComplete=true evaluatedHypotheses=2 configuredHypotheses=2 completePlans=2 costValidPlans=2 timingAdmissiblePlans=2 hypothesis=2 eventLead=6.000s presentationTime=16.000s candidate=b route=ring motionJ=0.400000000 scheduleWait=+3.500s selectedGlobalJ=0.540000000 minimumGlobalJ=0.540000000 selectedWithinMinimumTolerance=true tieTolerance=1.000e-09 objective=time_grasp_route noRetiming=true noReplanning=true
[success] [GlobalTimePlanSearchSummary] committed=true scheduleComplete=true evaluatedHypotheses=2 configuredHypotheses=2 feasibleHypotheses=2 geometryFailures=0 selectedLead=6.000s selectedMotionJ=0.400000000 selectedGlobalJ=0.540000000 scheduleWait=+3.500s elapsed=1.000s robotMoved=[0.0000,0.0000] oneCommit=true globalArgmin=true
[success] [Completed] full plan-once handover completed: grasp confirmed and carried-object retreat finished mode=MOVING
"""

# B: the GROUND_NEAR hypothesis-10 shape, 9 complete plans at one
# hypothesis, 8 individually cost-valid, 1 cost-invalid. The invalid record
# is given the numerically lowest cost on purpose.
_GROUND_NEAR_COST_LINES = "\n".join(
    "[success] [GlobalPlanCost] hypothesis=10 eventLead=5.500s "
    f"presentationTime=15.500s candidate=axisP_side_45deg route=route_{i} "
    f"valid=true motionJ={0.55 + 0.02 * i:.9f} scheduleWait=+1.000s "
    f"searchToCompletion=7.000s globalJ={0.60 + 0.02 * i:.9f} "
    "timeTerm=search_to_completion hardFeasibilityUnchanged=true"
    for i in range(8)
)
GROUND_NEAR_PARTIAL_INVALID = f"""
[warning] [PlanSelectionConfiguration] mode=binding_cost costConfigurationValid=true tieTolerance=1.000e-09 allowPhysicalExecution=false weightSumBeforeNormalization=1.000000 eventTimePolicy=global_time_plan timeTerm=search_to_completion
[success] [GlobalTimePlanSearchConfiguration] enabled=true fixedSearchEpoch=10.000s fixedAbsoluteEvents=true configuredHypotheses=1 leadRange=[5.500,5.500] objective=min_time_grasp_route timeTerm=search_to_completion scheduleMustComplete=true finalTimingReadmission=true robotStationary=true
{_GROUND_NEAR_COST_LINES}
[success] [GlobalPlanCost] hypothesis=10 eventLead=5.500s presentationTime=15.500s candidate=axisP_side_337deg route=ring140mm_1of8 valid=false motionJ=1000000000.000000 scheduleWait=+1.000s searchToCompletion=1000000001.000s globalJ=1000000000.000000 timeTerm=search_to_completion hardFeasibilityUnchanged=true
[success] [GlobalTimePlanCapture] success=true hypothesis=10 eventLead=5.500s completePlansAdded=8 cumulativeCompletePlans=8 deferredCommit=true robotMotion=false
[success] [GlobalTimePlanSelection] success=true scheduleComplete=true evaluatedHypotheses=1 configuredHypotheses=1 completePlans=8 costValidPlans=8 timingAdmissiblePlans=8 hypothesis=10 eventLead=5.500s presentationTime=15.500s candidate=axisP_side_45deg route=route_0 motionJ=0.550000000 scheduleWait=+1.000s selectedGlobalJ=0.600000000 minimumGlobalJ=0.600000000 selectedWithinMinimumTolerance=true tieTolerance=1.000e-09 tieBreak=[completion,event,candidateReach,clearance,candidate,route,hypothesis,index]
[success] [GlobalTimePlanCommitProof] committed=true scheduleComplete=true evaluatedHypotheses=1 configuredHypotheses=1 completePlans=8 costValidPlans=8 timingAdmissiblePlans=8 hypothesis=10 eventLead=5.500s presentationTime=15.500s candidate=axisP_side_45deg route=route_0 motionJ=0.550000000 scheduleWait=+1.000s selectedGlobalJ=0.600000000 minimumGlobalJ=0.600000000 selectedWithinMinimumTolerance=true tieTolerance=1.000e-09 objective=time_grasp_route noRetiming=true noReplanning=true
[success] [GlobalTimePlanSearchSummary] committed=true scheduleComplete=true evaluatedHypotheses=1 configuredHypotheses=1 feasibleHypotheses=1 geometryFailures=0 selectedLead=5.500s selectedMotionJ=0.550000000 selectedGlobalJ=0.600000000 scheduleWait=+1.000s elapsed=1.000s robotMoved=[0.0000,0.0000] oneCommit=true globalArgmin=true
[success] [Completed] full plan-once handover completed: grasp confirmed and carried-object retreat finished mode=MOVING
"""

# C: hypothesis 1 has two complete plans that are both cost-invalid
# (contributes zero alternatives, tagged GlobalTimePlanCostAuditReject, not
# BoundedEventGeometryReject); hypothesis 2 succeeds normally and the run
# still completes.
ALL_INVALID_EVENT_AMONG_OTHERS = """
[warning] [PlanSelectionConfiguration] mode=binding_cost costConfigurationValid=true tieTolerance=1.000e-09 allowPhysicalExecution=false weightSumBeforeNormalization=1.000000 eventTimePolicy=global_time_plan timeTerm=search_to_completion
[success] [GlobalTimePlanSearchConfiguration] enabled=true fixedSearchEpoch=10.000s fixedAbsoluteEvents=true configuredHypotheses=2 leadRange=[3.000,6.000] objective=min_time_grasp_route timeTerm=search_to_completion scheduleMustComplete=true finalTimingReadmission=true robotStationary=true
[success] [GlobalPlanCost] hypothesis=1 eventLead=3.000s presentationTime=13.000s candidate=x route=direct valid=false motionJ=1000000000.000000 scheduleWait=+1.000s searchToCompletion=1000000001.000s globalJ=1000000000.000000 timeTerm=search_to_completion hardFeasibilityUnchanged=true
[success] [GlobalPlanCost] hypothesis=1 eventLead=3.000s presentationTime=13.000s candidate=y route=ring valid=false motionJ=1000000000.000000 scheduleWait=+1.000s searchToCompletion=1000000001.000s globalJ=1000000000.000000 timeTerm=search_to_completion hardFeasibilityUnchanged=true
[warning] [GlobalTimePlanCostAuditReject] hypothesis=1 eventLead=3.000s completePlans=2 costInvalidPlans=2 reason=all_candidates_cost_invalid deferredCommit=true; searching another future event
[success] [GlobalTimePlanCapture] success=true hypothesis=1 eventLead=3.000s completePlansAdded=0 cumulativeCompletePlans=0 deferredCommit=true robotMotion=false
[success] [GlobalPlanCost] hypothesis=2 eventLead=6.000s presentationTime=16.000s candidate=b route=ring valid=true motionJ=0.400000000 scheduleWait=+3.500s searchToCompletion=9.500s globalJ=0.540000000 timeTerm=search_to_completion hardFeasibilityUnchanged=true
[success] [GlobalTimePlanCapture] success=true hypothesis=2 eventLead=6.000s completePlansAdded=1 cumulativeCompletePlans=1 deferredCommit=true robotMotion=false
[success] [GlobalTimePlanSelection] success=true scheduleComplete=true evaluatedHypotheses=2 configuredHypotheses=2 completePlans=1 costValidPlans=1 timingAdmissiblePlans=1 hypothesis=2 eventLead=6.000s presentationTime=16.000s candidate=b route=ring motionJ=0.400000000 scheduleWait=+3.500s selectedGlobalJ=0.540000000 minimumGlobalJ=0.540000000 selectedWithinMinimumTolerance=true tieTolerance=1.000e-09 tieBreak=[completion,event,candidateReach,clearance,candidate,route,hypothesis,index]
[success] [GlobalTimePlanCommitProof] committed=true scheduleComplete=true evaluatedHypotheses=2 configuredHypotheses=2 completePlans=1 costValidPlans=1 timingAdmissiblePlans=1 hypothesis=2 eventLead=6.000s presentationTime=16.000s candidate=b route=ring motionJ=0.400000000 scheduleWait=+3.500s selectedGlobalJ=0.540000000 minimumGlobalJ=0.540000000 selectedWithinMinimumTolerance=true tieTolerance=1.000e-09 objective=time_grasp_route noRetiming=true noReplanning=true
[success] [GlobalTimePlanSearchSummary] committed=true scheduleComplete=true evaluatedHypotheses=2 configuredHypotheses=2 feasibleHypotheses=2 geometryFailures=0 selectedLead=6.000s selectedMotionJ=0.400000000 selectedGlobalJ=0.540000000 scheduleWait=+3.500s elapsed=1.000s robotMoved=[0.0000,0.0000] oneCommit=true globalArgmin=true
[success] [Completed] full plan-once handover completed: grasp confirmed and carried-object retreat finished mode=MOVING
"""

# D: every hypothesis is entirely cost-invalid; the global search must fail
# closed with no commit and no completion.
GLOBAL_ALL_INVALID = """
[warning] [PlanSelectionConfiguration] mode=binding_cost costConfigurationValid=true tieTolerance=1.000e-09 allowPhysicalExecution=false weightSumBeforeNormalization=1.000000 eventTimePolicy=global_time_plan timeTerm=search_to_completion
[success] [GlobalTimePlanSearchConfiguration] enabled=true fixedSearchEpoch=10.000s fixedAbsoluteEvents=true configuredHypotheses=2 leadRange=[3.000,6.000] objective=min_time_grasp_route timeTerm=search_to_completion scheduleMustComplete=true finalTimingReadmission=true robotStationary=true
[success] [GlobalPlanCost] hypothesis=1 eventLead=3.000s presentationTime=13.000s candidate=x route=direct valid=false motionJ=1000000000.000000 scheduleWait=+1.000s searchToCompletion=1000000001.000s globalJ=1000000000.000000 timeTerm=search_to_completion hardFeasibilityUnchanged=true
[warning] [GlobalTimePlanCostAuditReject] hypothesis=1 eventLead=3.000s completePlans=1 costInvalidPlans=1 reason=all_candidates_cost_invalid deferredCommit=true; searching another future event
[success] [GlobalTimePlanCapture] success=true hypothesis=1 eventLead=3.000s completePlansAdded=0 cumulativeCompletePlans=0 deferredCommit=true robotMotion=false
[success] [GlobalPlanCost] hypothesis=2 eventLead=6.000s presentationTime=16.000s candidate=y route=ring valid=false motionJ=1000000000.000000 scheduleWait=+1.000s searchToCompletion=1000000001.000s globalJ=1000000000.000000 timeTerm=search_to_completion hardFeasibilityUnchanged=true
[warning] [GlobalTimePlanCostAuditReject] hypothesis=2 eventLead=6.000s completePlans=1 costInvalidPlans=1 reason=all_candidates_cost_invalid deferredCommit=true; searching another future event
[success] [GlobalTimePlanCapture] success=true hypothesis=2 eventLead=6.000s completePlansAdded=0 cumulativeCompletePlans=0 deferredCommit=true robotMotion=false
[error] [GlobalTimePlanSelection] success=false reason=no_complete_time_plan_alternatives scheduleComplete=true evaluatedHypotheses=2 configuredHypotheses=2 completePlans=0 costValidPlans=0 timingAdmissiblePlans=0 no fallback permitted
"""

# E: a deliberately malformed log claiming the cost-invalid candidate was
# nevertheless selected and committed.
INVALID_RECORD_SELECTED = """
[warning] [PlanSelectionConfiguration] mode=binding_cost costConfigurationValid=true tieTolerance=1.000e-09 allowPhysicalExecution=false weightSumBeforeNormalization=1.000000 eventTimePolicy=global_time_plan timeTerm=search_to_completion
[success] [GlobalTimePlanSearchConfiguration] enabled=true fixedSearchEpoch=10.000s fixedAbsoluteEvents=true configuredHypotheses=1 leadRange=[5.000,5.000] objective=min_time_grasp_route timeTerm=search_to_completion scheduleMustComplete=true finalTimingReadmission=true robotStationary=true
[success] [GlobalPlanCost] hypothesis=1 eventLead=5.000s presentationTime=15.000s candidate=good route=direct valid=true motionJ=0.400000000 scheduleWait=+1.000s searchToCompletion=7.000s globalJ=0.450000000 timeTerm=search_to_completion hardFeasibilityUnchanged=true
[success] [GlobalPlanCost] hypothesis=1 eventLead=5.000s presentationTime=15.000s candidate=bad route=ring valid=false motionJ=1000000000.000000 scheduleWait=+1.000s searchToCompletion=1000000001.000s globalJ=1000000000.000000 timeTerm=search_to_completion hardFeasibilityUnchanged=true
[success] [GlobalTimePlanCapture] success=true hypothesis=1 eventLead=5.000s completePlansAdded=1 cumulativeCompletePlans=1 deferredCommit=true robotMotion=false
[success] [GlobalTimePlanSelection] success=true scheduleComplete=true evaluatedHypotheses=1 configuredHypotheses=1 completePlans=1 costValidPlans=1 timingAdmissiblePlans=1 hypothesis=1 eventLead=5.000s presentationTime=15.000s candidate=bad route=ring motionJ=1000000000.000000 scheduleWait=+1.000s selectedGlobalJ=1000000000.000000 minimumGlobalJ=1000000000.000000 selectedWithinMinimumTolerance=true tieTolerance=1.000e-09 tieBreak=[completion,event,candidateReach,clearance,candidate,route,hypothesis,index]
[success] [GlobalTimePlanCommitProof] committed=true scheduleComplete=true evaluatedHypotheses=1 configuredHypotheses=1 completePlans=1 costValidPlans=1 timingAdmissiblePlans=1 hypothesis=1 eventLead=5.000s presentationTime=15.000s candidate=bad route=ring motionJ=1000000000.000000 scheduleWait=+1.000s selectedGlobalJ=1000000000.000000 minimumGlobalJ=1000000000.000000 selectedWithinMinimumTolerance=true tieTolerance=1.000e-09 objective=time_grasp_route noRetiming=true noReplanning=true
[success] [GlobalTimePlanSearchSummary] committed=true scheduleComplete=true evaluatedHypotheses=1 configuredHypotheses=1 feasibleHypotheses=1 geometryFailures=0 selectedLead=5.000s selectedMotionJ=1000000000.000000 selectedGlobalJ=1000000000.000000 scheduleWait=+1.000s elapsed=1.000s robotMoved=[0.0000,0.0000] oneCommit=true globalArgmin=true
[success] [Completed] full plan-once handover completed: grasp confirmed and carried-object retreat finished mode=MOVING
"""

# F: a cost-valid alternative with a lower globalJ exists (hypothesis 1,
# negative scheduleWait) but is not the winner, because it is not
# timing-admissible. Per-candidate timing admissibility is not logged, so
# the checker must not treat "a cheaper valid record exists" as a failure
# by itself -- this reproduces the real GROUND_NEAR hypothesis-1 shape.
LOWER_COST_INADMISSIBLE_ALTERNATIVE = """
[warning] [PlanSelectionConfiguration] mode=binding_cost costConfigurationValid=true tieTolerance=1.000e-09 allowPhysicalExecution=false weightSumBeforeNormalization=1.000000 eventTimePolicy=global_time_plan timeTerm=search_to_completion
[success] [GlobalTimePlanSearchConfiguration] enabled=true fixedSearchEpoch=10.000s fixedAbsoluteEvents=true configuredHypotheses=2 leadRange=[1.800,6.000] objective=min_time_grasp_route timeTerm=search_to_completion scheduleMustComplete=true finalTimingReadmission=true robotStationary=true
[success] [GlobalPlanCost] hypothesis=1 eventLead=1.800s presentationTime=11.800s candidate=early route=ring valid=true motionJ=0.500000000 scheduleWait=-1.100s searchToCompletion=6.900s globalJ=0.400000000 timeTerm=search_to_completion hardFeasibilityUnchanged=true
[success] [GlobalTimePlanCapture] success=true hypothesis=1 eventLead=1.800s completePlansAdded=1 cumulativeCompletePlans=1 deferredCommit=true robotMotion=false
[success] [GlobalPlanCost] hypothesis=2 eventLead=6.000s presentationTime=16.000s candidate=b route=ring valid=true motionJ=0.400000000 scheduleWait=+3.500s searchToCompletion=9.500s globalJ=0.540000000 timeTerm=search_to_completion hardFeasibilityUnchanged=true
[success] [GlobalTimePlanCapture] success=true hypothesis=2 eventLead=6.000s completePlansAdded=1 cumulativeCompletePlans=2 deferredCommit=true robotMotion=false
[success] [GlobalTimePlanSelection] success=true scheduleComplete=true evaluatedHypotheses=2 configuredHypotheses=2 completePlans=2 costValidPlans=2 timingAdmissiblePlans=1 hypothesis=2 eventLead=6.000s presentationTime=16.000s candidate=b route=ring motionJ=0.400000000 scheduleWait=+3.500s selectedGlobalJ=0.540000000 minimumGlobalJ=0.540000000 selectedWithinMinimumTolerance=true tieTolerance=1.000e-09 tieBreak=[completion,event,candidateReach,clearance,candidate,route,hypothesis,index]
[success] [GlobalTimePlanCommitProof] committed=true scheduleComplete=true evaluatedHypotheses=2 configuredHypotheses=2 completePlans=2 costValidPlans=2 timingAdmissiblePlans=1 hypothesis=2 eventLead=6.000s presentationTime=16.000s candidate=b route=ring motionJ=0.400000000 scheduleWait=+3.500s selectedGlobalJ=0.540000000 minimumGlobalJ=0.540000000 selectedWithinMinimumTolerance=true tieTolerance=1.000e-09 objective=time_grasp_route noRetiming=true noReplanning=true
[success] [GlobalTimePlanSearchSummary] committed=true scheduleComplete=true evaluatedHypotheses=2 configuredHypotheses=2 feasibleHypotheses=2 geometryFailures=0 selectedLead=6.000s selectedMotionJ=0.400000000 selectedGlobalJ=0.540000000 scheduleWait=+3.500s elapsed=1.000s robotMoved=[0.0000,0.0000] oneCommit=true globalArgmin=true
[success] [Completed] full plan-once handover completed: grasp confirmed and carried-object retreat finished mode=MOVING
"""

# G: same shape as F, but now carrying the GlobalPlanTimingAdmissibility
# ground truth: hypothesis 1's cheaper candidate is genuinely
# timing-inadmissible (remaining=0.8s < minimumSafeCommitLead=1.6s),
# hypothesis 2's winner is genuinely admissible. This is what lets the
# checker verify the committed candidate is the *exact* argmin over
# F_J intersect F_timing, not merely "not obviously wrong".
TIMING_ADMISSIBILITY_VALID = LOWER_COST_INADMISSIBLE_ALTERNATIVE.replace(
    "[success] [GlobalTimePlanCapture] success=true hypothesis=1 eventLead=1.800s completePlansAdded=1 cumulativeCompletePlans=1 deferredCommit=true robotMotion=false\n",
    "[success] [GlobalTimePlanCapture] success=true hypothesis=1 eventLead=1.800s completePlansAdded=1 cumulativeCompletePlans=1 deferredCommit=true robotMotion=false\n"
    "[info] [GlobalPlanTimingAdmissibility] hypothesis=1 candidate=early route=ring costValid=true globalJ=0.400000000 eventPresentationTime=11.800000s now=11.000000s remaining=0.800000s minimumSafeCommitLead=1.600000s predictedPresentationDuration=2.900000s minimumReachEntryLead=0.050000s eventWindowAdmissible=false timingAdmissible=false\n",
).replace(
    "[success] [GlobalTimePlanCapture] success=true hypothesis=2 eventLead=6.000s completePlansAdded=1 cumulativeCompletePlans=2 deferredCommit=true robotMotion=false\n",
    "[success] [GlobalTimePlanCapture] success=true hypothesis=2 eventLead=6.000s completePlansAdded=1 cumulativeCompletePlans=2 deferredCommit=true robotMotion=false\n"
    "[info] [GlobalPlanTimingAdmissibility] hypothesis=2 candidate=b route=ring costValid=true globalJ=0.540000000 eventPresentationTime=16.000000s now=11.000000s remaining=5.000000s minimumSafeCommitLead=1.600000s predictedPresentationDuration=2.500000s minimumReachEntryLead=0.050000s eventWindowAdmissible=true timingAdmissible=true\n",
)

# H: frozen seven-term objective (T,E,L,C,Q,K,V; R excluded). T=E=L=V=1,
# C=Q=K=0 => J = 0.4210526+0.1052632+0.1052632+0.0526316 = 0.684211. R is
# deliberately set to a large, arbitrary nonzero value (0.5000) to prove its
# value has no bearing on whether the reconstruction succeeds.
FROZEN_OBJECTIVE_VALID = (
    "[success] [CompletePlanCost] candidate=testGrasp route=testRoute valid=true "
    "J=0.684211 weightsNormalized=true auditTime=8.000s effort=8.000 path=0.500m "
    "rotation=1.571 clearance=[reach:0.0800,retreat:0.0800] jointMargin=0.2000 "
    "conditionIndex=0.1000 velocityUtil=1.0000 velocityLaw=rho4 finiteBoundary=true "
    "terms=[T:1.0000,E:1.0000,L:1.0000,R:0.5000,C:0.0000,Q:0.0000,K:0.0000,V:1.0000] "
    "hardFeasibilityUnchanged=true selectionMode=binding_cost binding=true"
)

# I: same as H but J is what a regressed implementation would report if R's
# old weight (~0.05) were accidentally still summed into the binding cost
# (0.684211 + 0.05*0.5 = 0.709211). Must be rejected.
FROZEN_OBJECTIVE_R_LEAK = FROZEN_OBJECTIVE_VALID.replace(
    "J=0.684211", "J=0.709211", 1
)

# J: same as H but velocityUtil no longer matches the logged V term's
# implied quartic law (clamp01(0.5)**4 = 0.0625, not 1.0000). J itself is
# left internally consistent with the (wrong) logged V=1.0000, so this
# isolates a pure V-law violation from a J-reconstruction violation.
FROZEN_OBJECTIVE_V_LAW_BROKEN = FROZEN_OBJECTIVE_VALID.replace(
    "velocityUtil=1.0000", "velocityUtil=0.5000", 1
)


def require_failure(text: str, description: str) -> None:
    if not verify_text(text):
        raise AssertionError(f"checker accepted invalid proof: {description}")


def require_failsafe_failure(text: str, description: str) -> None:
    if not verify_failsafe_text(text):
        raise AssertionError(
            f"checker accepted invalid fail-safe proof: {description}"
        )


def main() -> int:
    assert verify_text(VALID) == []

    require_failure(
        VALID.replace("evaluatedHypotheses=2 configuredHypotheses=2",
                      "evaluatedHypotheses=1 configuredHypotheses=2", 1),
        "incomplete schedule",
    )
    require_failure(
        VALID.replace("selectedGlobalJ=0.540000000 minimumGlobalJ=0.540000000",
                      "selectedGlobalJ=0.560000000 minimumGlobalJ=0.540000000", 1),
        "non-minimum selection",
    )
    require_failure(
        VALID.replace("candidate=b route=ring motionJ=0.400000000",
                      "candidate=c route=ring motionJ=0.400000000", 1),
        "selection/commit identity mismatch",
    )
    require_failure(
        VALID.replace(
            "[success] [Completed] full plan-once handover completed: grasp confirmed and carried-object retreat finished mode=MOVING",
            "[error] Failure",
        ),
        "handover not completed",
    )

    # A: valid=false is not by itself a failure; a well-accounted partial
    # exclusion is a legitimate successful proof.
    assert verify_text(PARTIAL_INVALID_HYPOTHESIS) == []

    # B: GROUND_NEAR hypothesis-10 shape (9 complete / 8 valid / 1 invalid)
    # is accepted, and the argmin is computed over the 8 valid alternatives.
    assert verify_text(GROUND_NEAR_PARTIAL_INVALID) == []
    require_failure(
        GROUND_NEAR_PARTIAL_INVALID.replace(
            "completePlansAdded=8 cumulativeCompletePlans=8",
            "completePlansAdded=7 cumulativeCompletePlans=7", 1),
        "pooled count does not match logged valid records",
    )

    # C: an event with all cost-invalid records contributes zero
    # alternatives but does not fail the otherwise-successful run.
    assert verify_text(ALL_INVALID_EVENT_AMONG_OTHERS) == []
    require_failure(
        ALL_INVALID_EVENT_AMONG_OTHERS.replace(
            "[warning] [GlobalTimePlanCostAuditReject] hypothesis=1 eventLead=3.000s completePlans=2 costInvalidPlans=2 reason=all_candidates_cost_invalid deferredCommit=true; searching another future event\n",
            "[warning] [BoundedEventGeometryReject] hypothesis=1 lead=3.000s source=global_fixed_schedule completePlan=false geometryFailures=1 robotStationary=true; searching another future event\n",
        ),
        "cost-audit rejection relabeled as geometry rejection",
    )

    # D: global all-invalid must fail safely, with no commit and no
    # completed handover.
    assert verify_failsafe_text(GLOBAL_ALL_INVALID) == []
    require_failsafe_failure(
        GLOBAL_ALL_INVALID + "\n[success] [GlobalTimePlanCommitProof] committed=true scheduleComplete=true evaluatedHypotheses=2 configuredHypotheses=2 completePlans=0 costValidPlans=0 timingAdmissiblePlans=0 hypothesis=1 eventLead=3.000s presentationTime=13.000s candidate=x route=direct motionJ=0.0 scheduleWait=+0.000s selectedGlobalJ=0.0 minimumGlobalJ=0.0 selectedWithinMinimumTolerance=true tieTolerance=1.000e-09 objective=time_grasp_route noRetiming=true noReplanning=true",
        "fail-safe log must not contain a commit",
    )

    # E: an invalid record must never be accepted as the selected/committed
    # plan.
    require_failure(INVALID_RECORD_SELECTED, "invalid record selected")

    # F: a cheaper cost-valid alternative that is simply not timing-
    # admissible must not fail the proof (real GROUND_NEAR hypothesis-1
    # shape); but a reported minimum below every logged valid record, or
    # not matching the winner's own record, is impossible and must fail.
    assert verify_text(LOWER_COST_INADMISSIBLE_ALTERNATIVE) == []
    require_failure(
        LOWER_COST_INADMISSIBLE_ALTERNATIVE.replace(
            "selectedGlobalJ=0.540000000 minimumGlobalJ=0.540000000",
            "selectedGlobalJ=0.100000000 minimumGlobalJ=0.100000000", 1),
        "reported minimum below every valid alternative",
    )

    # G: with GlobalPlanTimingAdmissibility ground truth present, the
    # checker must verify the committed candidate is the *exact* argmin
    # over F_J intersect F_timing, not just "plausibly not wrong".
    assert verify_text(TIMING_ADMISSIBILITY_VALID) == []
    require_failure(
        TIMING_ADMISSIBILITY_VALID.replace(
            "eventWindowAdmissible=false timingAdmissible=false",
            "eventWindowAdmissible=true timingAdmissible=true", 1),
        "a timing-admissible cheaper alternative exists but was not selected",
    )
    require_failure(
        TIMING_ADMISSIBILITY_VALID.replace(
            "costValid=true globalJ=0.400000000",
            "costValid=false globalJ=0.400000000", 1),
        "cost-invalid record present in the pooled timing-admissibility set",
    )
    # Regression: eventWindowAdmissible and timingAdmissible are adjacent
    # boolean fields in the log line; a record can satisfy the former and
    # still fail the latter (event window open, but not enough reach-entry
    # lead). The checker must key off timingAdmissible specifically, not
    # eventWindowAdmissible -- diverging only the first field here must NOT
    # change the verdict, since hypothesis 1 remains genuinely inadmissible.
    assert verify_text(TIMING_ADMISSIBILITY_VALID.replace(
        "eventWindowAdmissible=false timingAdmissible=false",
        "eventWindowAdmissible=true timingAdmissible=false", 1)) == []

    # H: frozen seven-term objective reconstruction, standalone. A correct
    # record (R present but excluded from J, V following the quartic law
    # from velocityUtil) must pass with zero errors.
    assert verify_frozen_objective(FROZEN_OBJECTIVE_VALID) == []

    # I: proves item A/D/F together -- if R's weight were accidentally
    # reinstated (or any binding weight drifted from the frozen vector), the
    # independent reconstruction from raw logged terms must catch it.
    assert verify_frozen_objective(FROZEN_OBJECTIVE_R_LEAK) != []

    # J: proves item B/C -- V must follow clamp01(velocityUtil)**4 exactly;
    # a mismatch (e.g. terminalVelocityUtilization leaking back into V, or
    # any other drift) must be caught independently of the J-reconstruction
    # check above.
    assert verify_frozen_objective(FROZEN_OBJECTIVE_V_LAW_BROKEN) != []

    # A log with no CompletePlanCost detail at all (every other fixture in
    # this suite) must not be forced through this check -- verify_text/
    # verify_failsafe_text only invoke it when the tag is present.
    assert verify_text(VALID) == []
    assert "[CompletePlanCost]" not in VALID

    # End-to-end: embedding a correct CompletePlanCost record inside an
    # otherwise-valid full log must not introduce new failures, and
    # embedding the R-leak variant must be caught through verify_text too.
    valid_with_frozen_objective = VALID.rstrip("\n") + "\n" + FROZEN_OBJECTIVE_VALID + "\n"
    assert verify_text(valid_with_frozen_objective) == []
    require_failure(
        VALID.rstrip("\n") + "\n" + FROZEN_OBJECTIVE_R_LEAK + "\n",
        "frozen objective reconstruction fails to detect a reinstated R contribution",
    )

    print("global time-plan log checker tests: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
