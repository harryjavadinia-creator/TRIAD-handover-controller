#!/usr/bin/env python3

from check_binding_cost_log import verify_text


VALID = """
[warning] [PlanSelectionConfiguration] mode=binding_cost costConfigurationValid=true tieTolerance=1.000e-09 allowPhysicalExecution=false weightSumBeforeNormalization=1.000000 eventTimePolicy=first_admissible_center_out
[success] [BindingCostSelection] success=true commitAdmissible=true completePlans=15 costValidPlans=15 timingAdmissiblePlans=12 candidate=axisP_side_337deg route=ring80mm_1of8 selectedJ=0.564517000 minimumAdmissibleJ=0.564517000 selectedWithinMinimumTolerance=true tieTolerance=1.000e-09 tieBreak=[reachTime,clearance,candidate,route,index]
[success] [BindingCostCommitProof] committed=true candidate=axisP_side_337deg route=ring80mm_1of8 selectedJ=0.564517000 minimumAdmissibleJ=0.564517000 selectedWithinMinimumTolerance=true tieTolerance=1.000e-09 completePlans=15 costValidPlans=15 timingAdmissiblePlans=12 eventTimePolicy=first_admissible_center_out
"""


def main() -> None:
    assert verify_text(VALID) == []
    assert verify_text(VALID.replace("costValidPlans=15", "costValidPlans=14"))
    assert verify_text(VALID.replace("selectedJ=0.564517000", "selectedJ=0.664517000"))
    assert verify_text(VALID.replace("committed=true", "committed=false"))
    print("binding-cost log checker tests: PASS")


if __name__ == "__main__":
    main()
