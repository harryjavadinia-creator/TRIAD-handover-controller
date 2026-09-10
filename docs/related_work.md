# Related work and contribution

TRIAD combines an explicit finite event-time–grasp–route decision, modeled
receiver-action checks through retreat, and live timing/prediction-consistency
admission before one-time commitment. Its contribution is this auditable
integration. The comparison below positions that formulation; it does not
establish priority over all handover methods or empirical superiority.

## Closest methodological comparisons

| Work | Relevant formulation | Evidence reported by the paper |
| --- | --- | --- |
| [Yang et al., 2022 — Model Predictive Control for Fluid Human-to-Robot Handovers](https://arxiv.org/abs/2204.00134) | Stochastic MPC, learned grasp reachability/manipulability, and a goal-set formulation that integrates grasp choice into motion optimization | Physical handovers and a user evaluation |
| [Oelerich, Hartl-Nesic and Kugi, 2024 — Model Predictive Trajectory Planning for Human-Robot Handovers](https://arxiv.org/abs/2404.07505) | Path-following MPC makes progress explicit; Gaussian-process prediction of the handover location adapts path-deviation bounds | Experiments with a collaborative seven-DoF arm |
| [Djeha et al., 2022 — Human-Robot Handovers using Task-Space Quadratic Programming](https://arxiv.org/abs/2206.09185) | Handover constraints in a task-space QP produce implicit timing and trajectory encounters | Panda receiving objects from a human |
| [Akinola et al., 2021 — Dynamic Grasping with Reachability and Motion Awareness](https://arxiv.org/abs/2103.10562) | Motion-aware grasp ranking, recurrent motion prediction, and motion generation seeded from the preceding solution; computation delay motivates prediction | Dynamic grasping experiments and real-robot validation |
| **TRIAD** | Finite complete-plan enumeration, model-relative hard checks, seven-term ranking, current timing/prediction-consistency admission, and governed execution after one commitment | Canonical simulation outcomes and corrected perception-latency ablation |

Yang's goal-set formulation jointly reasons about grasp choice and motion; it is
not accurately described as simply choosing a grasp and then planning motion.
Its approach uses MPC, while its grasp phase uses a blocking policy. See the
[primary paper, Sections III and V](https://arxiv.org/pdf/2204.00134).

## Scope of the distinction

TRIAD makes a complete finite alternative and its admission decision explicit
and independently inspectable. That differs in formulation from receding-horizon
motion optimization or implicit task-space encounters. It does not imply that
those methods ignore timing, lack joint grasp/motion reasoning, or cannot
achieve safe handovers.

TRIAD's evaluation does not include matched runs against these systems. No claim
of better completion, speed, physical safety, or human preference follows from
this table. The [method](mathematics.md) and [results](results.md) define the
claims supported by the documented evidence.
