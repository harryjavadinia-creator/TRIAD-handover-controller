# FINAL TRIAD — Phase 1: Literature Design Matrix

Scope: the design questions the final TRIAD generation laws must answer, i.e. how
the interception-time set T_k, the grasp set G_k(τ) and the route set R_k(τ,g) are
generated from the current information I_k; how feasibility, ranking, persistence,
replanning and commitment are handled. The papers were read to find principles
and gaps. They were **not** read to justify TRIAD's inherited constants
(14 leads, 32 grasps, 17 routes, the seven weights, L_commit = 1.6 s, …).
Section 4 lists the constants that have no support in this literature.

Evidence tags:

- **LIT-FULL**: extracted from the full text, which I read. Extraction files are
  kept outside the repo; the quotes are reproduced here.
- **LIT-ABS**: abstract or landing page only.
- **CODE / MEASURED**: TRIAD facts from this repository and its earlier reports.

A field marked "not stated" means I did not find it in the text. It does not mean
the paper lacks the feature.

---

## 1. Per-paper extraction

### 1.1 Menon, Cohen, Likhachev — *Motion Planning for Smooth Pickup of Moving Objects*, ICRA 2014, pp. 453–460 (LIT-FULL)

| field | extraction |
|---|---|
| problem | PR2 picks a can or martini glass off a 0.1 m/s conveyor. The object trajectory is **known a priori**, and perfect external sensing is assumed. |
| candidate representation | Search state (joint positions, joint velocities, time). Goal edges are adaptive grasp primitives at candidate pregrasp poses and times. |
| candidate generation | Fixed acceleration primitives (one joint at ±1.0 at a time, fixed Δt) integrated through the manipulator dynamics. The grasp primitive is attempted only when the end effector is within 0.10 m. "there are many poses around the object varying orientation around the object, and we test a handful of such poses", radially arranged about the cylinder axis and hand-picked. |
| fixed vs state-dependent count | Grasp poses are a fixed hand-picked set. Search expansion is state-dependent. |
| timing / interception rule | Least-time objective, so the pickup is at "the earliest feasible point in its trajectory". The heuristic samples object poses forward from t_current in steps of Δt_sample, up to ΔT_max (the time until the object leaves the workspace). It returns the first sample whose straight-line trapezoidal/triangular time of flight (v_max^e, a_max^e) plus a nominal grasp time is below t_sample − t_current. |
| grasp rule | Pregrasp pose facing the object's radial axis; Jacobian pseudo-inverse approach that keeps pace, then tracks while closing, then lifts. |
| trajectory rule | Graph search (ARA*) over dynamic primitives, followed by the adaptive grasp primitive. The result is spline-fitted and validated. |
| hard constraints | Joint torques, velocities and limits; collisions with static obstacles and with the moving goal. |
| soft objective | Execution time (reach + grasp + lift). |
| replanning / update | None. Dynamic replanning is listed as future work. |
| prediction assumptions | Exact known object trajectory. |
| switching / commitment | Single plan; execution is triggered on object release onto the belt. |
| real-time rate | 30 s planning allowance. 110/112 plans found (98.2%). "may not compute a trajectory in real time if the object is too close to the robot". |
| validation | 112 simulated start poses; PR2 video demonstrations without success statistics. |

### 1.2 Kim, Shukla, Billard — *Catching Objects in Flight*, IEEE T-RO 30(5):1049–1065, 2014 (LIT-FULL)

| field | extraction |
|---|---|
| problem | Catching thrown, unevenly shaped objects (bottle, racket, box, hammer) with an LWR 4+ and an Allegro hand. |
| candidate representation | Hand pose per time slice of the predicted object trajectory. |
| candidate generation | Graspable-space GMM (object frame, learned from human demonstrations) × reachable-space GMM (robot frame). A joint GMM is evaluated per time slice. Gradient ascent starts at the Gaussian centre closest to the current hand pose. |
| fixed vs state-dependent count | One optimum per time slice. Slices extend "until the predicted end of flight", so the count depends on state. |
| timing / interception rule | "The optimal catching configuration and catching time is the configuration and time slice with the highest likelihood". This is not earliest-time. The predicted time becomes "the desired reaching time in the timing controller". |
| grasp rule | Likelihood above ρ_grasp. Palm direction must oppose the object velocity: dot < d = 0.5 (60°). |
| trajectory rule | Coupled dynamical systems (reach + fingers) plus a timing controller; damped least-squares IK at 500 Hz. |
| hard constraints | Probabilistic reachability/graspability thresholds; conservative joint limits. **No execution-time feasibility check.** |
| soft objective | Joint likelihood. |
| replanning / update | Trajectory, configuration and time are re-predicted at every measurement. |
| prediction assumptions | Learned object dynamics + EKF; the whole trajectory is integrated forward. |
| switching / commitment | "we stop predicting the best catching posture when the predicted time at contact is inferior to 0.09 s", because of camera occlusion near the robot. |
| real-time rate | Best catching configuration ≈ 0.2 ms; control at 500 Hz. |
| validation | 52/71 = 73.2% real catches (9 of 80 throws excluded for never entering the reachable space). **12 of 19 failures: "the robot cannot reach the target at the desired time".** |

### 1.3 Mirrazavi Salehian, Figueroa, Billard — *Coordinated multi-arm motion planning: Reaching for moving objects in the face of uncertainty*, RSS 2016 (LIT-FULL)

| field | extraction |
|---|---|
| problem | Dual-arm interception of a large box carried by a blindfolded human, or of a thrown 150 cm rod (≈0.56 s flight). |
| candidate representation | User-defined reaching points on the object (one per arm); no grasp enumeration. |
| candidate generation | Fixed per object. |
| fixed vs state-dependent count | Fixed. |
| timing / interception rule | A forward model "determines a point along this trajectory where the object will become reachable by all robot arms" (the feasible intercept point at T*). Reachability is a GMM workspace model; the threshold δ_j is set so that 99% of training postures exceed it. |
| grasp rule | Fixed reaching points; orientation from marker geometry. |
| trajectory rule | LPV dynamical system of a virtual object, with proven asymptotic convergence. The coordination parameter γ blends from the predicted intercept point (γ=0) to tracking the real object (γ=1) as the object approaches or enters the workspace. |
| hard constraints | Workspace likelihood threshold. |
| soft objective | None beyond DS convergence. |
| replanning / update | Intercept point "continuously recomputed". For the rod: "The new feasible intercept point is chosen in the vicinity of the previous one to minimize the convergence time". |
| prediction assumptions | Ballistic prediction; "there is no guarantee that the object will pass through the predicted point at T*". |
| switching / commitment | Finger closure is triggered when end effectors are < 2 cm from the reaching points. |
| real-time rate | Control 500 Hz; motion capture 240 Hz. |
| validation | Success (< 2 cm): box 85%, rod 37.5% (tracking loss, IK). For the carried box, "the prediction of the intercept point does not play a vital role in grabbing the box". |

Related (LIT-ABS): Salehian, Khoramshahi, Billard, T-RO 32(2) 2016, soft catching. The robot moves with the object for a short period to leave time for finger closure.

### 1.4 Marturi et al. — *Dynamic grasp and trajectory planning for moving objects*, Auton. Robots 43:1241–1256, 2019 (LIT-FULL)

| field | extraction |
|---|---|
| problem | A robot follows, then grasps, an object moved arbitrarily by a human (handover); vision tracked. |
| candidate representation | Grasp trajectories (approach waypoint … grip waypoint; wrist pose + hand joints). |
| candidate generation | Learned generative grasp planner, run **once** before motion. The first N most likely are kept (N = 10/20/30 studied) and rigidly transformed by the tracked pose update each cycle. |
| fixed vs state-dependent count | Fixed N. |
| timing / interception rule | None: the current tracked pose, no future prediction. |
| grasp rule | Anytime differential-evolution IK for every hypothesis gives a task-space error ε. The trajectory switches to argmin only if ε_k + δ_w < ε_r*. δ_w = 2.0 was "selected by trial and error". |
| trajectory rule | Local optimisation planner if ‖c_c − c_1r‖ ≤ δ_c, otherwise a non-real-time PRM global planner. δ_c ∈ {0.2, 0.4, 0.6, 0.8} studied, default 0.4. |
| hard constraints | Collision-free IK (joint limits, collision bounds). |
| soft objective | Task-space error, which can be non-zero: "handling of situations where no error-free trajectories exist". |
| replanning / update | Every control cycle; tracker at 7.62 clouds/s. |
| prediction assumptions | None. |
| switching / commitment | **A human operator decides** when to stop tracking and execute the final grasp. |
| real-time rate | 7.62 clouds/s perception. |
| validation | KUKA real robot, 10 trials per object. More hypotheses gave more switches and lower task error; a smaller δ_c gave more switching. Failures came from tracking. |

### 1.5 Akinola, Xu, Song, Allen — *Dynamic Grasping with Reachability and Motion Awareness*, IROS 2021 (LIT-FULL)

| field | extraction |
|---|---|
| problem | Pick a moving object with unknown motion among static obstacles, as fast as possible. |
| candidate representation | Grasp + pregrasp (backed off b = 5 cm Mico / 7.5 cm Robotiq), object frame. |
| candidate generation | Offline: 5000 grasps, simulated 50 times each with noise, top 100 kept. Online: transform to the predicted pose, then keep top-5 by reachability (precomputed 6D reachability space) ∪ top-5 by a motion-aware success network M(g, pg, v, θ), giving 10. Pick the grasp whose IK is closest to the current configuration. |
| fixed vs state-dependent count | Fixed (10). The combination rule was chosen empirically: "top 5 + top 5 outperforms weighted sum". |
| timing / interception rule | Prediction horizon as a step function of end-effector distance d: 2 s if d > 0.3 m, 1 s if 0.1 < d ≤ 0.3 m, 0 s if d ≤ 0.1 m. The authors write: "t, t′, and t″ are configured experimentally but the optimal prediction horizon should be a function of the end-effector speed, the distance between the end-effector and the planned grasp pose, and the motion of the target. We leave this for future research." |
| grasp rule | Reachability and motion-aware ranking, then IK. |
| trajectory rule | Planner seeded with the previous solution (PRM best among CHOMP/RRT/PRM). Each new plan interrupts and is retimed to blend with current velocities. |
| hard constraints | IK reachable; collision-free with static obstacles. |
| soft objective | Ranking scores; closeness to the current configuration. |
| replanning / update | Every loop iteration. |
| prediction assumptions | LSTM trained on linear, circular and sinusoidal planar motions. |
| switching / commitment | CANGRASP: d_p ≤ 1.1 b and d_q ≤ 20°. Then re-detect, predict t′ = 1 s, move, close while moving for t″ = 0.1 s. Single commit. |
| real-time rate | Not stated as one number. |
| validation | Bullet sim, 7 objects × 100 trials per condition; R+M success 0.75–0.95. Removing prediction drops UR5 linear success from 0.874 to 0.284. Real UR5 conveyor at 4.46 cm/s: 4/5, 5/5, 3/5. |

### 1.6 Yang, Paxton, Mousavian, Chao, Cakmak, Fox — *Reactive Human-to-Robot Handovers of Arbitrary Objects*, ICRA 2021 (LIT-FULL)

| field | extraction |
|---|---|
| problem | Reactive H2R handover of unknown objects; Franka Panda; RGB-D. |
| candidate representation | 6-DoF grasps (SE(3)) plus a 180° flip about the grasp Z axis, which doubles the pool for a 2-finger gripper. Standoff pose 10 cm back; final grasp pushed 5 cm forward. |
| candidate generation | Temporally consistent GraspNet: the previous set is perturbed (ΔT ~ U[−2, 2] cm) and accepted by Metropolis–Hastings on evaluator score. Grasps colliding with the hand are removed. **Resampled when the number drops below "a certain threshold"**. |
| fixed vs state-dependent count | **Variable.** Regeneration is event-triggered by depletion. |
| timing / interception rule | None (no prediction). Users were told the robot moves "once the object is not moving". |
| grasp rule | Score C = w_s·min(s − s_min, 0) + w_prev·d(x, x_prev) + w_home·d(x, x_home), with w_s = 1, w_q = 0.1, w_home = 5, w_prev = 5 (no derivation given). |
| trajectory rule | "we always attempt a straight-line path first", with RRT-Connect "when necessary". RMP execution, with fast collision checks before following. |
| hard constraints | Candidates are looped in score order: Trac-IK, a collision-free joint path to the standoff, and a collision-free Cartesian path from standoff to grasp. **The first grasp passing all checks is selected.** |
| soft objective | C above. It orders the feasibility search; it is not an argmin over certified candidates. |
| replanning / update | Standoff updated at ≈10 Hz. |
| prediction assumptions | None. |
| switching / commitment | On arrival at the standoff, an open-loop grasp; retry on failure. |
| real-time rate | ≈10 Hz planning; 15 fps body tracking. |
| validation | 26 objects; 6 lab users; Household-A mean success 81.8%, approach time 10.7 ± 3.6 s. Rotation after the robot started moving: success within 3 attempts. The paper reports that in Rosenberger et al. (RA-L 2021), most failures came from the object moving after the robot started. |

### 1.7 Yang, Sundaralingam, Paxton, Akinola, Chao, Cakmak, Fox — *Model Predictive Control for Fluid Human-to-Robot Handovers*, ICRA 2022 (LIT-FULL)

| field | extraction |
|---|---|
| problem | Smooth, reactive H2R handover motion with grasp selection. |
| candidate representation | GraspNet grasps (5 Hz); standoff 15 cm from the final grasp. |
| candidate generation | As in 1.6, plus a learned reachability model (|JᵀWJ|, 1.3 ms). |
| fixed vs state-dependent count | Variable. |
| timing / interception rule | None; MPC horizon 2 s. |
| grasp rule | Baseline score plus reachability; or MPC-GoalSet, whose cost is the minimum pose distance over all grasps. |
| trajectory rule | Sampling-based MPC (STORM), 50–100 Hz. |
| hard constraints | **Handled as costs** (weight 5000). |
| soft objective | Goal, straight-line (30), manipulability and stop costs. α₁ = 70 and α₂ = 220 were tuned by "starting with a small value and slowly increasing … a total of 30 minutes". |
| replanning / update | Receding horizon. |
| prediction assumptions | None for the object. |
| switching / commitment | The grasp stage is **blocking**: "as the robot gets closer … body tracking, segmentation, and grasp estimation all become less reliable". A force/torque contact classifier ends the approach early. A failed grasp returns to Approach. |
| real-time rate | 50–100 Hz. |
| validation | MPC vs 1.6: approach 11.5 → 7.1 s, success 85.7 → 88.3%. Reachability gives +7.5% success for MPC. GoalSet had the lowest approach time but reduced success under orientation change. Trials were repeated until 3 successes per condition; N = 4 users. |

### 1.8 Islam, Salzman, Agarwal, Likhachev — *Provably Constant-time Planning and Replanning for Real-time Grasping Objects off a Conveyor Belt*, RSS 2020 (LIT-FULL)

| field | extraction |
|---|---|
| problem | PR2 conveyor pickup (0.2 m/s, known speed); pose estimates improve as the object approaches. |
| candidate representation | A discrete goal set G_full of poses (x, y, θ) around x_exec: x range ±2ε_P with ε_P = 2.5 cm, y 20 cm, resolution 1 cm and 10°. |
| candidate generation | Offline preprocessing (≈3.5 h, < 20 MB) of reusable time-parameterised paths. |
| fixed vs state-dependent count | Fixed goal set. |
| timing / interception rule | "The plan is computed for the sensed object pose projected forward by T_bound time, giving the planner T_bound time to plan. If the plan comes in earlier, the robot waits". New estimates are back-projected in time at the known belt speed. |
| grasp rule | Dynamic grasp primitive. |
| trajectory rule | Query of precomputed paths (latching). |
| hard constraints | Collision-free, including with the target. |
| soft objective | Path cost (time). |
| replanning / update | On every new estimate, with a provable constant-time bound. |
| prediction assumptions | A1: a replan cutoff t_rc exists, after which the robot executes the last plan (t_rc = 3.5 s). A2: no collision is possible before t_rc. A3: bounded perception error ε_P. |
| switching / commitment | The replan cutoff t_rc. |
| real-time rate | T_bound; for example, the real-robot table lists T_b = 0.2. |
| validation | Replanning on every estimate gave the highest pickup success. First-pose single-shot was worst. Best-pose single-shot often failed "since a large number of goals are unreachable due to limited time remaining". |

### 1.9 Oelerich, Hartl-Nesic, Kugi — *Model Predictive Trajectory Planning for Human-Robot Handovers*, VDI Mechatroniktagung 2024, pp. 65–72, arXiv 2404.07505 (LIT-FULL)

| field | extraction |
|---|---|
| problem | H2R handover trajectory planning with BoundMPC (path-following MPC with Cartesian error bounds); 7-DoF cobot. |
| candidate representation | One handover location; a linear reference path from the start through an approach point to the *initial* predicted location. |
| candidate generation | None (single). |
| fixed vs state-dependent count | Single. |
| timing / interception rule | No explicit time. Path progress is synchronised with the human: φ̇_d = φ̇_d,max·tanh(s(d_r,HO − d_h,HO + b)), which can be negative. |
| grasp rule | Orientation from the current hand rotation. |
| trajectory rule | MPC with orthogonal error bounds (second-order functions) that narrow to the predicted location. The new bounds coincide with the previous bounds at the current path parameter. |
| hard constraints | Joint and velocity limits; error bounds (keeping the end effector in front of the hand). |
| soft objective | Tangential path error; terminal tracking cost (w_T,p > w_T,o). |
| replanning / update | Receding horizon; bounds re-planned each step. |
| prediction assumptions | Per-axis GP regression from hand position and velocity to the final handover location. Blend p̃_HO = w·μ_HO + (1 − w)·p_h with w = ½ + ½·tanh(α_p·d_pred − d_p): trust the prediction far away and the measurement near. |
| switching / commitment | Not specified. |
| real-time rate | Not stated in the extracted text. |
| validation | One demonstrated cup handover (OptiTrack); arrival slightly late because of the joint-2 velocity bound. No statistics. |

### 1.10 Djeha, Dallard, Zermane, Gergondet, Kheddar — *Human-Robot Handovers using Task-Space Quadratic Programming*, RO-MAN 2022 (LIT-FULL)

| field | extraction |
|---|---|
| problem | H2R handover inside **mc_rtc** task-space QP (TRIAD's low-level framework). |
| candidate representation | None. An object-pose observation task feeds a grasp-frame trajectory-tracking task. |
| timing / interception rule | "both meet at the HOL without explicit time or object configuration specification" (implicit encounter by closed-loop tracking). |
| grasp rule | For symmetric objects, orientation "as close as possible to the current end-effector". |
| hard constraints | QP joint position, velocity and acceleration limits; collision constraints. |
| switching / commitment | Not specified; focused "mainly on the reaching phase". |
| validation | Panda demonstrations; no statistics. |

### 1.11 Burgess-Limerick, Lehnert, Leitner, Corke — *An Architecture for Reactive Mobile Manipulation On-The-Move*, arXiv 2212.06991, 2022 (LIT-FULL)

| field | extraction |
|---|---|
| problem | Mobile manipulator grasps static and moving objects while the base moves. |
| timing / interception rule | The arrival time is derived from state. The base controller estimates the time until the object is within manipulation range (manipulability at arrival > 0.08, ≈0.75 m). Arm start time comes from expected transit time (EE distance / (v_B·k_A), k_A = 0.5; 0.25–1.0 tested, "all result in stable performance"). |
| trajectory rule | Quintic polynomial recomputed every control step to arrive as the base enters range. |
| switching / commitment | Distance to target < d_T = 0.1 m switches to a PBVS final-phase grasp controller (k_P = 5). The gripper is stabilised over the object while closing because closing with relative velocity needs "precise timing of the close command, which is difficult to achieve with a reactive method". |
| validation | > 99% real success (1 failure / 120). The open-loop planned baseline failed every dynamic grasp "because the object has moved between the time of trajectory generation and the time of grasping". |

### 1.12 Christen, Yang, Pérez-D'Arpino, Hilliges, Fox, Chao — *Learning Human-to-Robot Handovers from Point Clouds*, CVPR 2023, arXiv 2303.17592 (LIT-FULL)

| field | extraction |
|---|---|
| problem | Learned vision policy for H2R handover in HandoverSim (DexYCB replay), with sim-to-real transfer. |
| candidate representation | None explicit (end-to-end policy to a pre-grasp). |
| switching / commitment | A learned grasp predictor outputs a success probability: "If the probability is above a tunable threshold, we execute an open-loop grasping motion", then a predetermined retract. |
| validation | Sequential (hand static) vs simultaneous settings. In the simultaneous setting "our method follows the hand and finds a feasible grasp once the hand has come to a stop"; GA-DDPG grasping while moving had low success. |

### 1.13 Zhang, Fang, Fang, Lu — *Flexible Handover with Real-Time Robust Dynamic Grasp Trajectory Generation*, arXiv 2308.15622, 2023 (LIT-FULL)

| field | extraction |
|---|---|
| candidate generation | GSNet dense grasps; a transformer tracks one grasp over T = 3 frames (tolerance τ = 0.01 m). |
| timing / interception rule | No time. The future grasp position is the last pose plus λ × a voted momentum, with λ_o = 1 for motion toward the robot and λ_p = 3 for motion away. The motivation given is that robot motion and closing "take an irreducible period of time". |
| switching / commitment | Grasp "when the predicted pose … is close enough to the robot's current pose" (value not stated). |
| validation | 6 FPS; Flexiv + Robotiq-85; 31 objects; 78.15% success on moving objects. |

### 1.14 Jia, Xu, Jayaraman, Song — *Dynamic Grasping with a Learned Meta-Controller*, arXiv 2302.08463 v3, 2024 (LIT-FULL)

| field | extraction |
|---|---|
| problem | The Akinola-style pipeline with the look-ahead time T_L ∈ [0, 8] s and the planning time budget T_T ∈ [0, 4] s chosen online. |
| timing / interception rule | A learned RL meta-controller chooses one T_L per iteration. Trade-off in the authors' words: a large look-ahead may land in reachable space and leave time "but worse prediction accuracy". |
| baselines | Fixed grid search best (2 s, 1 s). BO best (0.82 s, 0.78 s). **Online-IK**: split [0, 8] s into N = 3 segments and choose the farthest predicted pose with valid IK. **Online-reachability**: repeated planning attempts; R = 1 − Σt_i/(M·T_T). |
| validation | PyBullet; up to +28% success in the most cluttered setting. The controller "learns to reason about the reachable workspace and maintain the predicted pose within the reachable region". |

### 1.15 Abstract-level only (LIT-ABS)

- **Wang et al., GenH2R, CVPR 2024**: large-scale simulated H2R demonstrations and 4D imitation with a future-forecasting objective; ≥ +10% success.
- **Li & Hauser 2015 (workshop)**: predicts human reaching end time and position (minimum jerk, regression). This belongs to the human-release/transfer-point predictor class that TRIAD does **not** claim.

---

## 2. Design matrix by question (A–J)

Each row gives three things:

- **adoptable principle**: general, transferable, and defensible without paper-specific numbers;
- **paper-specific detail**: do not import it as a TRIAD constant;
- **TRIAD gap**: what the literature leaves unsolved and what a later phase must derive or measure.

### A. State-dependent interception-time generation (→ Phase 3, T_k)

| source | what is done |
|---|---|
| Menon 2014 | Earliest feasible time. The first forward time sample at which a bounded-velocity/acceleration straight-line time of flight plus grasp time fits before the object arrives; bounded above by the workspace-exit time. |
| Islam 2020 | Target time = now + computation bound T_bound (plan for the pose projected forward by T_bound). |
| Burgess-Limerick 2022 | Arrival time = time until the object enters manipulation range; arm start from estimated transit time. |
| Salehian 2016 | First time along the prediction at which the object becomes reachable (workspace likelihood). |
| Kim 2014 | All time slices until end of flight; select maximum likelihood. No execution-time feasibility check, and 12/19 failures were late arrival. |
| Akinola 2021 | Horizon from a distance step function; the authors say it *should* be a function of EE speed, distance and target motion, and leave that open. |
| Jia 2023 | A fixed horizon is a hyperparameter. Online-IK uses the farthest valid-IK horizon; a learned controller picks one horizon per iteration. |

- **Adoptable principles**
  - (A1) A physical lower bound: an interception time is admissible only if a robot time-to-reach bound plus closure time plus computation latency fits before it (Menon, Islam, Burgess-Limerick).
  - (A2) An upper bound from the interaction/workspace window: the object leaving the reachable region or the prediction ending (Menon ΔT_max, Salehian, Kim).
  - (A3) Execution-time feasibility must be checked, not assumed (Kim's failure breakdown is direct negative evidence).
  - (A4) The horizon choice trades prediction accuracy against time and reachability (Jia; Akinola's open problem).
- **Paper-specific detail (do not import)**: Akinola's 2 s / 1 s / 0 s steps; Jia's 8 s predictable range and (2 s, 1 s); Islam's t_rc = 3.5 s; Kim's slice resolution; Menon's 2.0 s gripper time.
- **TRIAD gap**: none of these generates a **finite set** of interception times as a law I_k → T_k whose bounds come from both robot capability and a *measured* prediction-validity envelope, and whose spacing comes from a physical pose-change criterion. TRIAD's current bank (CODE) is a fixed schedule from 1.8 to 8.0 s at 0.45 s steps (14 leads), regardless of state. Phase 2 must supply the prediction-error-vs-horizon envelope and the preview/runtime reach-duration error that A1 and A4 need.

### B. Future-state sampling and adaptive temporal resolution (→ Phase 3)

| source | what is done |
|---|---|
| Menon 2014 | Uniform Δt_sample in the heuristic; search time resolution set by primitive Δt. |
| Kim 2014 | Time slices of the predicted trajectory (resolution not stated in the extracted text). |
| Islam 2020 | Spatial goal resolution of 1 cm and 10°, fixed. |
| Jia 2023 | N = 3 horizon segments for Online-IK, chosen as a computation/accuracy balance. |

- **Adoptable principle**: resolution is a trade between coverage and computation, so it must be justified by a convergence study, not asserted (Jia's own N trade-off; Marturi's N study in D).
- **Paper-specific**: all the resolutions above.
- **TRIAD gap**: no paper derives temporal spacing from how much the *object pose* changes between samples relative to the capture tolerance. That is the criterion Phase 3 must define: sample spacing so that consecutive predicted poses differ by at most a capture-relevant distance or angle, refined where the pose changes fast and coarsened when the object is at rest. At rest, all future poses are identical (CODE: memoisation already collapses them), so an at-rest T_k could legitimately be tiny. This is a state-dependent count the literature does not provide.

### C. Online grasp-hypothesis generation and filtering (→ Phase 4, G_k(τ))

| source | what is done |
|---|---|
| Marturi 2019 | Fixed N from a learned generator, run once and rigidly transformed. More hypotheses gave more switching and lower error. |
| Akinola 2021 | Fixed database; filtered per step to 10 by reachability ∪ motion-aware quality. |
| Yang 2021 | Temporally consistent refinement of the previous set; hand-collision removal; **resample when depleted**; 180° flip for a 2-finger gripper. |
| Menon 2014 | "a handful" of radially arranged grasps about a cylinder axis. |
| Kim 2014 | Continuous optimum per time slice; palm direction must oppose object velocity (60°). |
| Zhang 2023 | Dense grasps, one tracked across frames. |

- **Adoptable principles**
  - (C1) Generate from object geometry, then transform to the predicted pose at τ (all).
  - (C2) Screen cheap necessary conditions before expensive ones (Akinola's reachability screen before IK; Yang's collision removal).
  - (C3) Exploit gripper symmetry explicitly (Yang's 180° flip, the same role as TRIAD's two axis signs).
  - (C4) Allow the set to change size and regenerate on an event (Yang's depletion trigger).
  - (C5) Approach-direction relative to object motion matters (Kim's constraint; Akinola's motion-aware model).
- **Paper-specific**: 100 / 5000 grasps, top-5 + top-5, 10 cm standoff, 60° palm threshold, learned models. TRIAD has a known CALL handle geometry and needs none of the learned generators.
- **TRIAD gap**: none derives the **angular resolution** of a grasp family around a known handle from capture/approach tolerances (jaw capture width, closure sweep tolerance), nor makes the family τ-dependent through the predicted pose. TRIAD's 16 angles × 2 signs (CODE) is inherited. Phase 4 must derive the initial resolution from gripper/handle geometry and run a convergence study (feasible recovery, selected-action stability, completion, computation).

### D. Reachability-conditioned grasp sets (→ Phase 4)

| source | what is done |
|---|---|
| Akinola 2021 | Precomputed 6D reachability ranking before IK. Reachability matters most for near and far objects. |
| Yang 2022 | Learned manipulability |JᵀWJ| ranking; +7.5% success. |
| Salehian 2016 / Kim 2014 | GMM reachable-space likelihood thresholds (Salehian: 99% of training postures above δ). |
| Burgess-Limerick 2022 | Manipulability threshold 0.08 defines the range. |

- **Adoptable principle**: condition the grasp set on robot reachability at the predicted pose, *as a filter or screen*, before costly certification.
- **Paper-specific**: learned or GMM models and their thresholds.
- **TRIAD gap**: TRIAD already has an exact but expensive reachability test (static F1 reach standoff, CODE; A1 verdict F1 USEFUL, MEASURED). The open question for Phase 4 is whether a cheaper *exact-or-conservative* necessary condition can shrink G_k(τ) without losing certified actions. The Track 1 exact timing prune (MEASURED lossless) is the model for what "lossless" must mean. A learned reachability screen would not be lossless and is out of scope.

### E. Moving-target trajectory generation, lazy or direct-first alternatives (→ Phase 5, R_k(τ,g))

| source | what is done |
|---|---|
| Yang 2021 | "we always attempt a straight-line path first", with RRT-Connect "when necessary". |
| Marturi 2019 | Local planner if the configuration distance is ≤ δ_c, otherwise the global planner. |
| Akinola 2021 | Previous-solution seeding; retime to current velocities. |
| Menon 2014 | Dynamic primitives plus an adaptive grasp primitive; single search. |
| Burgess-Limerick 2022 | Analytic quintic re-computed each step. |
| Yang 2022 / Oelerich 2024 | MPC around a reference; error bounds. |

- **Adoptable principles**
  - (E1) **Direct first; generate alternatives only on failure** (Yang 2021 explicitly; Marturi by distance).
  - (E2) Reuse the previous solution for continuity (Akinola seeding; Oelerich bound continuity).
- **Paper-specific**: RRT-Connect, PRM, δ_c values, MPC weights.
- **TRIAD gap**: the literature does not characterise *why* the direct motion fails in a way that tells a finite alternative generator where to put via-points. TRIAD's 8 directions × 2 offsets (CODE) is inherited. Phase 5 must log the direct-route failure stage and geometry (A1 instrumentation already gives stage and reason), then derive a failure-conditioned generator and compare direct-only, reduced, adaptive-lazy and the full reference.

### F. Reactive / receding candidate regeneration (→ Phases 3–5 and 7)

| source | what is done |
|---|---|
| Islam 2020 | Replan on every estimate, within a provable time bound. Late accurate single-shot planning loses reachable goals. |
| Yang 2021 / 2022 | 10 Hz or 5 Hz regeneration. |
| Kim 2014 | Re-predict at every measurement. |
| Akinola 2021 | Every iteration. |
| Salehian 2016 | Continuously. |
| Burgess-Limerick 2022 | Every control step. |

- **Adoptable principle**: regenerate the candidate set from the current state each update. A bounded computation time is a first-class requirement, because the admissible window shrinks while computing (Islam).
- **Paper-specific**: rates.
- **TRIAD gap**: MEASURED in this project, V2 produces no certified plan while the object moves in two of four scenarios. The binding constraint is enumeration latency against the motion window (Phase D), and speed alone did not raise completion (Track 1: 11/12 → 8/12). Islam's bounded-time guarantee rests on offline precomputation over a *known, repetitive* goal region. TRIAD cannot assume that, which is exactly why the state-conditioned law must shrink the sets rather than only speed up checks.

### G. Prediction-validity horizons and uncertainty (→ Phase 2, then Phase 3)

| source | what is done |
|---|---|
| Islam 2020 | Pose error decreases as the object approaches; bounded ε_P assumption; replan cutoff. |
| Kim 2014 | Stops re-predicting below 0.09 s to contact because the near-robot view is unreliable. |
| Jia 2023 | Look-ahead accuracy trade-off. |
| Oelerich 2024 | GP variance shapes bounds; prediction blends into measurement as the hand nears. |
| Akinola 2021 | Removing prediction is the largest ablation loss. |
| Salehian 2016 | For a slow carried object, the prediction "does not play a vital role". |

- **Adoptable principles**
  - (G1) Prediction trust should decay with horizon and be bounded by a measured error envelope.
  - (G2) Near contact, measurement or closed-loop tracking should dominate prediction (Oelerich blend, Salehian γ, Kim cutoff).
- **Paper-specific**: 0.09 s, GP models, ε_P = 2.5 cm.
- **TRIAD gap**: no paper reports an error-vs-horizon curve for a constant-twist predictor under the accelerations and decelerations of a handover and turns it into an *upper horizon* for candidate generation. Phase 2A must measure e_p(h) and e_R(h). Phase 3 then sets τ_max − t_k where the error exceeds the capture tolerance, which is the commit-freshness tube (15 mm / 0.12 rad, CODE) if that tube survives Phase 9.

### H. Ranking only after hard feasibility (→ Phase 6)

| source | ordering | tuning / derivation |
|---|---|---|
| Yang 2021 | Rank by soft score, then first-feasible in rank order. | Weights set without derivation. |
| Akinola 2021 | Rank/filter, then IK; closest configuration. | Combination found empirically. |
| Yang 2022 | Hard constraints as large-weight costs (not hard). | Weights tuned for 30 minutes. |
| Marturi 2019 | Soft task-space error; infeasible targets tracked. | δ_w by trial and error. |
| Menon 2014 | Least time among dynamically feasible trajectories (hard, then time). | — |
| Kim 2014 | Maximum likelihood, no timing feasibility; 12/19 failures from timing. | — |

- **Adoptable principles**
  - (H1) Keep hard safety/feasibility separate from preference. Evidence against soft handling of hard requirements: Kim's timing failures; Yang 2022 GoalSet reduced success.
  - (H2) Where a single objective is used with a clear physical meaning, *time* is common (Menon; Akinola's "as fast as possible").
  - (H3) Multi-term weights in this literature are hand-tuned. No paper provides a validated weight derivation that TRIAD could inherit.
- **Paper-specific**: every weight.
- **TRIAD gap**: TRIAD's seven weights (8, 2, 2, 3, 1.6, 1.4, 1)/19 have no literature support (CODE, Track 2 inventory). The earlier audit found argmin ≈ earliest-feasible (MEASURED, earlier audit memory). Phase 6 must compare the full J, earliest feasible, lexicographic (time → clearance → effort), and reduced objectives on completion, clearance, regret and switching. If J adds nothing material, remove it.

### I. Persistence, switching hysteresis, replanning triggers (→ Phase 7B/C)

| source | what is done |
|---|---|
| Marturi 2019 | Hysteresis ε_k + δ_w < ε_r*; the switching threshold trades flexibility against switch count. |
| Yang 2021 | w_prev term penalises moving away from the previous grasp. |
| Salehian 2016 | New intercept point chosen near the previous one. |
| Akinola 2021 | Seeding from the previous trajectory; "reduces the number of grasp switches" via reachable grasps. |
| Oelerich 2024 | New bounds coincide with the old bounds at the current progress. |
| Yang 2021 | Regeneration triggered by candidate depletion. |

- **Adoptable principles**
  - (I1) Persistence matters for smoothness and success; unconstrained re-selection causes oscillation (all).
  - (I2) Switching should need a margin or an invalidity event, not merely a marginally better score.
- **Paper-specific**: δ_w = 2.0, w_prev = 5, δ_c.
- **TRIAD gap**: all of these realise persistence through **soft cost margins** tuned by hand. TRIAD's frozen rule is retain-while-certified, replace-only-when-invalid: a validity-based rule with no tunable margin. It is not established by this literature. Phase 7B must compare it against cost-driven and hysteretic switching on replacements, oscillation, stale-plan duration and completion.

### J. Commitment and capture triggers (→ Phase 7A, Phase 9)

| source | trigger | after the trigger |
|---|---|---|
| Akinola 2021 | Proximity/orientation (d_p ≤ 1.1 b, 20°) | Short prediction, then close while moving. |
| Yang 2021 | Arrival at standoff | Open-loop grasp. |
| Yang 2022 | Blocking grasp stage at standoff | F/T contact classifier. |
| Christen 2023 | Pre-grasp plus learned success probability over a tunable threshold | Open-loop grasp. |
| Burgess-Limerick 2022 | d_T = 0.1 m | Closed-loop PBVS; stabilised over the object while closing. |
| Zhang 2023 | Predicted pose close enough | — |
| Salehian 2016 | < 2 cm | Finger closure. |
| Kim 2014 | Prediction freezes at 0.09 s to contact | — |
| Islam 2020 | Replan cutoff t_rc | Execute last plan. |
| Marturi 2019 | Human operator | — |

- **Adoptable principles**
  - (J1) A single, late, one-way transition from reactive approach to capture is universal. The reason given is that perception and prediction degrade near contact and closure needs a stable relative pose (Yang 2022, Kim, Burgess-Limerick).
  - (J2) The capture phase is short and uses closed-loop or local motion.
- **Paper-specific**: every distance, angle and probability threshold.
- **TRIAD gap**: the literature triggers on *proximity heuristics or learned confidence*. None commits on "the complete remaining action, including closure and carried retreat, is certified fresh and its timing reserve still holds". TRIAD's single terminal latch (CODE; L_commit = 1.6 s, L_entry = 0.05 s) is that kind of rule, but its reserves are inherited. Phase 7A must compare early (V1-like), late terminal, and a later alternative. Phase 9 must derive L_commit from the measured remaining reach, closure and dwell durations plus the measured preview/runtime error (Phase 2B), not from the literature numbers above.

---

## 3. What the literature supports in the frozen architecture, and what it does not

| frozen element | literature status |
|---|---|
| Robot interception time τ as a decision variable (not a human release time) | Supported in kind: Menon's earliest feasible pickup time, Kim's catching time, Salehian's T*, Burgess-Limerick's arrival time. Handover papers mostly avoid explicit time (Yang, Oelerich, Djeha, Zhang). Human-release predictors (Li & Hauser) are a different claim, not used. |
| Joint (τ, g, r) candidates | Pairs exist (Kim: time × configuration; Menon: pregrasp pose × time). A **finite joint time × grasp × route set with complete-action certification** was not found in the papers read. This is a gap/contribution candidate, not an established method. |
| Complete-action certification before ranking | Partially: Menon validates full grasp primitives before accepting a goal edge; Yang 2021 checks the standoff path and the Cartesian grasp path. No paper certifies closure plus carried retreat as part of candidate admission. Kim's failures are evidence for timing certification. |
| Provisional motion while the object moves, with recertification | Supported: Islam (start early, replan), Akinola, Yang, Burgess-Limerick. |
| Retain while certified, replace only when invalid | Not established; the literature uses soft hysteresis (section I). Phase 7B must test it. |
| Single late terminal commitment | Supported in kind (section J); the trigger definition differs. |
| Carried-object retreat in the certified action | Not found as a certified component (Yang 2022 retreats after grasp; Christen uses a predetermined retract). |
| mc_rtc QP below, supervisory decision above | Djeha 2022 shows QP-only implicit encounter without candidates, certification or commitment. TRIAD's layer is additive, not a replacement. |

**No finding in this literature contradicts the frozen architecture.** Two findings bound expectations and must be kept visible:

1. Christen 2023 learned in the simultaneous setting to wait until the hand stops. Yang 2021 instructed users that the robot moves once the object is still. This matches TRIAD's MEASURED behaviour (no plan while moving in 2/4 scenarios). Planning while the object moves is hard across methods; it is not a TRIAD-specific defect, and it is not solved by speed alone (Track 1).
2. Salehian 2016 found that for a slow carried object, closed-loop tracking made intercept prediction non-critical. For slow givers, TRIAD's prediction contribution may be small. Phase 2 must quantify this rather than assume a benefit.

## 4. Inherited TRIAD constants with no support in this literature

These have to be derived, validated or removed in Phases 3–9. Literature values are not substitutes.

| TRIAD quantity (CODE) | status after Phase 1 |
|---|---|
| lead bank 1.8–8.0 s, step 0.45 s (14 leads) | No support; principles A1–A4 and B define what must replace it. |
| 16 grasp angles × 2 signs (32) | Symmetry doubling is principled (Yang flip); 16 has no support (C). |
| 17 routes (direct + 8 directions × 80/140 mm) | Direct-first is principled (Yang); the alternative ring has no support (E). |
| seven cost weights (8, 2, 2, 3, 1.6, 1.4, 1)/19, w_T = 0.4210526, T_ref = 8 | No support; literature weights are hand-tuned (H). |
| L_commit = 1.6 s, L_entry = 0.05 s | No support; must come from measured durations and errors (J). |
| commit-freshness tube 15 mm / 0.12 rad; rest thresholds 0.004 m/s, 0.08 rad/s | No support; must come from capture tolerance and the Phase 2 prediction envelope (G). |
| standoff 0.120 m, retreat 0.180 m, entry depth 0.135 m | Literature standoffs (5–15 cm) differ per gripper; must come from Robotiq 2F-85 and CALL handle geometry (Phase 9). |

## 5. Phase handover

Phase 2 is next: measure prediction error vs horizon for the constant-twist predictor (4A) and preview/runtime parity (4B). The later laws depend on it:

- A1 needs the measured preview reach-duration error to set a conservative time-to-reach bound.
- G1 and the upper T_k horizon need e_p(h) and e_R(h).
- J needs the runtime-vs-preview duration distribution to derive L_commit.
