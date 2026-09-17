# Scientific repair log

Reference: bd815da31326c0d32dfb41a60345d38b81fe249f. Original reports,
pre/post-fix evidence and the original branch are preserved. No outcome claim
is transferred to this branch. Investigation branch: research/triad-scientific-repair.

## Gate 1: physical identity

Defect: receiving-family integer IDs used a robot-relative angular origin.
Invariant: a fixed family specification and ID determine immutable object-relative
capture, standoff and retreat transforms, independent of robot motion.

The basis is now formed in object coordinates: project object +X orthogonally
to the handle axis, with object +Y fallback; e2 = h cross e1. Geometry is built
in this frame and placed in the world by the object transform. Robot position
is used only by reachability/feasibility, never to redefine this basis.

Changing family discretization is a new experiment, not an identity-preserving
runtime operation. Historical ID numbers must NOT be compared physically across
this repair. The old family remains in its frozen commit.

Tests: all 530 grasps, all three target transforms, arbitrary rigid object
placement; old robot-relative construction explicitly violates the invariant.
Unit suites passed. Full controller build recorded separately before commit.

## Gate 2: geometry ownership

The provisional plan owns immutable object-relative standoff, capture and retreat
transforms. Candidate world transforms are evaluated at that candidate's object
pose; they must reduce to the same object-relative transforms on retention/patch.
A numerical identity tolerance of 1e-8 (matrix/vector norms, not a physical
acceptance margin) rejects nonfinite/mismatched geometry before selector mutation.
Patching takes its target from the stored plan, not a second geometry owner.

| Quantity | Frame/time | Mutability |
| --- | --- | --- |
| candidate W_T_M_* | World, candidate evaluation object pose | New per evaluation |
| plan O_T_M_* | Object, adoption | Immutable for that physical grasp |
| reference patch O_T_M_standoff | Object, patch start | Copied from plan |
| reference world pose | World, each control tick | Updated with pose estimate/time |
| terminal gate targets | World, current object estimate | Recomputed from plan |
| terminal certificate targets | World, certificate snapshot | Recomputed from plan |

The selector still uses integer IDs within a fixed receiving-family specification;
Gate 1 makes these physically stable. Legacy-ring experiments are not part of the
repaired receiving-family comparison. Pure geometry tests include rotation,
axial displacement and nonfinite mismatch rejection. Existing historical evidence
is not reclassified as evidence for the repaired branch.
