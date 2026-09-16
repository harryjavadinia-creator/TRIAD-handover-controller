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
