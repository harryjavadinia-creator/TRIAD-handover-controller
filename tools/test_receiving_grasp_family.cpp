// Deterministic unit tests for src/ReceivingGraspFamily.h (TRIAD-lite Phase B).
// Build: tools/run_control_aware_supervisor_unit_tests.sh

#include "../src/ControlAwareGraspSupervisor.h"
#include "../src/ReceivingGraspFamily.h"

#include <cstdio>
#include <cstdlib>

namespace
{
int failures = 0;
#define CHECK(cond)                                                                   \
  do                                                                                  \
  {                                                                                   \
    if(!(cond))                                                                       \
    {                                                                                 \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);            \
      ++failures;                                                                     \
    }                                                                                 \
  } while(0)

using namespace call_handover;

bool near(double a, double b, double tol) { return std::abs(a - b) <= tol; }

void testFrameAndContacts()
{
  HandleGeometry h;
  h.center = Eigen::Vector3d(0.6, -0.1, 0.3);
  h.axis = Eigen::Vector3d(0.2, 0.9, -0.3).normalized();
  GripperInterface gi;
  const Eigen::Vector3d reference(-0.4, 0.2, 0.5);
  ReceivingGraspFamilySpec spec;
  spec.thetaSamples = 24;
  spec.axialSamples = 3;
  spec.axialMax = 0.035;
  for(const auto & g : generateReceivingGraspFamily(spec))
  {
    const auto p = receivingGraspPoses(h, gi, reference, g);
    const Eigen::Matrix3d & R = p.rotation;
    CHECK((R.transpose() * R - Eigen::Matrix3d::Identity()).norm() < 1e-9);
    CHECK(near(R.determinant(), 1.0, 1e-9));
    CHECK(std::abs(R.col(0).dot(h.axis)) < 1e-9);                   // closing axis orthogonal to the handle
    CHECK(near(R.col(2).dot(h.axis), static_cast<double>(g.sign), 1e-9));
    // antipodal contacts: on the surface, opposite normals, through the axis
    const Eigen::Vector3d onAxis = h.center + g.s * h.axis;
    CHECK(near((p.contactPlus - onAxis).norm(), h.radius, 1e-12));
    CHECK(near((p.contactMinus - onAxis).norm(), h.radius, 1e-12));
    CHECK(((p.contactPlus - onAxis) + (p.contactMinus - onAxis)).norm() < 1e-12);
    CHECK(std::abs((p.contactPlus - onAxis).dot(h.axis)) < 1e-12);
    // approach offsets along +y_M
    CHECK(((p.standoff - p.capture) - gi.standoffDistance * R.col(1)).norm() < 1e-12);
    CHECK(((p.retreat - p.capture) - gi.retreatDistance * R.col(1)).norm() < 1e-12);
    CHECK(((p.capture - onAxis) - gi.captureDepth * R.col(1)).norm() < 1e-12);
  }
}

void testTaskConstraintReduction()
{
  GripperInterface gi;
  HandleGeometry h;
  CHECK(near(receivingAxialLimit(h, gi), 0.035, 1e-12));  // corridor tolerance binds
  gi.corridorAxialTolerance = 1.0;
  CHECK(near(receivingAxialLimit(h, gi), 0.0687 - 0.0125 - 0.003, 1e-12));  // handle length binds
  ReceivingGraspFamilySpec centred;
  centred.thetaSamples = 10;
  centred.axialSamples = 5;
  centred.axialMax = 0.0;  // explicit centred-grasp task constraint
  const auto fam = generateReceivingGraspFamily(centred);
  CHECK(fam.size() == 20u);
  for(const auto & g : fam) { CHECK(g.s == 0.0); }
  ReceivingGraspFamilySpec axial;
  axial.thetaSamples = 4;
  axial.axialSamples = 5;
  axial.axialMax = 0.03;
  const auto fa = generateReceivingGraspFamily(axial);
  CHECK(fa.size() == 40u);
  CHECK(near(fa[0].s, -0.03, 1e-12) && near(fa[2 * 4].s, 0.0, 1e-12) && near(fa[4 * 4].s, 0.03, 1e-12));
  // mechanical screen enforces the axial bound
  ReceivingGrasp out;
  out.s = 0.04;
  ReceivingGraspPoses p;
  p.capture = p.standoff = p.retreat = Eigen::Vector3d(0, 0, 1);
  CHECK(!receivingMechanicalScreen(out, p, 0.035, 0.0).passed);
  CHECK(receivingMechanicalScreen(out, p, 0.035, 0.0).reason == "axial_limit");
  out.s = 0.02;
  CHECK(receivingMechanicalScreen(out, p, 0.035, 0.0).passed);
  p.retreat.z() = -0.01;
  CHECK(receivingMechanicalScreen(out, p, 0.035, 0.0).reason == "mouth_below_ground");
}

void testResolutionAndConvergenceHelper()
{
  // Phase 4 floor: eps_p 15 mm, eps_R 0.12 rad, lever 0.126 m -> dtheta 0.119 rad -> 53 samples
  CHECK(thetaSamplesForResolution(0.015, 0.12, 0.126) == 53);
  // spacing of the generated family never exceeds the floor when N meets it
  ReceivingGraspFamilySpec spec;
  spec.thetaSamples = thetaSamplesForResolution(0.015, 0.12, 0.126);
  const auto fam = generateReceivingGraspFamily(spec);
  CHECK(fam[1].theta - fam[0].theta <= std::min(0.015 / 0.126, 0.12) + 1e-12);
  CHECK(fam.size() == 106u);
}

void testLegacyEquivalence()
{
  const Eigen::Vector3d axis = Eigen::Vector3d(0.0, 0.3, 0.95).normalized();
  const Eigen::Vector3d outward(0.7, -0.2, 0.1);
  HandleGeometry h;
  h.axis = axis;
  GripperInterface gi;
  for(int sign : {1, -1})
  {
    for(int k = 0; k < 16; ++k)
    {
      ReceivingGrasp g;
      g.sign = sign;
      g.theta = 2.0 * M_PI * k / 16.0;
      GraspParameters legacy;
      legacy.sign = sign;
      legacy.phi = legacyPhiFromTheta(sign, g.theta);
      const Eigen::Matrix3d Rn = receivingGraspPoses(h, gi, outward, g).rotation;
      const Eigen::Matrix3d Rl = graspFrameRotation(axis, outward, legacy);
      CHECK((Rn - Rl).norm() < 1e-9);
    }
  }
}

void testReachabilitySdf()
{
  using namespace gen3_wrist_reachability;
  // far outside the arm's reach
  CHECK(gen3WristReachabilitySdf(Eigen::Vector3d(2.0, 0.0, 0.3)) < -0.5);
  // grid node values are reproduced exactly (bilinear at nodes)
  const int ir = kNRho / 3;
  const int iz = kNZ / 2;
  const Eigen::Vector3d node(kRho0 + ir * kStep, 0.0, kZ0 + iz * kStep);
  CHECK(near(gen3WristReachabilitySdf(node), kSdf[iz * kNRho + ir], 1e-6));
  // solid of revolution: invariant to azimuth
  const double r = 0.45;
  const double z = 0.35;
  const double v0 = gen3WristReachabilitySdf(Eigen::Vector3d(r, 0.0, z));
  CHECK(near(gen3WristReachabilitySdf(Eigen::Vector3d(0.0, -r, z)), v0, 1e-12));
  CHECK(near(gen3WristReachabilitySdf(Eigen::Vector3d(r / std::sqrt(2.0), r / std::sqrt(2.0), z)), v0, 1e-9));
  CHECK(v0 > 0.0);  // a typical receiving location is inside
  // wrist point from a tool pose: fixed offset along the tool axes
  const Eigen::Matrix3d R = Eigen::AngleAxisd(0.7, Eigen::Vector3d(0.3, 0.1, 0.9).normalized()).toRotationMatrix();
  const Eigen::Vector3d p(0.4, 0.2, 0.5);
  CHECK((gen3WristPoint(R, p) - (p + R * Eigen::Vector3d(kWristInToolX, kWristInToolY, kWristInToolZ))).norm() < 1e-12);
  CHECK(near((gen3WristPoint(R, p) - p).norm(), 0.061525, 1e-6));
}

void testShortlist()
{
  std::vector<FunnelCandidate> c;
  auto add = [&c](int id, int sign, int axial, double theta, double score)
  {
    FunnelCandidate f;
    f.index = id;
    f.id = id;
    f.sign = sign;
    f.axialIndex = axial;
    f.theta = theta;
    f.score = score;
    c.push_back(f);
  };
  add(0, 1, 0, 0.00, 0.30);
  add(1, 1, 0, 0.05, 0.29);   // same theta bin as 0 -> deferred by diversity
  add(2, 1, 0, 0.50, 0.20);
  add(3, -1, 0, 0.00, 0.25);
  add(4, 1, 0, 1.00, -0.05);  // pruned at tolerance 0.02
  add(5, 1, 0, 1.50, -0.01);  // kept (within tolerance)
  const auto top3 = reachabilityShortlist(c, 3, 0.02, 0.2);
  CHECK(top3.size() == 3u && top3[0] == 0 && top3[1] == 3 && top3[2] == 2);
  const auto all = reachabilityShortlist(c, 0, 0.02, 0.2);
  CHECK(all.size() == 5u);  // 4 pruned only
  CHECK(std::find(all.begin(), all.end(), 4) == all.end());
  const auto fill = reachabilityShortlist(c, 5, 0.02, 0.2);
  CHECK(fill.size() == 5u && fill[4] == 1);  // diversity first, then fill by score (5 then 1)
  // interception order: earlier surrogate event before better score
  auto moving = c;
  moving[0].eventIndex = 3;
  moving[3].eventIndex = 1;
  const auto m3 = reachabilityShortlist(moving, 3, 0.02, 0.2);
  CHECK(m3.size() == 3u && m3[0] == 1 && m3[1] == 2 && m3[2] == 5);
}
} // namespace

int main()
{
  testFrameAndContacts();
  testTaskConstraintReduction();
  testResolutionAndConvergenceHelper();
  testLegacyEquivalence();
  testReachabilitySdf();
  testShortlist();
  if(failures != 0)
  {
    std::fprintf(stderr, "%d check(s) failed\n", failures);
    return EXIT_FAILURE;
  }
  std::printf("test_receiving_grasp_family: all checks passed\n");
  return EXIT_SUCCESS;
}
