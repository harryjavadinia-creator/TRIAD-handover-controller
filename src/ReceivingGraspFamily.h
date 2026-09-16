#pragma once

// TRIAD-lite receiving-grasp front end: mechanically derived grasp family,
// necessary-condition filters and a cheap Gen3 reachability funnel.
// Pure (Eigen + standard library + generated map); no controller state.
//
// Derivation (research/triad_lite/TRIAD_PREDICTIVE_INTERCEPTION_AUDIT.md, sec. 4):
//  * parallel-jaw antipodal contact on a cylinder of radius R: the closing axis
//    passes through, and is orthogonal to, the handle axis h. Contacts
//    c+/- = p_H + s h +/- R n(theta), n(theta) = e1 cos(theta) + e2 sin(theta);
//  * pad line contact / insertion corridor fix the remaining hand rotation about
//    the closing axis: z_M = sigma h (psi = 0 up to the corridor tolerance);
//  * (sigma = -1, theta) and (sigma = +1, theta + pi) are the same contact set with
//    the fingers swapped: one mechanical grasp, two robot configurations. sigma is
//    kept as a discrete kinematic branch;
//  * the handle and the coaxial sensor core / grey handle are axisymmetric, so
//    contact mechanics does not restrict theta; |s| is bounded by the handle
//    length, the finger width and the acquisition corridor's axial tolerance.
// Frame: x_M = n(theta) (closing axis), z_M = sigma h, y_M = z_M x x_M (approach,
// pointing away from the handle). capture = p_H + s h + d_c y_M,
// standoff = capture + d_s y_M, retreat = capture + d_r y_M.

#include "Gen3WristReachabilityMap.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <set>
#include <string>
#include <tuple>
#include <vector>

namespace call_handover
{

struct ReceivingGrasp
{
  int id = -1;
  int sign = 1;        ///< sigma: z_M = sign * handle axis
  double theta = 0.0;  ///< closing direction n(theta) about the handle axis
  double s = 0.0;      ///< axial offset along the handle axis (m)
  int thetaIndex = 0;
  int axialIndex = 0;
};

struct ReceivingGraspFamilySpec
{
  int thetaSamples = 53;  ///< numerical; floor from ceil(2 pi / dtheta_min)
  int axialSamples = 1;   ///< numerical; odd counts include s = 0
  double axialMax = 0.0;  ///< task constraint; 0 imposes a centred grasp
};

/** Ordered sign +1 then -1, then s ascending, then theta ascending. */
inline std::vector<ReceivingGrasp> generateReceivingGraspFamily(const ReceivingGraspFamilySpec & spec)
{
  const int nT = std::max(1, spec.thetaSamples);
  const bool centred = spec.axialMax <= 0.0 || spec.axialSamples <= 1;
  const int nS = centred ? 1 : spec.axialSamples;
  std::vector<ReceivingGrasp> out;
  out.reserve(static_cast<std::size_t>(2 * nT * nS));
  for(int sg = 0; sg < 2; ++sg)
  {
    for(int is = 0; is < nS; ++is)
    {
      const double s = centred ? 0.0 : -spec.axialMax + 2.0 * spec.axialMax * static_cast<double>(is) / (nS - 1);
      for(int it = 0; it < nT; ++it)
      {
        ReceivingGrasp g;
        g.id = static_cast<int>(out.size());
        g.sign = sg == 0 ? 1 : -1;
        g.theta = 2.0 * M_PI * static_cast<double>(it) / static_cast<double>(nT);
        g.s = s;
        g.thetaIndex = it;
        g.axialIndex = is;
        out.push_back(g);
      }
    }
  }
  return out;
}

/** Smallest theta sample count whose spacing changes the grasp pose by at most
 * epsP at lever arm leverArm and by at most epsR in rotation (Phase 4 floor). */
inline int thetaSamplesForResolution(double epsP, double epsR, double leverArm)
{
  const double d = std::min(epsP / std::max(1e-9, leverArm), epsR);
  return static_cast<int>(std::ceil(2.0 * M_PI / std::max(1e-9, d)));
}

struct HandleGeometry
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  double radius = 0.01125;
  double halfLength = 0.0687;
};

struct GripperInterface
{
  double captureDepth = 0.006;
  double standoffDistance = 0.120;
  double retreatDistance = 0.180;
  double fingerHalfWidth = 0.0125;  ///< Robotiq tip body lattice |y| 8 mm + 4.5 mm
  double axialMargin = 0.003;       ///< geometry safety margin
  double corridorAxialTolerance = 0.035;
};

/** Axial task bound: min(L_H - w/2 - m, corridor axial tolerance), >= 0. */
inline double receivingAxialLimit(const HandleGeometry & h, const GripperInterface & g)
{
  return std::max(0.0, std::min(h.halfLength - g.fingerHalfWidth - g.axialMargin, g.corridorAxialTolerance));
}

/** e1 = reference projected orthogonally to the axis (deterministic fallbacks),
 * e2 = axis x e1, n(theta) = e1 cos + e2 sin. */
inline Eigen::Vector3d receivingContactNormal(const Eigen::Vector3d & axis, const Eigen::Vector3d & reference, double theta)
{
  const Eigen::Vector3d h = axis.normalized();
  Eigen::Vector3d e1 = reference - h * h.dot(reference);
  if(e1.norm() < 1e-9) { e1 = Eigen::Vector3d::UnitX() - h * h.x(); }
  if(e1.norm() < 1e-9) { e1 = Eigen::Vector3d::UnitY() - h * h.y(); }
  e1.normalize();
  const Eigen::Vector3d e2 = h.cross(e1);
  return std::cos(theta) * e1 + std::sin(theta) * e2;
}

struct ReceivingGraspPoses
{
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity(); ///< world rotation of the mouth frame
  Eigen::Vector3d capture = Eigen::Vector3d::Zero();
  Eigen::Vector3d standoff = Eigen::Vector3d::Zero();
  Eigen::Vector3d retreat = Eigen::Vector3d::Zero();
  Eigen::Vector3d contactPlus = Eigen::Vector3d::Zero();
  Eigen::Vector3d contactMinus = Eigen::Vector3d::Zero();
};

inline ReceivingGraspPoses receivingGraspPoses(const HandleGeometry & handle, const GripperInterface & gripper,
                                               const Eigen::Vector3d & reference, const ReceivingGrasp & g)
{
  ReceivingGraspPoses p;
  const Eigen::Vector3d h = handle.axis.normalized();
  const Eigen::Vector3d xM = receivingContactNormal(h, reference, g.theta);
  const Eigen::Vector3d zM = (g.sign >= 0 ? 1.0 : -1.0) * h;
  const Eigen::Vector3d yM = zM.cross(xM);
  p.rotation.col(0) = xM;
  p.rotation.col(1) = yM;
  p.rotation.col(2) = zM;
  const Eigen::Vector3d onAxis = handle.center + g.s * h;
  p.contactPlus = onAxis + handle.radius * xM;
  p.contactMinus = onAxis - handle.radius * xM;
  p.capture = onAxis + gripper.captureDepth * yM;
  p.standoff = p.capture + gripper.standoffDistance * yM;
  p.retreat = p.capture + gripper.retreatDistance * yM;
  return p;
}

/** Legacy TRIAD (sigma, phi) ring angle equivalent to (sigma, theta):
 * sigma = +1: theta = phi - pi/2;  sigma = -1: theta = pi/2 - phi. */
inline double legacyPhiFromTheta(int sign, double theta)
{
  return sign >= 0 ? theta + 0.5 * M_PI : 0.5 * M_PI - theta;
}

struct MechanicalScreen
{
  bool axialOk = false;
  bool groundOk = false;
  bool passed = false;
  std::string reason = "not_evaluated";
};

/** Necessary conditions only (no false negatives by construction): the axial
 * task bound, and the mouth centre of the capture, standoff and retreat points
 * above the ground plane (a mouth centre below the ground always collides). */
inline MechanicalScreen receivingMechanicalScreen(const ReceivingGrasp & g, const ReceivingGraspPoses & p,
                                                  double axialLimit, double groundZ)
{
  MechanicalScreen m;
  m.axialOk = std::abs(g.s) <= axialLimit + 1e-12;
  m.groundOk = p.capture.z() >= groundZ && p.standoff.z() >= groundZ && p.retreat.z() >= groundZ;
  m.passed = m.axialOk && m.groundOk;
  m.reason = !m.axialOk ? "axial_limit" : (!m.groundOk ? "mouth_below_ground" : "passed");
  return m;
}

// ---------------------------------------------------------------------------
// Gen3 wrist-point reachability SDF (necessary condition)
// ---------------------------------------------------------------------------

/** Wrist point (gen3_joint_7 origin) from a tool (gen3_robotiq_85_base_link) pose. */
inline Eigen::Vector3d gen3WristPoint(const Eigen::Matrix3d & R_W_tool, const Eigen::Vector3d & p_W_tool)
{
  using namespace gen3_wrist_reachability;
  return p_W_tool + R_W_tool * Eigen::Vector3d(kWristInToolX, kWristInToolY, kWristInToolZ);
}

/** Bilinear SDF at a wrist point expressed in the robot base frame (m, positive
 * inside the sampled reachable solid of revolution). Outside the grid the value
 * is bounded above by minus the distance to the grid. */
inline double gen3WristReachabilitySdf(const Eigen::Vector3d & p_base_wrist)
{
  using namespace gen3_wrist_reachability;
  const double rho = std::hypot(p_base_wrist.x(), p_base_wrist.y());
  const double z = p_base_wrist.z();
  const double rhoMax = kRho0 + (kNRho - 1) * kStep;
  const double zMax = kZ0 + (kNZ - 1) * kStep;
  const double outR = std::max(0.0, std::max(kRho0 - rho, rho - rhoMax));
  const double outZ = std::max(0.0, std::max(kZ0 - z, z - zMax));
  const double fr = std::min(std::max((rho - kRho0) / kStep, 0.0), static_cast<double>(kNRho - 1) - 1e-9);
  const double fz = std::min(std::max((z - kZ0) / kStep, 0.0), static_cast<double>(kNZ - 1) - 1e-9);
  const int i0 = std::min(static_cast<int>(std::floor(fr)), kNRho - 2);
  const int k0 = std::min(static_cast<int>(std::floor(fz)), kNZ - 2);
  const double a = fr - i0;
  const double b = fz - k0;
  auto S = [](int iz, int ir) { return static_cast<double>(kSdf[iz * kNRho + ir]); };
  const double v = (1 - a) * (1 - b) * S(k0, i0) + a * (1 - b) * S(k0, i0 + 1) + (1 - a) * b * S(k0 + 1, i0)
      + a * b * S(k0 + 1, i0 + 1);
  if(outR > 0.0 || outZ > 0.0) { return std::min(v, 0.0) - std::hypot(outR, outZ); }
  return v;
}

// ---------------------------------------------------------------------------
// Shortlist
// ---------------------------------------------------------------------------

struct FunnelCandidate
{
  int index = -1;       ///< index into the hypothesis vector
  int id = -1;
  int sign = 1;
  int axialIndex = 0;
  double theta = 0.0;
  double score = -std::numeric_limits<double>::infinity(); ///< reachability SDF (m)
  /** Index of the first interception event at which the surrogate admits the
   * grasp (a lower bound on its earliest encounter). 0 for an object at rest,
   * which reduces the order to the score order. */
  int eventIndex = 0;
};

/** Candidates with score >= -pruneTolerance, earliest surrogate event first,
 * then best score (ties by id).
 * Diversity pass: at most one per (sign, axial index, theta bin of width
 * thetaBinWidth); then fill remaining slots by score. K <= 0 keeps all. */
inline std::vector<int> reachabilityShortlist(std::vector<FunnelCandidate> candidates, int K, double pruneTolerance,
                                              double thetaBinWidth)
{
  candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                                  [&](const FunnelCandidate & c) { return !(c.score >= -pruneTolerance); }),
                   candidates.end());
  std::stable_sort(candidates.begin(), candidates.end(), [](const FunnelCandidate & a, const FunnelCandidate & b)
                   {
                     if(a.eventIndex != b.eventIndex) { return a.eventIndex < b.eventIndex; }
                     return a.score > b.score || (a.score == b.score && a.id < b.id);
                   });
  std::vector<int> out;
  const std::size_t limit = K <= 0 ? candidates.size() : std::min(candidates.size(), static_cast<std::size_t>(K));
  std::set<std::tuple<int, int, int>> used;
  std::vector<char> taken(candidates.size(), 0);
  for(std::size_t i = 0; i < candidates.size() && out.size() < limit; ++i)
  {
    const auto & c = candidates[i];
    const int bin = thetaBinWidth > 0.0 ? static_cast<int>(std::floor(c.theta / thetaBinWidth)) : c.id;
    if(used.insert(std::make_tuple(c.sign, c.axialIndex, bin)).second)
    {
      out.push_back(c.index);
      taken[i] = 1;
    }
  }
  for(std::size_t i = 0; i < candidates.size() && out.size() < limit; ++i)
  {
    if(!taken[i]) { out.push_back(candidates[i].index); }
  }
  return out;
}

} // namespace call_handover
