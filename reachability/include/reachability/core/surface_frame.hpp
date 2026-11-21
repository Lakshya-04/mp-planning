// The local frame of the work surface at a point: origin, outward (free-side) normal, tangents u and v.
//
// A plane has one frame everywhere; a future curved surface (column, mesh patch) can supply one frame per
// candidate point. The free-space footprint filter (free_space.hpp) does not depend on the surface type.
//
// The stand search used to build its in-plane axis as `UnitZ x n`, which is the zero vector for a
// ceiling or a floor. This is the one place that builds the frame now.
//   - Vertical (or tilted) plane: u = normalize(UnitZ x n), exactly the old axis, so every wall
//     result is unchanged. v = n x u (points up the wall); stands are searched along u only.
//   - Horizontal plane (UnitZ x n is ~0): u = +x projected into the plane, v = n x u. The base
//     stands on the floor under (or over) the patch, so stands are searched over the (u, v) area.
#pragma once

#include <Eigen/Dense>

namespace reachability
{

struct SurfaceFrame
{
  Eigen::Vector3d origin{Eigen::Vector3d::Zero()};   ///< Point on the surface the frame is taken at (zero for surfaceFrame(normal)).
  Eigen::Vector3d n{Eigen::Vector3d::Zero()};   ///< Unit normal (into the free side).
  Eigen::Vector3d u{Eigen::Vector3d::Zero()};   ///< In-plane axis 1.
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};   ///< In-plane axis 2.
  bool ok{false};                               ///< False for a zero (or non-finite) normal.
  bool horizontal{false};                       ///< The plane is a ceiling or a floor.
};

inline SurfaceFrame surfaceFrame(const Eigen::Vector3d & normal, const Eigen::Vector3d & origin = Eigen::Vector3d::Zero())
{
  SurfaceFrame f;
  f.origin = origin;
  if (!normal.allFinite() || normal.norm() < 1e-9) {return f;}
  f.n = normal.normalized();
  Eigen::Vector3d u = Eigen::Vector3d::UnitZ().cross(f.n);
  if (u.norm() < 1e-6) {
    f.horizontal = true;
    u = Eigen::Vector3d::UnitX() - Eigen::Vector3d::UnitX().dot(f.n) * f.n;
  }
  f.u = u.normalized();
  f.v = f.n.cross(f.u);
  f.ok = true;
  return f;
}

}  // namespace reachability
