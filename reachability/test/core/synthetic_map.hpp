#pragma once

#include <cmath>

#include "reachability/core/map4d.hpp"

namespace reachability_test
{

/// A hand-made map, no arm behind it: for tool poses whose approach axis is +x (the tool points at a wall on
/// the +x side) every cell with x* in [x_lo, x_hi), y* in [-y_half, y_half] and z in [z_lo, z_hi] (tool
/// height above the arm base) holds `count`; the outer `edge_band` of x* (towards x_hi) holds `edge_count`.
/// Everything else is empty. Lets a test say exactly which pose is reachable and how strongly.
inline reachability::Pose towardPlusX(double x, double y, double z)
{
  reachability::Pose T = reachability::Pose::Identity();
  T.translation() = reachability::Vec3(x, y, z);
  T.linear() = Eigen::AngleAxisd(M_PI / 2, reachability::Vec3::UnitY()).toRotationMatrix();
  return T;
}

struct SyntheticReach
{
  double x_lo{0.30}, x_hi{0.90};
  double y_half{0.80};
  double z_lo{-0.50}, z_hi{0.70};
  int count{20};
  double edge_band{0.05};
  int edge_count{1};
};

inline reachability::Map4D syntheticMap(const SyntheticReach & r = {})
{
  reachability::Map4D map;   // default params: extent 1.8, z -1..1.8, 0.05 m cells
  const auto & p = map.params();
  const int itheta = map.keyOf(towardPlusX(0.5, 0.0, 0.0)).itheta;
  for (int iz = 0; iz < map.nz(); ++iz) {
    const double z = p.z_min + (iz + 0.5) * p.resolution;
    if (z < r.z_lo || z > r.z_hi) {continue;}
    for (int ix = 0; ix < map.nxy(); ++ix) {
      const double x = -p.xy_extent + (ix + 0.5) * p.resolution;
      if (x < r.x_lo || x >= r.x_hi) {continue;}
      for (int iy = 0; iy < map.nxy(); ++iy) {
        const double y = -p.xy_extent + (iy + 0.5) * p.resolution;
        if (std::fabs(y) > r.y_half) {continue;}
        map.raw()[map.index(iz, itheta, ix, iy)] =
          static_cast<uint8_t>(x >= r.x_hi - r.edge_band ? r.edge_count : r.count);
      }
    }
  }
  return map;
}

/// A hand-made map for a horizontal plane: a tool whose approach axis is vertical (pointing up for a
/// ceiling, down for a floor or table top) at `z` above the arm base is reachable at horizontal radii
/// r_lo..r_hi (the azimuth of a vertical approach is free, so one azimuth is recorded).
inline reachability::Map4D syntheticRingMap(double r_lo, double r_hi, double z, bool tool_up)
{
  reachability::Map4D map;
  for (double r = r_lo; r <= r_hi + 1e-9; r += 0.05) {
    reachability::Pose T = reachability::Pose::Identity();
    T.translation() = reachability::Vec3(r + 0.025, 0.0, z);
    if (!tool_up) {T.linear() = Eigen::AngleAxisd(M_PI, reachability::Vec3::UnitX()).toRotationMatrix();}
    map.record(T);
  }
  return map;
}

}  // namespace reachability_test
