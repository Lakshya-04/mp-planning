// RM4D: a combined reachability and inverse reachability map reduced to four dimensions.
//
// Method: Rudorfer, "RM4D: A Combined Reachability and Inverse Reachability Map for Common
// 6-/7-Axis Robot Arms by Dimensionality Reduction to 4D", ICRA 2025, arXiv:2410.06968.
// Implemented here from the paper's idea; ROS-free (C++17 + Eigen).
//
// Idea. For an arm whose first joint rotates a full turn about the vertical axis and whose last
// joint rotates a full turn about the tool axis, the set of reachable tool poses is invariant
// under (1) rotating the whole arm about the base z axis and (2) rotating the tool about its own
// approach axis. A pose therefore reduces to four numbers:
//   pz     height of the tool above the arm base,
//   theta  polar angle of the tool approach axis from vertical (0 = pointing up, pi = down),
//   x*, y* tool position after rotating about z so the approach axis's azimuth is the +x axis.
// One map of these cells answers both questions:
//   forward  - can this arm reach that pose (given in the arm base frame)?
//   inverse  - where must the base stand to reach a pose given in the world frame?
//
// Assumptions (the paper reports that accuracy holds even when they are only partly met): the
// first joint has a full 360 degree range and the last joint rotates fully.
#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Geometry>

namespace reachability
{

using Vec3 = Eigen::Vector3d;
using Pose = Eigen::Isometry3d;

struct MapParams
{
  double xy_extent{1.8};   ///< Half-width of the (x*, y*) window around the base (m): cover the arm's reach.
  double z_min{-1.0};      ///< Tool height range relative to the arm base (m).
  double z_max{1.8};
  double resolution{0.05};  ///< Cell edge for x*, y* and z (m).
  double theta_bin{0.2617993877991494};  ///< Polar-angle bin (rad), pi / 12 by default.

  bool operator==(const MapParams & o) const
  {
    return xy_extent == o.xy_extent && z_min == o.z_min && z_max == o.z_max &&
           resolution == o.resolution && theta_bin == o.theta_bin;
  }
};

/// Cell address. `valid` is false when the pose falls outside the map window.
struct CellKey
{
  int iz{0};
  int itheta{0};
  int ix{0};
  int iy{0};
  bool valid{false};
};

/// A place the base can stand so that a world pose is reachable.
struct BaseCandidate
{
  double x{0.0};
  double y{0.0};
  uint8_t count{0};  ///< Number of sampled arm configurations in the cell (a quality proxy).
};

class Map4D
{
public:
  explicit Map4D(const MapParams & params = {});

  const MapParams & params() const {return params_;}
  int nz() const {return nz_;}
  int ntheta() const {return ntheta_;}
  int nxy() const {return nxy_;}
  size_t totalCells() const {return cells_.size();}
  size_t nonzeroCells() const;

  /// Canonical cell of a tool pose given in the arm base frame (tool z axis = approach axis).
  CellKey keyOf(const Pose & tool_in_arm_base) const;

  /// Records one reachable tool pose (arm base frame). Counts saturate at 255.
  void record(const Pose & tool_in_arm_base);
  void recordKey(const CellKey & key);

  /// Forward query: is the pose (arm base frame) reachable?
  bool isReachable(const Pose & tool_in_arm_base) const;
  uint8_t cellCount(const Pose & tool_in_arm_base) const;

  /// Inverse query: base positions (world x, y) from which `tool_in_world` is reachable when the
  /// arm base sits `arm_base_z` above the world origin plane. Base yaw is free (the arm's first
  /// joint absorbs it). For a vertical approach axis the azimuth is undefined, so candidates on a
  /// ring around the tool are returned (`ring_samples` per radius).
  std::vector<BaseCandidate> inverseQuery(
    const Pose & tool_in_world, double arm_base_z, int ring_samples = 24) const;

  /// Adds the counts of other maps built with the same parameters (parallel build shards).
  void mergeFrom(const Map4D & other);

  const std::vector<uint8_t> & raw() const {return cells_;}
  std::vector<uint8_t> & raw() {return cells_;}
  size_t index(int iz, int itheta, int ix, int iy) const;

private:
  MapParams params_;
  int nz_{0};
  int ntheta_{0};
  int nxy_{0};
  std::vector<uint8_t> cells_;
};

}  // namespace reachability
