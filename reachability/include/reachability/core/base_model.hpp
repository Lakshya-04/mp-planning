// The mobile base as the stationing layer sees it: footprint, where the arm stands on it, and how
// it can move. Loaded from the `base:` section of a robot profile (mobile_manipulator_description).
//
// The reachability map is indexed by the ARM base position, but the thing that must stay clear of
// a wall is the whole footprint, and which yaw it takes is free at a stand for every drive type.
// Between stands the drive type matters: a base that can strafe slides along the wall, one that
// cannot has to turn, drive and turn back.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace reachability
{

enum class DriveType { kOmni, kDiff, kSkid, kFourWheelSteer };

struct BaseModel
{
  std::string name{"generic"};
  DriveType drive{DriveType::kOmni};
  bool can_strafe{true};
  std::vector<Eigen::Vector2d> footprint;  ///< Polygon in base_footprint (x forward, y left), m.
  Eigen::Vector2d arm_mount{0.0, 0.0};     ///< Arm base position in base_footprint (m).
  std::optional<double> lift_mount_height;  ///< lift_mount_link above the ground (m), when the profile has it.
  double max_linear{1.0};                  ///< m/s
  double max_angular{1.0};                 ///< rad/s
};

/// Where the base is: the arm base position in the world plane and the base yaw.
struct Stand
{
  Eigen::Vector2d arm_xy{0.0, 0.0};
  double yaw{0.0};
};

/// Reads the `base:` section of a profile YAML (drive_type, can_strafe, footprint,
/// max_*_velocity_*, optional arm_mount_xy). Throws std::runtime_error when it is missing or invalid.
BaseModel loadBaseProfile(const std::string & path);

/// Height of the arm base above the ground with the lift at zero stroke: the base's
/// `lift_mount_height_m` plus the telescopic lift's fixed column (0.8 m) and arm mount (0.02 m),
/// the defaults of urdf/lift/telescopic_lift.xacro. Pass it as `arm_mount_offset_z`. Empty when the
/// profile carries no mount height.
std::optional<double> armBaseHeightAtLiftZero(const BaseModel & base);

Eigen::Vector2d baseCentre(const BaseModel & base, const Stand & stand);
std::vector<Eigen::Vector2d> footprintWorld(const BaseModel & base, const Stand & stand);

/// Distance from the wall line to the nearest footprint point, along the wall normal (the normal
/// points into the free side). Negative if the footprint crosses the wall.
double wallClearance(
  const BaseModel & base, const Stand & stand, const Eigen::Vector2d & wall_point,
  const Eigen::Vector2d & wall_normal);

/// The yaw (sampled in 2*pi/steps) that leaves the most clearance for an arm base at `arm_xy`;
/// ties go to the yaw that faces the wall. `clearance` receives the value.
double bestYaw(
  const BaseModel & base, const Eigen::Vector2d & arm_xy, const Eigen::Vector2d & wall_point,
  const Eigen::Vector2d & wall_normal, double * clearance = nullptr, int steps = 24);

/// Time (s) to drive the base from one stand to the next. Omni bases translate and turn
/// together; four-wheel-steer bases crab or spin one after the other; the others turn toward the goal (or away from it, to reverse), drive, then turn to the
/// final yaw.
double travelTime(const BaseModel & base, const Stand & from, const Stand & to);

/// Visiting order over `stands` (indices), greedy nearest by travelTime starting at `start`.
std::vector<size_t> orderStands(
  const BaseModel & base, const std::vector<Stand> & stands, const Stand & start);

}  // namespace reachability
