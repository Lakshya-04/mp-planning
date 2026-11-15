#include "reachability/core/base_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace reachability
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

double wrap(double a)
{
  while (a > kPi) {a -= 2.0 * kPi;}
  while (a < -kPi) {a += 2.0 * kPi;}
  return a;
}

Eigen::Matrix2d rot(double yaw)
{
  Eigen::Matrix2d r;
  r << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
  return r;
}

DriveType driveFrom(const std::string & s)
{
  if (s == "omni") {return DriveType::kOmni;}
  if (s == "diff") {return DriveType::kDiff;}
  if (s == "skid") {return DriveType::kSkid;}
  if (s == "4ws") {return DriveType::kFourWheelSteer;}
  throw std::runtime_error("unknown drive_type '" + s + "' (omni | diff | skid | 4ws)");
}
}  // namespace

BaseModel loadBaseProfile(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node b = root["base"];
  if (!b || !b.IsMap()) {
    throw std::runtime_error(path + ": no `base:` section");
  }
  BaseModel m;
  m.name = b["name"] ? b["name"].as<std::string>() : "unnamed";
  if (!b["drive_type"]) {throw std::runtime_error(path + ": base.drive_type missing");}
  m.drive = driveFrom(b["drive_type"].as<std::string>());
  // Omni and four-wheel-steer bases strafe by default; diff and skid-steer bases cannot.
  const bool strafing_drive = m.drive == DriveType::kOmni || m.drive == DriveType::kFourWheelSteer;
  m.can_strafe = b["can_strafe"] ? b["can_strafe"].as<bool>() : strafing_drive;
  if (m.can_strafe && !strafing_drive) {
    throw std::runtime_error(path + ": a diff or skid base cannot strafe (can_strafe: true)");
  }
  const YAML::Node fp = b["footprint"];
  if (!fp || !fp.IsSequence() || fp.size() < 3) {
    throw std::runtime_error(path + ": base.footprint needs at least 3 [x, y] points");
  }
  for (const auto & p : fp) {
    if (!p.IsSequence() || p.size() != 2) {throw std::runtime_error(path + ": footprint points are [x, y]");}
    m.footprint.emplace_back(p[0].as<double>(), p[1].as<double>());
  }
  if (b["arm_mount_xy"]) {
    m.arm_mount = Eigen::Vector2d(b["arm_mount_xy"][0].as<double>(), b["arm_mount_xy"][1].as<double>());
  }
  if (b["lift_mount_height_m"]) {m.lift_mount_height = b["lift_mount_height_m"].as<double>();}
  if (b["max_linear_velocity_mps"]) {m.max_linear = b["max_linear_velocity_mps"].as<double>();}
  if (b["max_angular_velocity_radps"]) {m.max_angular = b["max_angular_velocity_radps"].as<double>();}
  if (!(m.max_linear > 0.0) || !(m.max_angular > 0.0)) {
    throw std::runtime_error(path + ": base velocity limits must be positive");
  }
  return m;
}

std::optional<double> armBaseHeightAtLiftZero(const BaseModel & base)
{
  constexpr double kLiftColumnHeight = 0.8;   // telescopic_lift.xacro column_height
  constexpr double kArmMountZ = 0.02;         // telescopic_lift.xacro arm_mount_xyz z
  if (!base.lift_mount_height) {return std::nullopt;}
  return *base.lift_mount_height + kLiftColumnHeight + kArmMountZ;
}

Eigen::Vector2d baseCentre(const BaseModel & base, const Stand & stand)
{
  return stand.arm_xy - rot(stand.yaw) * base.arm_mount;
}

std::vector<Eigen::Vector2d> footprintWorld(const BaseModel & base, const Stand & stand)
{
  const Eigen::Vector2d c = baseCentre(base, stand);
  const Eigen::Matrix2d r = rot(stand.yaw);
  std::vector<Eigen::Vector2d> out;
  out.reserve(base.footprint.size());
  for (const auto & p : base.footprint) {out.push_back(c + r * p);}
  return out;
}

double wallClearance(
  const BaseModel & base, const Stand & stand, const Eigen::Vector2d & wall_point,
  const Eigen::Vector2d & wall_normal)
{
  const Eigen::Vector2d n = wall_normal.normalized();
  double best = std::numeric_limits<double>::infinity();
  for (const auto & p : footprintWorld(base, stand)) {best = std::min(best, (p - wall_point).dot(n));}
  return best;
}

double bestYaw(
  const BaseModel & base, const Eigen::Vector2d & arm_xy, const Eigen::Vector2d & wall_point,
  const Eigen::Vector2d & wall_normal, double * clearance, int steps)
{
  const Eigen::Vector2d n = wall_normal.normalized();
  const double facing = std::atan2(-n.y(), -n.x());   // +x of the base pointing at the wall
  double best_yaw = facing, best_c = -std::numeric_limits<double>::infinity();
  for (int k = 0; k < std::max(steps, 1); ++k) {
    const double yaw = wrap(facing + 2.0 * kPi * k / std::max(steps, 1));
    const double c = wallClearance(base, {arm_xy, yaw}, wall_point, n);
    if (c > best_c + 1e-9) {      // strict: ties keep the first, which is the facing yaw
      best_c = c;
      best_yaw = yaw;
    }
  }
  if (clearance) {*clearance = best_c;}
  return best_yaw;
}

double travelTime(const BaseModel & base, const Stand & from, const Stand & to)
{
  const Eigen::Vector2d d = baseCentre(base, to) - baseCentre(base, from);
  const double dist = d.norm();
  const double turn_final = std::abs(wrap(to.yaw - from.yaw));
  if (base.can_strafe) {
    // Omni wheels translate and spin together. Four-wheel steering crabs or spins but does not do
    // both at once, so the two add up.
    if (base.drive == DriveType::kFourWheelSteer) {
      return dist / base.max_linear + turn_final / base.max_angular;
    }
    return std::max(dist / base.max_linear, turn_final / base.max_angular);
  }
  if (dist < 1e-3) {return turn_final / base.max_angular;}
  const double heading = std::atan2(d.y(), d.x());
  double best = std::numeric_limits<double>::infinity();
  for (const double h : {heading, wrap(heading + kPi)}) {      // forward, or reversing
    const double t = (std::abs(wrap(h - from.yaw)) + std::abs(wrap(to.yaw - h))) / base.max_angular +
      dist / base.max_linear;
    best = std::min(best, t);
  }
  return best;
}

std::vector<size_t> orderStands(
  const BaseModel & base, const std::vector<Stand> & stands, const Stand & start)
{
  std::vector<size_t> order, left(stands.size());
  for (size_t i = 0; i < left.size(); ++i) {left[i] = i;}
  Stand at = start;
  while (!left.empty()) {
    auto next = std::min_element(left.begin(), left.end(), [&](size_t a, size_t b) {
        return travelTime(base, at, stands[a]) < travelTime(base, at, stands[b]);
      });
    order.push_back(*next);
    at = stands[*next];
    left.erase(next);
  }
  return order;
}

}  // namespace reachability
