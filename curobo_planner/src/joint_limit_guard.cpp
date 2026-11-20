#include "curobo_planner/joint_limit_guard.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace curobo_planner
{
namespace mp = motion_planner;

LimitCheck enforceJointLimits(core_types::Trajectory & trajectory, const JointLimitPolicy & policy)
{
  LimitCheck out;
  auto fail = [&out](const std::string & code, const std::string & message) {
      out.ok = false;
      out.reason_code = code;
      out.message = message;
      return out;
    };
  if (!policy.enabled()) {return out;}
  if (!(policy.safety_factor > 0.0 && policy.safety_factor <= 1.0)) {
    return fail("limits_invalid", "safety_factor must be in (0, 1]");
  }
  const size_t n = trajectory.joint_names.size();
  std::vector<mp::JointLimit> lim(n);
  for (size_t j = 0; j < n; ++j) {
    const auto it = policy.limits.find(trajectory.joint_names[j]);
    if (it == policy.limits.end()) {
      return fail("limit_missing", "no velocity / acceleration limit for joint '" + trajectory.joint_names[j] + "'");
    }
    lim[j] = it->second;
    if (!(lim[j].velocity > 0.0) || !(lim[j].acceleration > 0.0) || !std::isfinite(lim[j].velocity) ||
      !std::isfinite(lim[j].acceleration))
    {
      return fail("limits_invalid", "limits of joint '" + trajectory.joint_names[j] + "' must be finite and > 0");
    }
  }
  const auto & pts = trajectory.points;
  out.duration_before_sec = out.duration_after_sec = pts.empty() ? 0.0 : pts.back().time_from_start_sec;
  for (size_t i = 0; i < pts.size(); ++i) {
    if (pts[i].positions.size() != n ||
      (!pts[i].velocities.empty() && pts[i].velocities.size() != n) ||
      (!pts[i].accelerations.empty() && pts[i].accelerations.size() != n))
    {
      return fail("invalid_trajectory", "a trajectory point does not match joint_names");
    }
    if (!std::isfinite(pts[i].time_from_start_sec) || (i > 0 && !(pts[i].time_from_start_sec > pts[i - 1].time_from_start_sec))) {
      return fail("invalid_trajectory", "time_from_start_sec must be finite and strictly increasing");
    }
    for (double q : pts[i].positions) {
      if (!std::isfinite(q)) {return fail("invalid_trajectory", "non-finite position");}
    }
  }
  if (pts.size() < 2) {return out;}

  // finite differences: v on each interval, a between consecutive intervals
  std::vector<std::vector<double>> vel(pts.size() - 1, std::vector<double>(n));
  std::vector<double> dt(pts.size() - 1);
  double vmax = 0.0;
  double amax = 0.0;
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    dt[i] = pts[i + 1].time_from_start_sec - pts[i].time_from_start_sec;
    for (size_t j = 0; j < n; ++j) {
      vel[i][j] = (pts[i + 1].positions[j] - pts[i].positions[j]) / dt[i];
      vmax = std::max(vmax, std::abs(vel[i][j]) / lim[j].velocity);
    }
  }
  for (size_t i = 0; i + 2 < pts.size(); ++i) {
    const double span = 0.5 * (dt[i] + dt[i + 1]);
    for (size_t j = 0; j < n; ++j) {
      amax = std::max(amax, std::abs((vel[i + 1][j] - vel[i][j]) / span) / lim[j].acceleration);
    }
  }
  // the planner's own vectors count too (they may be tighter than the differences)
  for (const auto & p : pts) {
    for (size_t j = 0; j < p.velocities.size(); ++j) {
      vmax = std::max(vmax, std::abs(p.velocities[j]) / lim[j].velocity);
    }
    for (size_t j = 0; j < p.accelerations.size(); ++j) {
      amax = std::max(amax, std::abs(p.accelerations[j]) / lim[j].acceleration);
    }
  }
  out.max_velocity_ratio = vmax;
  out.max_acceleration_ratio = amax;
  const double f = policy.safety_factor;
  const double scale = std::max({1.0, vmax / f, std::sqrt(amax / f)});
  if (scale <= 1.0) {return out;}  // inside limits * factor: untouched

  std::ostringstream detail;
  detail << "peak velocity " << vmax << " and acceleration " << amax << " of the limits (factor " << f << ")";
  if (policy.mode == LimitMode::kReject) {
    return fail("limit_violation", "trajectory exceeds the joint limits: " + detail.str());
  }
  for (auto & p : trajectory.points) {
    p.time_from_start_sec *= scale;
    for (double & v : p.velocities) {v /= scale;}
    for (double & a : p.accelerations) {a /= (scale * scale);}
  }
  out.time_scale = scale;
  out.duration_after_sec = out.duration_before_sec * scale;
  out.message = "retimed x" + std::to_string(scale) + ": " + detail.str();
  return out;
}

bool parseMoveItJointLimits(
  const std::string & yaml_text, std::map<std::string, mp::JointLimit> & limits, std::string & error,
  double * velocity_scaling, double * acceleration_scaling)
{
  try {
    const YAML::Node root = YAML::Load(yaml_text);
    const YAML::Node jl = root["joint_limits"];
    if (!jl || !jl.IsMap()) {
      error = "no joint_limits map";
      return false;
    }
    if (velocity_scaling) {
      *velocity_scaling = root["default_velocity_scaling_factor"] ? root["default_velocity_scaling_factor"].as<double>() : 1.0;
    }
    if (acceleration_scaling) {
      *acceleration_scaling =
        root["default_acceleration_scaling_factor"] ? root["default_acceleration_scaling_factor"].as<double>() : 1.0;
    }
    limits.clear();
    for (const auto & kv : jl) {
      const YAML::Node & v = kv.second;
      const bool has_v = v["has_velocity_limits"] && v["has_velocity_limits"].as<bool>() && v["max_velocity"];
      const bool has_a = v["has_acceleration_limits"] && v["has_acceleration_limits"].as<bool>() && v["max_acceleration"];
      if (!has_v || !has_a) {continue;}
      mp::JointLimit l;
      l.velocity = v["max_velocity"].as<double>();
      l.acceleration = v["max_acceleration"].as<double>();
      limits[kv.first.as<std::string>()] = l;
    }
    return true;
  } catch (const YAML::Exception & e) {
    error = std::string("invalid joint limits YAML: ") + e.what();
    return false;
  }
}

bool loadMoveItJointLimits(
  const std::string & path, std::map<std::string, mp::JointLimit> & limits, std::string & error,
  double * velocity_scaling, double * acceleration_scaling)
{
  std::ifstream in(path);
  if (!in) {
    error = "cannot read joint limits file '" + path + "'";
    return false;
  }
  std::stringstream ss;
  ss << in.rdbuf();
  if (!parseMoveItJointLimits(ss.str(), limits, error, velocity_scaling, acceleration_scaling)) {
    error += " (" + path + ")";
    return false;
  }
  return true;
}

bool parseInlineJointLimits(const std::string & text, std::map<std::string, mp::JointLimit> & limits, std::string & error)
{
  std::stringstream entries(text);
  std::string entry;
  while (std::getline(entries, entry, ',')) {
    if (entry.find_first_not_of(" \t") == std::string::npos) {continue;}
    std::stringstream parts(entry);
    std::string name, v, a, extra;
    std::getline(parts, name, ':');
    std::getline(parts, v, ':');
    std::getline(parts, a, ':');
    const bool extra_part = static_cast<bool>(std::getline(parts, extra, ':'));
    const auto trim = [](std::string s) {
        const auto b = s.find_first_not_of(" \t");
        const auto e = s.find_last_not_of(" \t");
        return b == std::string::npos ? std::string() : s.substr(b, e - b + 1);
      };
    name = trim(name);
    mp::JointLimit l;
    try {
      size_t used_v = 0, used_a = 0;
      l.velocity = std::stod(trim(v), &used_v);
      l.acceleration = std::stod(trim(a), &used_a);
      if (used_v != trim(v).size() || used_a != trim(a).size()) {throw std::invalid_argument("trailing");}
    } catch (const std::exception &) {
      error = "joint limit entry '" + entry + "' is not name:max_velocity:max_acceleration";
      return false;
    }
    if (name.empty() || extra_part || !(l.velocity > 0.0) || !(l.acceleration > 0.0) || !std::isfinite(l.velocity) ||
      !std::isfinite(l.acceleration))
    {
      error = "joint limit entry '" + entry + "' needs a name and finite limits > 0";
      return false;
    }
    limits[name] = l;
  }
  return true;
}

}  // namespace curobo_planner
