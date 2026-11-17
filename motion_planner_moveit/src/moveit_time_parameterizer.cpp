#include "motion_planner_moveit/moveit_time_parameterizer.hpp"

#include <algorithm>
#include <exception>
#include <unordered_map>

#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>

namespace motion_planner
{

MoveItTimeParameterizer::MoveItTimeParameterizer(
  std::shared_ptr<const moveit::core::RobotModel> model, std::string group, std::string tcp_link)
: model_(std::move(model)), group_(std::move(group)), tcp_link_(std::move(tcp_link))
{
  const auto * jmg = model_ ? model_->getJointModelGroup(group_) : nullptr;
  if (jmg && tcp_link_.empty() && !jmg->getActiveJointModels().empty()) {
    tcp_link_ = jmg->getActiveJointModels().back()->getChildLinkModel()->getName();
  }
}

std::vector<std::string> MoveItTimeParameterizer::jointNames() const
{
  const auto * jmg = model_ ? model_->getJointModelGroup(group_) : nullptr;
  return jmg ? jmg->getActiveJointModelNames() : std::vector<std::string>{};
}

std::vector<JointLimit> MoveItTimeParameterizer::modelLimits(double default_acc, double jerk) const
{
  std::vector<JointLimit> out;
  const auto * jmg = model_ ? model_->getJointModelGroup(group_) : nullptr;
  if (!jmg) {
    return out;
  }
  for (const auto * joint : jmg->getActiveJointModels()) {
    const auto & b = joint->getVariableBounds(joint->getName());
    JointLimit l;
    l.velocity = b.velocity_bounded_ ? b.max_velocity_ : 0.0;
    l.acceleration = b.acceleration_bounded_ ? b.max_acceleration_ : default_acc;
    l.jerk = jerk;
    out.push_back(l);
  }
  return out;
}

bool MoveItTimeParameterizer::retime(
  core_types::Trajectory & piece, SegmentRole role, const std::vector<JointLimit> & limits,
  double stroke_speed, std::string & message)
{
  const auto * jmg = model_ ? model_->getJointModelGroup(group_) : nullptr;
  if (!jmg) {
    message = "unknown planning group '" + group_ + "'";
    return false;
  }
  const auto & names = jmg->getActiveJointModelNames();
  const size_t n = names.size();
  if (piece.joint_names.size() != n || limits.size() != n) {
    message = "joint count differs from the planning group";
    return false;
  }
  // index of each group joint in the piece
  std::vector<size_t> src(n);
  for (size_t j = 0; j < n; ++j) {
    const auto it = std::find(piece.joint_names.begin(), piece.joint_names.end(), names[j]);
    if (it == piece.joint_names.end()) {
      message = "joint '" + names[j] + "' missing from the trajectory";
      return false;
    }
    src[j] = static_cast<size_t>(it - piece.joint_names.begin());
  }
  std::unordered_map<std::string, double> vel, acc, jerk;
  for (size_t j = 0; j < n; ++j) {
    const auto & l = limits[src[j]];
    vel[names[j]] = l.velocity;
    acc[names[j]] = l.acceleration;
    jerk[names[j]] = l.jerk;
  }

  auto toGroup = [&](const std::vector<double> & v) {
      std::vector<double> g(n);
      for (size_t j = 0; j < n; ++j) {
        g[j] = v[src[j]];
      }
      return g;
    };
  auto tcpOf = [&](const std::vector<double> & q) -> Eigen::Vector3d {
      moveit::core::RobotState s(model_);
      s.setToDefaultValues();
      s.setJointGroupPositions(jmg, toGroup(q));
      s.update();
      return s.getGlobalLinkTransform(tcp_link_).translation();
    };

  try {
    if (role == SegmentRole::kContactSeam) {
      for (const auto & l : limits) {
        if (l.jerk <= 0.0) {
          message = "a contact seam needs a jerk limit on every joint";
          return false;
        }
      }
      constantSpeedStamps(
        piece, stroke_speed,
        [&](const std::vector<double> & a, const std::vector<double> & b) {
          return (tcpOf(b) - tcpOf(a)).norm();
        });
    }

    robot_trajectory::RobotTrajectory rt(model_, group_);
    double previous_t = 0.0;
    for (const auto & p : piece.points) {
      auto s = std::make_shared<moveit::core::RobotState>(model_);
      s->setToDefaultValues();
      s->setJointGroupPositions(jmg, toGroup(p.positions));
      const bool at_end = &p == &piece.points.front() || &p == &piece.points.back();
      if (role == SegmentRole::kContactSeam && p.velocities.size() == n && !at_end) {   // rest at both ends
        s->setJointGroupVelocities(jmg, toGroup(p.velocities));
      }
      s->update();
      const double dt = role == SegmentRole::kContactSeam ? p.time_from_start_sec - previous_t : 0.0;
      previous_t = p.time_from_start_sec;
      rt.addSuffixWayPoint(s, rt.empty() ? 0.0 : dt);
    }

    bool ok = false;
    if (role == SegmentRole::kTransition) {
      trajectory_processing::TimeOptimalTrajectoryGeneration totg(0.1, 0.05);
      ok = totg.computeTimeStamps(rt, vel, acc, 1.0, 1.0);
    } else {
      ok = trajectory_processing::RuckigSmoothing::applySmoothing(
        rt, vel, acc, jerk, 1.0, 1.0, true, 0.01);
    }
    if (!ok) {
      message = role == SegmentRole::kTransition ? "TOTG failed" : "Ruckig smoothing failed";
      return false;
    }

    core_types::Trajectory out;
    out.joint_names = piece.joint_names;
    std::vector<double> g(n);
    for (size_t i = 0; i < rt.getWayPointCount(); ++i) {
      const auto & s = rt.getWayPoint(i);
      core_types::TrajectoryPoint p;
      p.positions.assign(n, 0.0);
      p.velocities.assign(n, 0.0);
      p.accelerations.assign(n, 0.0);
      s.copyJointGroupPositions(jmg, g);
      for (size_t j = 0; j < n; ++j) {
        p.positions[src[j]] = g[j];
      }
      if (s.hasVelocities()) {
        s.copyJointGroupVelocities(jmg, g);
        for (size_t j = 0; j < n; ++j) {
          p.velocities[src[j]] = g[j];
        }
      }
      if (s.hasAccelerations()) {
        s.copyJointGroupAccelerations(jmg, g);
        for (size_t j = 0; j < n; ++j) {
          p.accelerations[src[j]] = g[j];
        }
      }
      p.time_from_start_sec = rt.getWayPointDurationFromStart(i);
      out.points.push_back(std::move(p));
    }
    if (out.points.size() < 2) {
      message = "retimer returned fewer than 2 points";
      return false;
    }
    piece = std::move(out);
  } catch (const std::exception & e) {
    message = e.what();
    return false;
  }
  return true;
}

}  // namespace motion_planner
