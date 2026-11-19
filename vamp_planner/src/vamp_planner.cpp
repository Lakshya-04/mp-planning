#include "vamp_planner/vamp_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <Eigen/Geometry>
#include <vamp/collision/capt.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/factory.hh>

#include "core_types/null_logger.hpp"
#include "robot_backend.hpp"

namespace vamp_planner
{
namespace mp = motion_planner;

namespace
{
mp::PlanningResult fail(const std::string & code, const std::string & message, mp::FailureStage stage)
{
  mp::PlanningResult r;
  r.status = "failed";
  r.message = message;
  r.failure_reason_code = code;
  r.failure_stage = stage;
  return r;
}

Eigen::Isometry3f toIso(const core_types::Pose & p)
{
  Eigen::Quaternionf q(
    static_cast<float>(p.qw), static_cast<float>(p.qx), static_cast<float>(p.qy), static_cast<float>(p.qz));
  q = q.norm() < 1e-6F ? Eigen::Quaternionf::Identity() : q.normalized();
  Eigen::Isometry3f t = Eigen::Isometry3f::Identity();
  t.linear() = q.toRotationMatrix();
  t.translation() = Eigen::Vector3f(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
  return t;
}
}  // namespace

struct VampPlanner::Impl
{
  Options options;
  std::shared_ptr<core_types::Logger> logger;
  std::unique_ptr<RobotBackend> robot;
  vamp::collision::Environment<float> env;
  bool world_set{false};
};

VampPlanner::VampPlanner(const Options & options, std::shared_ptr<core_types::Logger> logger)
: impl_(std::make_unique<Impl>())
{
  impl_->options = options;
  impl_->logger = logger ? std::move(logger) : std::make_shared<core_types::NullLogger>();
  impl_->robot = makeBackend(options.robot);
  if (!impl_->robot) {
    impl_->logger->warn("vamp_planner: robot '" + options.robot + "' is not compiled in");
  }
}

VampPlanner::~VampPlanner() = default;

std::vector<std::string> VampPlanner::availableRobots() {return backendNames();}
bool VampPlanner::ok() const {return impl_->robot != nullptr;}
size_t VampPlanner::dimension() const {return impl_->robot ? impl_->robot->dimension() : 0;}
std::vector<std::string> VampPlanner::jointNames() const
{
  return impl_->robot ? impl_->robot->jointNames() : std::vector<std::string>{};
}
bool VampPlanner::hasWorld() const {return impl_->world_set;}

void VampPlanner::clearWorld()
{
  impl_->env = vamp::collision::Environment<float>();
  impl_->world_set = false;
}

void VampPlanner::setWorld(const mp::WorldGeometryUpdate & world)
{
  vamp::collision::Environment<float> env;
  for (const auto & b : world.boxes) {
    const Eigen::Quaternionf q(
      static_cast<float>(b.orientation_w), static_cast<float>(b.orientation_x),
      static_cast<float>(b.orientation_y), static_cast<float>(b.orientation_z));
    const Eigen::Matrix3f R = (q.norm() < 1e-6F ? Eigen::Quaternionf::Identity() : q.normalized()).toRotationMatrix();
    env.cuboids.emplace_back(
      static_cast<float>(b.center_x), static_cast<float>(b.center_y), static_cast<float>(b.center_z),
      R(0, 0), R(1, 0), R(2, 0), R(0, 1), R(1, 1), R(2, 1), R(0, 2), R(1, 2), R(2, 2),
      static_cast<float>(b.size_x) / 2, static_cast<float>(b.size_y) / 2, static_cast<float>(b.size_z) / 2);
  }
  if (!world.points.empty() && impl_->robot) {
    const Eigen::Isometry3f T = toIso(world.points_origin);
    std::vector<vamp::collision::Point> pts;
    pts.reserve(world.points.size());
    for (const auto & p : world.points) {
      const Eigen::Vector3f w = T * Eigen::Vector3f(
        static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
      pts.push_back({w.x(), w.y(), w.z()});
    }
    env.pointclouds.emplace_back(
      pts, impl_->robot->minRadius(), impl_->robot->maxRadius(), static_cast<float>(impl_->options.point_radius));
  }
  env.sort();
  impl_->env = std::move(env);
  impl_->world_set = true;
}

void VampPlanner::setBaseFrame(const std::string & frame) {impl_->options.base_frame = frame;}

bool VampPlanner::updateWorldGeometry(const mp::WorldGeometryUpdate & update, std::string & error_message)
{
  error_message.clear();
  if (!impl_->options.base_frame.empty() && !update.frame_id.empty() &&
    update.frame_id != impl_->options.base_frame)
  {
    error_message = "world frame '" + update.frame_id + "' differs from the planning base frame '" +
      impl_->options.base_frame + "' and VAMP cannot transform it";
    return false;
  }
  setWorld(update);
  return true;
}

bool VampPlanner::clearWorldGeometry(std::string & error_message)
{
  error_message.clear();
  clearWorld();
  return true;
}

mp::PlanningResult VampPlanner::plan(const mp::PlanningRequest & request)
{
  const auto t0 = std::chrono::steady_clock::now();
  if (!impl_->robot) {
    return fail("unsupported", "robot '" + impl_->options.robot + "' is not compiled in", mp::FailureStage::kPrecheck);
  }
  const size_t n = impl_->robot->dimension();
  if (request.start_joint_state.size() != n || request.target_joint_state.size() != n) {
    return fail(
      "invalid_request",
      "start_joint_state and target_joint_state must both have " + std::to_string(n) + " values",
      mp::FailureStage::kPrecheck);
  }
  if (impl_->options.require_world && !impl_->world_set) {
    return fail("no_world", "no world loaded: refusing to plan blind", mp::FailureStage::kPrecheck);
  }
  if (!impl_->robot->stateValid(request.start_joint_state, impl_->env)) {
    return fail("start_in_collision", "start state is in collision", mp::FailureStage::kPrecheck);
  }
  if (!impl_->robot->stateValid(request.target_joint_state, impl_->env)) {
    return fail("goal_in_collision", "target state is in collision", mp::FailureStage::kPrecheck);
  }

  std::vector<std::vector<double>> path;
  if (impl_->robot->motionValid(request.start_joint_state, request.target_joint_state, impl_->env)) {
    path = {request.start_joint_state, request.target_joint_state};  // straight line is free: skip the planner
  } else {
    PlanSettings s;
    s.max_iterations = impl_->options.max_iterations;
    s.range = static_cast<float>(impl_->options.range);
    s.simplify = impl_->options.simplify;
    const auto raw = impl_->robot->plan(request.start_joint_state, request.target_joint_state, impl_->env, s);
    if (!raw.found) {
      auto r = fail("no_path", "no collision-free path within the iteration budget", mp::FailureStage::kPlanning);
      r.planning_time_ms = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - t0).count());
      return r;
    }
    path = raw.waypoints;
  }

  mp::PlanningResult r;
  r.success = true;
  r.status = "success";
  r.path_fraction = 1.0;
  r.waypoints_completed = request.waypoint_indices.size();
  r.trajectory.joint_names = impl_->robot->jointNames();
  const double v = std::max(1e-3, impl_->options.nominal_joint_speed);
  double t = 0.0;
  for (size_t i = 0; i < path.size(); ++i) {
    if (i > 0) {
      double dmax = 0.0;
      for (size_t j = 0; j < n; ++j) {
        dmax = std::max(dmax, std::abs(path[i][j] - path[i - 1][j]));
      }
      t += dmax / v;
    }
    core_types::TrajectoryPoint p;
    p.positions = path[i];
    p.time_from_start_sec = t;
    r.trajectory.points.push_back(std::move(p));
  }
  r.planning_time_ms = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - t0).count());
  return r;
}

mp::IkResult VampPlanner::ik(const mp::IkRequest &)
{
  mp::IkResult r;
  r.message = "VAMP does not solve IK; use an IK backend";
  r.failure_reason_code = "unsupported";
  r.failure_stage = mp::FailureStage::kPrecheck;
  return r;
}

mp::CollisionResult VampPlanner::collision(const mp::CollisionRequest & request)
{
  mp::CollisionResult r;
  r.failure_stage = mp::FailureStage::kPrecheck;
  if (!impl_->robot) {
    r.message = "robot is not compiled in";
    r.failure_reason_code = "unsupported";
    return r;
  }
  const size_t n = impl_->robot->dimension();
  if (request.joint_values.empty() && request.trajectory_joint_values.empty()) {
    r.message = "need joint_values or trajectory_joint_values";
    r.failure_reason_code = "invalid_request";
    return r;
  }
  auto sized = [n](const std::vector<double> & q) {return q.size() == n;};
  bool free = true;
  if (!request.joint_values.empty()) {
    if (!sized(request.joint_values)) {
      r.message = "joint_values has the wrong length";
      r.failure_reason_code = "invalid_request";
      return r;
    }
    free = impl_->robot->stateValid(request.joint_values, impl_->env);
  }
  const auto & traj = request.trajectory_joint_values;
  for (size_t i = 0; i < traj.size() && free; ++i) {
    if (!sized(traj[i])) {
      r.message = "trajectory_joint_values[" + std::to_string(i) + "] has the wrong length";
      r.failure_reason_code = "invalid_request";
      return r;
    }
    free = (i == 0) ? impl_->robot->stateValid(traj[0], impl_->env) :
      impl_->robot->motionValid(traj[i - 1], traj[i], impl_->env);
  }
  r.success = true;
  r.collision_free = free;
  r.failure_stage = mp::FailureStage::kNone;
  return r;
}

}  // namespace vamp_planner
