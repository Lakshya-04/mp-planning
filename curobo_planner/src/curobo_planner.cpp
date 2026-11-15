#include "curobo_planner/curobo_planner.hpp"

#include <chrono>
#include <cstring>
#include <future>

#include <Eigen/Geometry>
#include <curobo_bridge_msgs/msg/box.hpp>
#include <curobo_bridge_msgs/srv/check_motion.hpp>
#include <curobo_bridge_msgs/srv/plan_joint.hpp>
#include <curobo_bridge_msgs/srv/plan_pose.hpp>
#include <curobo_bridge_msgs/srv/set_esdf.hpp>
#include <curobo_bridge_msgs/srv/set_world.hpp>
#include <curobo_bridge_msgs/srv/solve_ik.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "core_types/null_logger.hpp"

namespace curobo_planner
{
namespace mp = motion_planner;
using curobo_bridge_msgs::srv::CheckMotion;
using curobo_bridge_msgs::srv::PlanJoint;
using curobo_bridge_msgs::srv::PlanPose;
using curobo_bridge_msgs::srv::SetEsdf;
using curobo_bridge_msgs::srv::SetWorld;
using curobo_bridge_msgs::srv::SolveIk;

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

mp::FailureStage stageFrom(const std::string & s)
{
  return s == "planning" ? mp::FailureStage::kPlanning : mp::FailureStage::kPrecheck;
}

template<typename Response>
mp::PlanningResult toResult(const Response & res)
{
  if (!res.success) {
    return fail(res.failure_reason_code, res.message, stageFrom(res.failure_stage));
  }
  mp::PlanningResult r;
  r.success = true;
  r.status = "success";
  r.path_fraction = 1.0;
  r.planning_time_ms = static_cast<uint64_t>(res.planning_time_s * 1000.0);
  r.trajectory.joint_names = res.trajectory.joint_names;
  for (const auto & p : res.trajectory.points) {
    core_types::TrajectoryPoint tp;
    tp.positions = p.positions;
    tp.time_from_start_sec = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9;
    r.trajectory.points.push_back(std::move(tp));
  }
  return r;
}

Eigen::Isometry3d toIso(const core_types::Pose & p)
{
  Eigen::Quaterniond q(p.qw, p.qx, p.qy, p.qz);
  q = q.norm() < 1e-9 ? Eigen::Quaterniond::Identity() : q.normalized();
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = q.toRotationMatrix();
  t.translation() = Eigen::Vector3d(p.x, p.y, p.z);
  return t;
}
}  // namespace

struct CuroboClientPlanner::Impl
{
  rclcpp::Node::SharedPtr node;
  Options options;
  std::shared_ptr<core_types::Logger> logger;
  rclcpp::Client<PlanJoint>::SharedPtr plan_joint;
  rclcpp::Client<PlanPose>::SharedPtr plan_pose;
  rclcpp::Client<SetWorld>::SharedPtr set_world;
  rclcpp::Client<SolveIk>::SharedPtr solve_ik;
  rclcpp::Client<SetEsdf>::SharedPtr set_esdf;
  rclcpp::Client<CheckMotion>::SharedPtr check_motion;

  template<typename Srv>
  typename Srv::Response::SharedPtr call(
    typename rclcpp::Client<Srv>::SharedPtr client, typename Srv::Request::SharedPtr request,
    std::string & error)
  {
    if (!client->wait_for_service(std::chrono::duration<double>(options.wait_for_service_sec))) {
      error = "cuRobo planner service is not available";
      return nullptr;
    }
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::duration<double>(options.timeout_sec)) != std::future_status::ready) {
      client->remove_pending_request(future);
      error = "cuRobo planner call timed out";
      return nullptr;
    }
    return future.get();
  }
};

CuroboClientPlanner::CuroboClientPlanner(
  rclcpp::Node::SharedPtr node, const Options & options, std::shared_ptr<core_types::Logger> logger)
: impl_(std::make_unique<Impl>())
{
  impl_->node = std::move(node);
  impl_->options = options;
  impl_->logger = logger ? std::move(logger) : std::make_shared<core_types::NullLogger>();
  impl_->plan_joint = impl_->node->create_client<PlanJoint>(options.node_name + "/plan_joint");
  impl_->plan_pose = impl_->node->create_client<PlanPose>(options.node_name + "/plan_pose");
  impl_->set_world = impl_->node->create_client<SetWorld>(options.node_name + "/set_world");
  impl_->solve_ik = impl_->node->create_client<SolveIk>(options.node_name + "/solve_ik");
  impl_->set_esdf = impl_->node->create_client<SetEsdf>(options.node_name + "/set_esdf");
  impl_->check_motion = impl_->node->create_client<CheckMotion>(options.node_name + "/check_motion");
}

CuroboClientPlanner::~CuroboClientPlanner() = default;

mp::PlanningResult CuroboClientPlanner::plan(const mp::PlanningRequest & request)
{
  if (request.start_joint_state.empty()) {
    return fail("invalid_request", "start_joint_state is required", mp::FailureStage::kPrecheck);
  }
  std::string error;
  if (!request.target_joint_state.empty()) {
    auto req = std::make_shared<PlanJoint::Request>();
    req->start = request.start_joint_state;
    req->goal = request.target_joint_state;
    auto res = impl_->call<PlanJoint>(impl_->plan_joint, req, error);
    return res ? toResult(*res) : fail("unavailable", error, mp::FailureStage::kPrecheck);
  }
  if (request.target_poses.size() == 1) {
    const auto & p = request.target_poses.front();
    auto req = std::make_shared<PlanPose::Request>();
    req->start = request.start_joint_state;
    req->goal.position.x = p.x;
    req->goal.position.y = p.y;
    req->goal.position.z = p.z;
    req->goal.orientation.x = p.qx;
    req->goal.orientation.y = p.qy;
    req->goal.orientation.z = p.qz;
    req->goal.orientation.w = p.qw;
    auto res = impl_->call<PlanPose>(impl_->plan_pose, req, error);
    return res ? toResult(*res) : fail("unavailable", error, mp::FailureStage::kPrecheck);
  }
  return fail(
    "invalid_request", "need target_joint_state, or exactly one target pose", mp::FailureStage::kPrecheck);
}

bool CuroboClientPlanner::setWorld(const mp::WorldGeometryUpdate & world, std::string * message)
{
  if (!impl_->options.base_frame.empty() && !world.frame_id.empty() && world.frame_id != impl_->options.base_frame) {
    if (message) {
      *message = "world frame '" + world.frame_id + "' differs from the robot base frame '" +
        impl_->options.base_frame + "' and cuRobo cannot transform it";
    }
    return false;
  }
  auto req = std::make_shared<SetWorld::Request>();
  for (const auto & b : world.boxes) {
    curobo_bridge_msgs::msg::Box box;
    box.id = b.id;
    box.pose.position.x = b.center_x;
    box.pose.position.y = b.center_y;
    box.pose.position.z = b.center_z;
    box.pose.orientation.x = b.orientation_x;
    box.pose.orientation.y = b.orientation_y;
    box.pose.orientation.z = b.orientation_z;
    box.pose.orientation.w = b.orientation_w;
    box.size = {b.size_x, b.size_y, b.size_z};
    req->boxes.push_back(box);
  }
  if (!world.points.empty()) {
    const Eigen::Isometry3d T = toIso(world.points_origin);
    auto & cloud = req->points;
    cloud.header.frame_id = world.frame_id;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(world.points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = 12;
    cloud.row_step = cloud.point_step * cloud.width;
    for (const char * n : {"x", "y", "z"}) {
      sensor_msgs::msg::PointField f;
      f.name = n;
      f.offset = static_cast<uint32_t>(cloud.fields.size()) * 4;
      f.datatype = sensor_msgs::msg::PointField::FLOAT32;
      f.count = 1;
      cloud.fields.push_back(f);
    }
    cloud.data.resize(cloud.row_step);
    for (size_t i = 0; i < world.points.size(); ++i) {
      const Eigen::Vector3d w = T * Eigen::Vector3d(world.points[i].x, world.points[i].y, world.points[i].z);
      const float xyz[3] = {static_cast<float>(w.x()), static_cast<float>(w.y()), static_cast<float>(w.z())};
      std::memcpy(&cloud.data[i * 12], xyz, 12);
    }
  }
  std::string error;
  auto res = impl_->call<SetWorld>(impl_->set_world, req, error);
  if (!res) {
    if (message) {*message = error;}
    return false;
  }
  if (!res->success && message) {*message = res->message;}
  return res->success;
}

bool CuroboClientPlanner::setEsdf(const scene_model::EsdfGrid & grid, std::string * message)
{
  if (grid.nx <= 0 || grid.ny <= 0 || grid.nz <= 0) {
    if (message) {*message = "ESDF grid dimensions must be positive";}
    return false;
  }
  const size_t n = static_cast<size_t>(grid.nx) * static_cast<size_t>(grid.ny) * static_cast<size_t>(grid.nz);
  if (grid.distance.size() != n) {
    if (message) {*message = "ESDF grid has inconsistent dimensions";}
    return false;
  }
  auto req = std::make_shared<SetEsdf::Request>();
  req->nx = static_cast<uint32_t>(grid.nx);
  req->ny = static_cast<uint32_t>(grid.ny);
  req->nz = static_cast<uint32_t>(grid.nz);
  req->voxel_size = grid.voxel_size;
  // our origin is the centre of voxel (0,0,0); the grid box is centred (n - 1) / 2 voxels further on
  const Eigen::Vector3d dims(grid.nx, grid.ny, grid.nz);
  const Eigen::Vector3d centre = grid.origin + grid.voxel_size * (dims.array() - 1.0).matrix() / 2.0;
  req->center = {centre.x(), centre.y(), centre.z()};
  req->distance.resize(n);
  for (int i = 0; i < grid.nx; ++i) {          // x-fastest in, x-slowest out
    for (int j = 0; j < grid.ny; ++j) {
      for (int k = 0; k < grid.nz; ++k) {
        req->distance[(static_cast<size_t>(i) * grid.ny + j) * grid.nz + k] = grid.distance[grid.index(i, j, k)];
      }
    }
  }
  std::string error;
  auto res = impl_->call<SetEsdf>(impl_->set_esdf, req, error);
  if (!res) {
    if (message) {*message = error;}
    return false;
  }
  if (!res->success && message) {*message = res->message;}
  return res->success;
}

bool CuroboClientPlanner::clearWorld(std::string * message)
{
  return setWorld(mp::WorldGeometryUpdate{}, message);
}

bool CuroboClientPlanner::updateWorldGeometry(const mp::WorldGeometryUpdate & update, std::string & error_message)
{
  error_message.clear();
  return setWorld(update, &error_message);
}

bool CuroboClientPlanner::clearWorldGeometry(std::string & error_message)
{
  error_message.clear();
  return clearWorld(&error_message);
}

mp::IkResult CuroboClientPlanner::ik(const mp::IkRequest & request)
{
  mp::IkResult out;
  out.failure_stage = mp::FailureStage::kPrecheck;
  const auto & t = request.target_pose;
  if (t.size() != 6 && t.size() != 7) {
    out.failure_reason_code = "invalid_request";
    out.message = "target_pose must be xyz+rpy (6) or xyz+quaternion x y z w (7)";
    return out;
  }
  auto req = std::make_shared<SolveIk::Request>();
  req->pose.position.x = t[0];
  req->pose.position.y = t[1];
  req->pose.position.z = t[2];
  if (t.size() == 7) {
    req->pose.orientation.x = t[3];
    req->pose.orientation.y = t[4];
    req->pose.orientation.z = t[5];
    req->pose.orientation.w = t[6];
  } else {
    const Eigen::Quaterniond q = Eigen::AngleAxisd(t[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(t[4], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(t[3], Eigen::Vector3d::UnitX());
    req->pose.orientation.x = q.x();
    req->pose.orientation.y = q.y();
    req->pose.orientation.z = q.z();
    req->pose.orientation.w = q.w();
  }
  req->seed = request.joint_seed;
  std::string error;
  auto res = impl_->call<SolveIk>(impl_->solve_ik, req, error);
  if (!res) {
    out.failure_reason_code = "unavailable";
    out.message = error;
    return out;
  }
  if (!res->success) {
    out.failure_reason_code = res->failure_reason_code;
    out.failure_stage = stageFrom(res->failure_stage);
    out.message = res->message;
    return out;
  }
  out.success = true;
  out.failure_stage = mp::FailureStage::kNone;
  out.joint_solution = res->joints;
  return out;
}

mp::CollisionResult CuroboClientPlanner::collision(const mp::CollisionRequest & request)
{
  mp::CollisionResult out;
  out.failure_stage = mp::FailureStage::kPrecheck;
  if (!request.target_pose.empty() || !request.trajectory_poses.empty()) {
    out.failure_reason_code = "unsupported";
    out.message = "pose-based collision checks are not supported; give joint values";
    return out;
  }
  if (request.joint_values.empty() && request.trajectory_joint_values.empty()) {
    out.failure_reason_code = "invalid_request";
    out.message = "need joint_values or trajectory_joint_values";
    return out;
  }
  auto req = std::make_shared<CheckMotion::Request>();
  auto add = [&req](const std::vector<double> & q) {
      trajectory_msgs::msg::JointTrajectoryPoint p;
      p.positions = q;
      req->states.points.push_back(p);
    };
  if (!request.joint_values.empty()) {add(request.joint_values);}
  for (const auto & q : request.trajectory_joint_values) {add(q);}
  req->max_step = request.trajectory_joint_values.size() > 1 ? 0.05 : 0.0;
  std::string error;
  auto res = impl_->call<CheckMotion>(impl_->check_motion, req, error);
  if (!res) {
    out.failure_reason_code = "unavailable";
    out.message = error;
    return out;
  }
  if (!res->success) {
    out.failure_reason_code = "invalid_request";
    out.message = res->message;
    return out;
  }
  out.success = true;
  out.collision_free = res->collision_free;
  out.failure_stage = mp::FailureStage::kNone;
  return out;
}

}  // namespace curobo_planner
