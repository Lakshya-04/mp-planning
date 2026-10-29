/**
 * @file coverage_pass_demo_node.cpp
 * @brief Minimal dock -> contact -> retract orchestrator, driving the real
 *        motion_planner::MotionPlanner (CartesianPlanner + OmplPlanner) and executing its
 *        planned trajectories against JTAC's FollowJointTrajectory action.
 * @details manipulation_manager (the real dock-pose/reachability-map-driven state machine
 *          this stands in for) isn't ported into this repo yet -- see motion_planner's own
 *          package.xml description. This is the deliberately-scoped placeholder: one column of a
 *          real coverage plan, a fixed dock standoff instead of a reachability-map-optimized base
 *          pose (a proper BPO is future work), FSP to dock, JTAC force mode engaged, CartesianPlanner
 *          with allow_target_contact=true for the actual wall pass, force mode off, FSP retract.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <algorithm>
#include <chrono>
#include <fstream>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

#include <nlohmann/json.hpp>

#include "motion_planner/motion_planner.hpp"
#include "motion_planner/moveit_planning_backend.hpp"
#include "motion_planner/planners/cartesian_planner.hpp"
#include "motion_planner/planners/ompl_planner.hpp"
#include "ros_adapters/ros_conversions.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

namespace
{

core_types::Pose makePose(
  double x, double y, double z, double qx, double qy, double qz, double qw)
{
  core_types::Pose pose;
  pose.x = x;
  pose.y = y;
  pose.z = z;
  pose.qx = qx;
  pose.qy = qy;
  pose.qz = qz;
  pose.qw = qw;
  return pose;
}

// Live /joint_states, in the arm_7dof group's own variable order (lift_prismatic_joint,
// joint1..6 -- see moveit_config's SRDF group definition). Reading actual current joints
// instead of trusting recordExecutionOutcome()'s "continuity" tracking (the raw planned
// trajectory's end state) matters right after a contact segment: admittance keeps nudging the
// real joints to relieve force for as long as it's engaged, so the robot's true position drifts
// away from what was planned -- confirmed live, planning retract from stale continuity produced
// a joint-space discontinuity at execution start, a violent snap the physics engine registered as
// a multi-hundred-newton non-physical force spike.
std::vector<double> readCurrentJointState(
  const rclcpp::Node::SharedPtr & node, const std::vector<std::string> & joint_order)
{
  std::map<std::string, double> latest;
  auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, [&latest](const sensor_msgs::msg::JointState::SharedPtr msg) {
      for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        latest[msg->name[i]] = msg->position[i];
      }
    });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (rclcpp::ok() && latest.size() < joint_order.size() &&
    std::chrono::steady_clock::now() < deadline)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::vector<double> result;
  for (const auto & name : joint_order) {
    result.push_back(latest.count(name) ? latest.at(name) : 0.0);
  }
  return result;
}

struct CoverageWaypoint
{
  double x, y, z, qx, qy, qz, qw;
};

// Loads coverage_planner's own real output (coverage_output.json, from
// wall_coverage_demo_node -- see coverage_planner/launch/wall_coverage_demo.launch.py), finds
// the strip column nearest target_x, and returns its real endpoint waypoints (same x/y/
// orientation, one flat vertical strip on the wall) sorted by z. z_offset shifts from the JSON's
// own frame (base_link, matching wall_scene.json's own "frame_id") into whatever frame this
// node's own targets are expressed in (base_footprint, so +0.31m -- see this file's own frame_id
// comment).
std::vector<CoverageWaypoint> loadNearestCoverageColumn(
  const std::string & json_path, double target_x, double z_offset)
{
  std::ifstream file(json_path);
  if (!file.is_open()) {
    throw std::runtime_error("loadNearestCoverageColumn: could not open " + json_path);
  }
  nlohmann::json doc;
  file >> doc;
  const auto & poses = doc.at("final_lifted_and_expanded").at("path").at("poses");

  double best_x = 0.0;
  double best_dist = std::numeric_limits<double>::max();
  for (const auto & p : poses) {
    const double x = p.at("position").at("x").get<double>();
    const double dist = std::abs(x - target_x);
    if (dist < best_dist) {
      best_dist = dist;
      best_x = x;
    }
  }

  std::vector<CoverageWaypoint> column;
  for (const auto & p : poses) {
    const double x = p.at("position").at("x").get<double>();
    if (std::abs(x - best_x) > 1e-6) {
      continue;
    }
    CoverageWaypoint wp;
    wp.x = x;
    wp.y = p.at("position").at("y").get<double>();
    wp.z = p.at("position").at("z").get<double>() + z_offset;
    wp.qx = p.at("orientation").at("x").get<double>();
    wp.qy = p.at("orientation").at("y").get<double>();
    wp.qz = p.at("orientation").at("z").get<double>();
    wp.qw = p.at("orientation").at("w").get<double>();
    column.push_back(wp);
  }
  std::sort(
    column.begin(), column.end(), [](const auto & a, const auto & b) {return a.z < b.z;});
  return column;
}

bool executeTrajectory(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp_action::Client<FollowJointTrajectory>::SharedPtr & client,
  const core_types::Trajectory & trajectory, const std::string & label)
{
  if (trajectory.points.empty()) {
    RCLCPP_ERROR(node->get_logger(), "%s: empty trajectory, nothing to execute", label.c_str());
    return false;
  }

  auto goal_msg = FollowJointTrajectory::Goal();
  goal_msg.trajectory = ros_adapters::toRos(trajectory);

  auto result_promise = std::make_shared<std::promise<bool>>();
  auto result_future = result_promise->get_future().share();

  auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  send_goal_options.result_callback =
    [result_promise, label, node](const GoalHandle::WrappedResult & result) {
      const bool ok = result.code == rclcpp_action::ResultCode::SUCCEEDED;
      RCLCPP_INFO(
        node->get_logger(), "%s: result code=%d success=%d", label.c_str(),
        static_cast<int>(result.code), ok);
      result_promise->set_value(ok);
    };

  auto goal_handle_future = client->async_send_goal(goal_msg, send_goal_options);
  if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "%s: send_goal call failed", label.c_str());
    return false;
  }
  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "%s: goal rejected", label.c_str());
    return false;
  }

  while (rclcpp::ok() &&
    result_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready)
  {
    rclcpp::spin_some(node);
  }
  return result_future.get();
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("coverage_pass_demo");

  const std::string planning_group = "arm_7dof";
  const std::string frame_id = "base_footprint";  // MoveGroupInterface's planning frame for ranger_mini (base_link and base_footprint are two disconnected tf trees here -- see this run's own resolvePoseToPlanningFrame() error)

  auto backend = std::make_shared<motion_planner::MoveItPlanningBackend>();
  auto cartesian = std::make_unique<motion_planner::CartesianPlanner>(backend);
  auto fsp = std::make_unique<motion_planner::OmplPlanner>(backend);
  motion_planner::MotionPlanner motion_planner(std::move(cartesian), std::move(fsp));
  motion_planner.resetContinuity();

  auto jtac_client = rclcpp_action::create_client<FollowJointTrajectory>(
    node, "/combined_arm_lift_controller/follow_joint_trajectory");
  RCLCPP_INFO(node->get_logger(), "Waiting for JTAC action server...");
  if (!jtac_client->wait_for_action_server(std::chrono::seconds(15))) {
    RCLCPP_ERROR(node->get_logger(), "JTAC action server unavailable");
    return 1;
  }

  auto force_pub = node->create_publisher<std_msgs::msg::Float64>(
    "/combined_arm_lift_controller/target_force_n", 10);
  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(
    node, "/combined_arm_lift_controller");
  if (!param_client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node->get_logger(), "combined_arm_lift_controller param service unavailable");
    return 1;
  }

  auto set_admittance_axis = [&](bool enabled) {
      std::vector<bool> axes = {false, false, enabled, false, false, false};
      auto future = param_client->set_parameters({rclcpp::Parameter("admittance.selected_axes",
        axes)});
      rclcpp::spin_until_future_complete(node, future);
    };

  // Retuning admittance damping/stiffness live was tried here and ruled OUT as the cause: an
  // instrumented ~/debug_offset topic on JtacController showed the reference-vs-desired Cartesian
  // offset never exceeded ~3cm during a run full of multi-hundred-N spikes, no correlation at all
  // -- the spikes were never a JTAC/admittance control problem. Root cause (confirmed via the
  // wall's own contact sensor: near-zero measured depth, ~1000N reported force) was the tool's
  // collision geometry being the raw detailed STL mesh instead of a simplified primitive -- see
  // cam_mop_tool.xacro's own fix. gz_controllers.yaml's real z-axis defaults are left alone.

  // Real coverage-planner output this time, not a hand-picked point: coverage_planner's own
  // wall_coverage_demo_node was rerun fresh against this session's actual wall + current docked
  // base pose (see wall_export/gen_wall_scene.py's own header), producing wall_export/
  // coverage_output.json's real 40-waypoint boustrophedon plan. Pick the strip column nearest
  // this base's reach (constant standoff instead of a real reachability-map BPO -- future work,
  // see this file's own header) and step through the sub-range of ITS real z extent that's
  // actually reachable -- everything else about wall_x/wall_y below comes straight from the
  // planner's own output, not a guess.
  const std::string coverage_output_path = "/tmp/wall_export/coverage_output.json";
  // +0.31m: coverage_output.json is in base_link (see wall_scene.json's own frame_id); this
  // node's own frame_id is base_footprint (base_footprint -> base_link is a fixed +0.31m, see
  // this file's own frame_id comment, above).
  const auto coverage_column = loadNearestCoverageColumn(coverage_output_path, 0.06, 0.31);
  if (coverage_column.size() < 2) {
    RCLCPP_ERROR(
      node->get_logger(), "loadNearestCoverageColumn: expected >=2 waypoints, got %zu",
      coverage_column.size());
    return 1;
  }
  const double wall_x = coverage_column.front().x;
  const double wall_y = coverage_column.front().y;  // wall surface, at this base position
  const double dock_standoff = 0.12;
  // Reaching down toward the base's own height (z~1.0) put the arm's own self-collision
  // checker (base_link/front_*_wheel/lift_*_link) in the way -- see this run's own log, a real
  // finding CartesianPlanner surfaced that the earlier hand-rolled compute_ik script's
  // avoid_collisions=false silently ignored. Higher on the wall clears the base's own silhouette
  // -- but not TOO high: the wall's own top edge is z=2.0 (see the wall spawn script), and z=1.91
  // put the tool's own collision mesh close enough to clip that edge instead of the flat face.
  // Confirmed live via the wall's own contact sensor: the reported contact position sat at
  // z=1.99999... (the exact top edge) with depth=8e-7 (no real penetration) but a reported
  // reaction force near 1000N -- a classic edge/corner contact numerical instability, not a
  // physics-engine tuning problem. z=1.40 (well within this column's own [z_min,z_max]) keeps a
  // real margin from every wall edge.
  const double dock_z = 1.40;

  // Tool orientation: taken directly from the coverage planner's own output (its local Z axis is
  // the tool's press/approach axis, pointing INTO the wall) -- not re-derived, matches exactly.
  const double approach_qx = coverage_column.front().qx;
  const double approach_qy = coverage_column.front().qy;
  const double approach_qz = coverage_column.front().qz;
  const double approach_qw = coverage_column.front().qw;
  // wall_reference_pose's local Z is the wall's OUTWARD normal (opposite direction).
  const double normal_qx = 0.7071067811865476;
  const double normal_qy = 0.0;
  const double normal_qz = 0.0;
  const double normal_qw = 0.7071067811865475;

  motion_planner::PlanningRequest dock_request;
  dock_request.frame_id = frame_id;
  dock_request.planning_group = planning_group;
  dock_request.waypoint_indices = {0};
  dock_request.target_poses = {makePose(
      wall_x, wall_y - dock_standoff, dock_z, approach_qx, approach_qy, approach_qz, approach_qw)};
  dock_request.min_path_fraction = 0.5;

  motion_planner::SegmentRoutingSpec dock_routing;
  dock_routing.label = "dock_approach";
  dock_routing.mode_hint = "fsp";

  RCLCPP_INFO(node->get_logger(), "Planning dock/approach segment (FSP)...");
  auto dock_result = motion_planner.planSegment(dock_request, dock_routing);
  if (!dock_result.planning_result.success) {
    RCLCPP_ERROR(
      node->get_logger(), "Dock segment planning failed: %s",
      dock_result.planning_result.message.c_str());
    return 1;
  }
  RCLCPP_INFO(
    node->get_logger(), "Dock segment planned (mode=%s), executing...",
    dock_result.mode_used.c_str());
  if (!executeTrajectory(node, jtac_client, dock_result.planning_result.trajectory, "dock")) {
    return 1;
  }
  motion_planner.recordExecutionOutcome(
    true, dock_result.mode_used, dock_result.planning_result.trajectory);

  RCLCPP_INFO(node->get_logger(), "Engaging force mode...");
  set_admittance_axis(true);
  {
    std_msgs::msg::Float64 msg;
    msg.data = 5.0;
    force_pub->publish(msg);
  }
  auto max_offset_pub = node->create_publisher<std_msgs::msg::Float64>(
    "/combined_arm_lift_controller/max_offset_m", 10);
  {
    // Default is 0.1m -- observed live, the offset-diverged safety clamp (kRunawayMultiplier=5x
    // this) was tripping constantly at ~0.5m = exactly 5 * 0.1, resetting the compliance
    // integrator on a repeating cycle and producing the periodic force spikes. Widening this
    // gives the admittance response real room to settle instead of getting reset mid-converge.
    std_msgs::msg::Float64 msg;
    msg.data = 0.3;
    max_offset_pub->publish(msg);
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Contact segment: step down through this REAL strip's own reachable sub-range (its full real
  // extent, per coverage_column, spans ~0.03m-1.97m -- see its own load call above -- but only a
  // window around dock_z is actually reachable from this docked base pose without a real
  // reachability-map BPO, future work per this file's own header). x/y/orientation are the
  // planner's own real output; only z stepping through that sub-range and the y overshoot below
  // are this node's own additions.
  //
  // The literal wall-surface y undershoots real contact -- an earlier "successful" 4.4N spike
  // turned out (per the live force log: zero the instant before, zero the instant after) to be a
  // transient graze mid-swing at high approach speed, not genuine contact at the resting target.
  // +0.045 is the smallest overshoot found live to produce real, sustained contact force at this
  // wall/tool geometry (below ~0.035 never registers above the gravity baseline).
  const double contact_y = wall_y + 0.045;
  const std::vector<double> contact_z = {1.20, 1.40, 1.60};

  motion_planner::PlanningRequest contact_request;
  contact_request.frame_id = frame_id;
  contact_request.planning_group = planning_group;
  for (size_t i = 0; i < contact_z.size(); ++i) {
    contact_request.waypoint_indices.push_back(i);
    contact_request.target_poses.push_back(
      makePose(wall_x, contact_y, contact_z[i], approach_qx, approach_qy, approach_qz, approach_qw));
  }
  contact_request.allow_target_contact = true;
  contact_request.wall_reference_pose =
    makePose(wall_x, wall_y, contact_z.front(), normal_qx, normal_qy, normal_qz, normal_qw);
  contact_request.max_wall_penetration = 0.12;
  contact_request.min_path_fraction = 0.05;
  contact_request.target_eef_speed = 0.02;  // 2 cm/s -- a controlled press, not a crash

  motion_planner::SegmentRoutingSpec contact_routing;
  contact_routing.label = "wall_contact_pass";
  contact_routing.mode_hint = "cartesian";

  RCLCPP_INFO(node->get_logger(), "Planning contact segment (Cartesian, allow_target_contact)...");
  auto contact_result = motion_planner.planSegment(contact_request, contact_routing);
  RCLCPP_INFO(
    node->get_logger(), "Contact segment: success=%d fraction=%.3f waypoints=%zu",
    contact_result.planning_result.success, contact_result.planning_result.path_fraction,
    contact_result.planning_result.waypoints_completed);
  if (contact_result.planning_result.success &&
    !contact_result.planning_result.trajectory.points.empty())
  {
    executeTrajectory(node, jtac_client, contact_result.planning_result.trajectory, "contact");
    motion_planner.recordExecutionOutcome(
      true, contact_result.mode_used, contact_result.planning_result.trajectory);
  }

  // Hold at the wall: admittance stays engaged, no new motion commanded, while the contact force
  // settles toward the 5N target -- log measured_wrench live so the steady-state (not a
  // transient spike) is visible in this node's own output.
  RCLCPP_INFO(node->get_logger(), "Holding at the wall, watching contact force settle...");
  auto wrench_sub = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/combined_arm_lift_controller/measured_wrench", 10,
    [node](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
      RCLCPP_INFO(
        node->get_logger(), "  measured force: x=%.3f y=%.3f z=%.3f N", msg->wrench.force.x,
        msg->wrench.force.y, msg->wrench.force.z);
    });
  const auto hold_until = std::chrono::steady_clock::now() + std::chrono::seconds(4);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < hold_until) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  RCLCPP_INFO(node->get_logger(), "Disengaging force mode, retracting...");
  set_admittance_axis(false);

  motion_planner::PlanningRequest retract_request;
  retract_request.frame_id = frame_id;
  retract_request.planning_group = planning_group;
  retract_request.waypoint_indices = {0};
  retract_request.target_poses = {makePose(
      wall_x, wall_y - dock_standoff, 1.40, approach_qx, approach_qy, approach_qz, approach_qw)};
  retract_request.min_path_fraction = 0.5;
  // target_eef_speed is CartesianPlanner-only (see PlanningRequest's own doc comment) -- retract
  // is fsp-routed and moves through free space, so its default OMPL timing is fine here; only the
  // approach-into-contact leg above needed slowing down.

  motion_planner::SegmentRoutingSpec retract_routing;
  retract_routing.label = "retract";
  retract_routing.mode_hint = "fsp";

  const std::vector<std::string> group_joint_order = {
    "lift_prismatic_joint", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  const auto live_joint_state = readCurrentJointState(node, group_joint_order);
  // OMPL's own sampling is randomized -- a genuine failure to find any path from the exact
  // post-contact (compliance-drifted) pose is worth one immediate retry before giving up, same
  // cost as a single extra planning call.
  motion_planner::MotionPlanner::SegmentPlanResult retract_result;
  for (int attempt = 0; attempt < 3; ++attempt) {
    retract_result = motion_planner.planSegmentWithStartState(
      retract_request, retract_routing, live_joint_state);
    if (retract_result.planning_result.success) {
      break;
    }
    RCLCPP_WARN(
      node->get_logger(), "Retract planning attempt %d failed, retrying...", attempt + 1);
  }
  if (retract_result.planning_result.success) {
    executeTrajectory(node, jtac_client, retract_result.planning_result.trajectory, "retract");
    motion_planner.recordExecutionOutcome(
      true, retract_result.mode_used, retract_result.planning_result.trajectory);
  }

  RCLCPP_INFO(node->get_logger(), "Coverage pass demo complete.");
  rclcpp::shutdown();
  return 0;
}
