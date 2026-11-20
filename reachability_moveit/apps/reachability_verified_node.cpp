// The reachability node with stand verification: ~/plan_verified_stands answers the plan_stands request with
// selectStands and the MoveIt IK + collision checks (PortStandVerifier), so a stand RM4D promises but IK or the
// scene refuses is dropped from the plan. ~/plan_stands and the other services are the plain ReachabilityNode's.
//
//   ros2 run reachability_moveit reachability_verified_node --ros-args -p map_file:=ur10e.rm4d \
//     -p base_profile:=<profile.yaml> -p arm_mount_offset_z:=1.17 -p lift_max:=1.5
//
// Needs a running move_group with the robot (docs/moveit_config.md). A request takes minutes on a real wall
// (about 5 minutes measured on one wall, docs/reachability.md), so call it with a long client timeout.
// ~/plan_stands answers with the same verifier when the request sets `verified` (or the node parameter
// `verify_by_default` is true); without either it stays RM4D only.
// The scene is whatever move_group holds (one fixed stand's view); stand-relative obstacle posing needs the
// PortStandVerifier boxes/points, not wired here.
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "motion_planner_moveit/moveit_planning_backend.hpp"
#include "reachability/ros/reachability_node.hpp"
#include "reachability_moveit/port_stand_verifier.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto reach = std::make_shared<reachability::ReachabilityNode>();
  if (!reach->ready()) {
    RCLCPP_ERROR(reach->get_logger(), "reachability node not ready (map or base profile); exiting");
    rclcpp::shutdown();
    return 1;
  }
  // Settings of the verifier live on a second node: ReachabilityNode declares its own parameters.
  auto cfgnode = std::make_shared<rclcpp::Node>("reachability_verified_config");
  auto P = [&](const char * n, auto d) {return cfgnode->declare_parameter(n, d);};
  reachability_moveit::PortVerifierConfig cfg;
  cfg.ik_group = P("ik_group", std::string("arm"));
  cfg.ik_link = P("ik_link", std::string("tcp_link"));
  cfg.collision_group = P("collision_group", std::string("lift_arm"));
  const double mount_z = P("arm_mount_offset_z", 1.17);
  cfg.arm_base_in_plan.translation() = Eigen::Vector3d(P("arm_base_x", 0.0), P("arm_base_y", 0.0), mount_z);
  cfg.lift_max = P("lift_max", 1.5);
  if (P("seed_set", true)) {
    cfg.seeds = {{0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0}, {0.0, -1.2, 1.0, -1.4, -1.5708, 0.0},
      {0.0, -2.0, 2.2, -1.8, -1.5708, 0.0}};
  }
  reachability::VerifyOptions vo;
  vo.max_lift_probes = static_cast<size_t>(P("max_lift_probes", 3));
  vo.jitter_m = P("jitter_m", 0.0);
  vo.max_rounds = static_cast<size_t>(P("max_rounds", 64));
  // Try each target's lift heights lowest first and stop at the first that solves (fewer IK probes, same confirmed
  // set and best heights; margins become a lowest-first ordinal). max_verifier_calls bounds the batches (0 = none).
  vo.stop_at_first_ok = P("stop_at_first_ok", true);
  vo.max_verifier_calls = static_cast<size_t>(P("max_verifier_calls", 0));

  // One backend per port: each keeps one MoveGroupInterface (rebuilding it per group costs about 0.3 s a call).
  auto ik_backend = std::make_shared<motion_planner::MoveItPlanningBackend>();
  auto col_backend = std::make_shared<motion_planner::MoveItPlanningBackend>();
  reach->setVerifier(std::make_shared<reachability_moveit::PortStandVerifier>(cfg, ik_backend, col_backend), vo);

  RCLCPP_INFO(reach->get_logger(), "verified stands ready: service %s/plan_verified_stands", reach->get_name());
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(reach);
  executor.add_node(cfgnode);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
