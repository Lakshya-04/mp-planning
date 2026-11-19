// cuRobo through the ROS node, as the manager reaches it: the benchmark problems (the text format of
// docker/tools/vamp_bench.cpp) planned by curobo_planner::CuroboClientPlanner against a running
// curobo_planner_node. Times are wall clock around CuroboClientPlanner::plan() (service request,
// the node's planning, the reply), so against the in-process numbers of
// motion_planner_moveit/scripts/planner_benchmark.py run-curobo they show the cost of the ROS layer.
//
//   curobo_planner_benchmark problems.txt out.jsonl LABEL [limit_per_scene] [stride]
//
// Per scene: the boxes go in with setWorld(), then the problems (every `stride`-th, at most
// `limit_per_scene`) are planned in order. Output: one JSON object per line (header, then one per
// problem with id, success, wall_us, the node's own planning time and the trajectory length/duration).
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "curobo_planner/curobo_planner.hpp"

namespace mp = motion_planner;
using Clock = std::chrono::steady_clock;

int main(int argc, char ** argv)
{
  if (argc < 4) {
    std::fprintf(stderr, "usage: %s problems.txt out.jsonl LABEL [limit_per_scene] [stride]\n", argv[0]);
    return 2;
  }
  rclcpp::init(argc, argv);
  const std::size_t limit = argc > 4 ? std::stoul(argv[4]) : 0;
  const std::size_t stride = argc > 5 ? std::max<std::size_t>(1, std::stoul(argv[5])) : 1;
  std::ifstream in(argv[1]);
  std::FILE * out = std::fopen(argv[2], "w");
  if (!in || !out) {
    std::fprintf(stderr, "cannot open files\n");
    return 2;
  }
  auto node = std::make_shared<rclcpp::Node>("curobo_planner_benchmark");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec] {exec.spin();});
  curobo_planner::Options options;
  options.wait_for_service_sec = 300.0;   // the node compiles / warms up on start
  options.timeout_sec = 120.0;
  curobo_planner::CuroboClientPlanner planner(node, options);
  std::fprintf(out, "{\"type\":\"header\",\"backend\":\"curobo_node\",\"set\":\"%s\"}\n", argv[3]);

  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] != 'S') {continue;}
    std::istringstream hs(line.substr(1));
    int scene, nb, np, nq;
    hs >> scene >> nb >> np >> nq;
    mp::WorldGeometryUpdate world;
    for (int i = 0; i < nb; ++i) {
      std::getline(in, line);
      mp::WorldGeometryBox b;
      std::istringstream l2(line.substr(1));  // cx cy cz, R columns (9), half extents (3)
      std::vector<double> a;
      for (double x; l2 >> x;) {a.push_back(x);}
      b.id = "box" + std::to_string(i);
      b.center_x = a[0]; b.center_y = a[1]; b.center_z = a[2];
      Eigen::Matrix3d M;
      M.col(0) = Eigen::Vector3d(a[3], a[4], a[5]);
      M.col(1) = Eigen::Vector3d(a[6], a[7], a[8]);
      M.col(2) = Eigen::Vector3d(a[9], a[10], a[11]);
      const Eigen::Quaterniond q(M);
      b.orientation_x = q.x(); b.orientation_y = q.y(); b.orientation_z = q.z(); b.orientation_w = q.w();
      b.size_x = 2 * a[12]; b.size_y = 2 * a[13]; b.size_z = 2 * a[14];
      world.boxes.push_back(b);
    }
    std::string message;
    const bool ok_world = world.boxes.empty() ? planner.clearWorld(&message) : planner.setWorld(world, &message);
    if (!ok_world) {
      std::fprintf(stderr, "scene %d: world update failed: %s\n", scene, message.c_str());
    }
    std::size_t taken = 0;
    for (int i = 0; i < np; ++i) {
      std::getline(in, line);
      if (static_cast<std::size_t>(i) % stride != 0 || (limit && taken >= limit)) {continue;}
      ++taken;
      std::istringstream ls(line.substr(1));
      int id;
      ls >> id;
      std::vector<double> q;
      for (double x; ls >> x;) {q.push_back(x);}
      const std::size_t dof = q.size() / 2;
      mp::PlanningRequest r;
      r.mode = "fsp";
      r.start_joint_state.assign(q.begin(), q.begin() + dof);
      r.target_joint_state.assign(q.begin() + dof, q.end());
      const auto t0 = Clock::now();
      const auto res = planner.plan(r);
      const double us = std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
      const double dur = res.trajectory.points.empty() ? 0.0 : res.trajectory.points.back().time_from_start_sec;
      std::fprintf(
        out,
        "{\"type\":\"plan\",\"scene\":%d,\"id\":%d,\"ok\":%s,\"us\":%.1f,\"node_ms\":%lu,\"points\":%zu,"
        "\"traj_s\":%.4f,\"reason\":\"%s\"}\n",
        scene, id, res.success ? "true" : "false", us, static_cast<unsigned long>(res.planning_time_ms),
        res.trajectory.points.size(), dur, res.failure_reason_code.c_str());
      std::fflush(out);
    }
  }
  std::fclose(out);
  exec.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
