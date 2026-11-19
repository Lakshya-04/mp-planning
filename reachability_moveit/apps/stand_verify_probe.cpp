// RM4D versus IK on a wall, against a running MoveIt demo (docs/moveit_config.md, mock hardware).
//
//   ros2 run reachability_moveit stand_verify_probe --ros-args -p map_file:=ur10e.rm4d -p wall_x:=1.4 ...
//
// Builds the candidate stands and wall targets of a StationRequest, asks RM4D which (stand, target, lift
// height) triples it promises, sends a deterministic sample of them to the MoveIt IK + collision checks
// (PortStandVerifier over MoveItPlanningBackend), and prints how many RM4D confirms. It also samples the
// triples RM4D rejects, to see how pessimistic it is, and runs selectStands with and without the verifier.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "motion_planner_moveit/moveit_planning_backend.hpp"
#include "reachability/core/global_stands.hpp"
#include "reachability/core/io.hpp"
#include "reachability/core/lift_bands.hpp"
#include "reachability_moveit/port_stand_verifier.hpp"

using namespace reachability;  // NOLINT

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("stand_verify_probe");
  auto P = [&](const char * n, auto d) {return node->declare_parameter(n, d);};
  const auto map_file = P("map_file", std::string());
  const double wall_x = P("wall_x", 1.4);
  const double y_lo = P("wall_y_min", -0.6), y_hi = P("wall_y_max", 0.6), y_step = P("wall_y_step", 0.3);
  const double z_lo = P("wall_z_min", 0.5), z_hi = P("wall_z_max", 2.1), z_step = P("wall_z_step", 0.4);
  const double mount_z = P("arm_mount_offset_z", 1.17);
  const double lift_max = P("lift_max", 1.5);
  const double lift_step = P("lift_step", 0.25);
  const int max_triples = static_cast<int>(P("max_triples", 400));
  const bool run_selection = P("run_selection", true);
  if (map_file.empty()) {std::fprintf(stderr, "map_file is required\n"); return 2;}
  const Map4D map = loadMap(map_file);

  // wall at x = wall_x, free side toward -x (the robot starts at the origin and faces +x), tool z into the wall
  StationRequest req;
  req.wall_point = Vec3(wall_x, 0, 0);
  req.wall_normal = Vec3(-1, 0, 0);
  req.arm_mount_offset_z = mount_z;
  req.lift_max = lift_max;
  req.lift_step = lift_step;
  req.min_clearance = 0.3;
  req.max_standoff = 1.2;
  req.grid_step = 0.15;
  req.lateral_margin = 0.2;
  for (double y = y_lo; y <= y_hi + 1e-9; y += y_step) {
    for (double z = z_lo; z <= z_hi + 1e-9; z += z_step) {
      Pose T = Pose::Identity();
      T.translation() = Vec3(wall_x, y, z);
      T.linear() = Eigen::AngleAxisd(M_PI / 2, Vec3::UnitY()).toRotationMatrix();
      req.targets_world.push_back(T);
    }
  }
  const auto candidates = makeCandidateStands(req);
  const auto heights = heightRange(req.lift_min, req.lift_max, req.lift_step);
  std::printf("targets %zu, candidate stands %zu, lift heights %zu\n", req.targets_world.size(), candidates.size(), heights.size());

  // Two backends: each keeps one MoveGroupInterface, and rebuilding it on every group switch (arm for IK,
  // lift_arm for collision) costs about 0.3 s per call.
  auto ik_backend = std::make_shared<motion_planner::MoveItPlanningBackend>();
  auto col_backend = std::make_shared<motion_planner::MoveItPlanningBackend>();
  reachability_moveit::PortVerifierConfig cfg;
  cfg.ik_group = P("ik_group", std::string("arm"));
  cfg.ik_link = P("ik_link", std::string("tcp_link"));
  cfg.collision_group = P("collision_group", std::string("lift_arm"));
  cfg.arm_base_in_plan.translation() = Eigen::Vector3d(P("arm_base_x", 0.0), P("arm_base_y", 0.0), mount_z);
  cfg.lift_max = lift_max;
  if (P("seed_set", false)) {
    // UR-style folded arm seeds (the backend otherwise solves from the model's zero state): the first
    // collision-free solution wins, so a solution that hits the lift column no longer counts as a refusal.
    cfg.seeds = {{0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0}, {0.0, -1.2, 1.0, -1.4, -1.5708, 0.0},
      {0.0, -2.0, 2.2, -1.8, -1.5708, 0.0}};
  }
  auto verifier = std::make_shared<reachability_moveit::PortStandVerifier>(cfg, ik_backend, col_backend);

  // Triples (candidate, target, height) by what RM4D says.
  struct Triple {size_t c, t; double h;};
  std::vector<Triple> promised, refused_by_rm4d;
  for (size_t c = 0; c < candidates.size(); ++c) {
    const auto & s = candidates[c].stand;
    const auto reach = reachableByHeight(map, req.targets_world, s.arm_xy.x(), s.arm_xy.y(), heights, mount_z);
    for (size_t t = 0; t < req.targets_world.size(); ++t) {
      for (size_t k = 0; k < heights.size(); ++k) {
        (reach[t][k] ? promised : refused_by_rm4d).push_back({c, t, heights[k]});
      }
    }
  }
  auto sample = [&](const std::vector<Triple> & all) {
      std::vector<Triple> out;
      const size_t stride = std::max<size_t>(1, all.size() / static_cast<size_t>(std::max(1, max_triples)));
      for (size_t i = 0; i < all.size(); i += stride) {out.push_back(all[i]);}
      return out;
    };
  auto run = [&](const char * label, const std::vector<Triple> & all) {
      const auto t0 = std::chrono::steady_clock::now();
      const auto picked = sample(all);
      std::map<std::string, size_t> counts;
      for (const auto & tr : picked) {
        const auto v = verifier->verify(candidates[tr.c].stand, tr.h, {req.targets_world[tr.t]});
        ++counts[toString(v[0].reason)];
      }
      std::printf("%s: %zu triples in total, %zu sampled:", label, all.size(), picked.size());
      for (const auto & [k, n] : counts) {std::printf("  %s %zu", k.c_str(), n);}
      const size_t ok = counts["ok"];
      std::printf(" (%.1f s)", std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
      std::printf("  -> IK confirms %.1f%%\n", picked.empty() ? 0.0 : 100.0 * static_cast<double>(ok) / static_cast<double>(picked.size()));
    };
  run("RM4D reachable    ", promised);
  run("RM4D not reachable", refused_by_rm4d);

  if (run_selection) {
    StandSelectionOptions plain;
    const auto a = selectStands(map, req, candidates, std::nullopt, plain);
    StandSelectionOptions ver;
    ver.verifier = verifier;
    ver.verify.max_lift_probes = 3;
    const auto b = selectStands(map, req, candidates, std::nullopt, ver);
    std::printf("selectStands RM4D only : %zu stand(s), %zu unreachable, code %s\n", a.stands.size(), a.unreachable.size(), toString(a.code));
    std::printf("selectStands verified  : %zu stand(s), %zu unreachable (%zu rejected by IK), code %s, rounds %zu, verified pairs %zu, refused %zu (optimism %.1f%%), %zu IK batches\n",
      b.stands.size(), b.unreachable.size(), b.rejected_by_verifier.size(), toString(b.code), b.verification.rounds,
      b.verification.verified_pairs, b.verification.refused.size(), 100.0 * b.verification.optimism(), b.verification.verifier_calls);
    for (const auto & s : b.stands) {
      std::printf("  stand (%.2f, %.2f): %zu target(s), confirmed %zu, mean margin %.2f\n", s.stand.arm_xy.x(), s.stand.arm_xy.y(),
        s.targets.size(), s.verification.confirmed, s.mean_margin);
    }
  }
  std::printf("verifier: %zu IK calls, %zu collision checks\n", verifier->ikCalls(), verifier->collisionCalls());
  rclcpp::shutdown();
  return 0;
}
