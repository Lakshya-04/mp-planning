// Command-line tool: build, inspect and query reachability maps. ROS-free.
//
//   rm4d_tool build chain.yaml out.rm4d [--samples N] [--threads T] [--resolution m]
//                                       [--extent m] [--zmin m] [--zmax m] [--seed S]
//   rm4d_tool info map.rm4d
//   rm4d_tool bands map.rm4d --wall-x X --z-min Z0 --z-max Z1 [--z-step dz] [--approach +x]
//                            [--lift-max H] [--lift-step dh] [--mount-z dz] [--base-x bx] [--base-y by]
//   rm4d_tool station map.rm4d --wall-x X --z-min Z0 --z-max Z1 [--width W] [--z-step dz] [--y-step dy]
//                              [--lift-max H] [--mount-z dz] [--min-clearance c] [--grid-step g] [--base-profile profile.yaml]
//                              [--multi 1 --start-x X --start-y Y --strategy greedy|set_cover|global [--margin-penalty w --time-budget s]]   several stands with travel time
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>

#include "reachability/core/arm_model.hpp"
#include "reachability/core/base_model.hpp"
#include "reachability/core/builder.hpp"
#include "reachability/core/io.hpp"
#include "reachability/core/lift_bands.hpp"
#include "reachability/core/global_stands.hpp"
#include "reachability/core/station.hpp"
#include "reachability/core/stands.hpp"

using namespace reachability;

namespace
{
std::map<std::string, std::string> options(int argc, char ** argv, int first)
{
  std::map<std::string, std::string> o;
  for (int i = first; i < argc; i += 2) {
    if (std::strncmp(argv[i], "--", 2) != 0) {
      std::fprintf(stderr, "unexpected argument '%s'\n", argv[i]);
      std::exit(2);
    }
    if (i + 1 >= argc) {
      std::fprintf(stderr, "option '%s' needs a value\n", argv[i]);
      std::exit(2);
    }
    o[argv[i] + 2] = argv[i + 1];
  }
  return o;
}

double num(const std::map<std::string, std::string> & o, const char * k, double dflt)
{
  const auto it = o.find(k);
  return it == o.end() ? dflt : std::atof(it->second.c_str());
}

// A loop step: finite and positive, or the tool stops (a zero step would loop forever).
double step(const std::map<std::string, std::string> & o, const char * k, double dflt)
{
  const double v = num(o, k, dflt);
  if (!std::isfinite(v) || v <= 0.0) {
    std::fprintf(stderr, "--%s must be a positive number\n", k);
    std::exit(2);
  }
  return v;
}

int usage()
{
  std::fprintf(stderr,
    "usage:\n  rm4d_tool build chain.yaml out.rm4d [--samples N --threads T --resolution m --extent m --zmin m --zmax m --seed S]\n"
    "  rm4d_tool info map.rm4d\n"
    "  rm4d_tool bands map.rm4d --wall-x X --z-min Z0 --z-max Z1 [--z-step dz --lift-max H --lift-step dh --mount-z dz --base-x bx --base-y by]\n"
    "  rm4d_tool station map.rm4d --wall-x X --z-min Z0 --z-max Z1 [--width W --z-step dz --y-step dy --lift-max H --mount-z dz --min-clearance c --grid-step g --base-profile profile.yaml\n"
    "    [--multi 1 --start-x X --start-y Y --strategy greedy|set_cover|global [--margin-penalty w --time-budget s]]]\n");
  return 2;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    return usage();
  }
  const std::string cmd = argv[1];
  try {
    if (cmd == "build" && argc >= 4) {
      const ChainModel arm = loadChainYaml(argv[2]);
      const auto o = options(argc, argv, 4);
      MapParams p;
      p.resolution = num(o, "resolution", p.resolution);
      p.xy_extent = num(o, "extent", p.xy_extent);
      p.z_min = num(o, "zmin", p.z_min);
      p.z_max = num(o, "zmax", p.z_max);
      BuildOptions b;
      b.samples = static_cast<size_t>(num(o, "samples", 2e6));
      b.threads = static_cast<int>(num(o, "threads", 0));
      b.seed = static_cast<uint32_t>(num(o, "seed", 1));
      BuildStats s;
      const Map4D map = buildMap(arm, p, b, &s);
      saveMap(argv[3], map, arm.fingerprint());
      std::printf("built %s: %zu samples (%zu valid, %zu in window) in %.1f s, %zu of %zu cells set (%zu saturated)\n",
        argv[3], s.samples, s.valid, s.in_window, s.seconds, s.nonzero_cells, map.totalCells(), s.saturated_cells);
      return 0;
    }
    if (cmd == "info") {
      const Map4D map = loadMap(argv[2]);
      const auto & p = map.params();
      std::printf("window +-%.2f m, z %.2f..%.2f m, resolution %.3f m, theta bin %.1f deg\ncells %zu, set %zu (%.2f%%)\n",
        p.xy_extent, p.z_min, p.z_max, p.resolution, p.theta_bin * 180.0 / M_PI, map.totalCells(),
        map.nonzeroCells(), 100.0 * static_cast<double>(map.nonzeroCells()) / static_cast<double>(map.totalCells()));
      return 0;
    }
    if (cmd == "bands") {
      const auto o = options(argc, argv, 3);   // bad options are reported before the map is read
      const double wall_x = num(o, "wall-x", 0.6), z0 = num(o, "z-min", 0.1), z1 = num(o, "z-max", 2.5);
      const double dz = step(o, "z-step", 0.1);
      const double dh = step(o, "lift-step", 0.05);
      const Map4D map = loadMap(argv[2]);
      std::vector<Pose> targets;
      for (double z = z0; z <= z1 + 1e-9; z += dz) {
        Pose T = Pose::Identity();
        T.translation() = Vec3(wall_x, 0.0, z);
        T.linear() = Eigen::AngleAxisd(M_PI / 2, Vec3::UnitY()).toRotationMatrix();  // tool points into the wall (+x)
        targets.push_back(T);
      }
      const auto plan = planLiftBands(map, targets, num(o, "base-x", 0.0), num(o, "base-y", 0.0),
        heightRange(0.0, num(o, "lift-max", 1.5), dh), num(o, "mount-z", 0.2));
      std::printf("%zu targets, %zu reachable, %zu band(s):\n", targets.size(), plan.reachable_count, plan.bands.size());
      for (const auto & b : plan.bands) {
        std::printf("  lift %.2f m covers %zu target(s)\n", b.lift_height, b.pose_indices.size());
      }
      if (!plan.unreachable.empty()) {
        std::printf("  %zu target(s) unreachable from this station\n", plan.unreachable.size());
      }
      return plan.unreachable.empty() ? 0 : 1;
    }
    if (cmd == "station") {
      const auto o = options(argc, argv, 3);   // bad options are reported before the map is read
      const double wall_x = num(o, "wall-x", 3.0), z0 = num(o, "z-min", 0.1), z1 = num(o, "z-max", 2.5);
      const double dz = step(o, "z-step", 0.2), dy = step(o, "y-step", 0.2), half = num(o, "width", 2.0) / 2.0;
      const double grid = step(o, "grid-step", 0.1);
      const Map4D map = loadMap(argv[2]);
      StationRequest r;
      for (double y = -half; y <= half + 1e-9; y += dy) {
        for (double z = z0; z <= z1 + 1e-9; z += dz) {
          Pose T = Pose::Identity();
          T.translation() = Vec3(wall_x, y, z);
          T.linear() = Eigen::AngleAxisd(M_PI / 2, Vec3::UnitY()).toRotationMatrix();
          r.targets_world.push_back(T);
        }
      }
      r.wall_point = Vec3(wall_x, 0.0, 0.0);
      r.wall_normal = Vec3(-1.0, 0.0, 0.0);
      r.lift_max = num(o, "lift-max", 1.5);
      r.arm_mount_offset_z = num(o, "mount-z", 0.2);
      r.min_clearance = num(o, "min-clearance", 0.3);
      r.grid_step = grid;
      const auto prof = o.find("base-profile");
      if (prof != o.end()) {
        r.base = loadBaseProfile(prof->second);
        if (!o.count("mount-z")) {
          if (const auto z = armBaseHeightAtLiftZero(*r.base)) {
            r.arm_mount_offset_z = *z;
            std::printf("arm base %.3f m above the ground at lift zero (from the profile; --mount-z overrides)\n", *z);
          }
        }
        std::printf("base %s (%s%s)\n", r.base->name.c_str(), r.base->can_strafe ? "strafes" : "no strafe",
          r.base->drive == DriveType::kDiff || r.base->drive == DriveType::kSkid ? ", diff/skid" : "");
      }
      if (num(o, "multi", 0) != 0) {
        const Stand start{Eigen::Vector2d(num(o, "start-x", 0.0), num(o, "start-y", -half)), 0.0};
        const auto strat = o.find("strategy");
        if (strat != o.end() && strat->second == "global") {
          StandSelectionOptions so;
          so.margin_penalty = num(o, "margin-penalty", so.margin_penalty);
          so.time_budget_s = num(o, "time-budget", so.time_budget_s);
          const StandSelection sel = selectStands(map, r, start, so);
          std::printf("%s (%s): %zu stand(s) for %zu targets, %zu unreachable, tour %.1f vs cover order %.1f%s, %.3f s solve\n",
            sel.success ? "ok" : "failed", toString(sel.code), sel.stands.size(), r.targets_world.size(),
            sel.unreachable.size(), sel.tour_cost, sel.input_order_cost, r.base ? " s" : " m", sel.solve_time_s);
          for (const auto & a : sel.stands) {
            std::printf("  arm base (%.2f, %.2f) yaw %.0f deg: %zu targets, %zu band(s), margin %.2f, +%.1f\n",
              a.stand.arm_xy.x(), a.stand.arm_xy.y(), a.stand.yaw * 180.0 / M_PI, a.targets.size(),
              a.lift_bands.bands.size(), a.mean_margin, a.travel_cost);
          }
          for (const size_t t : sel.unreachable) {std::printf("  unreachable target %zu\n", t);}
          return sel.success ? 0 : 1;
        }
        const StandPlan sp = planStands(map, r, start, 32,
          parseStandStrategy(strat == o.end() ? "greedy" : strat->second));
        std::printf("%zu stand(s) for %zu targets, %zu unreachable, %zu not planned (stand cap), %.1f s driving\n",
          sp.stands.size(), r.targets_world.size(), sp.unreachable.size(), sp.not_planned.size(), sp.total_travel_s);
        for (const auto & a : sp.stands) {
          std::printf("  arm base (%.2f, %.2f) yaw %.0f deg: %zu targets, %zu band(s), +%.1f s\n", a.station.x,
            a.station.y, a.station.yaw * 180.0 / M_PI, a.targets.size(), a.station.plan.bands.size(), a.travel_s);
        }
        return sp.unreachable.empty() && sp.not_planned.empty() ? 0 : 1;
      }
      const Station s = chooseStation(map, r);
      if (!s.found) {
        std::printf("no station reaches any of the %zu targets\n", r.targets_world.size());
        return 1;
      }
      std::printf("station: arm base (%.2f, %.2f), %.2f m from the wall, %zu band(s), %zu unreachable of %zu targets\n",
        s.x, s.y, s.standoff, s.plan.bands.size(), s.plan.unreachable.size(), r.targets_world.size());
      if (r.base) {
        std::printf("  base yaw %.0f deg, footprint clearance %.2f m\n", s.yaw * 180.0 / M_PI, s.footprint_clearance);
      }
      return s.complete() ? 0 : 1;   // like `bands`: a partial station is a failure for scripts
    }
  } catch (const std::exception & e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return usage();
}
