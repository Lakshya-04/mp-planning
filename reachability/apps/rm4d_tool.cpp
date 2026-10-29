// Command-line tool: build, inspect and query reachability maps. ROS-free.
//
//   rm4d_tool build chain.yaml out.rm4d [--samples N] [--threads T] [--resolution m]
//                                       [--extent m] [--zmin m] [--zmax m] [--seed S]
//   rm4d_tool info map.rm4d
//   rm4d_tool bands map.rm4d --wall-x X --z-min Z0 --z-max Z1 [--z-step dz] [--approach +x]
//                            [--lift-max H] [--lift-step dh] [--mount-z dz] [--base-x bx] [--base-y by]
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>

#include "reachability/core/arm_model.hpp"
#include "reachability/core/builder.hpp"
#include "reachability/core/io.hpp"
#include "reachability/core/lift_bands.hpp"

using namespace reachability;

namespace
{
std::map<std::string, std::string> options(int argc, char ** argv, int first)
{
  std::map<std::string, std::string> o;
  for (int i = first; i + 1 < argc; i += 2) {
    if (std::strncmp(argv[i], "--", 2) != 0) {
      std::fprintf(stderr, "unexpected argument '%s'\n", argv[i]);
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

int usage()
{
  std::fprintf(stderr,
    "usage:\n  rm4d_tool build chain.yaml out.rm4d [--samples N --threads T --resolution m --extent m --zmin m --zmax m --seed S]\n"
    "  rm4d_tool info map.rm4d\n"
    "  rm4d_tool bands map.rm4d --wall-x X --z-min Z0 --z-max Z1 [--z-step dz --lift-max H --lift-step dh --mount-z dz --base-x bx --base-y by]\n");
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
      const Map4D map = loadMap(argv[2]);
      const auto o = options(argc, argv, 3);
      const double wall_x = num(o, "wall-x", 0.6), z0 = num(o, "z-min", 0.1), z1 = num(o, "z-max", 2.5);
      const double dz = num(o, "z-step", 0.1);
      std::vector<Pose> targets;
      for (double z = z0; z <= z1 + 1e-9; z += dz) {
        Pose T = Pose::Identity();
        T.translation() = Vec3(wall_x, 0.0, z);
        T.linear() = Eigen::AngleAxisd(M_PI / 2, Vec3::UnitY()).toRotationMatrix();  // tool points into the wall (+x)
        targets.push_back(T);
      }
      const auto plan = planLiftBands(map, targets, num(o, "base-x", 0.0), num(o, "base-y", 0.0),
        heightRange(0.0, num(o, "lift-max", 1.5), num(o, "lift-step", 0.05)), num(o, "mount-z", 0.2));
      std::printf("%zu targets, %zu reachable, %zu band(s):\n", targets.size(), plan.reachable_count, plan.bands.size());
      for (const auto & b : plan.bands) {
        std::printf("  lift %.2f m covers %zu target(s)\n", b.lift_height, b.pose_indices.size());
      }
      if (!plan.unreachable.empty()) {
        std::printf("  %zu target(s) unreachable from this station\n", plan.unreachable.size());
      }
      return plan.unreachable.empty() ? 0 : 1;
    }
  } catch (const std::exception & e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return usage();
}
