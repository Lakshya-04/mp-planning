// How much of a column plan can the arm do? Reads an RM4D map and a plan (JSON or YAML), plans stands that
// keep columns whole where possible, prints the analysis as JSON. ROS-free.
//
//   column_reach_tool map.rm4d plan.json [--out report.json] [--base-profile profile.yaml]
//                                        [--start-x X --start-y Y --start-yaw A]
//
// plan.json (every key but `columns` and `wall_normal` is optional; lengths in metres):
//   {"wall_point": [x,y,z], "wall_normal": [nx,ny,nz],          (normal points into the free side)
//    "arm_mount_offset_z": 0.2, "lift_min": 0, "lift_max": 1.5, "lift_step": 0.05,
//    "grid_step": 0.1, "min_clearance": 0.3, "max_standoff": 1.6, "lateral_margin": 0.5,
//    "sequence": {"standoff_m": 0.12, "approach_extra_m": 0.1, "press_depth_m": 0.02, "retract_m": -1,
//                 "dock_at_first_stroke": false},
//    "margin": {"min_score": 0.15, "count_ref": 8},
//    "margin_penalty": 0.5, "allow_splits": true, "max_stands": 32,
//    "columns": [{"label": "c0", "poses": [[x,y,z,qx,qy,qz,qw], ...]}, ...]}
// Poses are tool poses, tool z axis = approach axis. The result is an RM4D-level estimate (no IK, no collision).
#include <cstdio>
#include <fstream>
#include <map>
#include <string>

#include <yaml-cpp/yaml.h>

#include "reachability/core/base_model.hpp"
#include "reachability/core/column_analysis.hpp"
#include "reachability/core/io.hpp"

using namespace reachability;

namespace
{
template<typename T>
T get(const YAML::Node & n, const char * key, T dflt)
{
  return n && n[key] ? n[key].as<T>() : dflt;
}

Vec3 vec(const YAML::Node & n)
{
  if (!n || n.size() != 3) {throw std::runtime_error("expected [x, y, z]");}
  return Vec3(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::fprintf(stderr, "usage: column_reach_tool map.rm4d plan.json [--out f] [--base-profile f] [--start-x X --start-y Y --start-yaw A]\n");
    return 2;
  }
  std::map<std::string, std::string> opt;
  for (int i = 3; i + 1 < argc; i += 2) {
    if (std::string(argv[i]).rfind("--", 0) != 0) {
      std::fprintf(stderr, "unexpected argument '%s'\n", argv[i]);
      return 2;
    }
    opt[argv[i] + 2] = argv[i + 1];
  }
  try {
    const Map4D map = loadMap(argv[1]);
    const YAML::Node plan = YAML::LoadFile(argv[2]);

    StationRequest st;
    st.wall_point = plan["wall_point"] ? vec(plan["wall_point"]) : Vec3::Zero();
    st.wall_normal = vec(plan["wall_normal"]);
    st.arm_mount_offset_z = get(plan, "arm_mount_offset_z", 0.0);
    st.lift_min = get(plan, "lift_min", 0.0);
    st.lift_max = get(plan, "lift_max", 1.5);
    st.lift_step = get(plan, "lift_step", 0.05);
    st.grid_step = get(plan, "grid_step", 0.1);
    st.min_clearance = get(plan, "min_clearance", 0.3);
    st.max_standoff = get(plan, "max_standoff", 1.6);
    st.lateral_margin = get(plan, "lateral_margin", 0.5);
    std::optional<Stand> start;
    if (opt.count("base-profile")) {st.base = loadBaseProfile(opt["base-profile"]);}
    if (opt.count("start-x") && opt.count("start-y")) {
      Stand s;
      s.arm_xy = Eigen::Vector2d(std::stod(opt["start-x"]), std::stod(opt["start-y"]));
      s.yaw = opt.count("start-yaw") ? std::stod(opt["start-yaw"]) : 0.0;
      start = s;
    }

    ColumnStandOptions o;
    const YAML::Node sq = plan["sequence"], mg = plan["margin"];
    o.sequence.standoff_m = get(sq, "standoff_m", o.sequence.standoff_m);
    o.sequence.approach_extra_m = get(sq, "approach_extra_m", o.sequence.approach_extra_m);
    o.sequence.press_depth_m = get(sq, "press_depth_m", o.sequence.press_depth_m);
    o.sequence.retract_m = get(sq, "retract_m", o.sequence.retract_m);
    o.sequence.dock_at_first_stroke = get(sq, "dock_at_first_stroke", false);
    o.margin.min_score = get(mg, "min_score", o.margin.min_score);
    o.margin.count_ref = get(mg, "count_ref", o.margin.count_ref);
    o.margin_penalty = get(plan, "margin_penalty", o.margin_penalty);
    o.allow_splits = get(plan, "allow_splits", true);
    o.max_stands = get(plan, "max_stands", static_cast<size_t>(32));

    std::vector<ColumnPlan> cols;
    for (const auto & c : plan["columns"]) {
      ColumnPlan cp;
      cp.label = get<std::string>(c, "label", "column_" + std::to_string(cols.size()));
      for (const auto & p : c["poses"]) {
        if (p.size() != 7) {throw std::runtime_error("a pose is [x, y, z, qx, qy, qz, qw]");}
        Pose T = Pose::Identity();
        T.translation() = Vec3(p[0].as<double>(), p[1].as<double>(), p[2].as<double>());
        T.linear() = Eigen::Quaterniond(p[6].as<double>(), p[3].as<double>(), p[4].as<double>(), p[5].as<double>())
          .normalized().toRotationMatrix();
        cp.strokes_world.push_back(T);
      }
      cols.push_back(std::move(cp));
    }
    const std::string json = toJson(analyzeColumnCoverage(map, st, cols, start, o));
    if (opt.count("out")) {
      std::ofstream(opt["out"]) << json;
    } else {
      std::fputs(json.c_str(), stdout);
    }
  } catch (const std::exception & e) {
    std::fprintf(stderr, "column_reach_tool: %s\n", e.what());
    return 1;
  }
  return 0;
}
