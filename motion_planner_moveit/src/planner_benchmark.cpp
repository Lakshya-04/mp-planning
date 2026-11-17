// MoveIt / OMPL side of scripts/planner_benchmark.py: plans the benchmark problems (the text format of
// docker/tools/vamp_bench.cpp) with OMPL RRT-Connect, validity from a MoveIt PlanningScene (FCL on the
// URDF collision geometry, the SRDF allowed-collision matrix, boxes as world objects), and writes one
// JSON object per line.
//
//   planner_benchmark robot.urdf robot.srdf GROUP problems.txt out.jsonl LABEL [timeout_s]
//
// This is the planner MoveIt's OMPL pipeline runs (RRT-Connect, state-validity resolution 0.1 percent of
// the extent, then path simplification) without the move_group node, request adapters or time
// parameterisation, so the numbers describe the planning core, not a full move_group round trip.
// Timing is wall clock around solve + simplify. Not a ROS node: no rclcpp, no sim.
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometric_shapes/shapes.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <srdfdom/model.h>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>

namespace
{

std::string slurp(const std::string & path)
{
  std::ifstream f(path);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

struct Box
{
  Eigen::Vector3d c;
  Eigen::Matrix3d R;
  Eigen::Vector3d half;
};

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 7) {
    std::fprintf(
      stderr, "usage: %s robot.urdf robot.srdf group problems.txt out.jsonl label [timeout_s]\n",
      argv[0]);
    return 2;
  }
  const std::string group = argv[3];
  const double timeout = argc > 7 ? std::stod(argv[7]) : 5.0;

  auto urdf_model = urdf::parseURDF(slurp(argv[1]));
  auto srdf_model = std::make_shared<srdf::Model>();
  if (!urdf_model || !srdf_model->initString(*urdf_model, slurp(argv[2]))) {
    std::fprintf(stderr, "cannot load urdf/srdf\n");
    return 2;
  }
  auto robot = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
  const auto * jmg = robot->getJointModelGroup(group);
  if (!jmg) {
    std::fprintf(stderr, "no group %s\n", group.c_str());
    return 2;
  }
  const std::size_t dim = jmg->getActiveVariableCount();

  std::FILE * out = std::fopen(argv[5], "w");
  std::ifstream in(argv[4]);
  if (!out || !in) {
    std::fprintf(stderr, "cannot open files\n");
    return 2;
  }
  std::fprintf(
    out, "{\"type\":\"header\",\"backend\":\"moveit_ompl\",\"set\":\"%s\",\"robot\":\"%s\","
    "\"n_spheres\":0,\"dim\":%zu}\n", argv[6], argv[1], dim);

  // Space bounds are the joint limits of the group; the benchmark's own configurations are within.
  auto space = std::make_shared<ompl::base::RealVectorStateSpace>(dim);
  ompl::base::RealVectorBounds bounds(dim);
  const auto & bvec = jmg->getActiveJointModelsBounds();
  for (std::size_t i = 0; i < dim; ++i) {
    const auto & b = *bvec[i];
    bounds.setLow(i, b[0].min_position_);
    bounds.setHigh(i, b[0].max_position_);
  }
  space->setBounds(bounds);

  std::string line;
  int scene_id = -1;
  std::size_t nbox = 0, nprob = 0, nprobe = 0;
  std::shared_ptr<planning_scene::PlanningScene> scene;
  bool first = true;
  moveit::core::RobotState state(robot);
  state.setToDefaultValues();
  auto set_state = [&](const double * q) {
      state.setJointGroupPositions(jmg, std::vector<double>(q, q + dim));
      state.update();
    };

  while (std::getline(in, line)) {
    std::istringstream ls(line);
    std::string tag;
    ls >> tag;
    if (tag == "S") {
      ls >> scene_id >> nbox >> nprob >> nprobe;
      scene = std::make_shared<planning_scene::PlanningScene>(robot);
      for (std::size_t b = 0; b < nbox; ++b) {
        std::getline(in, line);
        std::istringstream bs(line);
        std::string t;
        bs >> t;
        Box box;
        double v[15];
        for (double & x : v) {bs >> x;}
        box.c = {v[0], v[1], v[2]};
        box.R.col(0) = Eigen::Vector3d(v[3], v[4], v[5]);
        box.R.col(1) = Eigen::Vector3d(v[6], v[7], v[8]);
        box.R.col(2) = Eigen::Vector3d(v[9], v[10], v[11]);
        box.half = {v[12], v[13], v[14]};
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.linear() = box.R;
        pose.translation() = box.c;
        scene->getWorldNonConst()->addToObject(
          "box" + std::to_string(b),
          std::make_shared<shapes::Box>(2 * box.half.x(), 2 * box.half.y(), 2 * box.half.z()), pose);
      }
      auto valid = [&](const double * q) {
          set_state(q);
          return scene->isStateValid(state, group);
        };
      for (std::size_t p = 0; p < nprob; ++p) {
        std::getline(in, line);
        std::istringstream ps(line);
        std::string t;
        int id;
        ps >> t >> id;
        std::vector<double> start(dim), goal(dim);
        for (auto & x : start) {ps >> x;}
        for (auto & x : goal) {ps >> x;}

        ompl::geometric::SimpleSetup ss(space);
        ss.setStateValidityChecker(
          [&](const ompl::base::State * s) {
            return valid(s->as<ompl::base::RealVectorStateSpace::StateType>()->values);
          });
        ss.getSpaceInformation()->setStateValidityCheckingResolution(0.001);
        ss.setPlanner(std::make_shared<ompl::geometric::RRTConnect>(ss.getSpaceInformation()));
        const bool sv = valid(start.data()), gv = valid(goal.data());
        bool ok = false;
        double us = 0;
        std::vector<std::vector<double>> path;
        if (sv && gv) {
          ompl::base::ScopedState<> s0(space), g0(space);
          for (std::size_t i = 0; i < dim; ++i) {s0[i] = start[i]; g0[i] = goal[i];}
          ss.setStartAndGoalStates(s0, g0);
          const auto t0 = std::chrono::steady_clock::now();
          ok = ss.solve(timeout) == ompl::base::PlannerStatus::EXACT_SOLUTION;
          if (ok) {ss.simplifySolution();}
          us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
          if (ok) {
            for (const auto * st : ss.getSolutionPath().getStates()) {
              const double * v = st->as<ompl::base::RealVectorStateSpace::StateType>()->values;
              path.emplace_back(v, v + dim);
            }
          }
        }
        if (first) {
          std::fprintf(out, "{\"type\":\"first_call\",\"us\":%.1f}\n", us);
          first = false;
        }
        std::fprintf(
          out, "{\"type\":\"plan\",\"scene\":%d,\"id\":%d,\"success\":%d,\"us\":%.1f,"
          "\"endpoints_valid\":[%d,%d],\"path\":[", scene_id, id, ok ? 1 : 0, us, sv ? 1 : 0, gv ? 1 : 0);
        for (std::size_t i = 0; i < path.size(); ++i) {
          std::fprintf(out, "%s[", i ? "," : "");
          for (std::size_t j = 0; j < dim; ++j) {
            std::fprintf(out, "%s%.5f", j ? "," : "", path[i][j]);
          }
          std::fprintf(out, "]");
        }
        std::fprintf(out, "]}\n");
        std::fflush(out);
      }
      for (std::size_t k = 0; k < nprobe; ++k) {std::getline(in, line);}  // probes unused here
    }
  }
  std::fclose(out);
  return 0;
}
