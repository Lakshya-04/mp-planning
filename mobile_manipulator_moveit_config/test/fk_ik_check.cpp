// Loads a rendered URDF + SRDF into MoveIt and reports what a planner would see for one group:
// the active joint names, then a forward/inverse-kinematics round trip with the KDL plugin.
//
//   fk_ik_check <urdf-file> <srdf-file> <group> [samples]
//
// Output, one key=value line each (parsed by test_moveit_swap.py):
//   dof=<n>   joints=<a,b,...>   tip=<link>   ik_ok=<n>   samples=<n>   max_pos_error_m=<m>   max_rot_error_rad=<rad>
// Each sample draws a random state inside the joint limits (collision not considered), takes the
// tip pose by FK, then solves IK for that pose from a seed perturbed by 0.1 rad and compares the
// FK of the solution to the target pose.
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "moveit/robot_model_loader/robot_model_loader.hpp"
#include "moveit/robot_state/robot_state.hpp"
#include "random_numbers/random_numbers.h"
#include "rclcpp/rclcpp.hpp"

namespace
{
std::string slurp(const std::string & path)
{
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 4) {
    std::cerr << "usage: fk_ik_check <urdf> <srdf> <group> [samples]\n";
    return 2;
  }
  const std::string group = argv[3];
  const int samples = argc > 4 ? std::stoi(argv[4]) : 20;
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  const std::string kin = "robot_description_kinematics." + group + ".";
  options.parameter_overrides(
    {{"robot_description", slurp(argv[1])},
      {"robot_description_semantic", slurp(argv[2])},
      {kin + "kinematics_solver", std::string("kdl_kinematics_plugin/KDLKinematicsPlugin")},
      {kin + "kinematics_solver_search_resolution", 0.005},
      {kin + "kinematics_solver_timeout", 0.1},
      {kin + "kinematics_solver_attempts", 5}});
  auto node = std::make_shared<rclcpp::Node>("fk_ik_check", options);

  robot_model_loader::RobotModelLoader loader(node, "robot_description", true);
  const auto model = loader.getModel();
  if (!model) {
    std::cerr << "robot model failed to load\n";
    return 3;
  }
  const auto * jmg = model->getJointModelGroup(group);
  if (!jmg) {
    std::cerr << "no such group: " << group << "\n";
    return 4;
  }
  std::string joints;
  for (const auto & name : jmg->getActiveJointModelNames()) {
    joints += (joints.empty() ? "" : ",") + name;
  }
  const std::string tip = jmg->getSolverInstance() ? jmg->getSolverInstance()->getTipFrame() : "";
  std::cout << "dof=" << jmg->getActiveVariableCount() << "\njoints=" << joints << "\ntip=" << tip
            << "\n";
  if (!jmg->getSolverInstance()) {
    std::cerr << "no kinematics solver for group " << group << "\n";
    return 5;
  }

  moveit::core::RobotState state(model);
  int ok = 0;
  double max_pos = 0.0, max_rot = 0.0;
  random_numbers::RandomNumberGenerator rng(42);   // fixed seed: the same samples every run
  for (int i = 0; i < samples; ++i) {
    state.setToRandomPositions(jmg, rng);
    state.update();
    const Eigen::Isometry3d target = state.getGlobalLinkTransform(tip);
    std::vector<double> q;
    state.copyJointGroupPositions(jmg, q);
    moveit::core::RobotState seeded(state);
    for (auto & v : q) {v += 0.1 * ((std::rand() % 2000) / 1000.0 - 1.0);}
    seeded.setJointGroupPositions(jmg, q);
    seeded.enforceBounds();
    if (!seeded.setFromIK(jmg, target, tip, 0.2)) {continue;}
    seeded.update();
    const Eigen::Isometry3d got = seeded.getGlobalLinkTransform(tip);
    const double pos = (got.translation() - target.translation()).norm();
    const double rot = Eigen::AngleAxisd(got.rotation().transpose() * target.rotation()).angle();
    max_pos = std::max(max_pos, pos);
    max_rot = std::max(max_rot, rot);
    ++ok;
  }
  std::cout << "samples=" << samples << "\nik_ok=" << ok << "\nmax_pos_error_m=" << max_pos
            << "\nmax_rot_error_rad=" << max_rot << "\n";
  rclcpp::shutdown();
  return 0;
}
