#pragma once

#include <vector>

#include "reachability/core/arm_model.hpp"

namespace reachability_test
{

// The generic 6-DOF reference arm of mobile_manipulator_description (UR-like proportions and axis
// pattern z, y, y, y, z, y), tool 0.165 m beyond the flange. Limits +-pi.
inline reachability::ChainModel genericArm()
{
  using reachability::ChainJoint;
  using reachability::Pose;
  using reachability::Vec3;
  auto joint = [](Vec3 xyz, Vec3 axis) {
      ChainJoint j;
      j.origin.translation() = xyz;
      j.axis = axis;
      j.lower = -M_PI;
      j.upper = M_PI;
      return j;
    };
  std::vector<ChainJoint> joints = {
    joint({0, 0, 0}, Vec3::UnitZ()),
    joint({0, 0, 0.18}, Vec3::UnitY()),
    joint({0, -0.12, 0.612}, Vec3::UnitY()),
    joint({0, 0, 0.572}, Vec3::UnitY()),
    joint({0, 0.12, 0}, Vec3::UnitZ()),
    joint({0, 0, 0.12}, Vec3::UnitY()),
  };
  Pose tool = Pose::Identity();
  tool.translation() = Vec3(0, 0.06, 0);
  tool.linear() = Eigen::AngleAxisd(-M_PI / 2, Vec3::UnitX()).toRotationMatrix();
  Pose tcp = Pose::Identity();
  tcp.translation() = Vec3(0, 0, 0.165);
  return reachability::ChainModel(joints, tool * tcp);
}

// A 7-DOF arm with the Franka Panda joint frames (published modified-DH values, the same chain as
// arms/panda_arm.xacro) and its limits, tool 0.2104 m beyond the flange (flange 0.107 + hand TCP
// 0.1034). Joint 3 (index 2) is the upper-arm roll, the null-space proxy used by the tests.
inline reachability::ChainModel sevenDofArm()
{
  using reachability::ChainJoint;
  using reachability::Pose;
  using reachability::Vec3;
  auto joint = [](Vec3 xyz, double roll, double lo, double hi) {
      ChainJoint j;
      j.origin.translation() = xyz;
      j.origin.linear() = Eigen::AngleAxisd(roll, Vec3::UnitX()).toRotationMatrix();
      j.axis = Vec3::UnitZ();
      j.lower = lo;
      j.upper = hi;
      return j;
    };
  const double h = M_PI / 2;
  std::vector<ChainJoint> joints = {
    joint({0, 0, 0.333}, 0, -2.8973, 2.8973),
    joint({0, 0, 0}, -h, -1.7628, 1.7628),
    joint({0, -0.316, 0}, h, -2.8973, 2.8973),
    joint({0.0825, 0, 0}, h, -3.0718, -0.0698),
    joint({-0.0825, 0.384, 0}, -h, -2.8973, 2.8973),
    joint({0, 0, 0}, h, -0.0175, 3.7525),
    joint({0.088, 0, 0}, h, -2.8973, 2.8973),
  };
  Pose tool = Pose::Identity();
  tool.translation() = Vec3(0, 0, 0.107 + 0.1034);
  return reachability::ChainModel(joints, tool);
}

inline reachability::MapParams coarse()
{
  reachability::MapParams p;
  p.xy_extent = 2.2;   // wider than the arm's ~1.5 m reach so out-of-reach poses are inside the window
  p.z_min = -2.0;
  p.z_max = 2.2;
  p.resolution = 0.1;
  p.theta_bin = M_PI / 9;
  return p;
}

}  // namespace reachability_test
