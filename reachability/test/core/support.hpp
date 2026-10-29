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
