// What the map builder needs from an arm: its joint limits and a forward-kinematics + validity
// evaluation. Backends (Pinocchio, VAMP, cuRobo, MoveIt) implement ArmModel; ChainModel is a
// dependency-free implementation from a joint chain description (e.g. generated from the URDF).
#pragma once

#include <functional>
#include <string>
#include <vector>

#include "reachability/core/map4d.hpp"

namespace reachability
{

class ArmModel
{
public:
  virtual ~ArmModel() = default;

  virtual int dof() const = 0;
  virtual double lower(int joint) const = 0;
  virtual double upper(int joint) const = 0;

  /// Forward kinematics of the tool in the arm base frame (tool z = approach axis).
  /// Returns false when the configuration is invalid (joint limits, self-collision, ...); the
  /// builder drops such samples. Must be safe to call from several threads.
  virtual bool evaluate(const std::vector<double> & q, Pose & tool_in_arm_base) const = 0;

  /// Stable text that changes when the robot changes (used to reject a map built for another
  /// robot). Default: empty = not checked.
  virtual std::string fingerprint() const {return "";}
};

struct ChainJoint
{
  Pose origin{Pose::Identity()};   ///< Joint frame in its parent link frame (at q = 0).
  Vec3 axis{Vec3::UnitZ()};        ///< Rotation axis in the joint frame.
  double lower{-M_PI};
  double upper{M_PI};
};

/// Serial chain of revolute joints: T = prod_i (origin_i * Rot(axis_i, q_i)) * tool.
class ChainModel : public ArmModel
{
public:
  ChainModel(std::vector<ChainJoint> joints, Pose tool = Pose::Identity());

  /// Optional extra validity test (self-collision, ...) applied after FK.
  void setValidity(std::function<bool(const std::vector<double> &)> valid) {valid_ = std::move(valid);}

  int dof() const override {return static_cast<int>(joints_.size());}
  double lower(int j) const override {return joints_.at(static_cast<size_t>(j)).lower;}
  double upper(int j) const override {return joints_.at(static_cast<size_t>(j)).upper;}
  bool evaluate(const std::vector<double> & q, Pose & tool) const override;
  std::string fingerprint() const override;

  const std::vector<ChainJoint> & joints() const {return joints_;}
  const Pose & tool() const {return tool_;}

private:
  std::vector<ChainJoint> joints_;
  Pose tool_;
  std::function<bool(const std::vector<double> &)> valid_;
};

/// Loads a chain from YAML:
///   joints:
///     - {xyz: [x, y, z], rpy: [r, p, y], axis: [ax, ay, az], lower: -3.14, upper: 3.14}
///   tool: {xyz: [..], rpy: [..]}
/// Written by scripts/urdf_to_chain.py from a URDF. Throws std::runtime_error on bad input.
ChainModel loadChainYaml(const std::string & path);

}  // namespace reachability
