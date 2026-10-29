#include "reachability/core/arm_model.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace reachability
{

ChainModel::ChainModel(std::vector<ChainJoint> joints, Pose tool)
: joints_(std::move(joints)), tool_(std::move(tool))
{
  if (joints_.empty()) {
    throw std::invalid_argument("ChainModel: no joints");
  }
  for (auto & j : joints_) {
    if (j.axis.norm() < 1e-9 || j.lower > j.upper) {
      throw std::invalid_argument("ChainModel: bad joint axis or limits");
    }
    j.axis.normalize();
  }
}

bool ChainModel::evaluate(const std::vector<double> & q, Pose & tool) const
{
  if (q.size() != joints_.size()) {
    return false;
  }
  Pose T = Pose::Identity();
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (q[i] < joints_[i].lower || q[i] > joints_[i].upper) {
      return false;
    }
    T = T * joints_[i].origin * Pose(Eigen::AngleAxisd(q[i], joints_[i].axis));
  }
  T = T * tool_;
  if (valid_ && !valid_(q)) {
    return false;
  }
  tool = T;
  return true;
}

std::string ChainModel::fingerprint() const
{
  std::ostringstream s;
  s << std::setprecision(9);
  for (const auto & j : joints_) {
    s << j.origin.translation().transpose() << '|' << Eigen::Quaterniond(j.origin.linear()).coeffs().transpose()
      << '|' << j.axis.transpose() << '|' << j.lower << ',' << j.upper << ';';
  }
  s << tool_.translation().transpose() << '|' << Eigen::Quaterniond(tool_.linear()).coeffs().transpose();
  return s.str();
}

namespace
{

Pose poseFrom(const YAML::Node & n)
{
  Pose T = Pose::Identity();
  if (!n) {
    return T;
  }
  if (n["xyz"]) {
    T.translation() = Vec3(n["xyz"][0].as<double>(), n["xyz"][1].as<double>(), n["xyz"][2].as<double>());
  }
  if (n["rpy"]) {
    const double r = n["rpy"][0].as<double>(), p = n["rpy"][1].as<double>(), y = n["rpy"][2].as<double>();
    T.linear() = (Eigen::AngleAxisd(y, Vec3::UnitZ()) * Eigen::AngleAxisd(p, Vec3::UnitY()) *
      Eigen::AngleAxisd(r, Vec3::UnitX())).toRotationMatrix();
  }
  return T;
}

}  // namespace

ChainModel loadChainYaml(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    throw std::runtime_error(path + ": cannot read chain description: " + e.what());
  }
  if (!root["joints"] || !root["joints"].IsSequence() || root["joints"].size() == 0) {
    throw std::runtime_error(path + ": 'joints' must be a non-empty list");
  }
  std::vector<ChainJoint> joints;
  for (const auto & n : root["joints"]) {
    ChainJoint j;
    j.origin = poseFrom(n);
    if (n["axis"]) {
      j.axis = Vec3(n["axis"][0].as<double>(), n["axis"][1].as<double>(), n["axis"][2].as<double>());
    }
    if (n["lower"]) {j.lower = n["lower"].as<double>();}
    if (n["upper"]) {j.upper = n["upper"].as<double>();}
    joints.push_back(j);
  }
  return ChainModel(std::move(joints), poseFrom(root["tool"]));
}

}  // namespace reachability
