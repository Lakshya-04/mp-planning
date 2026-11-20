#include "reachability/core/redundancy.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace reachability
{

void RedundancyPolicy::validate(const ArmModel & arm) const
{
  if (!active()) {
    return;
  }
  if (joint >= arm.dof()) {
    throw std::invalid_argument(
            "redundancy: joint " + std::to_string(joint + 1) + " does not exist on a " +
            std::to_string(arm.dof()) + "-joint arm");
  }
  if (values.empty()) {
    throw std::invalid_argument("redundancy: a restricted joint needs at least one value");
  }
  if (tolerance < 0.0) {
    throw std::invalid_argument("redundancy: tolerance must not be negative");
  }
  for (double v : values) {
    if (v < arm.lower(joint) - tolerance - 1e-9 || v > arm.upper(joint) + tolerance + 1e-9) {
      throw std::invalid_argument(
              "redundancy: value " + std::to_string(v) + " is outside the limits of joint " +
              std::to_string(joint + 1));
    }
  }
}

double RedundancyPolicy::draw(std::mt19937_64 & rng, double lower, double upper) const
{
  double v = values[std::uniform_int_distribution<size_t>(0, values.size() - 1)(rng)];
  if (tolerance > 0.0) {
    v += std::uniform_real_distribution<double>(-tolerance, tolerance)(rng);
  }
  return std::min(upper, std::max(lower, v));
}

std::string RedundancyPolicy::describe() const
{
  return describeRedundancy(0, *this);
}

std::string describeRedundancy(int arm_dof, const RedundancyPolicy & policy)
{
  if (!policy.active()) {
    return arm_dof > 6 ? "union over every value of the null space (no restriction)" : "none";
  }
  std::ostringstream s;
  s << "joint " << policy.joint + 1 << " in {";
  for (size_t i = 0; i < policy.values.size(); ++i) {
    s << (i ? ", " : "") << policy.values[i];
  }
  s << "} +- " << policy.tolerance << " rad";
  return s.str();
}

}  // namespace reachability
