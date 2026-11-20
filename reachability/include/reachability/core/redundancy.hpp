// Redundancy policy for maps of arms with more than six joints (7-DOF arms and up).
//
// RM4D needs only the tool pose, so for a redundant arm the same pose is reached by a whole
// family of joint configurations (the null space; a 7-axis arm has a one-parameter family, the
// "elbow angle"). Building the map by uniform joint sampling already marks a cell when ANY
// null-space value reaches it. That union is the default and is what a 6-DOF map is too (no
// freedom, nothing to choose). A RedundancyPolicy narrows the sampling instead: one joint (the
// null-space proxy, usually the upper-arm roll, joint 3 of a 7-axis arm) is drawn only from a
// configured set of values, so the map answers "reachable with that joint near one of these
// values". The policy is written into the map header (see MapInfo in io.hpp).
#pragma once

#include <random>
#include <string>
#include <vector>

#include "reachability/core/arm_model.hpp"

namespace reachability
{

struct RedundancyPolicy
{
  int joint{-1};               ///< Index (0-based) of the restricted joint; -1 = no restriction (union).
  std::vector<double> values;  ///< Allowed values (rad); each sample picks one uniformly.
  double tolerance{0.0};       ///< Half-width (rad) of a uniform spread around the chosen value.

  bool active() const {return joint >= 0;}

  /// Throws std::invalid_argument when the joint index is out of range, no value is given, or a
  /// value lies outside the joint's limits by more than the tolerance.
  void validate(const ArmModel & arm) const;

  /// One value for the restricted joint, clipped to [lower, upper].
  double draw(std::mt19937_64 & rng, double lower, double upper) const;

  /// Text stored in the map header, e.g. "none" or "joint 3 in {-0.5, 0, 0.5} +- 0.05 rad".
  std::string describe() const;
};

/// What a map says about the arm it was built for, beyond the robot fingerprint.
std::string describeRedundancy(int arm_dof, const RedundancyPolicy & policy);

}  // namespace reachability
