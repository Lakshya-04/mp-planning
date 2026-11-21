// Per-pose quality: how far a pose is from the edge of what the arm reaches, and (with a joint solution)
// how far the arm is from a singularity or a joint limit.
//
// The RM4D map stores, per cell, how many sampled arm configurations land in it (saturating at 255). That
// is a density, not a manipulability index, but it is what the map has and it falls off towards the
// workspace boundary, where the arm is stretched out or folded (the reach limit and the usual singular
// configurations). scorePose turns it into a score in [0, 1]:
//   count score     min(count, count_ref) / count_ref;
//   boundary score  share of the 8 neighbouring cells (one step in x*, y*, z and theta) that are
//                   reachable: 1 inside the workspace, lower at its edge.
//   score           the smaller of the two; 0 for an empty cell.
// The score is an estimate, not a measurement of manipulability: sample counts depend on the sampler and
// the build's sample count, so `count_ref` has to be set against the map in use (see docs/reachability.md).
//
// When a joint solution is known (a StandVerifier returned one) minSingularValue and jointLimitMargin give
// the real quantities for the arm model. They are unscaled (metres and radians share one Jacobian), so
// the threshold is a starting value, not a physical constant.
#pragma once

#include <vector>

#include <Eigen/Dense>

#include "reachability/core/arm_model.hpp"
#include "reachability/core/map4d.hpp"

namespace reachability
{

struct MarginConfig
{
  /// A pose with a score under this is not usable. 0 = only the cell's emptiness counts.
  double min_score{0.15};
  /// Map count at which the count score saturates (>= 1).
  double count_ref{8.0};
  /// Verified checks with a joint solution: smallest Jacobian singular value under this makes the pose
  /// unusable. 0 = off.
  double min_singular_value{0.0};
  /// ... and a joint within this share of its range of a limit. 0 = off.
  double min_limit_margin{0.0};
};

struct PoseScore
{
  bool reachable{false};       ///< The cell is non-empty.
  uint8_t count{0};
  double count_score{0.0};
  double boundary_score{0.0};
  double score{0.0};           ///< min(count_score, boundary_score).
};

/// Score of a tool pose given in the arm base frame.
PoseScore scorePose(const Map4D & map, const Pose & tool_in_arm_base, const MarginConfig & config);

/// 6 x dof Jacobian at `q` (columns: linear velocity then rotation vector of the tool, by finite differences
/// of ArmModel::evaluate). False when evaluate refuses `q` or the size is wrong.
bool armJacobian(const ArmModel & arm, const std::vector<double> & q, Eigen::MatrixXd & J, double eps = 1e-5);

/// Smallest singular value of a Jacobian of any shape. -1 for an empty matrix. Zero or near it is a
/// singular configuration.
double minSingularValue(const Eigen::MatrixXd & J);
/// Largest over smallest singular value; +infinity when the smallest is zero (relative 1e-12). -1 if empty.
double conditionNumber(const Eigen::MatrixXd & J);
/// Yoshikawa measure: product of the singular values (sqrt(det(J J^T)) for full row rank). -1 if empty.
double manipulability(const Eigen::MatrixXd & J);

/// Same three for an arm model at `q`; -1 when evaluate refuses `q`. Unscaled (metres and radians share one
/// Jacobian). Work for any dof.
double minSingularValue(const ArmModel & arm, const std::vector<double> & q, double eps = 1e-5);
double conditionNumber(const ArmModel & arm, const std::vector<double> & q, double eps = 1e-5);
double manipulability(const ArmModel & arm, const std::vector<double> & q, double eps = 1e-5);

/// Smallest distance of a joint to its limit as a share of that joint's range, in [0, 0.5]. Joints with a
/// range of 2*pi or more (continuous) are ignored; 0.5 when every joint is ignored. -1 for a size mismatch.
double jointLimitMargin(const ArmModel & arm, const std::vector<double> & q);

}  // namespace reachability
