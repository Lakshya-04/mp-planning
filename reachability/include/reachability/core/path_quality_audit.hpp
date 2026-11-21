// Path quality audit: how close a joint trajectory comes to a singularity or a joint limit.
//
// ROS-free. The audit takes the stroke trajectories a planner produced (one joint-space waypoint list per
// stroke) and, per stroke, reports the smallest Jacobian singular value and the smallest joint-limit margin
// over its waypoints, the index of the worst waypoint and a reason: `near_singular`, `near_limit` or `ok`.
// The Jacobian and the limit margin come from injected callables, so the audit runs over a real arm
// (armJacobianFn / armLimitMarginFn below) or a fake one in a test. It only reports; it never changes a path.
//
// Thresholds (PathQualityConfig) are starting values, not calibrated on hardware: the Jacobian is unscaled
// (metres and radians share it), so a singular value of 0.02 means "a unit joint speed can lose all but
// 0.02 of tool speed in the weakest direction". A threshold of 0 switches that check off.
#pragma once

#include <functional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "reachability/core/arm_model.hpp"

namespace reachability
{

struct PathQualityConfig
{
  /// A waypoint whose smallest Jacobian singular value is under this is near a singularity. 0 = off.
  /// Default matches the value the singularity gtests use to tell a bent arm (above) from a stretched one.
  double min_sigma{0.02};
  /// A waypoint whose smallest joint-limit margin (share of the joint's range, 0..0.5) is under this is near
  /// a limit. 0 = off.
  double min_limit_margin{0.02};
};

enum class PathQualityReason { kOk, kNearSingular, kNearLimit };

/// "ok", "near_singular" or "near_limit".
const char * toString(PathQualityReason reason);

/// Jacobian at q (any shape, rows = task dimensions). False when q is invalid for the model.
using JacobianFn = std::function<bool (const std::vector<double> & q, Eigen::MatrixXd & J)>;
/// Smallest limit margin at q as a share of range in [0, 0.5]; negative when q is invalid.
using LimitMarginFn = std::function<double (const std::vector<double> & q)>;

/// One stroke's joint trajectory.
struct PathStroke
{
  std::string id;
  std::vector<std::vector<double>> waypoints;
};

struct StrokeQuality
{
  std::string id;
  int waypoints{0};              ///< Waypoints in the stroke.
  int evaluated{0};              ///< Waypoints the callables accepted.
  double min_sigma{-1.0};        ///< -1 when nothing was evaluated or no Jacobian was given.
  int min_sigma_index{-1};
  double min_limit_margin{-1.0}; ///< -1 when nothing was evaluated or no margin callable was given.
  int min_limit_margin_index{-1};
  int worst_index{-1};           ///< Waypoint that caused `reason`; for ok, the smallest-sigma one (or -1).
  PathQualityReason reason{PathQualityReason::kOk};
  bool near_limit_too{false};    ///< Reason is near_singular and a limit threshold was crossed as well.
};

struct PathQualityReport
{
  std::vector<StrokeQuality> strokes;
  int total{0};
  int ok{0};
  int near_singular{0};
  int near_limit{0};
  int unevaluated{0};            ///< Strokes with no waypoint the callables accepted (counted as ok).
  double min_sigma{-1.0};        ///< Over all strokes.
  double min_limit_margin{-1.0};
  /// True when any stroke is near_singular or near_limit.
  bool flagged() const {return near_singular > 0 || near_limit > 0;}
};

/// Audit every stroke. A null callable skips that measure. near_singular takes precedence over near_limit.
PathQualityReport auditPathQuality(
  const std::vector<PathStroke> & strokes, const JacobianFn & jacobian, const LimitMarginFn & margin,
  const PathQualityConfig & config = PathQualityConfig{});

/// Margin of q against explicit bounds, same rule as jointLimitMargin: share of the range, joints of
/// range >= 2*pi (or <= 0) ignored, 0.5 when all are ignored, -1 on size mismatch.
double limitMarginFromBounds(
  const std::vector<double> & lower, const std::vector<double> & upper, const std::vector<double> & q);

/// Callables over an ArmModel (must outlive the returned functions).
JacobianFn armJacobianFn(const ArmModel & arm, double eps = 1e-5);
LimitMarginFn armLimitMarginFn(const ArmModel & arm);

}  // namespace reachability
