/**
 * @file time_parameterization.hpp
 * @brief Time parameterisation of a planned segment behind a port, ROS-free.
 * @details A planned path (joint positions) gets timestamps by the role of the segment:
 *          - kContactSeam: constant tool speed (the stroke speed from the tool profile), jerk
 *            limited, and a limit violation is a failure (a seam that cannot be run at the
 *            stroke speed within the joint limits must not be run slower silently);
 *          - kTransition: time-optimal within the joint velocity and acceleration limits;
 *            limit violations are reported, not fatal.
 *          The path is cleaned (near-duplicate points dropped) and split where the direction of
 *          travel reverses by more than reversal_angle_deg, so each piece starts and ends at
 *          rest and the retimer never sees a cusp. The retimer itself is the port
 *          (TimeParameterizer); motion_planner_moveit adapts MoveIt's TOTG and Ruckig smoothing.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "core_types/trajectory.hpp"

namespace motion_planner
{

enum class SegmentRole : uint8_t
{
  kContactSeam = 0,  ///< Constant speed, jerk limited, violations are failures.
  kTransition,       ///< Time-optimal, violations reported.
};

enum class TimingError : uint8_t
{
  kNone = 0,
  kTooFewPoints,         ///< Fewer than 2 distinct points: nothing to time.
  kJointCountMismatch,   ///< A point or the limits do not match joint_names.
  kNonFinite,            ///< NaN or inf in a position.
  kBadLimit,             ///< Velocity or acceleration limit not > 0, or jerk < 0, or not finite.
  kBadStrokeSpeed,       ///< A contact seam without a positive stroke speed.
  kBackendFailed,        ///< The retimer refused a piece.
  kLimitViolation,       ///< A contact seam exceeds a limit after retiming.
};

const char * toString(TimingError error);

/// Per-joint limits. jerk == 0 means unbounded (not checked, not enforced).
struct JointLimit
{
  double velocity{0.0};      ///< rad/s or m/s; must be > 0.
  double acceleration{0.0};  ///< rad/s^2 or m/s^2; must be > 0.
  double jerk{0.0};          ///< rad/s^3 or m/s^3; 0 = unbounded, never negative.
};

struct TimingRequest
{
  SegmentRole role{SegmentRole::kTransition};
  std::vector<JointLimit> limits;       ///< One per joint, in joint_names order.
  double stroke_speed{0.0};             ///< Tool speed (m/s) on a contact seam; ignored otherwise.
  double reversal_angle_deg{150.0};     ///< Split where the direction changes by more than this.
  double duplicate_epsilon{1e-6};       ///< Points closer than this (max joint delta) are merged.
  double violation_tolerance{0.02};     ///< Relative slack on a limit before it is a violation.
};

struct LimitViolation
{
  enum class Kind : uint8_t { kVelocity, kAcceleration, kJerk };
  size_t piece{0};
  size_t point{0};
  size_t joint{0};
  Kind kind{Kind::kVelocity};
  double value{0.0};
  double limit{0.0};
};

struct TimingResult
{
  bool success{false};
  TimingError error{TimingError::kNone};
  std::string message;
  /// Timed pieces, each starting at time 0 and at rest at both ends; execute in order.
  std::vector<core_types::Trajectory> pieces;
  /// Index (into the de-duplicated input path) of every point a split happened at.
  std::vector<size_t> split_points;
  std::vector<LimitViolation> violations;
  size_t dropped_points{0};   ///< Near-duplicates removed.
  double total_duration_sec{0.0};
};

/// Distance between two configurations (joint space, or the tool path length through FK).
using PathMetric = std::function<double(const std::vector<double> &, const std::vector<double> &)>;

/**
 * @class TimeParameterizer
 * @brief Port: give one reversal-free piece timestamps (and velocities / accelerations).
 * @details The piece arrives with positions only, 2 or more distinct points, validated limits.
 *          The result must start and end at rest. Return false with a message to refuse.
 */
class TimeParameterizer
{
public:
  virtual ~TimeParameterizer() = default;
  virtual bool retime(
    core_types::Trajectory & piece, SegmentRole role, const std::vector<JointLimit> & limits,
    double stroke_speed, std::string & message) = 0;
};

/// Validate the input and the request; kNone when fine.
TimingError validateTimingInput(
  const core_types::Trajectory & path, const TimingRequest & request, std::string & message);

/// Drop points whose max joint delta from the previous kept point is below epsilon.
core_types::Trajectory removeNearDuplicates(
  const core_types::Trajectory & path, double epsilon, size_t & dropped);

/// Indices of interior points where the joint-space direction changes by more than the angle.
std::vector<size_t> findReversals(const core_types::Trajectory & path, double angle_deg);

/// Cut the path at the given interior indices; neighbours share the cut point.
std::vector<core_types::Trajectory> splitAt(
  const core_types::Trajectory & path, const std::vector<size_t> & indices);

/// Constant-speed timestamps by cumulative path length; velocities by finite difference.
/// The metric defaults to the Euclidean joint-space distance.
void constantSpeedStamps(
  core_types::Trajectory & piece, double speed, const PathMetric & metric = nullptr);

/// Velocity, acceleration and (when accelerations are stored) jerk beyond the limits.
std::vector<LimitViolation> findLimitViolations(
  const core_types::Trajectory & piece, const std::vector<JointLimit> & limits,
  double tolerance, size_t piece_index = 0);

/// Join timed pieces (each at rest at its ends) into one trajectory with offset times.
core_types::Trajectory concatenate(const std::vector<core_types::Trajectory> & pieces);

/// Validate, clean, split, retime each piece through the port, check limits.
TimingResult parameterizeSegment(
  TimeParameterizer & retimer, const core_types::Trajectory & path, const TimingRequest & request);

}  // namespace motion_planner
