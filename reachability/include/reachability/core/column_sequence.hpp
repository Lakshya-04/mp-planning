// The pose sequence of one wall column and its reachability check.
//
// A column is not only its stroke poses. The manager docks at an approach pose (the dock pose pulled back
// by `dock_approach_extra_m`), closes to the dock / standoff pose, presses to the contact depth, strokes, and
// retracts. A column whose strokes are reachable but whose approach or retract is not cannot be run, so the
// whole sequence is checked against the RM4D map for one stand and one lift height (the lift is locked
// while a column runs).
//
// Each pose gets a score in [0, 1] from the map (scorePose, see pose_quality.hpp); a pose under
// MarginConfig::min_score is not usable even when its cell is reachable. A column is
//   kWhole        every pose of the sequence is usable at one lift height,
//   kSplit        the approach / retract poses are usable but only some strokes are: the column would be
//                 broken into pieces (another stand has to do the rest),
//   kUnreachable  no stroke is usable, or an end pose (approach, contact, retract) is not.
// Everything here is RM4D-level: it is an optimistic estimate (no IK, no collision), like the rest of the
// map. confirmColumnWithVerifier re-checks one chosen (stand, column) with a StandVerifier.
#pragma once

#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "reachability/core/arm_model.hpp"
#include "reachability/core/map4d.hpp"
#include "reachability/core/pose_quality.hpp"
#include "reachability/core/stand_verifier.hpp"

namespace reachability
{

enum class PoseRole
{
  kApproach,   ///< Free-space goal short of the dock pose.
  kContact,    ///< Dock pose pressed to the contact depth (only when press_depth_m > 0).
  kStroke,     ///< A pose of the column's plan.
  kRetract,    ///< Away from the wall after the last stroke pose.
};

const char * toString(PoseRole role);

/// Where the extra poses go. Distances in metres along the wall normal. `wall_normal` points into the free
/// side (the StationRequest convention; the manager's WallDirection points the other way).
struct SequenceParams
{
  Vec3 wall_normal{-Vec3::UnitX()};
  /// Stand-off of the dock pose from the plan poses (the manager's `dock_standoff_m`).
  double standoff_m{0.12};
  /// The approach pose is this much further out (the manager's `dock_approach_extra_m`).
  double approach_extra_m{0.0};
  /// How far past the plan surface the tool presses (target depth). 0 = no contact pose (non-contact tool).
  double press_depth_m{0.0};
  /// Retract distance from the last stroke pose; negative = back to the approach pose
  /// (standoff_m + approach_extra_m).
  double retract_m{-1.0};
  /// The dock pose is built from the first stroke pose (manager `dock_at_stroke_start`), else from the middle one.
  bool dock_at_first_stroke{false};
};

constexpr size_t kNoStroke = std::numeric_limits<size_t>::max();

struct ColumnPoseSequence
{
  std::vector<Pose> poses;              ///< approach, [contact], strokes in order, retract.
  std::vector<PoseRole> roles;          ///< Same size.
  std::vector<size_t> stroke_index;     ///< Index into the input stroke list for kStroke, else kNoStroke.
  size_t strokes{0};
};

/// Builds the sequence of a column from its stroke poses (world frame, tool z = approach axis, wall
/// normal as in `params`). Empty strokes give an empty sequence. Approach / retract poses at zero distance
/// are omitted.
ColumnPoseSequence buildColumnSequence(const std::vector<Pose> & strokes_world, const SequenceParams & params);

enum class PoseFailure
{
  kNone,
  kOutOfReach,    ///< The map cell is empty (or outside the window).
  kLowMargin,     ///< Reachable but the score is under MarginConfig::min_score.
  kRefused,       ///< The verifier refused it (IK, collision, limits).
  kSingular,      ///< The verifier's joint solution has a smallest singular value under the threshold.
  kJointLimit,    ///< ... or a joint closer to its limit than the threshold.
};

const char * toString(PoseFailure failure);

struct PoseCheck
{
  PoseRole role{PoseRole::kStroke};
  size_t stroke{kNoStroke};
  bool usable{false};
  PoseFailure failure{PoseFailure::kOutOfReach};
  PoseScore score;
  double singular_value{-1.0};   ///< Verified checks with a joint solution; -1 = not computed.
  double limit_margin{-1.0};
};

struct SequenceCheck
{
  double lift_height{0.0};
  std::vector<PoseCheck> poses;   ///< One per sequence pose.
  size_t strokes_total{0};
  size_t strokes_usable{0};
  bool ends_usable{true};         ///< Approach, contact and retract poses all usable.
  double min_score{0.0};          ///< Lowest score over the usable poses (0 when none).
  /// The pose that limits the column: the first unusable end pose, else the first unusable stroke, else
  /// (a whole column) the lowest-scoring pose. Index into `poses`; empty for an empty sequence.
  std::optional<size_t> limiting;
  bool whole() const {return strokes_total > 0 && strokes_usable == strokes_total && ends_usable;}
};

/// Recomputes the totals and `limiting` from `poses` (call after editing `usable` flags).
void summarize(SequenceCheck * check);

/// Checks every pose of `seq` for an arm base at (`base_xy`, lift_height + arm_mount_offset_z).
SequenceCheck checkSequenceAt(
  const Map4D & map, const ColumnPoseSequence & seq, const Eigen::Vector2d & base_xy, double lift_height,
  double arm_mount_offset_z, const MarginConfig & margin);

enum class ColumnStatus { kWhole, kSplit, kUnreachable };

const char * toString(ColumnStatus status);

ColumnStatus statusOf(const SequenceCheck & check);

struct ColumnFeasibility
{
  ColumnStatus status{ColumnStatus::kUnreachable};
  SequenceCheck best;        ///< The check at the chosen lift height.
  bool verified{false};      ///< confirmColumnWithVerifier ran on it.
};

/// The best of `heights`: a height where the whole sequence is usable if any, else the one with usable end
/// poses and the most usable strokes, then the highest minimum score; ties go to the lower height.
ColumnFeasibility evaluateColumn(
  const Map4D & map, const ColumnPoseSequence & seq, const Eigen::Vector2d & base_xy,
  const std::vector<double> & heights, double arm_mount_offset_z, const MarginConfig & margin);

/// Re-checks `feas` (its lift height) with the verifier: a refused pose becomes unusable, and with `arm`
/// the joint solution's smallest singular value and joint-limit margin are held against `margin`. The
/// status is recomputed; the RM4D scores are kept. Only the poses RM4D let through are sent.
ColumnFeasibility confirmColumnWithVerifier(
  StandVerifier & verifier, const ArmModel * arm, const Stand & stand, const ColumnPoseSequence & seq,
  const ColumnFeasibility & feas, const MarginConfig & margin);

}  // namespace reachability
