/**
 * @file motion_planner.hpp
 * @brief Owns the Cartesian/FSP planner backends and decides, per segment,
 *        which one actually gets used.
 * @details `ManipulationManager` is the interface exposing manipulation to
 *          the rest of the system (mission-level sequencing, retry-on-
 *          execution-failure, reporting). Getting a single requested motion
 *          actually planned -- which backend, whether a routing hint should
 *          be overridden by what a real attempt achieves -- is a distinct
 *          responsibility that belongs here, next to the two planners it
 *          uses (mirrors the Planner port pattern already established by
 *          CartesianPlanner/OmplPlanner). Never executes -- this stays a
 *          planning-only seam (Planner::plan()'s existing plan/execute
 *          decoupling), so ExecutionPort remains owned by ManipulationManager
 *          and this class never depends on manipulation_manager types.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "core_types/logger.hpp"
#include "core_types/null_logger.hpp"
#include "core_types/trajectory.hpp"
#include "motion_planner/planners/planner.hpp"
#include "motion_planner/planners/planning_types.hpp"

namespace motion_planner
{

/**
 * @struct SegmentRoutingSpec
 * @brief The routing-relevant fields of a caller's segment description --
 *        deliberately not manipulation_manager::MotionSegmentSpec itself,
 *        so this class never depends on that package's types.
 */
struct SegmentRoutingSpec
{
  std::string label;          ///< Used for label-heuristic mode routing.
  std::string mode_hint;      ///< "" | "cartesian" | "fsp" (raw, unresolved).
  std::string planner_hint;   ///< "" | explicit planner name (e.g. "ompl").
};

/**
 * @struct TrajectorySegmentProvenance
 * @brief One contiguous range of a stitched SegmentPlanResult's
 *        planning_result.trajectory.points, tagged with which planner
 *        actually produced it -- "cartesian" | "fsp". end_index is
 *        exclusive. A single-planner result (the common case) has exactly
 *        one entry spanning the whole trajectory; a bridged result has one
 *        entry per kept cartesian/fsp cycle, in order -- lets a caller
 *        (e.g. a marker-based visualization) attribute each chunk of a
 *        stitched trajectory to the planner that actually produced it,
 *        rather than only knowing a single segment-level summary
 *        (mode_used/bridge_count) for the whole thing.
 */
struct TrajectorySegmentProvenance
{
  std::string planner_name;
  size_t start_index{0};
  size_t end_index{0};
};

/**
 * @class MotionPlanner
 * @brief Resolves mode/planner routing for one segment and plans it,
 *        attempting Cartesian first for any fsp-routed segment and falling
 *        back to FSP only if that attempt can't clear min_path_fraction --
 *        judged by what the attempt actually achieved, not by any static
 *        fact about how the segment was authored (e.g. waypoint-index
 *        adjacency to the previous segment). A cartesian-routed segment
 *        (via explicit hint or a strip/sweep label) is never rerouted
 *        regardless of achieved fraction -- see planSegment()'s doc comment.
 */
class MotionPlanner
{
public:
  MotionPlanner(
    std::unique_ptr<Planner> cartesian_planner,
    std::unique_ptr<Planner> free_space_planner);

  void setLogger(std::shared_ptr<core_types::Logger> logger);

  /// Default mode when a segment has no mode_hint and no matching label
  /// heuristic. Must be "cartesian" or "fsp" -- anything else warns and
  /// falls back to "cartesian".
  void setMotionModePolicy(const std::string & policy);

  /// Requested default planner name for fsp-routed segments. Only "ompl" is
  /// supported today (see resolvePlannerSelection() in the .cpp) -- anything
  /// else warns and uses the fallback planner name instead.
  void setPlannerName(const std::string & planner_name);
  void setPlannerFallback(const std::string & fallback_name);

  std::string selectedMode() const;
  std::string selectedPlanner() const;

  /// Borrowed reference to the Cartesian planner for feasibility analysis
  /// (CoverageFeasibilityAnalyzer) -- MotionPlanner keeps ownership.
  Planner & cartesianPlanner();

  struct SegmentPlanResult
  {
    PlanningResult planning_result;
    std::string mode_used;
    std::string planner_used;
    bool mode_switch_used{false};
    bool fallback_used{false};
    /// How many bridge-then-resume cycles survived into the final result
    /// (0 if none were needed, or if a low-gain cycle was reverted, or if
    /// the segment fell back to a wholesale free_space_planner_ plan) --
    /// lets a caller tell a bridged Cartesian pass apart from a clean one
    /// even though mode_used stays "cartesian" for both.
    int bridge_count{0};
    /// Per-chunk planner attribution across planning_result.trajectory --
    /// see TrajectorySegmentProvenance. Always non-empty on a successful
    /// result (one entry for a single-planner result, more for a bridged
    /// one).
    std::vector<TrajectorySegmentProvenance> trajectory_segments;
  };

  /**
   * @brief Resolve routing and plan one segment.
   * @details `request` should already carry target_poses/frame_id/
   *          planning_group/eef_step/min_path_fraction/etc; `.mode`,
   *          `.planner`, and `.start_joint_state` are overwritten internally
   *          (start_joint_state seeded from tracked continuity -- see
   *          recordExecutionOutcome()/resetContinuity()).
   *
   *          Routing: resolve mode_hint/label/default exactly like today's
   *          label-heuristic system already does. If the resolved mode is
   *          "fsp", always attempt the Cartesian planner first regardless of
   *          *why* it resolved to fsp (explicit hint or label) -- generalizes
   *          the old "only for an explicit fsp hint" preflight. Keep that
   *          attempt if it succeeds AND clears min_path_fraction; otherwise
   *          plan with the free-space planner instead. If the resolved mode
   *          is "cartesian", always use the Cartesian planner, accepting
   *          whatever it achieves (even a near-zero fraction) -- these
   *          segments carry real coverage value once any contact/spray is
   *          engaged, so a low fraction is legitimate partial progress, not
   *          a failure to reroute away from -- *except* when the shortfall is
   *          a jump discontinuity or a genuine collision (CartesianTruncationReason
   *          ::kJumpDiscontinuity/::kCollision, from CartesianPlanner's own
   *          genuine diagnosis -- the collision case only reached after its
   *          own consistency-limited local IK recovery is exhausted, so it's
   *          an equally strong "genuinely stuck" signal): bridge past it via
   *          free_space_planner_ (a single-pose joint-space move to a point
   *          just past the discontinuity/collision), then resume
   *          cartesian_planner_ for the remainder, looping up to a small
   *          attempt cap. ik_failure-truncated results are never bridged this
   *          way -- no nearby configuration reaches that pose at all, so
   *          bridging to a different point doesn't help reach an unreachable
   *          one (see cartesian_planner.hpp's own doc comment).
   *
   *          Each bridge+resume cycle must advance the TCP at least
   *          kMinBridgeProgressMeters along the requested path (default 5cm)
   *          or it's reverted and the loop stops -- a cycle that only buys a
   *          sliver of progress (observed live: a 1.5cm gain from a full
   *          RRTConnect solve) isn't worth its planning cost, and the
   *          resume's own request only asks for whatever original waypoints
   *          are still ahead of the bridge target (never the ones already
   *          passed), so a resume never re-walks backward to an earlier
   *          waypoint before continuing forward.
   *
   *          If bridging still leaves a cartesian-routed segment below
   *          min_path_fraction: a contact/spray segment
   *          (request.allow_target_contact) keeps that partial result as-is
   *          (substituting a joint-space wander while the tool is touching
   *          the wall would drag it across the surface unpredictably, a
   *          safety concern, not just an efficiency one); a non-contact
   *          segment falls back to planning the whole thing with
   *          free_space_planner_ instead, reported as mode_used="fsp".
   */
  SegmentPlanResult planSegment(PlanningRequest request, const SegmentRoutingSpec & routing);

  /**
   * @brief Same as planSegment(), except the start state is supplied
   *        explicitly instead of read from tracked continuity.
   * @details planSegment() is a thin wrapper around this: it resolves
   *          start_joint_state from previous_end_joint_state_ (main-thread-
   *          only) and forwards. This entry point never reads or writes
   *          previous_end_joint_state_/has_previous_segment_/
   *          previous_mode_used_ (only recordExecutionOutcome()/
   *          resetContinuity() touch those) -- which is what makes it safe
   *          to call from a background thread (e.g. speculatively planning
   *          segment N+1 while segment N is still executing -- see
   *          ManipulationManager::executeMotionSegments()'s look-ahead
   *          pipelining) without racing the main thread's own continuity
   *          state. Internally serialized by planning_mutex_ against
   *          cartesian_planner_/free_space_planner_, which are themselves
   *          not thread-safe (shared MoveGroupInterface/PlanningSceneMonitor
   *          members, and CartesianPlanner's own ACM push/restore against
   *          the shared live planning scene) -- a concurrent call here waits
   *          rather than racing that shared state.
   * @param[in] start_joint_state Seed for the segment's own start state,
   *            e.g. the caller's own already-known expected end state of
   *            whatever segment precedes this one.
   */
  SegmentPlanResult planSegmentWithStartState(
    PlanningRequest request, const SegmentRoutingSpec & routing,
    const std::vector<double> & start_joint_state);

  /**
   * @brief Record what actually happened after the caller executed (or
   *        didn't) the trajectory from the most recent planSegment() call --
   *        continuity (used to seed the next planSegment()'s start state)
   *        only advances on genuine execution success, matching the
   *        pre-extraction behavior exactly.
   * @param[in] execution_succeeded Whether ExecutionPort::execute() (if
   *            called at all) succeeded.
   * @param[in] mode_used The mode planSegment() actually reported using.
   * @param[in] planned_trajectory The trajectory from that planSegment()'s
   *            result -- its last point's positions become the next
   *            planSegment() call's start_joint_state.
   */
  void recordExecutionOutcome(
    bool execution_succeeded,
    const std::string & mode_used,
    const core_types::Trajectory & planned_trajectory);

  /// Clears continuity state (previous end joint state / mode) -- call once
  /// per fresh mission before the first planSegment() call.
  void resetContinuity();

private:
  std::string resolveModePolicy(const SegmentRoutingSpec & routing) const;
  void resolvePlannerSelection();

  std::unique_ptr<Planner> cartesian_planner_;
  std::unique_ptr<Planner> free_space_planner_;
  std::shared_ptr<core_types::Logger> logger_{std::make_shared<core_types::NullLogger>()};

  std::string motion_mode_policy_{"cartesian"};
  std::string requested_planner_{"ompl"};
  std::string fallback_planner_{"ompl"};
  std::string effective_planner_{"ompl"};
  bool used_fallback_{false};

  bool has_previous_segment_{false};
  std::string previous_mode_used_;
  std::vector<double> previous_end_joint_state_;

  /// Serializes planSegmentWithStartState() bodies -- cartesian_planner_/
  /// free_space_planner_ are not internally thread-safe (see that method's
  /// own doc comment), so a background look-ahead call and the main
  /// thread's own call must never run concurrently against them.
  std::mutex planning_mutex_;
};

}  // namespace motion_planner
