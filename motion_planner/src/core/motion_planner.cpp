#include "motion_planner/core/motion_planner.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <sstream>

#include <Eigen/Geometry>

namespace motion_planner
{

namespace
{

constexpr int kMaxBridgeAttempts = 3;
// How far past the truncation point to bridge to, in meters -- a small step,
// not the whole remaining path; the goal is only to get past the one
// discontinuous point, then let cartesian_planner_ resume from a state near
// the original path again. Distance-based (not a fixed fraction) so the
// free-space planner is never asked for a sub-centimeter move regardless of
// the segment's total length -- confirmed live: a fraction-based step (2% of
// a 0.55m pass ~= 1.1cm) produced a full RRTConnect solve for a ~1cm TCP
// jump, taking over a minute to execute for essentially no progress.
constexpr double kMinBridgeAheadMeters = 0.05;
// Fallback ahead-step (as a fraction of the path) only for the degenerate
// case of a near-zero-length requested path, where a meters-based step isn't
// meaningful.
constexpr double kBridgeAheadFractionFloor = 0.02;
// A bridge+resume cycle that advances the TCP less than this along the
// requested path isn't worth its planning cost (a full free-space solve) --
// reverted and the loop stops rather than accumulating wasted wandering.
// Same value/reasoning as kMinBridgeAheadMeters (both express "what counts as
// a meaningful move here"), kept as a separate named constant since they
// answer different questions (how far to aim vs. how much progress to
// require) even though they currently share a value.
constexpr double kMinBridgeProgressMeters = 0.05;
// Enforced at every stitch seam in appendTrajectoryWithOffset() below -- a
// fresh sub-plan's own first point conventionally starts at
// time_from_start_sec=0.0 (representing "start of this sub-plan"), which
// after offsetting by accumulated's own current total duration lands
// EXACTLY on accumulated's own last point's time -- an exact tie a real
// FollowJointTrajectory controller rejects outright ("Time between points
// N and N+1 is not strictly increasing"), confirmed live against a
// genuine bridge-then-resume stitch. Same value as CartesianPlanner's own
// kMinTimeStepSeconds, for consistency.
constexpr double kMinSeamTimeStepSeconds = 1e-3;

// Position LERP + orientation SLERP along an (possibly multi-waypoint)
// requested Cartesian path, treating each consecutive waypoint pair as an
// equal-fraction segment. This is an approximation of CartesianPlanner's own
// (arc-length-based) fraction semantics -- exact for the common case of a
// single straight pass (2 waypoints, 1 segment, where the two notions of
// fraction coincide), and a reasonable one for multi-waypoint paths too.
core_types::Pose interpolateAlongPath(
  const std::vector<core_types::Pose> & target_poses, double fraction)
{
  if (target_poses.empty()) {
    return core_types::Pose{};
  }
  if (target_poses.size() == 1) {
    return target_poses.front();
  }

  fraction = std::clamp(fraction, 0.0, 1.0);
  const size_t segment_count = target_poses.size() - 1;
  const double scaled = fraction * static_cast<double>(segment_count);
  size_t segment_index = std::min(
    static_cast<size_t>(std::floor(scaled)), segment_count - 1);
  const double local_t = scaled - static_cast<double>(segment_index);

  const auto & a = target_poses[segment_index];
  const auto & b = target_poses[segment_index + 1];

  core_types::Pose result;
  result.x = a.x + local_t * (b.x - a.x);
  result.y = a.y + local_t * (b.y - a.y);
  result.z = a.z + local_t * (b.z - a.z);

  const Eigen::Quaterniond qa = Eigen::Quaterniond(a.qw, a.qx, a.qy, a.qz).normalized();
  const Eigen::Quaterniond qb = Eigen::Quaterniond(b.qw, b.qx, b.qy, b.qz).normalized();
  const Eigen::Quaterniond q = qa.slerp(local_t, qb);
  result.qx = q.x();
  result.qy = q.y();
  result.qz = q.z();
  result.qw = q.w();
  return result;
}

double poseDistance(const core_types::Pose & a, const core_types::Pose & b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double dz = b.z - a.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Straight-line sum across consecutive target_poses -- the same "distance is
// uniform per fraction unit" assumption interpolateAlongPath() already makes
// (exact for the common single-segment case, an approximation for a
// genuinely multi-waypoint chain). Used to convert a fraction-of-path gain
// into a physical distance for the bridge-progress guard below.
double totalPathArcLength(const std::vector<core_types::Pose> & target_poses)
{
  double total = 0.0;
  for (size_t i = 1; i < target_poses.size(); ++i) {
    total += poseDistance(target_poses[i - 1], target_poses[i]);
  }
  return total;
}

// The original target_poses still ahead of `fraction` along the path --
// matches interpolateAlongPath()'s own equal-fraction-per-segment
// convention, so a resume request built from this never re-walks a waypoint
// already behind it. The segment currently being traversed is dropped too --
// the resume's own start_joint_state already reflects wherever within that
// segment progress stopped (via forward kinematics), so only genuinely
// upcoming waypoints need restating; re-including the current segment's own
// start would make the resume walk backward to it first.
std::vector<core_types::Pose> remainingWaypointsAfter(
  const std::vector<core_types::Pose> & target_poses, double fraction)
{
  if (target_poses.size() < 2) {
    return target_poses;
  }
  fraction = std::clamp(fraction, 0.0, 1.0);
  const size_t segment_count = target_poses.size() - 1;
  const size_t passed_segment_index = std::min(
    static_cast<size_t>(std::floor(fraction * static_cast<double>(segment_count))),
    segment_count - 1);
  return std::vector<core_types::Pose>(
    target_poses.begin() + static_cast<std::ptrdiff_t>(passed_segment_index) + 1,
    target_poses.end());
}

// Appends `next`'s points to `accumulated`, offsetting each time_from_start_sec
// by accumulated's own current total duration -- MotionPlanner only sees each
// sub-plan's already-time-parameterized PlanningResult (re-timing internals
// like CartesianPlanner's TOTG/IPTP chain are private to that class, not part
// of the Planner interface), so a from-scratch single-pass re-parameterization
// of the whole stitched trajectory isn't available here; each sub-trajectory's
// own boundary velocities (zero, by construction of TOTG/IPTP) stay zero at
// the seam, which is an honest reflection of a genuine bridge-and-resume
// (a deliberate pause to recover from a discontinuity), not a smoothed-through
// artifact to hide.
void appendTrajectoryWithOffset(
  core_types::Trajectory & accumulated, const core_types::Trajectory & next)
{
  if (accumulated.joint_names.empty()) {
    accumulated.joint_names = next.joint_names;
  }
  const bool had_previous_points = !accumulated.points.empty();
  const double time_offset = had_previous_points ?
    accumulated.points.back().time_from_start_sec : 0.0;
  // A fresh sub-plan's own first point conventionally starts at
  // time_from_start_sec=0.0, which after offsetting lands EXACTLY on
  // accumulated's own last point's time -- bump the seam forward by at
  // least kMinSeamTimeStepSeconds so the stitched trajectory is never
  // handed to a real controller with a repeated (or reversed) timestamp at
  // the join. Only the seam itself needs this: each sub-plan's own points
  // are already strictly increasing among themselves (this only guards
  // the boundary between two sub-plans, not within either one).
  double last_time = had_previous_points ?
    accumulated.points.back().time_from_start_sec :
    -std::numeric_limits<double>::infinity();
  for (const auto & point : next.points) {
    auto offset_point = point;
    offset_point.time_from_start_sec += time_offset;
    if (offset_point.time_from_start_sec <= last_time) {
      offset_point.time_from_start_sec = last_time + kMinSeamTimeStepSeconds;
    }
    last_time = offset_point.time_from_start_sec;
    accumulated.points.push_back(std::move(offset_point));
  }
}

std::string toLowerCopy(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

bool containsToken(const std::string & value, const std::string & token)
{
  return value.find(token) != std::string::npos;
}

// Only used for bridge-loop log messages below -- not a caller-facing API, so
// kept local rather than a shared planning_types.hpp helper.
std::string truncationReasonToString(CartesianTruncationReason reason)
{
  switch (reason) {
    case CartesianTruncationReason::kJumpDiscontinuity:
      return "a jump discontinuity";
    case CartesianTruncationReason::kCollision:
      return "a collision";
    case CartesianTruncationReason::kIkFailure:
      return "an IK failure";
    case CartesianTruncationReason::kNone:
    default:
      return "a truncation";
  }
}

/// Label-heuristic mode routing -- unchanged from the pre-extraction
/// selectSegmentModePolicy() in manipulation_manager.cpp, moved here as this
/// class's own routing logic (not shared -- manipulation_manager no
/// longer needs it once this class owns mode resolution).
std::string modePolicyFromLabel(
  const std::string & segment_label,
  const std::string & default_policy)
{
  const std::string label = toLowerCopy(segment_label);

  if (containsToken(label, "connector") ||
    containsToken(label, "boundary_pass") ||
    containsToken(label, "obstacle_gap") ||
    containsToken(label, "recovery") ||
    containsToken(label, "transition") ||
    containsToken(label, "home"))
  {
    return "fsp";
  }

  if (containsToken(label, "sweep_strip") || containsToken(label, "strip")) {
    return "cartesian";
  }

  return default_policy;
}

}  // namespace

MotionPlanner::MotionPlanner(
  std::unique_ptr<Planner> cartesian_planner,
  std::unique_ptr<Planner> free_space_planner)
: cartesian_planner_(std::move(cartesian_planner)),
  free_space_planner_(std::move(free_space_planner))
{
}

void MotionPlanner::setLogger(std::shared_ptr<core_types::Logger> logger)
{
  if (!logger) {
    return;
  }
  logger_ = std::move(logger);
}

void MotionPlanner::setMotionModePolicy(const std::string & policy)
{
  motion_mode_policy_ = policy;
  if (motion_mode_policy_ != "cartesian" && motion_mode_policy_ != "fsp") {
    std::ostringstream oss;
    oss << "Unsupported motion_mode_policy='" << motion_mode_policy_
        << "'. Falling back to cartesian. Supported: cartesian|fsp.";
    logger_->warn(oss.str());
    motion_mode_policy_ = "cartesian";
  }
}

void MotionPlanner::setPlannerName(const std::string & planner_name)
{
  requested_planner_ = planner_name;
  resolvePlannerSelection();
}

void MotionPlanner::setPlannerFallback(const std::string & fallback_name)
{
  fallback_planner_ = fallback_name;
  if (fallback_planner_ != "ompl") {
    std::ostringstream oss;
    oss << "Unsupported planner_fallback='" << fallback_planner_
        << "'. Forcing fallback to ompl.";
    logger_->warn(oss.str());
    fallback_planner_ = "ompl";
  }
  resolvePlannerSelection();
}

void MotionPlanner::resolvePlannerSelection()
{
  effective_planner_ = requested_planner_;
  used_fallback_ = false;

  // Bookkeeping only -- which planner NAME is in effect. cartesian_planner_
  // has exactly one implementation today (no alternate backend names yet,
  // e.g. a future cuRobo planner -- see §6.4) and isn't part of this
  // "ompl"-only name selection, which governs free_space_planner_'s backend
  // name only.
  if (requested_planner_ == "ompl") {
    return;
  }

  std::ostringstream oss;
  oss << "Requested planner='" << requested_planner_
      << "' unsupported in current phase. Falling back globally to '" << fallback_planner_ << "'.";
  logger_->warn(oss.str());

  used_fallback_ = true;
  effective_planner_ = fallback_planner_;
}

std::string MotionPlanner::selectedMode() const
{
  return motion_mode_policy_;
}

std::string MotionPlanner::selectedPlanner() const
{
  return effective_planner_;
}

Planner & MotionPlanner::cartesianPlanner()
{
  return *cartesian_planner_;
}

void MotionPlanner::setSegmentKindModes(const std::map<std::string, std::string> & kind_to_mode)
{
  segment_kind_modes_.clear();
  for (const auto & [kind, mode] : kind_to_mode) {
    const std::string m = toLowerCopy(mode);
    if ((kind != "contact" && kind != "transition") ||
      (m != "cartesian" && m != "free_space" && m != "fsp"))
    {
      logger_->warn("ignoring segment mode override '" + kind + "' -> '" + mode + "'");
      continue;
    }
    segment_kind_modes_[kind] = (m == "cartesian") ? "cartesian" : "fsp";
  }
}

std::string MotionPlanner::resolveModePolicy(const SegmentRoutingSpec & routing) const
{
  const std::string mode_hint = toLowerCopy(routing.mode_hint);
  if (mode_hint == "cartesian" || mode_hint == "fsp") {
    return mode_hint;
  }
  if (!mode_hint.empty()) {
    std::ostringstream oss;
    oss << "segment label=" << routing.label << " has unsupported mode hint='" << mode_hint
        << "'. Using label/default routing.";
    logger_->warn(oss.str());
  }
  return modePolicyFromLabel(routing.label, motion_mode_policy_);
}

MotionPlanner::SegmentPlanResult MotionPlanner::planSegment(
  PlanningRequest request,
  const SegmentRoutingSpec & routing)
{
  return planSegmentWithStartState(std::move(request), routing, previous_end_joint_state_);
}

MotionPlanner::SegmentPlanResult MotionPlanner::planSegmentWithStartState(
  PlanningRequest request,
  const SegmentRoutingSpec & routing,
  const std::vector<double> & start_joint_state)
{
  std::lock_guard<std::mutex> lock(planning_mutex_);
  SegmentPlanResult result;

  std::string routed_mode_policy = resolveModePolicy(routing);
  {
    const auto override_it =
      segment_kind_modes_.find(request.allow_target_contact ? "contact" : "transition");
    if (override_it != segment_kind_modes_.end()) {
      routed_mode_policy = override_it->second;
    }
  }

  const std::string planner_hint = toLowerCopy(routing.planner_hint);
  std::string planner_for_segment = effective_planner_;
  bool segment_fallback_used = false;
  if (!planner_hint.empty()) {
    if (planner_hint == "ompl") {
      planner_for_segment = "ompl";
    } else {
      planner_for_segment = fallback_planner_;
      segment_fallback_used = true;
      std::ostringstream oss;
      oss << "segment label=" << routing.label << " has unsupported planner hint='" << planner_hint
          << "'. Falling back to '" << planner_for_segment << "'.";
      logger_->warn(oss.str());
    }
  }

  request.mode = routed_mode_policy;
  request.planner = planner_for_segment;
  request.start_joint_state = start_joint_state;

  uint64_t total_planning_time_ms = 0;
  std::string mode_used = routed_mode_policy;

  // The fix: for an fsp-routed segment -- whether via an explicit hint or a
  // connector/transition-type label -- always attempt Cartesian first,
  // judged by what it actually achieves (path_fraction vs
  // request.min_path_fraction), not by any static fact about how the
  // segment was authored. RRTConnect finds *a* valid joint-space path, not
  // the most direct one, so a genuinely-unobstructed fsp-routed move can end
  // up on a needlessly convoluted path if we don't at least try Cartesian
  // first -- confirmed live (a wide, looping TCP path to reach a pose 15cm
  // away). A cartesian-routed segment (routed_mode_policy == "cartesian")
  // never goes through this -- it always uses cartesian_planner_ and
  // accepts whatever fraction it achieves, since these segments carry real
  // coverage value once contact/spray is engaged.
  PlanningResult planning_result;
  bool cartesian_preflight_succeeded = false;
  if (routed_mode_policy == "fsp") {
    PlanningResult cartesian_attempt = cartesian_planner_->plan(request);
    total_planning_time_ms += cartesian_attempt.planning_time_ms;
    if (cartesian_attempt.success && cartesian_attempt.path_fraction >= request.min_path_fraction) {
      planning_result = std::move(cartesian_attempt);
      mode_used = "cartesian";
      cartesian_preflight_succeeded = true;
    }
  }

  if (!cartesian_preflight_succeeded) {
    Planner * active_planner = (routed_mode_policy == "cartesian") ?
      cartesian_planner_.get() :
      free_space_planner_.get();
    planning_result = active_planner->plan(request);
    total_planning_time_ms += planning_result.planning_time_ms;
  }

  // Attribute the whole trajectory-so-far to whichever single planner just
  // produced it -- the common case (no bridging needed). The cartesian-only
  // block below may append more entries (a bridged result); Part C's
  // fallback (further below) may reset this to a single fsp entry instead.
  std::vector<TrajectorySegmentProvenance> trajectory_segments = {
    {mode_used, 0, planning_result.trajectory.points.size()}
  };

  // Bridge a jump discontinuity or a genuine collision via free_space_planner_,
  // then resume cartesian_planner_ for the remainder -- only for a genuinely
  // cartesian-routed segment (an fsp-routed segment already fell back to
  // free_space_planner_ wholesale above if its own Cartesian preflight didn't
  // clear min_path_fraction, so it never reaches CartesianPlanner's own
  // truncation diagnosis at all). Collision used to be excluded here (routing
  // around a genuine obstacle without a deliberately designed recovery
  // strategy would be a real safety-relevant behavior change) -- but
  // CartesianPlanner now searches a bounded, jump-safe neighborhood via
  // consistency-limited IK retry (exploiting the arm's redundant lift DOF)
  // *before* ever reporting kCollision, so by the time it's seen here it's
  // already the same strength of "genuinely stuck, local options exhausted"
  // signal a jump discontinuity is -- the deliberately-designed strategy that
  // exclusion was deferring to. kIkFailure stays excluded: no nearby
  // configuration reaches that pose at all, so bridging to a different point
  // doesn't help reach an unreachable one.
  int kept_bridge_count = 0;
  if (routed_mode_policy == "cartesian") {
    core_types::Trajectory accumulated_trajectory = planning_result.trajectory;
    const double total_arc_length = totalPathArcLength(request.target_poses);
    double covered_fraction = planning_result.path_fraction;
    int bridge_attempts = 0;
    // !request.allow_target_contact: a contact segment must never receive an FSP bridge spliced
    // into the middle of its trajectory -- the same drag hazard the wholesale fallback below
    // already refuses to risk (a joint-space wander while the tool is touching the wall would
    // drag it across the surface unpredictably), except worse here because it happens mid-
    // execution with no opportunity to unload first. A truncated contact segment keeps its
    // partial Cartesian result via that same fallback's contact-aware branch below instead.
    while (planning_result.success && !request.allow_target_contact &&
      (planning_result.truncation_reason == CartesianTruncationReason::kJumpDiscontinuity ||
      planning_result.truncation_reason == CartesianTruncationReason::kCollision) &&
      bridge_attempts < kMaxBridgeAttempts && !accumulated_trajectory.points.empty())
    {
      ++bridge_attempts;

      const double ahead_fraction = total_arc_length > 1e-6 ?
        kMinBridgeAheadMeters / total_arc_length : kBridgeAheadFractionFloor;
      const double bridge_target_fraction = std::min(1.0, covered_fraction + ahead_fraction);

      PlanningRequest bridge_request;
      bridge_request.planning_group = request.planning_group;
      bridge_request.frame_id = request.frame_id;
      bridge_request.start_joint_state = accumulated_trajectory.points.back().positions;
      bridge_request.target_poses = {
        interpolateAlongPath(request.target_poses, bridge_target_fraction)};

      PlanningResult bridge_result = free_space_planner_->plan(bridge_request);
      total_planning_time_ms += bridge_result.planning_time_ms;
      if (!bridge_result.success || bridge_result.trajectory.points.empty()) {
        std::ostringstream oss;
        oss << "segment label=" << routing.label << ": could not bridge past "
            << truncationReasonToString(planning_result.truncation_reason) << " ("
            << bridge_result.message <<
          ") -- keeping the partial Cartesian result achieved so far.";
        logger_->warn(oss.str());
        break;
      }

      // Only the original waypoints still ahead of the bridge target -- not
      // the full request.target_poses again -- so the resume walks forward
      // from where the bridge just placed it, never backward to an
      // already-passed waypoint (confirmed live: re-sending the full list
      // made every resume detour all the way back to the segment's first
      // waypoint before continuing, multiplying the planned Cartesian
      // distance several-fold over what the segment actually needed).
      PlanningRequest resume_request = request;
      resume_request.start_joint_state = bridge_result.trajectory.points.back().positions;
      resume_request.target_poses = remainingWaypointsAfter(
        request.target_poses, bridge_target_fraction);
      PlanningResult resume_result = cartesian_planner_->plan(resume_request);
      total_planning_time_ms += resume_result.planning_time_ms;
      if (!resume_result.success || resume_result.trajectory.points.empty()) {
        std::ostringstream oss;
        oss << "segment label=" << routing.label << ": resumed Cartesian plan after bridging past "
            << truncationReasonToString(planning_result.truncation_reason) << " failed ("
            << resume_result.message << ") -- discarding this bridge attempt and keeping the "
          "partial Cartesian result achieved so far.";
        logger_->warn(oss.str());
        break;
      }

      // The resume's own path_fraction is relative to remainingWaypointsAfter()
      // above (what was still left when the bridge landed), not the whole
      // original request -- combine with how far the bridge itself already
      // got to express progress in the original path's own terms.
      const double new_covered_fraction = bridge_target_fraction +
        (1.0 - bridge_target_fraction) * resume_result.path_fraction;
      const double gain_meters = (new_covered_fraction - covered_fraction) * total_arc_length;
      // A near-zero total_arc_length (a degenerate/zero-length requested
      // path) makes any physical-distance gain meaningless to measure --
      // don't let that masquerade as "no progress" and discard a cycle that
      // may have made real fractional progress.
      if (total_arc_length > 1e-6 && gain_meters < kMinBridgeProgressMeters) {
        std::ostringstream oss;
        oss << "segment label=" << routing.label << ": bridge+resume cycle only advanced "
            << gain_meters << "m (below the " << kMinBridgeProgressMeters
            << "m minimum) -- discarding it and keeping the partial Cartesian result achieved "
          "so far, rather than paying for a wander that bought essentially nothing.";
        logger_->warn(oss.str());
        break;
      }

      // Only mutate accumulated_trajectory/trajectory_segments once the
      // cycle is confirmed worth keeping -- everything above (both plan()
      // calls, the gain check) only ever reads accumulated_trajectory, so
      // there's nothing to revert on any of the early-exit paths above.
      const size_t index_before_bridge = accumulated_trajectory.points.size();
      appendTrajectoryWithOffset(accumulated_trajectory, bridge_result.trajectory);
      const size_t index_before_resume = accumulated_trajectory.points.size();
      trajectory_segments.push_back({"fsp", index_before_bridge, index_before_resume});
      appendTrajectoryWithOffset(accumulated_trajectory, resume_result.trajectory);
      trajectory_segments.push_back(
        {"cartesian", index_before_resume, accumulated_trajectory.points.size()});

      covered_fraction = new_covered_fraction;
      planning_result = std::move(resume_result);
      ++kept_bridge_count;
    }
    // Report against the whole stitched sequence, not just the last
    // sub-attempt's own (much shorter) trajectory. path_fraction is now the
    // tracked covered_fraction (progress in the *original* request's own
    // terms across every kept cycle), not just the last sub-attempt's own
    // (much narrower) fraction -- fixes the previously-accepted
    // approximation now that covered_fraction is tracked anyway.
    planning_result.trajectory = accumulated_trajectory;
    planning_result.path_fraction = covered_fraction;
  }

  // Contact-aware fallback: if bridging couldn't recover a genuinely
  // meaningful Cartesian run, a contact/spray segment (allow_target_contact)
  // keeps the partial result as-is -- substituting a joint-space wander
  // while the tool is touching the wall would drag it across the surface
  // unpredictably, a safety concern, not just an efficiency one. A
  // non-contact segment has no coverage value in a partial pass, so it gets
  // the same wholesale free-space fallback an fsp-routed segment's own
  // failed Cartesian preflight already gets above.
  if (routed_mode_policy == "cartesian" &&
    planning_result.path_fraction < request.min_path_fraction &&
    !request.allow_target_contact)
  {
    PlanningResult fsp_fallback = free_space_planner_->plan(request);
    total_planning_time_ms += fsp_fallback.planning_time_ms;
    if (fsp_fallback.success) {
      trajectory_segments = {{"fsp", 0, fsp_fallback.trajectory.points.size()}};
      planning_result = std::move(fsp_fallback);
      mode_used = "fsp";
      kept_bridge_count = 0;
    } else {
      std::ostringstream oss;
      oss << "segment label=" << routing.label << ": Cartesian achieved only "
          << planning_result.path_fraction << " of the path and the free-space fallback also "
        "failed (" << fsp_fallback.message << ") -- keeping the partial Cartesian result "
        "achieved so far.";
      logger_->warn(oss.str());
    }
  }

  result.mode_switch_used = has_previous_segment_ ?
    (mode_used != previous_mode_used_) :
    (mode_used != motion_mode_policy_);

  planning_result.planning_time_ms = total_planning_time_ms;
  result.planning_result = std::move(planning_result);
  result.mode_used = mode_used;
  result.planner_used = planner_for_segment;
  result.fallback_used = used_fallback_ || segment_fallback_used;
  result.bridge_count = kept_bridge_count;
  result.trajectory_segments = std::move(trajectory_segments);
  return result;
}

void MotionPlanner::recordExecutionOutcome(
  bool execution_succeeded,
  const std::string & mode_used,
  const core_types::Trajectory & planned_trajectory)
{
  has_previous_segment_ = true;
  previous_mode_used_ = mode_used;
  if (execution_succeeded && !planned_trajectory.points.empty()) {
    previous_end_joint_state_ = planned_trajectory.points.back().positions;
  }
}

void MotionPlanner::resetContinuity()
{
  has_previous_segment_ = false;
  previous_mode_used_.clear();
  previous_end_joint_state_.clear();
}

}  // namespace motion_planner
