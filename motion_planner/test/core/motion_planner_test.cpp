#include <gtest/gtest.h>

#include <future>
#include <memory>
#include <string>
#include <vector>

#include "core_types/trajectory.hpp"
#include "motion_planner/core/motion_planner.hpp"

namespace motion_planner
{
namespace
{

/// Scripted Planner double: returns a fixed success/fraction/failure-code,
/// records how many times plan() was called, and (if it "succeeds") always
/// hands back a one-point trajectory ending at end_positions so continuity
/// tests can assert on it.
class ScriptedPlanner : public Planner
{
public:
  ScriptedPlanner(std::string name, bool succeeds, double path_fraction)
  : name_(std::move(name)), succeeds_(succeeds), path_fraction_(path_fraction)
  {
  }

  std::string name() const override {return name_;}

  PlanningResult plan(const PlanningRequest & request) override
  {
    ++plan_call_count;
    last_start_joint_state = request.start_joint_state;

    PlanningResult result;
    result.success = succeeds_;
    result.status = succeeds_ ? "planned" : "failed";
    result.failure_reason_code = succeeds_ ? "none" : "fake_plan_failed";
    result.path_fraction = path_fraction_;
    if (succeeds_) {
      core_types::TrajectoryPoint point;
      point.positions = end_positions;
      result.trajectory.points = {point};
      result.waypoints_completed = request.waypoint_indices.size();
    }
    return result;
  }

  IkResult ik(const IkRequest & /*request*/) override {return IkResult{};}
  CollisionResult collision(const CollisionRequest & /*request*/) override
  {
    return CollisionResult{};
  }

  int plan_call_count{0};
  std::vector<double> end_positions{0.1, 0.2, 0.3};
  std::vector<double> last_start_joint_state;

private:
  std::string name_;
  bool succeeds_;
  double path_fraction_;
};

PlanningRequest makeRequest(double min_path_fraction = 0.5)
{
  PlanningRequest request;
  request.frame_id = "base_link";
  request.planning_group = "arm_7dof";
  request.waypoint_indices = {0, 1};
  request.min_path_fraction = min_path_fraction;
  return request;
}

TEST(MotionPlannerTest, CartesianRoutedContactSegmentKeepsPartialInsteadOfFspFallback)
{
  // A contact/spray segment (allow_target_contact) must never fall back to
  // FSP no matter how low its achieved fraction is -- these carry real
  // coverage value once contact/spray is engaged, so a near-zero fraction is
  // legitimate partial progress, not a failure to reroute away from.
  // Substituting a joint-space wander while the tool is touching the wall
  // would drag it across the surface unpredictably -- a safety concern, not
  // just an efficiency one.
  auto cartesian = std::make_unique<ScriptedPlanner>("cartesian", true, 0.007);
  auto * cartesian_ptr = cartesian.get();
  auto fsp = std::make_unique<ScriptedPlanner>("fsp", true, 1.0);
  auto * fsp_ptr = fsp.get();
  MotionPlanner planner(std::move(cartesian), std::move(fsp));

  SegmentRoutingSpec routing;
  routing.label = "sweep_strip_0";

  PlanningRequest request = makeRequest();
  request.allow_target_contact = true;
  const auto result = planner.planSegment(request, routing);

  EXPECT_EQ(result.mode_used, "cartesian");
  EXPECT_TRUE(result.planning_result.success);
  EXPECT_DOUBLE_EQ(result.planning_result.path_fraction, 0.007);
  EXPECT_TRUE(result.free_space_backend.empty());  // the free-space plugin planned nothing
  EXPECT_EQ(cartesian_ptr->plan_call_count, 1);
  EXPECT_EQ(fsp_ptr->plan_call_count, 0)
    << "FSP must never be attempted for a contact-flagged cartesian-routed segment";
  ASSERT_EQ(result.trajectory_segments.size(), 1u);
  EXPECT_EQ(result.trajectory_segments[0].planner_name, "cartesian");
  EXPECT_EQ(result.trajectory_segments[0].start_index, 0u);
  EXPECT_EQ(
    result.trajectory_segments[0].end_index,
    result.planning_result.trajectory.points.size());
}

TEST(MotionPlannerTest, CartesianRoutedNonContactSegmentFallsBackToFspWhenNoMeaningfulRun)
{
  // The same low-fraction result, but without allow_target_contact -- a
  // non-contact segment has no coverage value in a partial pass, so it gets
  // the same wholesale free-space fallback an fsp-routed segment's own
  // failed Cartesian preflight already gets.
  auto cartesian = std::make_unique<ScriptedPlanner>("cartesian", true, 0.007);
  auto * cartesian_ptr = cartesian.get();
  auto fsp = std::make_unique<ScriptedPlanner>("fsp", true, 1.0);
  auto * fsp_ptr = fsp.get();
  MotionPlanner planner(std::move(cartesian), std::move(fsp));

  SegmentRoutingSpec routing;
  routing.label = "sweep_strip_0";

  const auto result = planner.planSegment(makeRequest(), routing);

  EXPECT_EQ(result.mode_used, "fsp");
  EXPECT_TRUE(result.planning_result.success);
  EXPECT_DOUBLE_EQ(result.planning_result.path_fraction, 1.0);
  EXPECT_EQ(result.free_space_backend, "fsp");  // name() of the plugin, not the legacy planner_used
  EXPECT_EQ(cartesian_ptr->plan_call_count, 1)
    << "the low-fraction Cartesian attempt still happens once before falling back";
  EXPECT_EQ(fsp_ptr->plan_call_count, 1)
    << "falls back to a wholesale free-space plan since this isn't a contact segment";
  ASSERT_EQ(result.trajectory_segments.size(), 1u)
    << "the fallback resets provenance to a single fsp entry, not a leftover cartesian one "
    "from before the fallback replaced the entire result";
  EXPECT_EQ(result.trajectory_segments[0].planner_name, "fsp");
  EXPECT_EQ(result.trajectory_segments[0].start_index, 0u);
  EXPECT_EQ(
    result.trajectory_segments[0].end_index,
    result.planning_result.trajectory.points.size());
}

/// Scripted CartesianPlanner double: a jump-truncated partial result on its
/// first plan() call, full success on every call after -- used to verify
/// planSegment()'s bridge-then-resume loop (bridges exactly once via
/// free_space_planner_, then resumes cartesian_planner_ and stops looping
/// once the truncation reason clears).
// Returns a truncated result (call-#1 reason configurable -- jump/collision
// both bridgeable, ik_failure not) then a fully-succeeded one on every
// subsequent call, so the same fake exercises both the "bridge fires" and
// "bridge correctly excluded" tests.
class JumpThenResumeCartesianPlanner : public Planner
{
public:
  explicit JumpThenResumeCartesianPlanner(
    CartesianTruncationReason first_call_reason = CartesianTruncationReason::kJumpDiscontinuity,
    double first_call_fraction = 0.4,
    double resume_call_fraction = 1.0)
  : first_call_reason_(first_call_reason),
    first_call_fraction_(first_call_fraction),
    resume_call_fraction_(resume_call_fraction)
  {
  }

  std::string name() const override {return "cartesian";}

  PlanningResult plan(const PlanningRequest & request) override
  {
    ++plan_call_count;
    last_start_joint_states.push_back(request.start_joint_state);
    last_target_poses.push_back(request.target_poses);

    PlanningResult result;
    result.success = true;
    result.trajectory.joint_names = {"j1"};
    core_types::TrajectoryPoint point;
    if (plan_call_count == 1) {
      result.path_fraction = first_call_fraction_;
      result.truncation_reason = first_call_reason_;
      point.positions = {0.1, 0.1, 0.1};
      point.time_from_start_sec = 1.0;
    } else {
      result.path_fraction = resume_call_fraction_;
      result.truncation_reason = resume_call_fraction_ >= 1.0 ?
        CartesianTruncationReason::kNone : first_call_reason_;
      point.positions = {0.5, 0.5, 0.5};
      point.time_from_start_sec = 2.0;
    }
    result.trajectory.points = {point};
    return result;
  }

  IkResult ik(const IkRequest & /*request*/) override {return IkResult{};}
  CollisionResult collision(const CollisionRequest & /*request*/) override
  {
    return CollisionResult{};
  }

  int plan_call_count{0};
  std::vector<std::vector<double>> last_start_joint_states;
  std::vector<std::vector<core_types::Pose>> last_target_poses;

private:
  CartesianTruncationReason first_call_reason_;
  double first_call_fraction_;
  double resume_call_fraction_;
};

TEST(MotionPlannerTest, CartesianRoutedSegmentBridgesJumpDiscontinuityThenResumes)
{
  auto cartesian = std::make_unique<JumpThenResumeCartesianPlanner>();
  auto * cartesian_ptr = cartesian.get();
  auto fsp = std::make_unique<ScriptedPlanner>("fsp", true, 1.0);
  auto * fsp_ptr = fsp.get();
  MotionPlanner planner(std::move(cartesian), std::move(fsp));

  SegmentRoutingSpec routing;
  routing.label = "sweep_strip_0";

  PlanningRequest request = makeRequest();
  request.target_poses = {core_types::Pose{}, core_types::Pose{}};

  const auto result = planner.planSegment(request, routing);

  EXPECT_TRUE(result.planning_result.success);
  EXPECT_EQ(result.planning_result.truncation_reason, CartesianTruncationReason::kNone)
    << "the final reported result should be the resumed (fully-succeeded) attempt";
  EXPECT_EQ(cartesian_ptr->plan_call_count, 2)
    << "one initial attempt + one resume after the bridge, then the loop stops "
    "because the resumed result's truncation reason is no longer kJumpDiscontinuity";
  EXPECT_EQ(fsp_ptr->plan_call_count, 1) << "should bridge exactly once via free_space_planner_";
  ASSERT_EQ(result.planning_result.trajectory.points.size(), 3u)
    << "stitched: initial partial (1pt) + bridge (1pt) + resumed (1pt)";
  EXPECT_EQ(cartesian_ptr->last_start_joint_states[1], fsp_ptr->end_positions)
    << "the resume call should start from the bridge's own end state";
  ASSERT_EQ(result.trajectory_segments.size(), 3u)
    << "one provenance entry per leg -- initial cartesian, fsp bridge, resumed cartesian";
  EXPECT_EQ(result.trajectory_segments[0].planner_name, "cartesian");
  EXPECT_EQ(result.trajectory_segments[0].start_index, 0u);
  EXPECT_EQ(result.trajectory_segments[0].end_index, 1u);
  EXPECT_EQ(result.trajectory_segments[1].planner_name, "fsp");
  EXPECT_EQ(result.trajectory_segments[1].start_index, 1u);
  EXPECT_EQ(result.trajectory_segments[1].end_index, 2u);
  EXPECT_EQ(result.trajectory_segments[2].planner_name, "cartesian");
  EXPECT_EQ(result.trajectory_segments[2].start_index, 2u);
  EXPECT_EQ(result.trajectory_segments[2].end_index, 3u);
}

TEST(MotionPlannerTest, CartesianRoutedContactSegmentDoesNotBridgeJumpDiscontinuity)
{
  // A contact segment must never receive an FSP bridge spliced into the middle of its
  // trajectory -- the same drag hazard CartesianRoutedContactSegmentKeepsPartialInsteadOfFspFallback
  // already refuses to risk via the wholesale fallback, except this would be worse: it happens
  // mid-execution, with no opportunity to unload the tool first. The truncated partial result is
  // kept as-is, exactly like the low-fraction case above -- allow_target_contact routes to the
  // same "keep the partial result" outcome whether the truncation came from a low fraction or a
  // jump/collision the bridge loop would otherwise have tried to route around.
  auto cartesian = std::make_unique<JumpThenResumeCartesianPlanner>();
  auto * cartesian_ptr = cartesian.get();
  auto fsp = std::make_unique<ScriptedPlanner>("fsp", true, 1.0);
  auto * fsp_ptr = fsp.get();
  MotionPlanner planner(std::move(cartesian), std::move(fsp));

  SegmentRoutingSpec routing;
  routing.label = "sweep_strip_0";

  PlanningRequest request = makeRequest();
  request.allow_target_contact = true;
  request.target_poses = {core_types::Pose{}, core_types::Pose{}};

  const auto result = planner.planSegment(request, routing);

  EXPECT_EQ(result.planning_result.truncation_reason, CartesianTruncationReason::kJumpDiscontinuity)
    << "no bridge attempted -- still whatever the initial (un-bridged) attempt reported";
  EXPECT_DOUBLE_EQ(result.planning_result.path_fraction, 0.4)
    << "the initial attempt's own fraction, unimproved by any bridge+resume cycle";
  EXPECT_EQ(cartesian_ptr->plan_call_count, 1)
    << "no resume attempt -- the bridge loop must never enter for a contact segment";
  EXPECT_EQ(fsp_ptr->plan_call_count, 0)
    << "no bridge attempt via free_space_planner_ either";
  ASSERT_EQ(result.planning_result.trajectory.points.size(), 1u)
    << "just the initial attempt's own point -- nothing stitched on";
  ASSERT_EQ(result.trajectory_segments.size(), 1u)
    << "a single cartesian provenance entry, not a phantom fsp/cartesian pair from a bridge "
    "that should never have started";
  EXPECT_EQ(result.trajectory_segments[0].planner_name, "cartesian");
}

TEST(MotionPlannerTest, BridgeResumeDoesNotResendAlreadyCoveredWaypoints)
{
  // Confirmed live this session: resending the full original target_poses on
  // resume made computeCartesianPath() walk backward to the first waypoint
  // before continuing forward again, multiplying the planned distance
  // several-fold. The resume request must only carry waypoints still ahead
  // of the bridge target -- never the segment's own first waypoint again.
  auto cartesian = std::make_unique<JumpThenResumeCartesianPlanner>();
  auto * cartesian_ptr = cartesian.get();
  auto fsp = std::make_unique<ScriptedPlanner>("fsp", true, 1.0);
  MotionPlanner planner(std::move(cartesian), std::move(fsp));

  SegmentRoutingSpec routing;
  routing.label = "sweep_strip_0";

  core_types::Pose start_pose;
  start_pose.x = 0.0;
  core_types::Pose end_pose;
  end_pose.x = 1.0;

  PlanningRequest request = makeRequest();
  request.target_poses = {start_pose, end_pose};

  planner.planSegment(request, routing);

  ASSERT_EQ(cartesian_ptr->last_target_poses.size(), 2u)
    << "one initial attempt + one resume";
  ASSERT_EQ(cartesian_ptr->last_target_poses[1].size(), 1u)
    << "the resume should only ask for the one waypoint still ahead, not the full "
    "original two-waypoint list";
  EXPECT_DOUBLE_EQ(cartesian_ptr->last_target_poses[1][0].x, end_pose.x)
    << "the resume's sole remaining waypoint should be the segment's actual final "
    "target, not a re-send of its already-passed first waypoint";
}

TEST(MotionPlannerTest, BridgeIsRevertedWhenProgressBelowThreshold)
{
  // A bridge+resume cycle that only advances the TCP a few centimeters along
  // a long requested path isn't worth its planning cost -- confirmed live: a
  // 1.5cm gain from a full RRTConnect solve, taking over a minute to
  // execute. Reverted rather than kept.
  auto cartesian = std::make_unique<JumpThenResumeCartesianPlanner>(
    CartesianTruncationReason::kJumpDiscontinuity, /*first_call_fraction=*/ 0.996,
    /*resume_call_fraction=*/ 1.0);
  auto * cartesian_ptr = cartesian.get();
  auto fsp = std::make_unique<ScriptedPlanner>("fsp", true, 1.0);
  auto * fsp_ptr = fsp.get();
  MotionPlanner planner(std::move(cartesian), std::move(fsp));

  SegmentRoutingSpec routing;
  routing.label = "sweep_strip_0";

  // 10m total path: bridging to fraction 1.0 (+0.5% ahead-step, clamped)
  // then a fully-successful resume of the tiny remainder only nets
  // (1.0 - 0.996) * 10m = 0.04m -- below the 5cm minimum.
  core_types::Pose start_pose;
  start_pose.x = 0.0;
  core_types::Pose end_pose;
  end_pose.x = 10.0;

  PlanningRequest request = makeRequest();
  request.target_poses = {start_pose, end_pose};

  const auto result = planner.planSegment(request, routing);

  EXPECT_DOUBLE_EQ(result.planning_result.path_fraction, 0.996)
    << "reverted -- the reported fraction stays the initial attempt's own, unimproved";
  EXPECT_EQ(result.planning_result.truncation_reason, CartesianTruncationReason::kJumpDiscontinuity)
    << "reverted -- still whatever the initial (un-bridged) attempt reported";
  EXPECT_EQ(cartesian_ptr->plan_call_count, 2)
    << "the resume attempt still happens once before its low gain is discovered";
  EXPECT_EQ(fsp_ptr->plan_call_count, 1) << "the bridge attempt still happens once, then reverted";
  ASSERT_EQ(result.planning_result.trajectory.points.size(), 1u)
    << "the reverted bridge+resume legs are discarded -- just the initial attempt's own point";
  ASSERT_EQ(result.trajectory_segments.size(), 1u)
    << "reverted -- no phantom fsp/cartesian entries for the discarded cycle";
  EXPECT_EQ(result.trajectory_segments[0].planner_name, "cartesian");
  EXPECT_EQ(result.trajectory_segments[0].start_index, 0u);
  EXPECT_EQ(result.trajectory_segments[0].end_index, 1u);
}

TEST(MotionPlannerTest, FspRoutedSegmentDoesNotBridgeJumpDiscontinuity)
{
  // The bridge-then-resume loop is scoped to cartesian-routed segments only
  // -- an fsp-routed segment's own Cartesian preflight attempt must never
  // trigger it, even if that preflight result happens to carry a jump
  // truncation reason (it's discarded wholesale in favor of free_space_planner_
  // once it doesn't clear min_path_fraction, same as before this feature).
  auto cartesian = std::make_unique<JumpThenResumeCartesianPlanner>();
  auto * cartesian_ptr = cartesian.get();
  auto fsp = std::make_unique<ScriptedPlanner>("fsp", true, 1.0);
  auto * fsp_ptr = fsp.get();
  MotionPlanner planner(std::move(cartesian), std::move(fsp));

  SegmentRoutingSpec routing;
  routing.label = "connector_0";
  routing.mode_hint = "fsp";

  PlanningRequest request = makeRequest(0.5);
  request.target_poses = {core_types::Pose{}, core_types::Pose{}};

  const auto result = planner.planSegment(request, routing);

  EXPECT_EQ(result.mode_used, "fsp");
  EXPECT_EQ(cartesian_ptr->plan_call_count, 1)
    << "only the fsp-preflight Cartesian attempt -- no bridge/resume loop";
  EXPECT_EQ(fsp_ptr->plan_call_count, 1);
}

TEST(MotionPlannerTest, CartesianRoutedSegmentBridgesCollisionThenResumes)
{
  // Same bridge-then-resume mechanism as the jump-discontinuity case above --
  // now also triggered by kCollision, since CartesianPlanner only reports it
  // after its own consistency-limited local recovery is already exhausted
  // (an equally strong "genuinely stuck" signal, not the weaker "collision
  // was ignored to get further" it used to mean).
  auto cartesian = std::make_unique<JumpThenResumeCartesianPlanner>(
    CartesianTruncationReason::kCollision);
  auto * cartesian_ptr = cartesian.get();
  auto fsp = std::make_unique<ScriptedPlanner>("fsp", true, 1.0);
  auto * fsp_ptr = fsp.get();
  MotionPlanner planner(std::move(cartesian), std::move(fsp));

  SegmentRoutingSpec routing;
  routing.label = "sweep_strip_0";

  PlanningRequest request = makeRequest();
  request.target_poses = {core_types::Pose{}, core_types::Pose{}};

  const auto result = planner.planSegment(request, routing);

  EXPECT_TRUE(result.planning_result.success);
  EXPECT_EQ(result.planning_result.truncation_reason, CartesianTruncationReason::kNone)
    << "the final reported result should be the resumed (fully-succeeded) attempt";
  EXPECT_EQ(cartesian_ptr->plan_call_count, 2)
    << "one initial attempt + one resume after the bridge, then the loop stops "
    "because the resumed result's truncation reason is no longer kCollision";
  EXPECT_EQ(fsp_ptr->plan_call_count, 1) << "should bridge exactly once via free_space_planner_";
  ASSERT_EQ(result.planning_result.trajectory.points.size(), 3u)
    << "stitched: initial partial (1pt) + bridge (1pt) + resumed (1pt)";
  EXPECT_EQ(cartesian_ptr->last_start_joint_states[1], fsp_ptr->end_positions)
    << "the resume call should start from the bridge's own end state";
  ASSERT_EQ(result.trajectory_segments.size(), 3u)
    << "one provenance entry per leg -- initial cartesian, fsp bridge, resumed cartesian";
  EXPECT_EQ(result.trajectory_segments[0].planner_name, "cartesian");
  EXPECT_EQ(result.trajectory_segments[0].start_index, 0u);
  EXPECT_EQ(result.trajectory_segments[0].end_index, 1u);
  EXPECT_EQ(result.trajectory_segments[1].planner_name, "fsp");
  EXPECT_EQ(result.trajectory_segments[1].start_index, 1u);
  EXPECT_EQ(result.trajectory_segments[1].end_index, 2u);
  EXPECT_EQ(result.trajectory_segments[2].planner_name, "cartesian");
  EXPECT_EQ(result.trajectory_segments[2].start_index, 2u);
  EXPECT_EQ(result.trajectory_segments[2].end_index, 3u);
}

TEST(MotionPlannerTest, CartesianRoutedSegmentDoesNotBridgeIkFailure)
{
  // kIkFailure means no nearby configuration reaches that pose at all --
  // bridging to a different point doesn't help reach an unreachable one, so
  // the bridge loop must never fire for it (unlike jump/collision above).
  // allow_target_contact=true isolates that from the separate contact-aware
  // fallback (Part C), which would otherwise also kick in below
  // min_path_fraction regardless of truncation_reason -- covered by its own
  // tests.
  auto cartesian = std::make_unique<JumpThenResumeCartesianPlanner>(
    CartesianTruncationReason::kIkFailure);
  auto * cartesian_ptr = cartesian.get();
  auto fsp = std::make_unique<ScriptedPlanner>("fsp", true, 1.0);
  auto * fsp_ptr = fsp.get();
  MotionPlanner planner(std::move(cartesian), std::move(fsp));

  SegmentRoutingSpec routing;
  routing.label = "sweep_strip_0";

  PlanningRequest request = makeRequest();
  request.target_poses = {core_types::Pose{}, core_types::Pose{}};
  request.allow_target_contact = true;

  const auto result = planner.planSegment(request, routing);

  EXPECT_TRUE(result.planning_result.success);
  EXPECT_EQ(result.planning_result.truncation_reason, CartesianTruncationReason::kIkFailure)
    << "the low-fraction ik_failure result is accepted as-is, never bridged";
  EXPECT_EQ(cartesian_ptr->plan_call_count, 1) << "no resume attempt -- the bridge never fires";
  EXPECT_EQ(fsp_ptr->plan_call_count, 0) << "no bridge attempt via free_space_planner_ either";
  ASSERT_EQ(result.planning_result.trajectory.points.size(), 1u)
    << "just the initial partial attempt, unstitched";
}

TEST(MotionPlannerTest, FspRoutedSegmentStaysCartesianWhenAttemptClearsThreshold)
{
  auto cartesian = std::make_unique<ScriptedPlanner>("cartesian", true, 1.0);
  auto * cartesian_ptr = cartesian.get();
  auto fsp = std::make_unique<ScriptedPlanner>("fsp", true, 1.0);
  auto * fsp_ptr = fsp.get();
  MotionPlanner planner(std::move(cartesian), std::move(fsp));

  SegmentRoutingSpec routing;
  routing.label = "connector_0";
  routing.mode_hint = "fsp";

  const auto result = planner.planSegment(makeRequest(), routing);

  EXPECT_EQ(result.mode_used, "cartesian");
  EXPECT_TRUE(result.planning_result.success);
  EXPECT_EQ(cartesian_ptr->plan_call_count, 1);
  EXPECT_EQ(fsp_ptr->plan_call_count, 0)
    << "a fsp-routed segment Cartesian can fully solve should stay on cartesian";
}

TEST(MotionPlannerTest, FspRoutedSegmentFallsBackWhenCartesianAttemptBelowThreshold)
{
  auto cartesian = std::make_unique<ScriptedPlanner>("cartesian", true, 0.2);
  auto * cartesian_ptr = cartesian.get();
  auto fsp = std::make_unique<ScriptedPlanner>("fsp", true, 1.0);
  auto * fsp_ptr = fsp.get();
  MotionPlanner planner(std::move(cartesian), std::move(fsp));

  SegmentRoutingSpec routing;
  routing.label = "connector_0";
  routing.mode_hint = "fsp";

  const auto result = planner.planSegment(makeRequest(0.5), routing);

  EXPECT_EQ(result.mode_used, "fsp");
  EXPECT_TRUE(result.planning_result.success);
  EXPECT_EQ(cartesian_ptr->plan_call_count, 1)
    << "Cartesian must still be attempted first even though it ends up rerouted";
  EXPECT_EQ(fsp_ptr->plan_call_count, 1);
}

TEST(MotionPlannerTest, FspRoutedSegmentReportsFailureWhenBothPlannersFail)
{
  auto cartesian = std::make_unique<ScriptedPlanner>("cartesian", false, 0.0);
  auto fsp = std::make_unique<ScriptedPlanner>("fsp", false, 0.0);
  MotionPlanner planner(std::move(cartesian), std::move(fsp));

  SegmentRoutingSpec routing;
  routing.label = "connector_0";
  routing.mode_hint = "fsp";

  const auto result = planner.planSegment(makeRequest(), routing);

  EXPECT_EQ(result.mode_used, "fsp");
  EXPECT_FALSE(result.planning_result.success);
}

TEST(MotionPlannerTest, ContinuityThreadsPreviousEndStateIntoNextPlanSegmentCall)
{
  auto cartesian = std::make_unique<ScriptedPlanner>("cartesian", true, 1.0);
  auto * cartesian_ptr = cartesian.get();
  MotionPlanner planner(std::move(cartesian), std::make_unique<ScriptedPlanner>("fsp", true, 1.0));

  SegmentRoutingSpec routing;
  routing.label = "sweep_strip_0";

  const auto first = planner.planSegment(makeRequest(), routing);
  EXPECT_TRUE(cartesian_ptr->last_start_joint_state.empty())
    << "the first segment has no previous segment to seed from";

  planner.recordExecutionOutcome(true, first.mode_used, first.planning_result.trajectory);

  planner.planSegment(makeRequest(), routing);
  EXPECT_EQ(cartesian_ptr->last_start_joint_state, cartesian_ptr->end_positions)
    << "the second call's start_joint_state should be seeded from the first segment's planned end";
}

TEST(MotionPlannerTest, ContinuityDoesNotAdvanceOnExecutionFailure)
{
  auto cartesian = std::make_unique<ScriptedPlanner>("cartesian", true, 1.0);
  auto * cartesian_ptr = cartesian.get();
  MotionPlanner planner(std::move(cartesian), std::make_unique<ScriptedPlanner>("fsp", true, 1.0));

  SegmentRoutingSpec routing;
  routing.label = "sweep_strip_0";

  const auto first = planner.planSegment(makeRequest(), routing);
  // Execution failed even though planning succeeded -- continuity must not
  // advance, matching the pre-extraction behavior exactly (the robot never
  // actually moved).
  planner.recordExecutionOutcome(false, first.mode_used, first.planning_result.trajectory);

  planner.planSegment(makeRequest(), routing);
  EXPECT_TRUE(cartesian_ptr->last_start_joint_state.empty());
}

TEST(MotionPlannerTest, ResetContinuityClearsPreviousEndState)
{
  auto cartesian = std::make_unique<ScriptedPlanner>("cartesian", true, 1.0);
  auto * cartesian_ptr = cartesian.get();
  MotionPlanner planner(std::move(cartesian), std::make_unique<ScriptedPlanner>("fsp", true, 1.0));

  SegmentRoutingSpec routing;
  routing.label = "sweep_strip_0";

  const auto first = planner.planSegment(makeRequest(), routing);
  planner.recordExecutionOutcome(true, first.mode_used, first.planning_result.trajectory);

  planner.resetContinuity();
  planner.planSegment(makeRequest(), routing);
  EXPECT_TRUE(cartesian_ptr->last_start_joint_state.empty())
    << "resetContinuity() should clear the previous mission's end state";
}

TEST(MotionPlannerTest, PlanSegmentWithStartStateUsesExplicitSeedNotContinuity)
{
  auto cartesian = std::make_unique<ScriptedPlanner>("cartesian", true, 1.0);
  auto * cartesian_ptr = cartesian.get();
  MotionPlanner planner(std::move(cartesian), std::make_unique<ScriptedPlanner>("fsp", true, 1.0));

  SegmentRoutingSpec routing;
  routing.label = "sweep_strip_0";
  const std::vector<double> explicit_seed = {9.0, 8.0, 7.0};

  planner.planSegmentWithStartState(makeRequest(), routing, explicit_seed);
  EXPECT_EQ(cartesian_ptr->last_start_joint_state, explicit_seed)
    << "planSegmentWithStartState() must seed from its own explicit parameter";

  // It must never have written previous_end_joint_state_/has_previous_segment_
  // as a side effect -- a subsequent continuity-based planSegment() call
  // should behave exactly as if the speculative call never happened (no
  // previous segment recorded yet).
  planner.planSegment(makeRequest(), routing);
  EXPECT_TRUE(cartesian_ptr->last_start_joint_state.empty())
    << "planSegmentWithStartState() must not mutate tracked continuity state";
}

TEST(MotionPlannerTest, PlanSegmentWithStartStateIsSafeUnderConcurrentCalls)
{
  // A smoke test for planning_mutex_ actually serializing access, not a
  // stress test of CartesianPlanner/OmplPlanner's own real thread-safety
  // (out of scope here -- ScriptedPlanner has no shared mutable state of its
  // own to race). Two concurrent calls against the same MotionPlanner
  // instance must both complete cleanly with their own distinct seed
  // reflected in the result, with no crash/deadlock.
  auto cartesian = std::make_unique<ScriptedPlanner>("cartesian", true, 1.0);
  MotionPlanner planner(std::move(cartesian), std::make_unique<ScriptedPlanner>("fsp", true, 1.0));

  SegmentRoutingSpec routing;
  routing.label = "sweep_strip_0";

  auto future_a = std::async(
    std::launch::async, [&planner, &routing]() {
      return planner.planSegmentWithStartState(makeRequest(), routing, {1.0, 1.0, 1.0});
    });
  auto future_b = std::async(
    std::launch::async, [&planner, &routing]() {
      return planner.planSegmentWithStartState(makeRequest(), routing, {2.0, 2.0, 2.0});
    });

  const auto result_a = future_a.get();
  const auto result_b = future_b.get();
  EXPECT_TRUE(result_a.planning_result.success);
  EXPECT_TRUE(result_b.planning_result.success);
}

TEST(MotionPlannerTest, SeedContinuityGivesTheFirstSegmentAStartStateOnce)
{
  auto fsp = std::make_unique<ScriptedPlanner>("fsp", true, 1.0);
  auto * fsp_ptr = fsp.get();
  MotionPlanner planner(std::make_unique<ScriptedPlanner>("cart", false, 0.0), std::move(fsp));

  SegmentRoutingSpec routing;
  routing.label = "prime";
  routing.mode_hint = "fsp";
  PlanningRequest request = makeRequest(0.5);
  request.target_joint_state = {0.5, 0.5, 0.5};

  planner.resetContinuity();
  planner.seedContinuity({1.0, 2.0, 3.0});
  planner.planSegment(request, routing);
  EXPECT_EQ(fsp_ptr->last_start_joint_state, (std::vector<double>{1.0, 2.0, 3.0}));

  // once a segment has run, a later seed must not override the continuity
  core_types::Trajectory traj;
  core_types::TrajectoryPoint p;
  p.positions = {9.0, 9.0, 9.0};
  traj.points = {p};
  planner.recordExecutionOutcome(true, "fsp", traj);
  planner.seedContinuity({1.0, 1.0, 1.0});
  planner.planSegment(request, routing);
  EXPECT_EQ(fsp_ptr->last_start_joint_state, (std::vector<double>{9.0, 9.0, 9.0}));

  // a fresh mission forgets it again
  planner.resetContinuity();
  planner.planSegment(request, routing);
  EXPECT_TRUE(fsp_ptr->last_start_joint_state.empty());
}

}  // namespace
}  // namespace motion_planner
