#include <cmath>

#include <gtest/gtest.h>

#include "motion_planner/core/time_parameterization.hpp"

namespace motion_planner
{
namespace
{

core_types::Trajectory path(const std::vector<std::vector<double>> & q)
{
  core_types::Trajectory t;
  for (size_t j = 0; j < q.front().size(); ++j) {
    t.joint_names.push_back("j" + std::to_string(j));
  }
  for (const auto & p : q) {
    core_types::TrajectoryPoint pt;
    pt.positions = p;
    t.points.push_back(pt);
  }
  return t;
}

TimingRequest request(SegmentRole role, size_t joints, double speed = 0.0)
{
  TimingRequest r;
  r.role = role;
  r.limits.assign(joints, JointLimit{1.0, 2.0, 0.0});
  r.stroke_speed = speed;
  return r;
}

// Fake retimer: constant speed in joint space, records what it was given.
class FakeRetimer : public TimeParameterizer
{
public:
  bool retime(
    core_types::Trajectory & piece, SegmentRole, const std::vector<JointLimit> &,
    double stroke_speed, std::string & message) override
  {
    ++calls;
    if (fail) {
      message = "refused";
      return false;
    }
    constantSpeedStamps(piece, stroke_speed > 0.0 ? stroke_speed : speed);
    last_points = piece.points.size();
    return true;
  }
  double speed{0.1};
  bool fail{false};
  int calls{0};
  size_t last_points{0};
};

TEST(TimeParameterizationTest, ReversalIsFoundAndSplitsIntoTwoPieces)
{
  const auto p = path({{0.0}, {0.1}, {0.2}, {0.1}, {0.0}});
  EXPECT_EQ(findReversals(p, 150.0), std::vector<size_t>({2}));
  FakeRetimer retimer;
  const auto r = parameterizeSegment(retimer, p, request(SegmentRole::kTransition, 1));
  ASSERT_TRUE(r.success) << r.message;
  EXPECT_EQ(r.pieces.size(), 2u);
  EXPECT_EQ(retimer.calls, 2);
  const auto joined = concatenate(r.pieces);
  EXPECT_EQ(joined.points.size(), 5u);
  EXPECT_NEAR(joined.points.back().time_from_start_sec, r.total_duration_sec, 1e-9);
}

TEST(TimeParameterizationTest, ShallowTurnIsNotSplit)
{
  const auto p = path({{0.0, 0.0}, {0.1, 0.0}, {0.2, 0.05}});
  EXPECT_TRUE(findReversals(p, 150.0).empty());
}

TEST(TimeParameterizationTest, NearDuplicatesAreDroppedAndGoalKept)
{
  const auto p = path({{0.0}, {1e-9}, {0.1}, {0.1 + 1e-9}});
  FakeRetimer retimer;
  const auto r = parameterizeSegment(retimer, p, request(SegmentRole::kTransition, 1));
  ASSERT_TRUE(r.success);
  EXPECT_EQ(r.dropped_points, 2u);
  EXPECT_EQ(retimer.last_points, 2u);
  EXPECT_NEAR(r.pieces[0].points.back().positions[0], 0.1 + 1e-9, 1e-12);
}

TEST(TimeParameterizationTest, AllDuplicatesIsTooFewPoints)
{
  FakeRetimer retimer;
  const auto r = parameterizeSegment(
    retimer, path({{0.0}, {0.0}, {0.0}}), request(SegmentRole::kTransition, 1));
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.error, TimingError::kTooFewPoints);
}

TEST(TimeParameterizationTest, ZeroAccelerationLimitIsTypedError)
{
  FakeRetimer retimer;
  auto req = request(SegmentRole::kTransition, 1);
  req.limits[0].acceleration = 0.0;
  const auto r = parameterizeSegment(retimer, path({{0.0}, {1.0}}), req);
  EXPECT_EQ(r.error, TimingError::kBadLimit);
  EXPECT_EQ(retimer.calls, 0);
}

TEST(TimeParameterizationTest, InputErrorsAreTyped)
{
  FakeRetimer retimer;
  EXPECT_EQ(
    parameterizeSegment(retimer, path({{0.0}, {NAN}}), request(SegmentRole::kTransition, 1)).error,
    TimingError::kNonFinite);
  EXPECT_EQ(
    parameterizeSegment(retimer, path({{0.0}, {1.0}}), request(SegmentRole::kTransition, 2)).error,
    TimingError::kJointCountMismatch);
  EXPECT_EQ(
    parameterizeSegment(retimer, path({{0.0}, {1.0}}), request(SegmentRole::kContactSeam, 1)).error,
    TimingError::kBadStrokeSpeed);
  EXPECT_EQ(
    parameterizeSegment(retimer, path({{0.0}}), request(SegmentRole::kTransition, 1)).error,
    TimingError::kTooFewPoints);
}

TEST(TimeParameterizationTest, BackendRefusalIsReported)
{
  FakeRetimer retimer;
  retimer.fail = true;
  const auto r = parameterizeSegment(
    retimer, path({{0.0}, {1.0}}), request(SegmentRole::kTransition, 1));
  EXPECT_EQ(r.error, TimingError::kBackendFailed);
}

TEST(TimeParameterizationTest, ConstantSpeedStampsUseTheMetric)
{
  auto p = path({{0.0}, {0.1}, {0.3}});
  constantSpeedStamps(p, 0.1, [](const auto & a, const auto & b) {return 2.0 * std::fabs(b[0] - a[0]);});
  EXPECT_NEAR(p.points[1].time_from_start_sec, 2.0, 1e-9);
  EXPECT_NEAR(p.points[2].time_from_start_sec, 6.0, 1e-9);
}

TEST(TimeParameterizationTest, ContactSeamViolationFailsTransitionOnlyReports)
{
  // 1 rad in 1 s on a 0.5 rad/s joint
  FakeRetimer retimer;
  auto seam = request(SegmentRole::kContactSeam, 1, 1.0);
  seam.limits[0].velocity = 0.5;
  const auto fail = parameterizeSegment(retimer, path({{0.0}, {1.0}}), seam);
  EXPECT_FALSE(fail.success);
  EXPECT_EQ(fail.error, TimingError::kLimitViolation);
  EXPECT_FALSE(fail.violations.empty());

  auto trans = request(SegmentRole::kTransition, 1);
  trans.limits[0].velocity = 0.5;
  retimer.speed = 1.0;
  const auto ok = parameterizeSegment(retimer, path({{0.0}, {1.0}}), trans);
  EXPECT_TRUE(ok.success);
  EXPECT_FALSE(ok.violations.empty());
  EXPECT_EQ(ok.violations[0].kind, LimitViolation::Kind::kVelocity);
}

TEST(TimeParameterizationTest, JerkIsCheckedOnlyWhenBounded)
{
  core_types::Trajectory t = path({{0.0}, {0.1}, {0.2}});
  const double acc[3] = {0.0, 1.0, 0.0};
  for (int i = 0; i < 3; ++i) {
    t.points[i].time_from_start_sec = 0.1 * i;
    t.points[i].accelerations = {acc[i]};
    t.points[i].velocities = {0.0};
  }
  std::vector<JointLimit> lim{{1.0, 2.0, 0.0}};
  EXPECT_TRUE(findLimitViolations(t, lim, 0.0).empty());
  lim[0].jerk = 5.0;
  EXPECT_FALSE(findLimitViolations(t, lim, 0.0).empty());
}

}  // namespace
}  // namespace motion_planner
