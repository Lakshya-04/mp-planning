#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "reachability/core/path_quality_audit.hpp"
#include "reachability/core/pose_quality.hpp"

using namespace reachability;

namespace
{
// Planar 2-link arm, unit links, task = tool xy. Singular (stretched or folded) at q2 = 0 or pi.
bool planar2(const std::vector<double> & q, Eigen::MatrixXd & J)
{
  if (q.size() != 2) {return false;}
  const double s1 = std::sin(q[0]), c1 = std::cos(q[0]);
  const double s12 = std::sin(q[0] + q[1]), c12 = std::cos(q[0] + q[1]);
  J.resize(2, 2);
  J << -s1 - s12, -s12,
    c1 + c12, c12;
  return true;
}

// Planar 3-link (redundant for xy): 2 x 3 Jacobian, singular only when fully stretched or folded.
bool planar3(const std::vector<double> & q, Eigen::MatrixXd & J)
{
  if (q.size() != 3) {return false;}
  const double a1 = q[0], a2 = q[0] + q[1], a3 = q[0] + q[1] + q[2];
  J.resize(2, 3);
  J << -std::sin(a1) - std::sin(a2) - std::sin(a3), -std::sin(a2) - std::sin(a3), -std::sin(a3),
    std::cos(a1) + std::cos(a2) + std::cos(a3), std::cos(a2) + std::cos(a3), std::cos(a3);
  return true;
}

const std::vector<double> kLo{-3.0, -2.0};
const std::vector<double> kHi{3.0, 2.0};

LimitMarginFn bounds2()
{
  return [](const std::vector<double> & q) {return limitMarginFromBounds(kLo, kHi, q);};
}

PathStroke elbowSweep(const std::string & id, double from, double to, int n)
{
  PathStroke s;
  s.id = id;
  for (int i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) / (n - 1);
    s.waypoints.push_back({0.3, from + t * (to - from)});
  }
  return s;
}
}  // namespace

TEST(JacobianMeasures, PlanarArmValues)
{
  Eigen::MatrixXd J;
  ASSERT_TRUE(planar2({0.3, M_PI / 2}, J));
  EXPECT_NEAR(manipulability(J), 1.0, 1e-12);      // |sin q2| l1 l2
  EXPECT_GE(conditionNumber(J), 1.0);
  const double smin = minSingularValue(J);
  EXPECT_NEAR(manipulability(J) / (smin * smin), conditionNumber(J), 1e-9);   // smax*smin / smin^2
  ASSERT_TRUE(planar2({0.3, 0.0}, J));
  EXPECT_LT(minSingularValue(J), 1e-9);
  EXPECT_TRUE(std::isinf(conditionNumber(J)));
  EXPECT_LT(manipulability(J), 1e-9);
  EXPECT_EQ(minSingularValue(Eigen::MatrixXd()), -1.0);
}

TEST(PathQualityAudit, PathThroughSingularPoseIsNearSingular)
{
  const auto rep = auditPathQuality({elbowSweep("s0", 1.0, -1.0, 21)}, planar2, bounds2());
  ASSERT_EQ(rep.strokes.size(), 1u);
  const auto & s = rep.strokes[0];
  EXPECT_EQ(s.reason, PathQualityReason::kNearSingular);
  EXPECT_STREQ(toString(s.reason), "near_singular");
  EXPECT_EQ(s.worst_index, 10);                    // q2 == 0 in the middle
  EXPECT_LT(s.min_sigma, 1e-6);
  EXPECT_EQ(rep.near_singular, 1);
  EXPECT_TRUE(rep.flagged());
}

TEST(PathQualityAudit, BentPathIsOk)
{
  const auto rep = auditPathQuality({elbowSweep("s0", 1.0, 1.5, 11)}, planar2, bounds2());
  EXPECT_EQ(rep.strokes[0].reason, PathQualityReason::kOk);
  EXPECT_GT(rep.strokes[0].min_sigma, 0.2);
  EXPECT_NEAR(rep.strokes[0].min_limit_margin, 0.125, 1e-12);   // 0.5 / 4 at q2 = 1.5
  EXPECT_EQ(rep.ok, 1);
  EXPECT_FALSE(rep.flagged());
}

TEST(PathQualityAudit, MarginNearJointLimit)
{
  // q2 limit is 2.0; 1.99 is 0.01 of a range of 4 from it = 0.0025 < 0.02. Sigma is healthy there.
  PathStroke s = elbowSweep("s0", 1.5, 1.99, 6);
  const auto rep = auditPathQuality({s}, planar2, bounds2());
  const auto & r = rep.strokes[0];
  EXPECT_EQ(r.reason, PathQualityReason::kNearLimit);
  EXPECT_STREQ(toString(r.reason), "near_limit");
  EXPECT_EQ(r.worst_index, 5);
  EXPECT_NEAR(r.min_limit_margin, 0.0025, 1e-9);
  EXPECT_EQ(rep.near_limit, 1);
}

TEST(PathQualityAudit, ThresholdsOffReportsOk)
{
  PathQualityConfig off;
  off.min_sigma = 0.0;
  off.min_limit_margin = 0.0;
  const auto rep = auditPathQuality(
    {elbowSweep("a", 1.0, -1.0, 21), elbowSweep("b", 1.5, 1.99, 6)}, planar2, bounds2(), off);
  EXPECT_EQ(rep.ok, 2);
  EXPECT_EQ(rep.near_singular + rep.near_limit, 0);
  EXPECT_FALSE(rep.flagged());
  EXPECT_LT(rep.strokes[0].min_sigma, 1e-6);       // still measured
}

TEST(PathQualityAudit, DefaultThresholdsAreNonZero)
{
  const PathQualityConfig c;
  EXPECT_GT(c.min_sigma, 0.0);
  EXPECT_GT(c.min_limit_margin, 0.0);
}

TEST(PathQualityAudit, AggregateCountsAndPrecedence)
{
  PathStroke both = elbowSweep("both", 1.99, 0.0, 5);   // starts at the limit, ends singular
  PathStroke bad;
  bad.id = "bad";
  bad.waypoints = {{0.0}};                               // wrong size: nothing evaluated
  const auto rep = auditPathQuality(
    {elbowSweep("ok", 1.0, 1.5, 5), elbowSweep("sing", 0.5, -0.5, 11), elbowSweep("lim", 1.5, 1.99, 4), both, bad},
    planar2, bounds2());
  EXPECT_EQ(rep.total, 5);
  EXPECT_EQ(rep.near_singular, 2);
  EXPECT_EQ(rep.near_limit, 1);
  EXPECT_EQ(rep.ok, 2);
  EXPECT_EQ(rep.unevaluated, 1);
  EXPECT_EQ(rep.strokes[3].reason, PathQualityReason::kNearSingular);
  EXPECT_TRUE(rep.strokes[3].near_limit_too);
  EXPECT_EQ(rep.strokes[3].worst_index, 4);
  EXPECT_LT(rep.min_sigma, 1e-6);
  EXPECT_NEAR(rep.min_limit_margin, 0.0025, 1e-9);
}

TEST(PathQualityAudit, ThreeLinkStretchedPoseAndNullCallables)
{
  PathStroke s;
  s.id = "p3";
  s.waypoints = {{0.2, 0.8, 0.8}, {0.2, 0.0, 0.0}};   // second is fully stretched
  const auto rep = auditPathQuality({s}, planar3, nullptr);
  EXPECT_EQ(rep.strokes[0].reason, PathQualityReason::kNearSingular);
  EXPECT_EQ(rep.strokes[0].worst_index, 1);
  EXPECT_EQ(rep.strokes[0].min_limit_margin, -1.0);   // no margin callable
  const auto none = auditPathQuality({s}, nullptr, nullptr);
  EXPECT_EQ(none.unevaluated, 1);
  EXPECT_EQ(none.ok, 1);
}

TEST(PathQualityAudit, LimitMarginFromBounds)
{
  EXPECT_NEAR(limitMarginFromBounds(kLo, kHi, {0.0, 0.0}), 0.5, 1e-12);
  EXPECT_NEAR(limitMarginFromBounds(kLo, kHi, {0.0, 1.0}), 0.25, 1e-12);
  EXPECT_EQ(limitMarginFromBounds(kLo, kHi, {0.0}), -1.0);
  EXPECT_EQ(limitMarginFromBounds({-M_PI, -M_PI}, {M_PI, M_PI}, {3.0, 3.0}), 0.5);   // continuous ignored
}
