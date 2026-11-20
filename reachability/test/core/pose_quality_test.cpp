#include <gtest/gtest.h>

#include "reachability/core/pose_quality.hpp"
#include "support.hpp"
#include "synthetic_map.hpp"

using namespace reachability;  // NOLINT
using namespace reachability_test;  // NOLINT

TEST(PoseQuality, EmptyCellAndOutsideWindowScoreZero)
{
  const Map4D map = syntheticMap();
  const MarginConfig m;
  EXPECT_FALSE(scorePose(map, towardPlusX(0.1, 0, 0), m).reachable);
  EXPECT_EQ(scorePose(map, towardPlusX(0.1, 0, 0), m).score, 0.0);
  EXPECT_FALSE(scorePose(map, towardPlusX(5.0, 0, 0), m).reachable);
}

TEST(PoseQuality, InteriorScoresHigherThanEdge)
{
  const Map4D map = syntheticMap();
  const MarginConfig m;
  const PoseScore inner = scorePose(map, towardPlusX(0.55, 0.0, 0.1), m);
  const PoseScore rim = scorePose(map, towardPlusX(0.87, 0.0, 0.1), m);
  EXPECT_TRUE(inner.reachable);
  EXPECT_EQ(inner.count_score, 1.0);          // 20 >= count_ref 8
  EXPECT_NEAR(inner.boundary_score, 6.0 / 8.0, 1e-12);   // theta neighbours are empty in this map
  EXPECT_NEAR(inner.score, 0.75, 1e-12);
  EXPECT_TRUE(rim.reachable);
  EXPECT_LT(rim.score, inner.score);
  EXPECT_NEAR(rim.count_score, 1.0 / 8.0, 1e-12);
  // The count reference scales the score.
  MarginConfig strict = m;
  strict.count_ref = 40.0;
  EXPECT_NEAR(scorePose(map, towardPlusX(0.55, 0.0, 0.1), strict).count_score, 0.5, 1e-12);
}

TEST(PoseQuality, SingularValueAndLimitMargin)
{
  const ChainModel arm = genericArm();
  EXPECT_LT(minSingularValue(arm, {0, 0, 0, 0, 0, 0}), 1e-3);
  EXPECT_GT(minSingularValue(arm, {0.3, -1.0, 1.6, -0.6, 0.4, 0.8}), 0.02);
  EXPECT_EQ(minSingularValue(arm, {0, 0}), -1.0);   // wrong size
  EXPECT_EQ(jointLimitMargin(arm, {0, 0, 0, 0, 0, 0}), 0.5);   // +-pi joints are continuous: ignored

  ChainJoint j;
  j.lower = -1.0;
  j.upper = 1.0;
  const ChainModel limited({j, j}, Pose::Identity());
  EXPECT_NEAR(jointLimitMargin(limited, {0.0, 0.9}), 0.05, 1e-12);   // 0.1 of a range of 2
  EXPECT_NEAR(jointLimitMargin(limited, {0.0, 0.0}), 0.5, 1e-12);
}
