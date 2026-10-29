#include <gtest/gtest.h>

#include <random>

#include "reachability/core/builder.hpp"
#include "support.hpp"

using namespace reachability;  // NOLINT
using namespace reachability_test;  // NOLINT

namespace
{
// One map shared by the tests below: building it is the expensive part.
const Map4D & sharedMap()
{
  static const Map4D map = [] {
      static const ChainModel arm = genericArm();
      BuildOptions o;
      o.samples = 1'500'000;
      o.threads = 4;
      return buildMap(arm, coarse(), o);
    }();
  return map;
}
}  // namespace

TEST(Builder, FreshSamplesAreAlmostAlwaysReachable)
{
  const ChainModel arm = genericArm();
  std::mt19937 rng(99);
  int in_window = 0, reachable = 0;
  std::vector<double> q(6);
  Pose T;
  for (int i = 0; i < 20000; ++i) {
    for (int j = 0; j < 6; ++j) {q[j] = std::uniform_real_distribution<double>(-M_PI, M_PI)(rng);}
    ASSERT_TRUE(arm.evaluate(q, T));
    if (!sharedMap().keyOf(T).valid) {continue;}
    ++in_window;
    reachable += sharedMap().isReachable(T) ? 1 : 0;
  }
  EXPECT_GT(in_window, 15000);
  EXPECT_GT(static_cast<double>(reachable) / in_window, 0.97);
}

TEST(Builder, PosesBeyondTheArmsReachAreUnreachable)
{
  // The arm reaches about 1.4 m horizontally (tool pointing out) and 1.6 m straight up. Poses
  // beyond that are inside the map window (xy_extent 2.2) but cannot be reached.
  Pose far = Pose::Identity();
  far.translation() = Vec3(1.8, 0.0, 0.2);
  far.linear() = Eigen::AngleAxisd(M_PI / 2, Vec3::UnitY()).toRotationMatrix();
  EXPECT_FALSE(sharedMap().isReachable(far));
  far.linear().setIdentity();
  far.translation() = Vec3(0.0, 0.0, 1.9);
  EXPECT_FALSE(sharedMap().isReachable(far));
  // Right at the limit: still reachable straight up.
  Pose limit = Pose::Identity();
  limit.translation() = Vec3(0.0, 0.0, 1.5);
  EXPECT_TRUE(sharedMap().isReachable(limit));
  // And something comfortably within reach is.
  Pose near = Pose::Identity();
  near.translation() = Vec3(0.5, 0.0, 0.4);
  near.linear() = Eigen::AngleAxisd(M_PI / 2, Vec3::UnitY()).toRotationMatrix();
  EXPECT_TRUE(sharedMap().isReachable(near));
}

TEST(Builder, InverseQueryFindsTheTrueBaseForRandomPlacements)
{
  const ChainModel arm = genericArm();
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> q_dist(-M_PI, M_PI), xy(-5.0, 5.0), yaw(-M_PI, M_PI);
  const MapParams p = coarse();
  int tested = 0, found = 0;
  std::vector<double> q(6);
  Pose T;
  while (tested < 300) {
    for (int j = 0; j < 6; ++j) {q[j] = q_dist(rng);}
    if (!arm.evaluate(q, T)) {continue;}
    const Vec3 axis = T.linear().col(2);
    if (std::hypot(axis.x(), axis.y()) < 0.1) {continue;}      // vertical: ring case tested apart
    if (!sharedMap().keyOf(T).valid) {continue;}
    const double bx = xy(rng), by = xy(rng), bz = 0.3 + 0.2 * q_dist(rng) / M_PI;
    Pose base = Pose::Identity();
    base.translation() = Vec3(bx, by, bz);
    base.linear() = Eigen::AngleAxisd(yaw(rng), Vec3::UnitZ()).toRotationMatrix();
    const Pose world = base * T;
    ++tested;
    const auto candidates = sharedMap().inverseQuery(world, bz);
    for (const auto & c : candidates) {
      if (std::hypot(c.x - bx, c.y - by) <= 1.5 * p.resolution) {
        ++found;
        break;
      }
    }
  }
  EXPECT_GT(static_cast<double>(found) / tested, 0.97)
    << found << " of " << tested << " true base positions were among the candidates";
}

TEST(Builder, EveryInverseCandidateReallyReachesThePose)
{
  // Forward check of the inverse answer: place the arm base at each candidate and the pose must
  // be reachable in the arm's own frame (the map is self-consistent by construction).
  Pose T = Pose::Identity();
  T.translation() = Vec3(0.55, 0.1, 0.35);
  T.linear() = Eigen::AngleAxisd(M_PI / 2, Vec3::UnitY()).toRotationMatrix();
  ASSERT_TRUE(sharedMap().isReachable(T));
  Pose world = Pose::Identity();
  world.translation() = Vec3(2.0, -1.0, 1.5);
  world.linear() = Eigen::AngleAxisd(0.8, Vec3::UnitZ()).toRotationMatrix() * T.linear();
  const auto candidates = sharedMap().inverseQuery(world, 1.15);
  ASSERT_FALSE(candidates.empty());
  for (const auto & c : candidates) {
    Pose in_base = world;
    in_base.translation() -= Vec3(c.x, c.y, 1.15);
    in_base.linear() = Pose(Eigen::AngleAxisd(0.0, Vec3::UnitZ())).linear() * in_base.linear();
    // Base yaw is free: undo the world azimuth so the check uses the same canonical frame.
    const Vec3 axis = world.linear().col(2);
    const double phi = std::atan2(axis.y(), axis.x());
    const Pose rot(Eigen::AngleAxisd(-phi, Vec3::UnitZ()));
    EXPECT_TRUE(sharedMap().isReachable(rot * in_base))
      << "candidate (" << c.x << ", " << c.y << ")";
  }
}

TEST(Builder, IsDeterministicForTheSameSeedAndThreads)
{
  const ChainModel arm = genericArm();
  BuildOptions o;
  o.samples = 80000;
  o.threads = 3;
  o.seed = 5;
  const Map4D a = buildMap(arm, coarse(), o);
  const Map4D b = buildMap(arm, coarse(), o);
  EXPECT_EQ(a.raw(), b.raw());
  o.seed = 6;
  EXPECT_NE(a.raw(), buildMap(arm, coarse(), o).raw());
}

TEST(Builder, StatsAreConsistent)
{
  const ChainModel arm = genericArm();
  BuildOptions o;
  o.samples = 50000;
  o.threads = 2;
  BuildStats s;
  const Map4D m = buildMap(arm, coarse(), o, &s);
  EXPECT_EQ(s.samples, 50000U);
  EXPECT_EQ(s.valid, 50000U);          // the chain accepts everything within +-pi
  EXPECT_LE(s.in_window, s.valid);
  EXPECT_EQ(s.nonzero_cells, m.nonzeroCells());
  EXPECT_GT(s.nonzero_cells, 1000U);
  EXPECT_GE(s.seconds, 0.0);
}

TEST(Builder, InvalidSamplesAreDropped)
{
  ChainModel arm = genericArm();
  arm.setValidity([](const std::vector<double> & q) {return q[0] > 0.0;});  // half the space
  BuildOptions o;
  o.samples = 20000;
  o.threads = 2;
  BuildStats s;
  buildMap(arm, coarse(), o, &s);
  EXPECT_GT(s.valid, 8000U);
  EXPECT_LT(s.valid, 12000U);
}
