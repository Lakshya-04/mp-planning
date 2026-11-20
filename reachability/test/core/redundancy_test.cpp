#include <gtest/gtest.h>

#include <unistd.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <random>

#include "reachability/core/builder.hpp"
#include "reachability/core/io.hpp"
#include "reachability/core/redundancy.hpp"
#include "support.hpp"

using namespace reachability;  // NOLINT
using namespace reachability_test;  // NOLINT
namespace fs = std::filesystem;

namespace
{
std::string tmp(const std::string & name)
{
  return (fs::temp_directory_path() / ("reachability_red_" + std::to_string(::getpid()) + "_" + name)).string();
}

// The 7-DOF arm reaches about 1.1 m, so a finer window than the generic arm's keeps cells populated.
MapParams sevenParams()
{
  MapParams p;
  p.xy_extent = 1.4;
  p.z_min = -1.4;
  p.z_max = 1.6;
  p.resolution = 0.1;
  p.theta_bin = M_PI / 9;
  return p;
}

const Map4D & unionMap()
{
  static const Map4D map = [] {
      static const ChainModel arm = sevenDofArm();
      BuildOptions o;
      o.samples = 1'500'000;
      o.threads = 2;
      return buildMap(arm, sevenParams(), o);
    }();
  return map;
}

// Reference of the pre-N-DOF builder: one thread, one uniform draw per joint in order.
Map4D referenceBuild(const ArmModel & arm, const MapParams & params, size_t samples, uint32_t seed)
{
  Map4D map(params);
  std::mt19937_64 rng(seed * 1000003ULL + 0);
  std::vector<double> q(static_cast<size_t>(arm.dof()));
  Pose tool;
  for (size_t n = 0; n < samples; ++n) {
    for (int j = 0; j < arm.dof(); ++j) {
      std::uniform_real_distribution<double> u(arm.lower(j), arm.upper(j));
      q[static_cast<size_t>(j)] = u(rng);
    }
    if (!arm.evaluate(q, tool)) {continue;}
    const CellKey key = map.keyOf(tool);
    if (key.valid) {map.recordKey(key);}
  }
  return map;
}
}  // namespace

TEST(Redundancy, SixDofResultsAreUnchanged)
{
  const ChainModel arm = genericArm();
  BuildOptions o;
  o.samples = 40000;
  o.threads = 1;
  o.seed = 5;
  EXPECT_FALSE(o.redundancy.active());
  const Map4D built = buildMap(arm, coarse(), o);
  EXPECT_EQ(built.raw(), referenceBuild(arm, coarse(), 40000, 5).raw());
}

TEST(Redundancy, SevenDofChainHasSevenJointsAndEvaluates)
{
  const ChainModel arm = sevenDofArm();
  ASSERT_EQ(arm.dof(), 7);
  Pose T;
  ASSERT_TRUE(arm.evaluate({0, 0, 0, -1.5, 0, 1.5, 0}, T));
  EXPECT_FALSE(arm.evaluate({0, 0, 0, 0.5, 0, 1.5, 0}, T));   // joint 4 limit
  EXPECT_FALSE(arm.evaluate({0, 0, 0, -1.5, 0, 1.5}, T));     // six values for a 7-joint chain
}

TEST(Redundancy, SevenDofMapIsReachableForFreshSamplesAndNotBeyondReach)
{
  const ChainModel arm = sevenDofArm();
  std::mt19937 rng(11);
  int in_window = 0, reachable = 0;
  std::vector<double> q(7);
  Pose T;
  for (int i = 0; i < 20000; ++i) {
    for (int j = 0; j < 7; ++j) {
      q[j] = std::uniform_real_distribution<double>(arm.lower(j), arm.upper(j))(rng);
    }
    ASSERT_TRUE(arm.evaluate(q, T));
    if (!unionMap().keyOf(T).valid) {continue;}
    ++in_window;
    reachable += unionMap().isReachable(T) ? 1 : 0;
  }
  EXPECT_GT(in_window, 15000);
  EXPECT_GT(static_cast<double>(reachable) / in_window, 0.95);
  Pose far = Pose::Identity();
  far.translation() = Vec3(1.35, 0.0, 0.3);
  far.linear() = Eigen::AngleAxisd(M_PI / 2, Vec3::UnitY()).toRotationMatrix();
  EXPECT_FALSE(unionMap().isReachable(far));
  Pose near = Pose::Identity();
  near.translation() = Vec3(0.5, 0.0, 0.4);
  near.linear() = Eigen::AngleAxisd(M_PI / 2, Vec3::UnitY()).toRotationMatrix();
  EXPECT_TRUE(unionMap().isReachable(near));
}

TEST(Redundancy, SevenDofInverseQueryFindsTheTrueBase)
{
  const ChainModel arm = sevenDofArm();
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> xy(-5.0, 5.0), yaw(-M_PI, M_PI);
  const MapParams p = sevenParams();
  int tested = 0, found = 0;
  std::vector<double> q(7);
  Pose T;
  while (tested < 200) {
    for (int j = 0; j < 7; ++j) {
      q[j] = std::uniform_real_distribution<double>(arm.lower(j), arm.upper(j))(rng);
    }
    if (!arm.evaluate(q, T)) {continue;}
    const Vec3 axis = T.linear().col(2);
    if (std::hypot(axis.x(), axis.y()) < 0.1 || !unionMap().keyOf(T).valid) {continue;}
    Pose base = Pose::Identity();
    base.translation() = Vec3(xy(rng), xy(rng), 0.0);
    base.linear() = Eigen::AngleAxisd(yaw(rng), Vec3::UnitZ()).toRotationMatrix();
    const Pose world = base * T;
    ++tested;
    for (const auto & c : unionMap().inverseQuery(world, 0.0)) {
      if (std::hypot(c.x - base.translation().x(), c.y - base.translation().y()) < 1.5 * p.resolution) {
        ++found;
        break;
      }
    }
  }
  EXPECT_GT(static_cast<double>(found) / tested, 0.9);
}

TEST(Redundancy, RestrictedJointShrinksTheMapAndHoldsForItsOwnSamples)
{
  const ChainModel arm = sevenDofArm();
  BuildOptions o;
  o.samples = 1'500'000;
  o.threads = 2;
  o.redundancy.joint = 2;
  o.redundancy.values = {0.0};
  const Map4D narrow = buildMap(arm, sevenParams(), o);
  EXPECT_LT(narrow.nonzeroCells(), unionMap().nonzeroCells());
  EXPECT_GT(narrow.nonzeroCells(), 0U);
  // Samples with joint 3 at its policy value are reachable in the restricted map; the union map
  // contains everything the restricted one does (same arm, more freedom), up to sampling noise.
  std::mt19937 rng(3);
  std::vector<double> q(7);
  Pose T;
  int in_window = 0, in_narrow = 0, narrow_not_in_union = 0;
  for (int i = 0; i < 20000; ++i) {
    for (int j = 0; j < 7; ++j) {
      q[j] = std::uniform_real_distribution<double>(arm.lower(j), arm.upper(j))(rng);
    }
    q[2] = 0.0;
    ASSERT_TRUE(arm.evaluate(q, T));
    if (!narrow.keyOf(T).valid) {continue;}
    ++in_window;
    in_narrow += narrow.isReachable(T) ? 1 : 0;
    narrow_not_in_union += (narrow.isReachable(T) && !unionMap().isReachable(T)) ? 1 : 0;
  }
  EXPECT_GT(static_cast<double>(in_narrow) / in_window, 0.9);
  EXPECT_LT(static_cast<double>(narrow_not_in_union) / in_window, 0.01);
}

TEST(Redundancy, PolicyDrawsOnlyConfiguredValuesWithinLimits)
{
  RedundancyPolicy pol;
  pol.joint = 2;
  pol.values = {-1.0, 0.5};
  pol.tolerance = 0.05;
  std::mt19937_64 rng(1);
  bool low = false, high = false;
  for (int i = 0; i < 500; ++i) {
    const double v = pol.draw(rng, -2.0, 2.0);
    EXPECT_TRUE((std::abs(v + 1.0) <= 0.05 + 1e-12) || (std::abs(v - 0.5) <= 0.05 + 1e-12));
    low |= v < 0;
    high |= v > 0;
  }
  EXPECT_TRUE(low && high);
  pol.values = {2.0};
  EXPECT_LE(pol.draw(rng, -2.0, 2.0), 2.0);   // clipped to the limit
}

TEST(Redundancy, InvalidPoliciesAreRejected)
{
  const ChainModel arm = sevenDofArm();
  BuildOptions o;
  o.samples = 100;
  o.redundancy.joint = 7;                       // there are only joints 0..6
  o.redundancy.values = {0.0};
  EXPECT_THROW(buildMap(arm, sevenParams(), o), std::invalid_argument);
  o.redundancy.joint = 2;
  o.redundancy.values = {};
  EXPECT_THROW(buildMap(arm, sevenParams(), o), std::invalid_argument);
  o.redundancy.values = {5.0};                  // beyond joint 3's limit
  EXPECT_THROW(buildMap(arm, sevenParams(), o), std::invalid_argument);
}

TEST(Redundancy, HeaderRecordsDofAndPolicyAndVersion1StillLoads)
{
  const ChainModel arm = sevenDofArm();
  RedundancyPolicy pol;
  pol.joint = 2;
  pol.values = {-0.5, 0.0, 0.5};
  pol.tolerance = 0.05;
  MapInfo info;
  info.arm_dof = 7;
  info.redundancy = describeRedundancy(7, pol);
  EXPECT_EQ(info.redundancy, "joint 3 in {-0.5, 0, 0.5} +- 0.05 rad");
  EXPECT_NE(describeRedundancy(7, RedundancyPolicy{}).find("union"), std::string::npos);
  EXPECT_EQ(describeRedundancy(6, RedundancyPolicy{}), "none");

  Map4D m(sevenParams());
  const std::string path = tmp("seven.rm4d");
  saveMap(path, m, arm.fingerprint(), info);
  MapInfo back;
  EXPECT_NO_THROW(loadMap(path, arm.fingerprint(), &back));
  EXPECT_EQ(back.arm_dof, 7U);
  EXPECT_EQ(back.redundancy, info.redundancy);
  fs::remove(path);

  // A version 1 file (no MapInfo block) is still read; its info says "not recorded".
  const std::string v1 = tmp("v1.rm4d");
  {
    std::ofstream o(v1, std::ios::binary);
    const char magic[8] = {'R', 'M', '4', 'D', 'M', 'A', 'P', '\0'};
    o.write(magic, 8);
    const uint32_t version = 1, fp_len = 0;
    o.write(reinterpret_cast<const char *>(&version), 4);
    const MapParams p = sevenParams();
    for (double d : {p.xy_extent, p.z_min, p.z_max, p.resolution, p.theta_bin}) {
      o.write(reinterpret_cast<const char *>(&d), sizeof(d));
    }
    o.write(reinterpret_cast<const char *>(&fp_len), 4);
    const uint64_t n = m.raw().size();
    o.write(reinterpret_cast<const char *>(&n), 8);
    o.write(reinterpret_cast<const char *>(m.raw().data()), static_cast<std::streamsize>(n));
  }
  MapInfo old;
  EXPECT_NO_THROW(loadMap(v1, "", &old));
  EXPECT_EQ(old.arm_dof, 0U);
  EXPECT_TRUE(old.redundancy.empty());
  fs::remove(v1);
}
