#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "motion_planner/core/planners/seed_preferring_ik.hpp"

namespace motion_planner
{
namespace
{

/// IkPort double driven by a callback.
class FakeIk : public IkPort
{
public:
  using Fn = std::function<IkResult(const IkRequest &)>;
  explicit FakeIk(Fn fn)
  : fn_(std::move(fn)) {}
  IkResult ik(const IkRequest & request) override
  {
    ++calls;
    return fn_(request);
  }
  int calls{0};

private:
  Fn fn_;
};

IkResult ok(std::vector<double> q)
{
  IkResult r;
  r.success = true;
  r.joint_solution = std::move(q);
  return r;
}

IkRequest withSeed(std::vector<double> seed)
{
  IkRequest r;
  r.target_pose = {0, 0, 0, 0, 0, 0};
  r.joint_seed = std::move(seed);
  return r;
}

constexpr double kTwoPi = 6.283185307179586;

TEST(SeedPreferringIk, NoSeedDelegatesOnce)
{
  auto inner = std::make_shared<FakeIk>([](const IkRequest &) {return ok({1.0, 2.0});});
  SeedPreferringIk ik(inner, {});
  auto r = ik.ik(withSeed({}));
  EXPECT_TRUE(r.success);
  EXPECT_EQ(inner->calls, 1);
}

TEST(SeedPreferringIk, WrapsFullTurnTowardSeed)
{
  auto inner = std::make_shared<FakeIk>(
    [](const IkRequest &) {return ok({0.5 + kTwoPi, -0.5 - kTwoPi});});
  SeedPreferringIkConfig cfg;
  cfg.extra_seeds = 1;
  SeedPreferringIk ik(inner, cfg);
  auto r = ik.ik(withSeed({0.4, -0.4}));
  ASSERT_TRUE(r.success);
  EXPECT_NEAR(r.joint_solution[0], 0.5, 1e-9);
  EXPECT_NEAR(r.joint_solution[1], -0.5, 1e-9);
}

TEST(SeedPreferringIk, PrismaticJointIsNotWrapped)
{
  auto inner = std::make_shared<FakeIk>([](const IkRequest &) {return ok({7.0, 0.5});});
  SeedPreferringIkConfig cfg;
  cfg.extra_seeds = 1;
  cfg.wrap_joint = {false, true};
  SeedPreferringIk ik(inner, cfg);
  auto r = ik.ik(withSeed({0.0, 0.5}));
  ASSERT_TRUE(r.success);
  EXPECT_DOUBLE_EQ(r.joint_solution[0], 7.0);
}

TEST(SeedPreferringIk, PicksClosestAcrossSeeds)
{
  // The first call (the caller's own seed) lands on a far branch, later calls on a near one.
  int n = 0;
  auto inner = std::make_shared<FakeIk>(
    [&n](const IkRequest &) {return ++n == 1 ? ok({2.5}) : ok({0.2});});
  SeedPreferringIkConfig cfg;
  cfg.extra_seeds = 3;
  SeedPreferringIk ik(inner, cfg);
  auto r = ik.ik(withSeed({0.0}));
  ASSERT_TRUE(r.success);
  EXPECT_NEAR(r.joint_solution[0], 0.2, 1e-9);
}

TEST(SeedPreferringIk, DropsSolutionsOutsideLimits)
{
  auto inner = std::make_shared<FakeIk>([](const IkRequest &) {return ok({3.0});});
  SeedPreferringIkConfig cfg;
  cfg.extra_seeds = 2;
  cfg.wrap_joint = {false};
  cfg.lower_limits = {-1.0};
  cfg.upper_limits = {1.0};
  SeedPreferringIk ik(inner, cfg);
  auto r = ik.ik(withSeed({0.0}));
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.failure_reason_code, "ik_outside_limits");
}

TEST(SeedPreferringIk, AcceptDistanceStopsEarly)
{
  auto inner = std::make_shared<FakeIk>([](const IkRequest &) {return ok({0.01});});
  SeedPreferringIkConfig cfg;
  cfg.extra_seeds = 10;
  cfg.accept_distance = 0.05;
  SeedPreferringIk ik(inner, cfg);
  auto r = ik.ik(withSeed({0.0}));
  EXPECT_TRUE(r.success);
  EXPECT_EQ(inner->calls, 1);
}

TEST(SeedPreferringIk, AllFailuresReportFailure)
{
  auto inner = std::make_shared<FakeIk>([](const IkRequest &) {
      IkResult r;
      r.failure_reason_code = "ik_failed";
      return r;
    });
  SeedPreferringIkConfig cfg;
  cfg.extra_seeds = 2;
  SeedPreferringIk ik(inner, cfg);
  auto r = ik.ik(withSeed({0.0}));
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.failure_reason_code, "ik_failed");
  EXPECT_EQ(inner->calls, 3);
}

TEST(SeedPreferringIk, NoiseSkipsNonTurnJointsAndStaysInLimits)
{
  std::vector<std::vector<double>> seeds;
  auto inner = std::make_shared<FakeIk>([&seeds](const IkRequest & r) {
      seeds.push_back(r.joint_seed);
      return ok(r.joint_seed);
    });
  SeedPreferringIkConfig cfg;
  cfg.extra_seeds = 20;
  cfg.perturbation = 1.0;
  cfg.wrap_joint = {false, true};
  cfg.lower_limits = {0.0, -0.1};
  cfg.upper_limits = {0.5, 0.1};
  SeedPreferringIk ik(inner, cfg);
  ik.ik(withSeed({0.25, 0.0}));
  ASSERT_EQ(seeds.size(), 21u);
  for (const auto & s : seeds) {
    EXPECT_DOUBLE_EQ(s[0], 0.25);          // lift untouched
    EXPECT_GE(s[1], -0.1 - 1e-12);         // turn joint clamped to its limits
    EXPECT_LE(s[1], 0.1 + 1e-12);
  }
}

TEST(SeedPreferringIk, WeightsDecideBetweenMetresAndRadians)
{
  // joints: lift (m), shoulder (rad). Branch A moves the lift 0.3 m, branch B the shoulder 0.35 rad.
  auto run = [](const std::vector<double> & weights) {
      int n = 0;
      auto inner = std::make_shared<FakeIk>([&n](const IkRequest &) {
          return ++n % 2 ? ok({0.3, 0.1}) : ok({0.0, 0.35});
        });
      SeedPreferringIkConfig config;
      config.extra_seeds = 1;
      config.weights = weights;
      return SeedPreferringIk(inner, config).ik(withSeed({0.0, 0.0})).joint_solution;
    };
  EXPECT_NEAR(run({})[0], 0.3, 1e-12);                 // unweighted: the slow lift is moved
  const auto weighted = run({36.0, 1.0});              // 1 m counts as 6 rad
  EXPECT_NEAR(weighted[0], 0.0, 1e-12);
  EXPECT_NEAR(weighted[1], 0.35, 1e-12);
}

TEST(SeedPreferringIk, FirstFailureReasonIsKept)
{
  int n = 0;
  auto inner = std::make_shared<FakeIk>([&n](const IkRequest &) {
      if (++n == 1) {
        IkResult r;
        r.failure_reason_code = "ik_in_collision";
        return r;
      }
      return ok({9.0});   // later attempts land outside the limits
    });
  SeedPreferringIkConfig cfg;
  cfg.extra_seeds = 3;
  cfg.wrap_joint = {false};
  cfg.lower_limits = {-1.0};
  cfg.upper_limits = {1.0};
  SeedPreferringIk ik(inner, cfg);
  auto r = ik.ik(withSeed({0.0}));
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.failure_reason_code, "ik_in_collision");
}

TEST(SeedPreferringIk, NegativePerturbationIsTreatedAsItsMagnitude)
{
  auto inner = std::make_shared<FakeIk>([](const IkRequest & r) {return ok(r.joint_seed);});
  SeedPreferringIkConfig cfg;
  cfg.extra_seeds = 2;
  cfg.perturbation = -0.1;
  SeedPreferringIk ik(inner, cfg);
  EXPECT_TRUE(ik.ik(withSeed({0.0})).success);
}

}  // namespace
}  // namespace motion_planner
