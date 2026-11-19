#include <gtest/gtest.h>

#include <cmath>

#include "reachability_moveit/port_stand_verifier.hpp"

using namespace reachability_moveit;  // NOLINT
using namespace motion_planner;       // NOLINT
using reachability::Pose;
using reachability::Stand;
using reachability::VerifyReason;

namespace
{
struct FakeIk : IkPort
{
  std::function<IkResult(const IkRequest &)> fn;
  std::vector<IkRequest> seen;
  IkResult ik(const IkRequest & r) override {seen.push_back(r); return fn(r);}
};
struct FakeCollision : CollisionPort
{
  bool free{true};
  bool ran{true};
  std::vector<CollisionRequest> seen;
  CollisionResult collision(const CollisionRequest & r) override
  {
    seen.push_back(r);
    CollisionResult c;
    c.success = ran;
    c.collision_free = free;
    return c;
  }
};
struct FakeScene : ScenePort
{
  std::vector<WorldGeometryUpdate> pushed;
  bool ok{true};
  bool updateWorldGeometry(const WorldGeometryUpdate & u, std::string & e) override
  {
    if (!ok) {e = "no scene"; return false;}
    pushed.push_back(u);
    return true;
  }
  bool clearWorldGeometry(std::string &) override {return true;}
};

IkResult solved(std::vector<double> q)
{
  IkResult r;
  r.success = true;
  r.joint_solution = std::move(q);
  return r;
}
IkResult failed(const std::string & code = "ik_failed")
{
  IkResult r;
  r.failure_reason_code = code;
  return r;
}

PortVerifierConfig config()
{
  PortVerifierConfig c;
  c.arm_base_in_plan.translation() = Eigen::Vector3d(0.1, 0.0, 1.17);   // arm mount 0.1 m ahead, 1.17 m up
  c.joint_lower = std::vector<double>(6, -3.2);
  c.joint_upper = std::vector<double>(6, 3.2);
  return c;
}

Pose target(double x, double y, double z)
{
  Pose T = Pose::Identity();
  T.translation() = Eigen::Vector3d(x, y, z);
  return T;
}

struct Rig
{
  std::shared_ptr<FakeIk> ik = std::make_shared<FakeIk>();
  std::shared_ptr<FakeCollision> col = std::make_shared<FakeCollision>();
  std::shared_ptr<FakeScene> scene = std::make_shared<FakeScene>();
  Rig() {ik->fn = [](const IkRequest &) {return solved({0, 0.5, 1, 1.5, 2, 2.5});};}
};
}  // namespace

TEST(PortStandVerifier, TheTargetIsExpressedForTheArmGroupWithTheLiftAtZero)
{
  // base yaw 90 deg, arm base at (2, 3) world; a target 1 m in front of the arm (world +y), at the arm's height
  const auto c = config();
  const Stand s{Eigen::Vector2d(2.0, 3.0), M_PI / 2};
  const auto t = PortStandVerifier::ikTarget(c, s, 0.4, target(2.0, 4.0, 1.17 + 0.4));
  ASSERT_EQ(t.size(), 7U);
  // in the arm base frame the point is 1 m ahead; the IK frame keeps the arm base at (0.1, 0, 1.17)
  EXPECT_NEAR(t[0], 0.1 + 1.0, 1e-9);
  EXPECT_NEAR(t[1], 0.0, 1e-9);
  EXPECT_NEAR(t[2], 1.17, 1e-9);   // lift 0.4 removed
  // the orientation turned with the base: world identity seen from a base yawed +90 deg is yaw -90 deg
  const Eigen::Quaterniond q(t[6], t[3], t[4], t[5]);
  EXPECT_NEAR(std::abs(q.toRotationMatrix()(0, 1)), 1.0, 1e-9);
}

TEST(PortStandVerifier, OkNeedsIkAndACollisionFreeSolutionAndSendsTheLiftToTheCollisionCheck)
{
  Rig r;
  PortStandVerifier v(config(), r.ik, r.col, r.scene);
  const auto res = v.verify(Stand{}, 0.5, {target(1.0, 0.0, 1.5)});
  ASSERT_EQ(res.size(), 1U);
  EXPECT_EQ(res[0].reason, VerifyReason::kOk);
  EXPECT_EQ(res[0].joints.size(), 6U);
  ASSERT_EQ(r.ik->seen.size(), 1U);
  EXPECT_EQ(r.ik->seen[0].planning_group, "arm");
  EXPECT_EQ(r.ik->seen[0].end_effector_link, "tcp_link");
  ASSERT_EQ(r.col->seen.size(), 1U);
  EXPECT_EQ(r.col->seen[0].planning_group, "lift_arm");
  ASSERT_EQ(r.col->seen[0].joint_values.size(), 7U);
  EXPECT_DOUBLE_EQ(r.col->seen[0].joint_values[0], 0.5);
}

TEST(PortStandVerifier, FailuresAreTyped)
{
  Rig r;
  PortStandVerifier v(config(), r.ik, r.col, r.scene);
  r.ik->fn = [](const IkRequest &) {return failed();};
  EXPECT_EQ(v.verify(Stand{}, 0.5, {target(1, 0, 1.5)})[0].reason, VerifyReason::kIkFailed);
  r.ik->fn = [](const IkRequest &) {return failed("invalid_target_pose");};
  EXPECT_EQ(v.verify(Stand{}, 0.5, {target(1, 0, 1.5)})[0].reason, VerifyReason::kError);
  r.ik->fn = [](const IkRequest &) {return solved({0, 0, 0, 0, 0, 9.0});};   // beyond +-3.2
  EXPECT_EQ(v.verify(Stand{}, 0.5, {target(1, 0, 1.5)})[0].reason, VerifyReason::kOutOfLimits);
  r.ik->fn = [](const IkRequest &) {return solved({0, 0, 0, 0, 0, 0});};
  r.col->free = false;
  EXPECT_EQ(v.verify(Stand{}, 0.5, {target(1, 0, 1.5)})[0].reason, VerifyReason::kInCollision);
  r.col->free = true;
  r.col->ran = false;
  EXPECT_EQ(v.verify(Stand{}, 0.5, {target(1, 0, 1.5)})[0].reason, VerifyReason::kError);
  r.col->ran = true;
  // lift outside its travel: no backend call at all
  const size_t before = r.ik->seen.size();
  EXPECT_EQ(v.verify(Stand{}, 2.0, {target(1, 0, 1.5)})[0].reason, VerifyReason::kOutOfLimits);
  EXPECT_EQ(r.ik->seen.size(), before);
}

TEST(PortStandVerifier, AnotherSeedCanFindTheCollisionFreeSolution)
{
  Rig r;
  auto c = config();
  c.seeds = {{0, 0, 0, 0, 0, 0}, {1, 1, 1, 1, 1, 1}};
  r.ik->fn = [](const IkRequest & q) {return solved({q.joint_seed[0], 0, 0, 0, 0, 0});};
  struct Picky : CollisionPort
  {
    CollisionResult collision(const CollisionRequest & q) override
    {
      CollisionResult c;
      c.success = true;
      c.collision_free = q.joint_values[1] > 0.5;   // the second seed's solution only
      return c;
    }
  };
  PortStandVerifier v(c, r.ik, std::make_shared<Picky>(), r.scene);
  const auto res = v.verify(Stand{}, 0.5, {target(1, 0, 1.5)});
  EXPECT_EQ(res[0].reason, VerifyReason::kOk);
  EXPECT_EQ(v.ikCalls(), 2U);
}

TEST(PortStandVerifier, TheSceneIsRePosedPerStandAndPushedOnlyWhenTheStandChanges)
{
  Rig r;
  auto c = config();
  WorldGeometryBox b;
  b.id = "box";
  b.center_x = 3.0;   // 1 m ahead of a stand at the origin... the arm mount is 0.1 m ahead of the base
  b.center_y = 0.0;
  b.center_z = 1.0;
  c.boxes.push_back(b);
  PortStandVerifier v(c, r.ik, r.col, r.scene);
  const Stand s0{Eigen::Vector2d(0.1, 0.0), 0.0};   // base_footprint at the world origin
  v.verify(s0, 0.3, {target(1, 0, 1.5)});
  v.verify(s0, 0.6, {target(1, 0, 1.5)});
  EXPECT_EQ(v.scenePushes(), 1U);
  ASSERT_EQ(r.scene->pushed.size(), 1U);
  EXPECT_NEAR(r.scene->pushed[0].boxes[0].center_x, 3.0, 1e-9);
  const Stand s1{Eigen::Vector2d(0.1, 1.0), M_PI / 2};   // yawed 90 degrees, so the box moves behind the base
  v.verify(s1, 0.3, {target(1, 0, 1.5)});
  EXPECT_EQ(v.scenePushes(), 2U);
  const auto & moved = r.scene->pushed[1].boxes[0];
  // base origin = arm - R(yaw) * (0.1, 0) = (0.1, 0.9); world (3, 0) -> base frame (R^T (p - o)) = (-0.9, -2.9)
  EXPECT_NEAR(moved.center_x, -0.9, 1e-9);
  EXPECT_NEAR(moved.center_y, -2.9, 1e-9);
  EXPECT_NEAR(moved.center_z, 1.0, 1e-9);
}

TEST(PortStandVerifier, WithoutASceneAnswerOrWithAFailingOneItIsAnErrorNeverOk)
{
  Rig r;
  r.scene->ok = false;
  PortStandVerifier v(config(), r.ik, r.col, r.scene);
  EXPECT_EQ(v.verify(Stand{}, 0.5, {target(1, 0, 1.5)})[0].reason, VerifyReason::kError);
  PortStandVerifier none(config(), nullptr, nullptr);
  EXPECT_EQ(none.verify(Stand{}, 0.5, {target(1, 0, 1.5)})[0].reason, VerifyReason::kError);
  EXPECT_TRUE(v.verify(Stand{}, 0.5, {}).empty());
}
