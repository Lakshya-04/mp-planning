#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

#include <unistd.h>

#include "reachability/core/base_model.hpp"

using namespace reachability;  // NOLINT

namespace
{
constexpr double kPi = 3.14159265358979323846;

BaseModel box(double length, double width, bool strafe)
{
  BaseModel b;
  b.drive = strafe ? DriveType::kOmni : DriveType::kDiff;
  b.can_strafe = strafe;
  b.footprint = {{-length / 2, -width / 2}, {length / 2, -width / 2}, {length / 2, width / 2},
    {-length / 2, width / 2}};
  b.max_linear = 1.0;
  b.max_angular = 1.0;
  return b;
}
}  // namespace

TEST(BaseModel, ClearanceUsesTheWholeFootprint)
{
  const BaseModel b = box(0.8, 0.4, true);
  // wall at x = 2 facing -x; arm base 1 m from it. Facing the wall the 0.8 m side leads: nose 0.4 m
  // ahead of the arm base. Turned sideways the 0.4 m side leads: 0.2 m ahead.
  const Eigen::Vector2d wall(2.0, 0.0), n(-1.0, 0.0);
  EXPECT_NEAR(wallClearance(b, {{1.0, 0.0}, 0.0}, wall, n), 0.6, 1e-9);
  EXPECT_NEAR(wallClearance(b, {{1.0, 0.0}, kPi / 2}, wall, n), 0.8, 1e-9);
}

TEST(BaseModel, BestYawTurnsTheShortSideToTheWall)
{
  const BaseModel b = box(0.8, 0.4, true);
  double c = 0.0;
  const double yaw = bestYaw(b, {1.0, 0.0}, {2.0, 0.0}, {-1.0, 0.0}, &c);
  EXPECT_NEAR(c, 0.8, 1e-9);
  EXPECT_NEAR(std::abs(std::sin(yaw)), 1.0, 1e-9);   // sideways to the wall
}

TEST(BaseModel, ArmMountOffsetMovesTheBodyNotTheArm)
{
  BaseModel b = box(0.8, 0.4, true);
  b.arm_mount = {0.2, 0.0};          // the arm sits 0.2 m ahead of the base centre
  const auto c = baseCentre(b, {{1.0, 0.0}, 0.0});
  EXPECT_NEAR(c.x(), 0.8, 1e-9);
  // facing the wall: nose is 0.4 - 0.2 = 0.2 m ahead of the arm base
  EXPECT_NEAR(wallClearance(b, {{1.0, 0.0}, 0.0}, {2.0, 0.0}, {-1.0, 0.0}), 0.8, 1e-9);
}

TEST(BaseModel, StrafingBeatsTurnDriveTurnAlongTheWall)
{
  const BaseModel omni = box(0.8, 0.5, true), diff = box(0.8, 0.5, false);
  const Stand a{{0.0, 0.0}, 0.0}, b{{0.0, 1.0}, 0.0};           // 1 m along the wall, same yaw
  EXPECT_NEAR(travelTime(omni, a, b), 1.0, 1e-9);
  EXPECT_NEAR(travelTime(diff, a, b), 1.0 + kPi, 1e-9);        // turn 90, drive, turn back
  EXPECT_GT(travelTime(diff, a, b), travelTime(omni, a, b));
}

TEST(BaseModel, FourWheelSteerCannotTranslateAndSpinAtOnce)
{
  BaseModel ws = box(0.8, 0.5, true);
  ws.drive = DriveType::kFourWheelSteer;
  const Stand a{{0.0, 0.0}, 0.0}, b{{0.0, 1.0}, kPi / 2};     // 1 m sideways and a quarter turn
  EXPECT_NEAR(travelTime(ws, a, b), 1.0 + kPi / 2, 1e-9);
  BaseModel omni = ws;
  omni.drive = DriveType::kOmni;
  EXPECT_NEAR(travelTime(omni, a, b), kPi / 2, 1e-9);        // the spin hides the 1 m slide
}

TEST(BaseModel, ADifferentialBaseReversesInsteadOfTurningAround)
{
  const BaseModel diff = box(0.8, 0.5, false);
  const Stand a{{0.0, 0.0}, 0.0}, b{{-1.0, 0.0}, 0.0};         // straight behind
  EXPECT_NEAR(travelTime(diff, a, b), 1.0, 1e-9);              // reversed, no turning
}

TEST(BaseModel, OrderStandsVisitsNeighboursFirst)
{
  const BaseModel omni = box(0.8, 0.5, true);
  const std::vector<Stand> stands = {{{3.0, 0.0}, 0.0}, {{1.0, 0.0}, 0.0}, {{2.0, 0.0}, 0.0}};
  const auto order = orderStands(omni, stands, {{0.0, 0.0}, 0.0});
  ASSERT_EQ(order.size(), 3U);
  EXPECT_EQ(order[0], 1U);
  EXPECT_EQ(order[1], 2U);
  EXPECT_EQ(order[2], 0U);
}

TEST(BaseModel, ArmBaseHeightFollowsTheProfilesLiftMount)
{
  const auto dir = std::filesystem::temp_directory_path() / ("base_model_test_" + std::to_string(::getpid()));
  std::filesystem::create_directories(dir);
  const auto path = (dir / "mount.yaml").string();
  std::ofstream(path) << "base:\n  drive_type: skid\n  lift_mount_height_m: 0.39\n"
                         "  footprint: [[-0.4, -0.2], [0.4, -0.2], [0.4, 0.2], [-0.4, 0.2]]\n";
  const BaseModel m = loadBaseProfile(path);
  ASSERT_TRUE(armBaseHeightAtLiftZero(m));
  EXPECT_NEAR(*armBaseHeightAtLiftZero(m), 0.39 + 0.8 + 0.02, 1e-12);
}

TEST(BaseModel, CanStrafeDefaultsToTheDriveTypeAndRejectsContradictions)
{
  const auto dir = std::filesystem::temp_directory_path() / ("base_model_test_" + std::to_string(::getpid()));
  std::filesystem::create_directories(dir);
  const std::string fp = "  footprint: [[-0.4, -0.2], [0.4, -0.2], [0.4, 0.2], [-0.4, 0.2]]\n";
  auto load = [&](const std::string & drive, const std::string & strafe) {
      const auto path = (dir / "strafe.yaml").string();
      std::ofstream(path) << "base:\n  drive_type: " << drive << "\n" << strafe << fp;
      return loadBaseProfile(path);
    };
  EXPECT_TRUE(load("4ws", "").can_strafe);
  EXPECT_TRUE(load("omni", "").can_strafe);
  EXPECT_FALSE(load("diff", "").can_strafe);
  EXPECT_FALSE(load("skid", "").can_strafe);
  EXPECT_THROW(load("diff", "  can_strafe: true\n"), std::runtime_error);
  EXPECT_THROW(load("skid", "  can_strafe: true\n"), std::runtime_error);
}

TEST(BaseModel, LoadsAProfileAndRejectsBadOnes)
{
  const auto dir = std::filesystem::temp_directory_path() / ("base_model_test_" + std::to_string(::getpid()));
  std::filesystem::create_directories(dir);
  const auto good = (dir / "good.yaml").string();
  std::ofstream(good) << "base:\n  name: t\n  drive_type: diff\n  can_strafe: false\n"
                         "  footprint: [[-0.4, -0.2], [0.4, -0.2], [0.4, 0.2], [-0.4, 0.2]]\n"
                         "  arm_mount_xy: [0.1, 0.0]\n  max_linear_velocity_mps: 0.8\n"
                         "  max_angular_velocity_radps: 1.2\n";
  const BaseModel m = loadBaseProfile(good);
  EXPECT_EQ(m.drive, DriveType::kDiff);
  EXPECT_FALSE(m.can_strafe);
  EXPECT_EQ(m.footprint.size(), 4U);
  EXPECT_NEAR(m.arm_mount.x(), 0.1, 1e-12);
  EXPECT_NEAR(m.max_linear, 0.8, 1e-12);
  EXPECT_FALSE(armBaseHeightAtLiftZero(m));   // no lift_mount_height_m in this profile

  const auto bad = (dir / "bad.yaml").string();
  std::ofstream(bad) << "base:\n  drive_type: hover\n  footprint: [[0,0],[1,0],[1,1]]\n";
  EXPECT_THROW(loadBaseProfile(bad), std::runtime_error);
  const auto none = (dir / "none.yaml").string();
  std::ofstream(none) << "robot:\n  name: x\n";
  EXPECT_THROW(loadBaseProfile(none), std::runtime_error);
}

#ifdef PROFILES_DIR
TEST(BaseModel, EveryShippedBaseProfileLoads)
{
  size_t n = 0;
  for (const auto & f : std::filesystem::directory_iterator(PROFILES_DIR)) {
    if (f.path().extension() != ".yaml") {continue;}
    const BaseModel m = loadBaseProfile(f.path().string());
    EXPECT_GE(m.footprint.size(), 3U) << f.path();
    EXPECT_GT(m.max_linear, 0.0) << f.path();
    // a strafing base is omni or 4-wheel steering; skid and diff bases cannot strafe
    if (m.drive == DriveType::kDiff || m.drive == DriveType::kSkid) {
      EXPECT_FALSE(m.can_strafe) << f.path();
    }
    ++n;
  }
  EXPECT_GE(n, 8U);
}
#else
TEST(BaseModel, EveryShippedBaseProfileLoads)
{
  GTEST_SKIP() << "mobile_manipulator_description/config/profiles not found next to this package";
}
#endif
