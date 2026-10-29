#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "motion_planner/point_cloud_file_loader.hpp"

namespace motion_planner
{
namespace
{

std::string fixturePath(const std::string & filename)
{
  return std::string(MOTION_PLANNER_TEST_CONFIG_DIR) + "/" + filename;
}

TEST(PointCloudFileLoaderTest, LoadsPcdFixtureWithExpectedPointCountAndValues)
{
  const WorldGeometryUpdate update =
    loadPointCloudFile(fixturePath("obstacle_blocking.pcd"), "base_link");

  EXPECT_EQ(update.frame_id, "base_link");
  // 6x6x6 grid (0.075m half-extent, 0.03m step -> 6 samples per axis).
  ASSERT_EQ(update.points.size(), 216u);

  // First generated point: center (-0.15, -0.70, 1.40) -- benchmark_cases.yaml's
  // fake_wall_stroke_2 midpoint -- minus the 0.075m half-extent on each axis.
  EXPECT_NEAR(update.points.front().x, -0.225, 1e-3);
  EXPECT_NEAR(update.points.front().y, -0.775, 1e-3);
  EXPECT_NEAR(update.points.front().z, 1.325, 1e-3);
}

TEST(PointCloudFileLoaderTest, LoadsSecondFixtureIndependently)
{
  const WorldGeometryUpdate update =
    loadPointCloudFile(fixturePath("obstacle_clear.pcd"), "base_link");

  ASSERT_EQ(update.points.size(), 216u);
  // Generated centered on (3.0, 3.0, 3.0) -- nowhere near the blocking
  // fixture's geometry.
  EXPECT_NEAR(update.points.front().x, 2.92, 1e-3);
}

TEST(PointCloudFileLoaderTest, MissingFileThrows)
{
  EXPECT_THROW(
    loadPointCloudFile(fixturePath("does_not_exist.pcd"), "base_link"),
    std::runtime_error);
}

TEST(PointCloudFileLoaderTest, UnsupportedExtensionThrows)
{
  EXPECT_THROW(
    loadPointCloudFile(fixturePath("obstacle_blocking.txt"), "base_link"),
    std::runtime_error);
}

}  // namespace
}  // namespace motion_planner
