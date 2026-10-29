#include "motion_planner/core/point_cloud_file_loader.hpp"

#include <algorithm>
#include <stdexcept>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

namespace motion_planner
{

namespace
{

std::string toLowerCopy(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

bool hasSuffix(const std::string & value, const std::string & suffix)
{
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

}  // namespace

WorldGeometryUpdate loadPointCloudFile(const std::string & path, const std::string & frame_id)
{
  const std::string lower_path = toLowerCopy(path);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  int load_result = -1;
  if (hasSuffix(lower_path, ".pcd")) {
    load_result = pcl::io::loadPCDFile<pcl::PointXYZ>(path, cloud);
  } else if (hasSuffix(lower_path, ".ply")) {
    load_result = pcl::io::loadPLYFile<pcl::PointXYZ>(path, cloud);
  } else {
    throw std::runtime_error(
            "loadPointCloudFile: unsupported file extension (expected .pcd or .ply): " + path);
  }

  if (load_result < 0) {
    throw std::runtime_error("loadPointCloudFile: failed to load point cloud from " + path);
  }

  WorldGeometryUpdate update;
  update.frame_id = frame_id;
  update.points.reserve(cloud.size());
  for (const auto & point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    WorldGeometryPoint world_point;
    world_point.x = point.x;
    world_point.y = point.y;
    world_point.z = point.z;
    update.points.push_back(world_point);
  }

  return update;
}

}  // namespace motion_planner
