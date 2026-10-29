/**
 * @file point_cloud_file_loader.hpp
 * @brief Loads a static .pcd/.ply point-cloud file into a WorldGeometryUpdate.
 * @details Stands in for a live perception feed until one is merged --
 *          mirrors the established pcl::io::loadPCDFile/loadPLYFile pattern
 *          already used in wall_detection/src/wall_reprojection.cpp.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <string>

#include "motion_planner/scene_port.hpp"

namespace motion_planner
{

/**
 * @brief Load a .pcd or .ply point-cloud file (sniffed by extension) into a
 *        WorldGeometryUpdate.
 * @param[in] path Absolute path to the .pcd/.ply file.
 * @param[in] frame_id Reference frame to stamp the resulting update with.
 * @throws std::runtime_error on an unsupported extension, missing file, or
 *         a PCL load failure -- callers should not need to guess why an
 *         empty update resulted.
 */
WorldGeometryUpdate loadPointCloudFile(const std::string & path, const std::string & frame_id);

}  // namespace motion_planner
