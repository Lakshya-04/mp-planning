/**
 * @file scene_port.hpp
 * @brief Backend-agnostic port for feeding world/obstacle collision geometry
 *        to whichever planner backend is active.
 * @details Two distinct kinds of geometry travel through this port, and callers
 *          that need contact-aware collision handling (CartesianPlanner's
 *          setWallContactAcmEntry(), see cartesian_planner.cpp) MUST keep them
 *          distinct rather than merging everything into one blob:
 *            - `points` (dense, e.g. a wall surface scan) become ONE octomap --
 *              the only thing a contact-mode ACM exclusion is ever scoped
 *              against, via kWallOctomapCollisionObjectId below.
 *            - `boxes` (discrete, named obstacles) become separate named
 *              CollisionObjects, each independently ACM-safe -- a contact
 *              exclusion for the wall octomap must never also silently
 *              exclude these.
 *          Which planner backend is active (MoveIt/OMPL today, cuRobo later --
 *          still an open evaluation, §6.4/§12) determines how that geometry
 *          actually needs to be consumed. This port is the seam: pure C++, no
 *          MoveIt/ROS types, so a future cuRobo adapter can implement it
 *          without depending on MoveIt at all -- mirrors the
 *          Planner/ExecutionPort/Logger port pattern already established in
 *          this codebase.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <string>
#include <vector>

#include "core_types/pose.hpp"

namespace motion_planner
{

/**
 * @brief The collision object ID MoveIt itself assigns to octomap-based world geometry
 *        (`planning_scene::PlanningScene::OCTOMAP_NS`, verified against the installed
 *        libmoveit_planning_scene.so). A contact segment's `target_collision_object_id`
 *        must be exactly this string for a wall-contact ACM exclusion to actually match
 *        the octomap `updateWorldGeometry()` builds from `WorldGeometryUpdate::points` --
 *        anything else silently adds an unused ACM entry. Never point this at a `boxes`
 *        obstacle's id; those must stay real hazards regardless of any contact segment.
 */
inline constexpr char kWallOctomapCollisionObjectId[] = "<octomap>";

/**
 * @struct WorldGeometryPoint
 * @brief One point of dense collision geometry (e.g. from a point cloud).
 */
struct WorldGeometryPoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/**
 * @struct WorldGeometryBox
 * @brief One discrete, named obstacle -- e.g. a coverage-planner-detected obstacle region's
 *        3D bounding box. Kept as a distinct primitive (not rasterized into `points`'
 *        octomap) specifically so it can never be excluded by a wall-contact ACM entry
 *        (see this file's own doc comment).
 */
struct WorldGeometryBox
{
  std::string id;                ///< Unique per update; becomes the CollisionObject's id.
  double center_x{0.0};
  double center_y{0.0};
  double center_z{0.0};
  double orientation_x{0.0};     ///< Box-local axes -> frame_id rotation, as a quaternion.
  double orientation_y{0.0};
  double orientation_z{0.0};
  double orientation_w{1.0};
  double size_x{0.0};            ///< Full (not half) extent along the box's own local x axis.
  double size_y{0.0};
  double size_z{0.0};
};

/**
 * @struct WorldGeometryUpdate
 * @brief A full replacement of the backend's current world/obstacle geometry.
 */
struct WorldGeometryUpdate
{
  std::string frame_id;                      ///< Reference frame points/boxes are expressed in.
  std::vector<WorldGeometryPoint> points;     ///< Dense collision geometry (e.g. a wall scan) --
                                               ///< becomes one octomap, see kWallOctomapCollisionObjectId.
  std::vector<WorldGeometryBox> boxes;        ///< Discrete named obstacles -- never ACM-excludable.
  /// Pose (in frame_id) that `points`' own coordinates are expressed relative to. Identity
  /// (default) means points are already directly in frame_id's own raw axes -- unchanged
  /// behavior for every existing caller. A caller that has ALREADY fit a dominant plane for
  /// this geometry (e.g. a WallPlane's own basis_u/basis_v/normal_outward -- the exact same
  /// per-wall fit the perception pipeline already computes, not something new/hardcoded per
  /// wall) can instead express `points` in that plane's own LOCAL (u, v, normal) axes and set
  /// this to the plane's real pose: the resulting octree's own voxel grid then aligns to the
  /// fitted plane (eliminating the raw-capture-frame-misalignment source of voxel staircasing
  /// for the dominant planar component) while ANY genuine depth variation off that plane
  /// (curvature, protrusions, noise) still shows up in full, unmodified 3D fidelity as nonzero
  /// local-normal offsets -- no flatness is assumed or enforced, only the voxel grid's own
  /// reference axes change. Never affects `boxes`, which are always directly in frame_id (they
  /// already carry their own independent orientation and don't suffer this artifact).
  core_types::Pose points_origin;
};

/**
 * @class ScenePort
 * @brief Feeds world/obstacle collision geometry to the active planner
 *        backend's collision representation. Never plans, never executes --
 *        purely a world-state seam.
 */
class ScenePort
{
public:
  virtual ~ScenePort() = default;

  /**
   * @brief Replace (not merge with) the backend's current world geometry.
   * @param[in] update The full new world geometry.
   * @param[out] error_message Set on failure.
   * @return true on success.
   */
  virtual bool updateWorldGeometry(
    const WorldGeometryUpdate & update,
    std::string & error_message) = 0;

  /**
   * @brief Remove all world geometry previously injected via this port.
   * @param[out] error_message Set on failure.
   * @return true on success.
   */
  virtual bool clearWorldGeometry(std::string & error_message) = 0;
};

}  // namespace motion_planner
