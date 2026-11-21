// Where the base footprint may stand: the floor-plane obstacles and the walls other than the work plane.
//
// The stand search tests the base against the work plane only. A request can add this so a stand is
// also rejected when its (inflated) footprint overlaps an adjacent wall, a column or any other
// obstacle. Everything is 2D in the floor plane; heights are not modelled.
#pragma once

#include <vector>

#include <Eigen/Dense>

#include "reachability/core/base_model.hpp"

namespace reachability
{

/// An infinite wall in the floor plane: the free side is where `normal` points.
struct FreeSpaceBoundary
{
  Eigen::Vector2d point{0.0, 0.0};
  Eigen::Vector2d normal{1.0, 0.0};
};

struct FreeSpace
{
  /// Walls other than the work plane (e.g. the perpendicular wall at an inside corner).
  std::vector<FreeSpaceBoundary> walls;
  /// Obstacles as polygons (any winding). A finite wall or a wall end is a thin polygon.
  std::vector<std::vector<Eigen::Vector2d>> obstacles;
  /// Optional finite extent of the work wall along the plane frame's u axis, measured from the
  /// request's wall_point: [lo, hi]. When set, the work-plane clearance only applies to footprint
  /// points inside the span, so a stand beyond a wall end or an outside corner is allowed. Put the
  /// wall end itself in `obstacles` (a thin polygon) to keep the footprint off it.
  bool has_work_span{false};
  double work_span_lo{0.0};
  double work_span_hi{0.0};

  bool empty() const {return walls.empty() && obstacles.empty() && !has_work_span;}

  /// Axis-aligned box as an obstacle polygon.
  static std::vector<Eigen::Vector2d> box(double x0, double y0, double x1, double y1);

  /// True when the polygon (1 vertex = a point, 2 = a segment) keeps at least `margin` from every wall and obstacle.
  bool footprintFree(const std::vector<Eigen::Vector2d> & footprint_world, double margin) const;
  /// Same for the base footprint at a stand (the arm-base point alone when `base` has no footprint).
  bool footprintFree(const Stand & pose, const BaseModel & base, double margin) const;
  /// A bare point (arm base, no base model).
  bool pointFree(const Eigen::Vector2d & p, double margin) const;
};

/// Smallest distance between two vertex lists (point, segment or polygon); 0 when they overlap.
double polygonDistance(const std::vector<Eigen::Vector2d> & a, const std::vector<Eigen::Vector2d> & b);

}  // namespace reachability
