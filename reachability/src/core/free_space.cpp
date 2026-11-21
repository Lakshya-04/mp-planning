#include "reachability/core/free_space.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace reachability
{
namespace
{
using V2 = Eigen::Vector2d;

double cross(const V2 & a, const V2 & b) {return a.x() * b.y() - a.y() * b.x();}

double pointSegment(const V2 & p, const V2 & a, const V2 & b)
{
  const V2 ab = b - a;
  const double len2 = ab.squaredNorm();
  const double t = len2 < 1e-18 ? 0.0 : std::clamp((p - a).dot(ab) / len2, 0.0, 1.0);
  return (p - (a + t * ab)).norm();
}

bool segmentsCross(const V2 & a, const V2 & b, const V2 & c, const V2 & d)
{
  const double d1 = cross(b - a, c - a), d2 = cross(b - a, d - a);
  const double d3 = cross(d - c, a - c), d4 = cross(d - c, b - c);
  return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0)) && d1 != 0.0 && d2 != 0.0 && d3 != 0.0 && d4 != 0.0;
}

double segmentSegment(const V2 & a, const V2 & b, const V2 & c, const V2 & d)
{
  if (segmentsCross(a, b, c, d)) {return 0.0;}
  return std::min(
    std::min(pointSegment(a, c, d), pointSegment(b, c, d)),
    std::min(pointSegment(c, a, b), pointSegment(d, a, b)));
}

bool insidePolygon(const V2 & p, const std::vector<V2> & poly)
{
  if (poly.size() < 3) {return false;}
  bool in = false;
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    if (((poly[i].y() > p.y()) != (poly[j].y() > p.y())) &&
      p.x() < (poly[j].x() - poly[i].x()) * (p.y() - poly[i].y()) / (poly[j].y() - poly[i].y()) + poly[i].x())
    {
      in = !in;
    }
  }
  return in;
}
}  // namespace

double polygonDistance(const std::vector<V2> & a, const std::vector<V2> & b)
{
  if (a.empty() || b.empty()) {return std::numeric_limits<double>::infinity();}
  for (const auto & p : a) {if (insidePolygon(p, b)) {return 0.0;}}
  for (const auto & p : b) {if (insidePolygon(p, a)) {return 0.0;}}
  auto edges = [](const std::vector<V2> & p) {
      std::vector<std::pair<V2, V2>> e;
      if (p.size() == 1) {e.emplace_back(p[0], p[0]);}
      else if (p.size() == 2) {e.emplace_back(p[0], p[1]);}
      else {for (size_t i = 0; i < p.size(); ++i) {e.emplace_back(p[i], p[(i + 1) % p.size()]);}}
      return e;
    };
  double best = std::numeric_limits<double>::infinity();
  for (const auto & ea : edges(a)) {
    for (const auto & eb : edges(b)) {
      best = std::min(best, segmentSegment(ea.first, ea.second, eb.first, eb.second));
      if (best == 0.0) {return 0.0;}
    }
  }
  return best;
}

std::vector<Eigen::Vector2d> FreeSpace::box(double x0, double y0, double x1, double y1)
{
  return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
}

bool FreeSpace::footprintFree(const std::vector<Eigen::Vector2d> & fp, double margin) const
{
  for (const auto & w : walls) {
    const V2 n = w.normal.norm() < 1e-12 ? V2(0.0, 0.0) : V2(w.normal.normalized());
    for (const auto & p : fp) {
      if ((p - w.point).dot(n) < margin - 1e-9) {return false;}
    }
  }
  for (const auto & o : obstacles) {
    if (polygonDistance(fp, o) < margin - 1e-9) {return false;}
    if (margin <= 0.0 && polygonDistance(fp, o) <= 0.0) {return false;}
  }
  return true;
}

bool FreeSpace::footprintFree(const Stand & pose, const BaseModel & base, double margin) const
{
  if (base.footprint.empty()) {return pointFree(pose.arm_xy, margin);}
  return footprintFree(footprintWorld(base, pose), margin);
}

bool FreeSpace::pointFree(const Eigen::Vector2d & p, double margin) const
{
  return footprintFree(std::vector<V2>{p}, margin);
}

}  // namespace reachability
