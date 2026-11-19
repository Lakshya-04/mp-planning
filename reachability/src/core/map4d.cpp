#include "reachability/core/map4d.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace reachability
{

Map4D::Map4D(const MapParams & params)
: params_(params)
{
  if (params.resolution <= 0.0 || params.theta_bin <= 0.0 || params.xy_extent <= 0.0 ||
    params.z_max <= params.z_min)
  {
    throw std::invalid_argument("Map4D: resolution, theta_bin, xy_extent and the z range must be positive");
  }
  nxy_ = std::max(1, static_cast<int>(std::ceil(2.0 * params.xy_extent / params.resolution)));
  nz_ = std::max(1, static_cast<int>(std::ceil((params.z_max - params.z_min) / params.resolution)));
  ntheta_ = std::max(1, static_cast<int>(std::ceil(M_PI / params.theta_bin)));
  cells_.assign(
    static_cast<size_t>(nz_) * static_cast<size_t>(ntheta_) * static_cast<size_t>(nxy_) *
    static_cast<size_t>(nxy_), 0);
}

size_t Map4D::index(int iz, int itheta, int ix, int iy) const
{
  return ((static_cast<size_t>(iz) * ntheta_ + itheta) * nxy_ + ix) * nxy_ + iy;
}

size_t Map4D::nonzeroCells() const
{
  return static_cast<size_t>(std::count_if(cells_.begin(), cells_.end(), [](uint8_t c) {return c != 0;}));
}

CellKey Map4D::keyOf(const Pose & T) const
{
  const Vec3 axis = T.linear().col(2);
  const Vec3 p = T.translation();
  const double s = std::hypot(axis.x(), axis.y());
  const double theta = std::atan2(s, axis.z());  // 0 .. pi

  // Azimuth that is rotated to +x. A (near) vertical axis has none: use the position's own.
  double phi;
  if (s > 1e-3) {
    phi = std::atan2(axis.y(), axis.x());
  } else {
    phi = std::hypot(p.x(), p.y()) > 1e-9 ? std::atan2(p.y(), p.x()) : 0.0;
  }
  const double c = std::cos(phi), sn = std::sin(phi);
  const double xs = c * p.x() + sn * p.y();
  // For a vertical axis phi is the position's own azimuth, so y* is exactly 0; computing it as
  // -sn*px + c*py leaves ~1e-16 of noise that picks the cell on either side of y* = 0.
  const double ys = s > 1e-3 ? -sn * p.x() + c * p.y() : 0.0;

  CellKey key;
  key.ix = static_cast<int>(std::floor((xs + params_.xy_extent) / params_.resolution));
  key.iy = static_cast<int>(std::floor((ys + params_.xy_extent) / params_.resolution));
  key.iz = static_cast<int>(std::floor((p.z() - params_.z_min) / params_.resolution));
  key.itheta = std::min(ntheta_ - 1, static_cast<int>(std::floor(theta / params_.theta_bin)));
  key.valid = key.ix >= 0 && key.ix < nxy_ && key.iy >= 0 && key.iy < nxy_ && key.iz >= 0 &&
    key.iz < nz_ && key.itheta >= 0;
  return key;
}

void Map4D::recordKey(const CellKey & key)
{
  if (!key.valid) {
    return;
  }
  uint8_t & c = cells_[index(key.iz, key.itheta, key.ix, key.iy)];
  if (c < 255) {
    ++c;
  }
}

void Map4D::record(const Pose & T) {recordKey(keyOf(T));}

uint8_t Map4D::cellCount(const Pose & T) const
{
  const CellKey k = keyOf(T);
  return k.valid ? cells_[index(k.iz, k.itheta, k.ix, k.iy)] : 0;
}

bool Map4D::isReachable(const Pose & T) const {return cellCount(T) > 0;}

std::vector<BaseCandidate> Map4D::inverseQuery(
  const Pose & tool_in_world, double arm_base_z, int ring_samples) const
{
  std::vector<BaseCandidate> out;
  const Vec3 axis = tool_in_world.linear().col(2);
  const Vec3 pw = tool_in_world.translation();
  const double s = std::hypot(axis.x(), axis.y());
  const double theta = std::atan2(s, axis.z());
  const double pz = pw.z() - arm_base_z;

  const int iz = static_cast<int>(std::floor((pz - params_.z_min) / params_.resolution));
  const int itheta = std::min(ntheta_ - 1, static_cast<int>(std::floor(theta / params_.theta_bin)));
  if (iz < 0 || iz >= nz_ || itheta < 0) {
    return out;
  }
  const bool vertical = s <= 1e-3;
  const double phi = vertical ? 0.0 : std::atan2(axis.y(), axis.x());
  const double c = std::cos(phi), sn = std::sin(phi);

  // keyOf puts a vertical axis at y* = 0, i.e. in this one row of cells.
  const int iy_vertical = static_cast<int>(std::floor((0.0 + params_.xy_extent) / params_.resolution));
  for (int ix = 0; ix < nxy_; ++ix) {
    const double xs = -params_.xy_extent + (ix + 0.5) * params_.resolution;
    for (int iy = 0; iy < nxy_; ++iy) {
      if (vertical && iy != iy_vertical) {
        continue;
      }
      const uint8_t count = cells_[index(iz, itheta, ix, iy)];
      if (count == 0) {
        continue;
      }
      const double ys = -params_.xy_extent + (iy + 0.5) * params_.resolution;
      if (!vertical) {
        // Base position = tool position - Rz(phi) * (x*, y*).
        out.push_back({pw.x() - (c * xs - sn * ys), pw.y() - (sn * xs + c * ys), count});
      } else {
        // No azimuth: any direction works, so only the radius matters. Sample a ring, once per
        // radius (the canonical frame for a vertical axis puts the position on +x: y* = 0).
        const double r = xs;
        for (int k = 0; k < ring_samples; ++k) {
          const double a = 2.0 * M_PI * k / ring_samples;
          out.push_back({pw.x() - r * std::cos(a), pw.y() - r * std::sin(a), count});
        }
      }
    }
  }
  return out;
}

void Map4D::mergeFrom(const Map4D & other)
{
  if (!(params_ == other.params_)) {
    throw std::invalid_argument("Map4D::mergeFrom: maps were built with different parameters");
  }
  for (size_t i = 0; i < cells_.size(); ++i) {
    const unsigned sum = static_cast<unsigned>(cells_[i]) + other.cells_[i];
    cells_[i] = static_cast<uint8_t>(std::min(sum, 255U));
  }
}

}  // namespace reachability
