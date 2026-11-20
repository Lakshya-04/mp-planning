#include "reachability/core/pose_quality.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/SVD>

namespace reachability
{

namespace
{
bool cellReachable(const Map4D & map, int iz, int it, int ix, int iy)
{
  if (iz < 0 || iz >= map.nz() || it < 0 || it >= map.ntheta() || ix < 0 || ix >= map.nxy() || iy < 0 ||
    iy >= map.nxy())
  {
    return false;
  }
  return map.raw()[map.index(iz, it, ix, iy)] > 0;
}
}  // namespace

PoseScore scorePose(const Map4D & map, const Pose & T, const MarginConfig & config)
{
  PoseScore s;
  const CellKey k = map.keyOf(T);
  if (!k.valid) {return s;}
  s.count = map.raw()[map.index(k.iz, k.itheta, k.ix, k.iy)];
  s.reachable = s.count > 0;
  if (!s.reachable) {return s;}
  const double ref = std::max(1.0, config.count_ref);
  s.count_score = std::min(static_cast<double>(s.count), ref) / ref;
  int hit = 0;
  const int dz[8] = {1, -1, 0, 0, 0, 0, 0, 0};
  const int dt[8] = {0, 0, 1, -1, 0, 0, 0, 0};
  const int dx[8] = {0, 0, 0, 0, 1, -1, 0, 0};
  const int dy[8] = {0, 0, 0, 0, 0, 0, 1, -1};
  for (int i = 0; i < 8; ++i) {
    // theta wraps nowhere: the polar angle is 0..pi, a step past either end has no neighbour.
    if (cellReachable(map, k.iz + dz[i], k.itheta + dt[i], k.ix + dx[i], k.iy + dy[i])) {++hit;}
  }
  s.boundary_score = hit / 8.0;
  s.score = std::min(s.count_score, s.boundary_score);
  return s;
}

double minSingularValue(const ArmModel & arm, const std::vector<double> & q, double eps)
{
  const int n = arm.dof();
  if (static_cast<int>(q.size()) != n || n < 1) {return -1.0;}
  Pose T0;
  if (!arm.evaluate(q, T0)) {return -1.0;}
  Eigen::MatrixXd J(6, n);
  for (int j = 0; j < n; ++j) {
    std::vector<double> qp = q;
    qp[static_cast<size_t>(j)] += eps;
    Pose T1;
    if (!arm.evaluate(qp, T1)) {
      // Limit hit on the forward step: difference backwards instead.
      qp[static_cast<size_t>(j)] = q[static_cast<size_t>(j)] - eps;
      if (!arm.evaluate(qp, T1)) {return -1.0;}
      J.block<3, 1>(0, j) = (T0.translation() - T1.translation()) / eps;
      const Eigen::AngleAxisd r(T0.linear() * T1.linear().transpose());
      J.block<3, 1>(3, j) = r.axis() * r.angle() / eps;
      continue;
    }
    J.block<3, 1>(0, j) = (T1.translation() - T0.translation()) / eps;
    const Eigen::AngleAxisd r(T1.linear() * T0.linear().transpose());
    J.block<3, 1>(3, j) = r.axis() * r.angle() / eps;
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
  const Eigen::VectorXd sv = svd.singularValues();
  // With fewer than 6 joints there are fewer singular values than task dimensions: the smallest of the
  // joints' own directions is returned (the arm lacks the missing task directions at every configuration).
  return sv.minCoeff();
}

double jointLimitMargin(const ArmModel & arm, const std::vector<double> & q)
{
  if (static_cast<int>(q.size()) != arm.dof()) {return -1.0;}
  double m = 0.5;
  for (int j = 0; j < arm.dof(); ++j) {
    const double lo = arm.lower(j), hi = arm.upper(j), range = hi - lo;
    if (!(range > 0.0) || range >= 2.0 * M_PI - 1e-9) {continue;}
    const double v = q[static_cast<size_t>(j)];
    m = std::min(m, std::max(0.0, std::min(v - lo, hi - v)) / range);
  }
  return m;
}

}  // namespace reachability
