#include "reachability/core/path_quality_audit.hpp"

#include <algorithm>
#include <cmath>

#include "reachability/core/pose_quality.hpp"

namespace reachability
{

const char * toString(PathQualityReason reason)
{
  switch (reason) {
    case PathQualityReason::kNearSingular: return "near_singular";
    case PathQualityReason::kNearLimit: return "near_limit";
    default: return "ok";
  }
}

PathQualityReport auditPathQuality(
  const std::vector<PathStroke> & strokes, const JacobianFn & jacobian, const LimitMarginFn & margin,
  const PathQualityConfig & config)
{
  PathQualityReport rep;
  for (const PathStroke & s : strokes) {
    StrokeQuality sq;
    sq.id = s.id;
    sq.waypoints = static_cast<int>(s.waypoints.size());
    for (int i = 0; i < sq.waypoints; ++i) {
      const std::vector<double> & q = s.waypoints[static_cast<size_t>(i)];
      bool used = false;
      if (jacobian) {
        Eigen::MatrixXd J;
        if (jacobian(q, J)) {
          const double sv = minSingularValue(J);
          if (sv >= 0.0) {
            used = true;
            if (sq.min_sigma_index < 0 || sv < sq.min_sigma) {
              sq.min_sigma = sv;
              sq.min_sigma_index = i;
            }
          }
        }
      }
      if (margin) {
        const double m = margin(q);
        if (m >= 0.0) {
          used = true;
          if (sq.min_limit_margin_index < 0 || m < sq.min_limit_margin) {
            sq.min_limit_margin = m;
            sq.min_limit_margin_index = i;
          }
        }
      }
      if (used) {++sq.evaluated;}
    }
    const bool singular =
      config.min_sigma > 0.0 && sq.min_sigma_index >= 0 && sq.min_sigma < config.min_sigma;
    const bool limited = config.min_limit_margin > 0.0 && sq.min_limit_margin_index >= 0 &&
      sq.min_limit_margin < config.min_limit_margin;
    if (singular) {
      sq.reason = PathQualityReason::kNearSingular;
      sq.worst_index = sq.min_sigma_index;
      sq.near_limit_too = limited;
    } else if (limited) {
      sq.reason = PathQualityReason::kNearLimit;
      sq.worst_index = sq.min_limit_margin_index;
    } else {
      sq.worst_index = sq.min_sigma_index;
    }

    ++rep.total;
    if (sq.evaluated == 0) {++rep.unevaluated;}
    switch (sq.reason) {
      case PathQualityReason::kNearSingular: ++rep.near_singular; break;
      case PathQualityReason::kNearLimit: ++rep.near_limit; break;
      default: ++rep.ok; break;
    }
    if (sq.min_sigma_index >= 0 && (rep.min_sigma < 0.0 || sq.min_sigma < rep.min_sigma)) {
      rep.min_sigma = sq.min_sigma;
    }
    if (sq.min_limit_margin_index >= 0 &&
      (rep.min_limit_margin < 0.0 || sq.min_limit_margin < rep.min_limit_margin))
    {
      rep.min_limit_margin = sq.min_limit_margin;
    }
    rep.strokes.push_back(std::move(sq));
  }
  return rep;
}

double limitMarginFromBounds(
  const std::vector<double> & lower, const std::vector<double> & upper, const std::vector<double> & q)
{
  if (lower.size() != upper.size() || q.size() != lower.size()) {return -1.0;}
  double m = 0.5;
  for (size_t j = 0; j < q.size(); ++j) {
    const double range = upper[j] - lower[j];
    if (!(range > 0.0) || range >= 2.0 * M_PI - 1e-9) {continue;}
    m = std::min(m, std::max(0.0, std::min(q[j] - lower[j], upper[j] - q[j])) / range);
  }
  return m;
}

JacobianFn armJacobianFn(const ArmModel & arm, double eps)
{
  return [&arm, eps](const std::vector<double> & q, Eigen::MatrixXd & J) {
           return armJacobian(arm, q, J, eps);
         };
}

LimitMarginFn armLimitMarginFn(const ArmModel & arm)
{
  return [&arm](const std::vector<double> & q) {return jointLimitMargin(arm, q);};
}

}  // namespace reachability
