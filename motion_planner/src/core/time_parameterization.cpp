#include "motion_planner/core/time_parameterization.hpp"

#include <algorithm>
#include <cmath>

namespace motion_planner
{

namespace
{
constexpr double kMinStepSec = 1e-3;
constexpr double kPi = 3.14159265358979323846;

double jointDistance(const std::vector<double> & a, const std::vector<double> & b)
{
  double sum = 0.0;
  for (size_t i = 0; i < a.size() && i < b.size(); ++i) {
    sum += (b[i] - a[i]) * (b[i] - a[i]);
  }
  return std::sqrt(sum);
}

double maxDelta(const std::vector<double> & a, const std::vector<double> & b)
{
  double m = 0.0;
  for (size_t i = 0; i < a.size() && i < b.size(); ++i) {
    m = std::max(m, std::fabs(b[i] - a[i]));
  }
  return m;
}
}  // namespace

const char * toString(TimingError error)
{
  switch (error) {
    case TimingError::kNone: return "none";
    case TimingError::kTooFewPoints: return "too_few_points";
    case TimingError::kJointCountMismatch: return "joint_count_mismatch";
    case TimingError::kNonFinite: return "non_finite";
    case TimingError::kBadLimit: return "bad_limit";
    case TimingError::kBadStrokeSpeed: return "bad_stroke_speed";
    case TimingError::kBackendFailed: return "backend_failed";
    case TimingError::kLimitViolation: return "limit_violation";
  }
  return "unknown";
}

TimingError validateTimingInput(
  const core_types::Trajectory & path, const TimingRequest & request, std::string & message)
{
  const size_t n = path.joint_names.size();
  if (n == 0 || request.limits.size() != n) {
    message = "limits must have one entry per joint";
    return TimingError::kJointCountMismatch;
  }
  for (const auto & l : request.limits) {
    if (!(std::isfinite(l.velocity) && l.velocity > 0.0) ||
      !(std::isfinite(l.acceleration) && l.acceleration > 0.0) ||
      !(std::isfinite(l.jerk) && l.jerk >= 0.0))
    {
      message = "velocity and acceleration limits must be > 0 and jerk >= 0";
      return TimingError::kBadLimit;
    }
  }
  if (request.role == SegmentRole::kContactSeam &&
    !(std::isfinite(request.stroke_speed) && request.stroke_speed > 0.0))
  {
    message = "a contact seam needs a positive stroke speed";
    return TimingError::kBadStrokeSpeed;
  }
  for (const auto & p : path.points) {
    if (p.positions.size() != n) {
      message = "a point does not match joint_names";
      return TimingError::kJointCountMismatch;
    }
    for (double x : p.positions) {
      if (!std::isfinite(x)) {
        message = "non-finite position";
        return TimingError::kNonFinite;
      }
    }
  }
  if (path.points.size() < 2) {
    message = "fewer than 2 points";
    return TimingError::kTooFewPoints;
  }
  return TimingError::kNone;
}

core_types::Trajectory removeNearDuplicates(
  const core_types::Trajectory & path, double epsilon, size_t & dropped)
{
  core_types::Trajectory out;
  out.joint_names = path.joint_names;
  dropped = 0;
  for (size_t i = 0; i < path.points.size(); ++i) {
    if (out.points.empty() ||
      maxDelta(out.points.back().positions, path.points[i].positions) > epsilon)
    {
      core_types::TrajectoryPoint p;
      p.positions = path.points[i].positions;
      out.points.push_back(std::move(p));
    } else if (i + 1 == path.points.size() && out.points.size() > 1) {
      // keep the true goal: it replaces the near-duplicate before it
      out.points.back().positions = path.points[i].positions;
      ++dropped;
    } else {
      ++dropped;
    }
  }
  return out;
}

std::vector<size_t> findReversals(const core_types::Trajectory & path, double angle_deg)
{
  std::vector<size_t> out;
  const double cos_limit = std::cos(angle_deg * kPi / 180.0);
  for (size_t i = 1; i + 1 < path.points.size(); ++i) {
    const auto & a = path.points[i - 1].positions;
    const auto & b = path.points[i].positions;
    const auto & c = path.points[i + 1].positions;
    double dot = 0.0;
    double na = 0.0;
    double nb = 0.0;
    for (size_t j = 0; j < b.size(); ++j) {
      const double u = b[j] - a[j];
      const double v = c[j] - b[j];
      dot += u * v;
      na += u * u;
      nb += v * v;
    }
    if (na <= 0.0 || nb <= 0.0) {
      continue;
    }
    if (dot / std::sqrt(na * nb) < cos_limit) {
      out.push_back(i);
    }
  }
  return out;
}

std::vector<core_types::Trajectory> splitAt(
  const core_types::Trajectory & path, const std::vector<size_t> & indices)
{
  std::vector<core_types::Trajectory> out;
  size_t start = 0;
  auto emit = [&](size_t end) {
      core_types::Trajectory t;
      t.joint_names = path.joint_names;
      t.points.assign(path.points.begin() + start, path.points.begin() + end + 1);
      out.push_back(std::move(t));
      start = end;
    };
  for (size_t idx : indices) {
    if (idx > start && idx + 1 < path.points.size()) {
      emit(idx);
    }
  }
  emit(path.points.size() - 1);
  return out;
}

void constantSpeedStamps(core_types::Trajectory & piece, double speed, const PathMetric & metric)
{
  auto & pts = piece.points;
  if (pts.empty() || speed <= 0.0) {
    return;
  }
  const PathMetric m = metric ? metric : PathMetric(jointDistance);
  const size_t n = piece.joint_names.size();
  pts[0].time_from_start_sec = 0.0;
  for (size_t i = 1; i < pts.size(); ++i) {
    const double dt = std::max(m(pts[i - 1].positions, pts[i].positions) / speed, kMinStepSec);
    pts[i].time_from_start_sec = pts[i - 1].time_from_start_sec + dt;
  }
  for (size_t i = 0; i < pts.size(); ++i) {
    const size_t lo = i == 0 ? 0 : i - 1;
    const size_t hi = std::min(i + 1, pts.size() - 1);
    const double dt = pts[hi].time_from_start_sec - pts[lo].time_from_start_sec;
    pts[i].velocities.assign(n, 0.0);
    pts[i].accelerations.assign(n, 0.0);
    if (dt > 1e-9) {
      for (size_t j = 0; j < n; ++j) {
        pts[i].velocities[j] = (pts[hi].positions[j] - pts[lo].positions[j]) / dt;
      }
    }
  }
}

std::vector<LimitViolation> findLimitViolations(
  const core_types::Trajectory & piece, const std::vector<JointLimit> & limits,
  double tolerance, size_t piece_index)
{
  std::vector<LimitViolation> out;
  const auto & pts = piece.points;
  const size_t jc = piece.joint_names.size();
  const size_t n = std::min(jc, limits.size());
  auto neighbours = [&](size_t i, size_t & lo, size_t & hi) {
      lo = i == 0 ? 0 : i - 1;
      hi = std::min(i + 1, pts.size() - 1);
    };
  auto add = [&](size_t i, size_t j, LimitViolation::Kind k, double v, double lim) {
      if (std::fabs(v) > lim * (1.0 + tolerance)) {
        out.push_back({piece_index, i, j, k, v, lim});
      }
    };
  auto vel = [&](size_t i, size_t j) {
      if (pts[i].velocities.size() == jc) {
        return pts[i].velocities[j];
      }
      size_t lo, hi;
      neighbours(i, lo, hi);
      const double dt = pts[hi].time_from_start_sec - pts[lo].time_from_start_sec;
      return dt > 1e-9 ? (pts[hi].positions[j] - pts[lo].positions[j]) / dt : 0.0;
    };
  for (size_t i = 0; i < pts.size(); ++i) {
    size_t lo, hi;
    neighbours(i, lo, hi);
    const double dt = pts[hi].time_from_start_sec - pts[lo].time_from_start_sec;
    const bool has_acc = pts[i].accelerations.size() == jc;
    for (size_t j = 0; j < n; ++j) {
      add(i, j, LimitViolation::Kind::kVelocity, vel(i, j), limits[j].velocity);
      const double acc = has_acc ? pts[i].accelerations[j] :
        (dt > 1e-9 ? (vel(hi, j) - vel(lo, j)) / dt : 0.0);
      add(i, j, LimitViolation::Kind::kAcceleration, acc, limits[j].acceleration);
      if (limits[j].jerk > 0.0 && has_acc && dt > 1e-9 &&
        pts[hi].accelerations.size() == jc && pts[lo].accelerations.size() == jc)
      {
        add(
          i, j, LimitViolation::Kind::kJerk,
          (pts[hi].accelerations[j] - pts[lo].accelerations[j]) / dt, limits[j].jerk);
      }
    }
  }
  return out;
}

core_types::Trajectory concatenate(const std::vector<core_types::Trajectory> & pieces)
{
  core_types::Trajectory out;
  double offset = 0.0;
  for (const auto & piece : pieces) {
    if (out.joint_names.empty()) {
      out.joint_names = piece.joint_names;
    }
    for (size_t i = 0; i < piece.points.size(); ++i) {
      if (i == 0 && !out.points.empty()) {
        continue;   // the shared cut point, already there and at rest
      }
      auto p = piece.points[i];
      p.time_from_start_sec += offset;
      out.points.push_back(std::move(p));
    }
    if (!piece.points.empty()) {
      offset += piece.points.back().time_from_start_sec;
    }
  }
  return out;
}

TimingResult parameterizeSegment(
  TimeParameterizer & retimer, const core_types::Trajectory & path, const TimingRequest & request)
{
  TimingResult result;
  result.error = validateTimingInput(path, request, result.message);
  if (result.error != TimingError::kNone) {
    return result;
  }
  const auto clean = removeNearDuplicates(path, request.duplicate_epsilon, result.dropped_points);
  if (clean.points.size() < 2) {
    result.error = TimingError::kTooFewPoints;
    result.message = "fewer than 2 distinct points";
    return result;
  }
  result.split_points = findReversals(clean, request.reversal_angle_deg);
  result.pieces = splitAt(clean, result.split_points);

  for (size_t k = 0; k < result.pieces.size(); ++k) {
    auto & piece = result.pieces[k];
    std::string msg;
    if (!retimer.retime(piece, request.role, request.limits, request.stroke_speed, msg)) {
      result.error = TimingError::kBackendFailed;
      result.message = "piece " + std::to_string(k) + ": " + msg;
      return result;
    }
    auto v = findLimitViolations(piece, request.limits, request.violation_tolerance, k);
    result.violations.insert(result.violations.end(), v.begin(), v.end());
    if (!piece.points.empty()) {
      result.total_duration_sec += piece.points.back().time_from_start_sec;
    }
  }
  if (request.role == SegmentRole::kContactSeam && !result.violations.empty()) {
    result.error = TimingError::kLimitViolation;
    result.message = std::to_string(result.violations.size()) +
      " limit violation(s) on a contact seam";
    return result;
  }
  result.success = true;
  return result;
}

}  // namespace motion_planner
