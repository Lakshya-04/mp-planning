#include "motion_planner/core/planners/seed_preferring_ik.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <utility>

namespace motion_planner
{

namespace
{
constexpr double kTwoPi = 6.283185307179586476925286766559;
}  // namespace

SeedPreferringIk::SeedPreferringIk(std::shared_ptr<IkPort> inner, SeedPreferringIkConfig config)
: inner_(std::move(inner)), config_(std::move(config))
{
  // The noise half-width is a magnitude: a negative or NaN value would be undefined for the distribution.
  config_.perturbation = std::isfinite(config_.perturbation) ? std::fabs(config_.perturbation) : 0.0;
}

double SeedPreferringIk::distance(const std::vector<double> & a, const std::vector<double> & b) const
{
  double sum = 0.0;
  for (size_t i = 0; i < a.size() && i < b.size(); ++i) {
    const double w = i < config_.weights.size() ? config_.weights[i] : 1.0;
    const double d = a[i] - b[i];
    sum += w * d * d;
  }
  return std::sqrt(sum);
}

bool SeedPreferringIk::wrapToSeed(
  std::vector<double> & solution, const std::vector<double> & seed) const
{
  const bool have_limits = config_.lower_limits.size() >= solution.size() &&
    config_.upper_limits.size() >= solution.size();
  for (size_t i = 0; i < solution.size() && i < seed.size(); ++i) {
    const bool wrap = config_.wrap_joint.empty() ||
      (i < config_.wrap_joint.size() && config_.wrap_joint[i]);
    double q = solution[i];
    if (wrap) {
      q += kTwoPi * std::round((seed[i] - q) / kTwoPi);
      if (have_limits) {
        // The nearest turn may sit outside the limits; step toward the interior.
        while (q > config_.upper_limits[i] + 1e-9) {q -= kTwoPi;}
        while (q < config_.lower_limits[i] - 1e-9) {q += kTwoPi;}
      }
    }
    if (have_limits &&
      (q < config_.lower_limits[i] - 1e-9 || q > config_.upper_limits[i] + 1e-9))
    {
      return false;
    }
    solution[i] = q;
  }
  return true;
}

IkResult SeedPreferringIk::ik(const IkRequest & request)
{
  if (request.joint_seed.empty() || config_.extra_seeds <= 0) {
    return inner_->ik(request);
  }

  std::mt19937 rng(config_.rng_seed);
  std::uniform_real_distribution<double> noise(-config_.perturbation, config_.perturbation);

  IkResult best;
  double best_distance = std::numeric_limits<double>::infinity();
  IkResult last_failure;
  bool any_attempt_failed = false;

  for (int attempt = 0; attempt <= config_.extra_seeds; ++attempt) {
    IkRequest trial = request;
    if (attempt > 0) {
      const bool have_limits = config_.lower_limits.size() >= trial.joint_seed.size() &&
        config_.upper_limits.size() >= trial.joint_seed.size();
      for (size_t i = 0; i < trial.joint_seed.size(); ++i) {
        // Noise is in radians: keep it off joints that are not turns (a lift is in metres).
        if (!config_.wrap_joint.empty() && !(i < config_.wrap_joint.size() && config_.wrap_joint[i])) {
          continue;
        }
        double q = trial.joint_seed[i] + noise(rng);
        if (have_limits) {q = std::min(std::max(q, config_.lower_limits[i]), config_.upper_limits[i]);}
        trial.joint_seed[i] = q;
      }
    }
    IkResult result = inner_->ik(trial);
    if (!result.success) {
      if (!any_attempt_failed) {last_failure = std::move(result);}   // the first reason is the real one
      any_attempt_failed = true;
      continue;
    }
    if (!wrapToSeed(result.joint_solution, request.joint_seed)) {
      if (!any_attempt_failed) {
        last_failure = result;
        last_failure.success = false;
        last_failure.message = "IK solution is outside the joint limits.";
        last_failure.failure_reason_code = "ik_outside_limits";
        last_failure.joint_solution.clear();
      }
      any_attempt_failed = true;
      continue;
    }
    const double d = distance(result.joint_solution, request.joint_seed);
    if (d < best_distance) {
      best_distance = d;
      best = std::move(result);
    }
    if (config_.accept_distance > 0.0 && best_distance <= config_.accept_distance) {break;}
  }

  if (best.success) {return best;}
  return any_attempt_failed ? last_failure : best;
}

}  // namespace motion_planner
