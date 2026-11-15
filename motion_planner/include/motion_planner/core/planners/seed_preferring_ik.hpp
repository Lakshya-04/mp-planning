/**
 * @file seed_preferring_ik.hpp
 * @brief IkPort decorator that returns the solution closest to the caller's seed.
 * @details Wraps any IkPort. A single solve from the seed can land on a different branch of the
 *          arm (a joint a full turn away, a flipped elbow) even when a nearby one exists. This
 *          retries from perturbed copies of the seed, wraps every solution's revolute joints by
 *          whole turns toward the seed, drops the ones outside the joint limits, and returns the
 *          one with the smallest weighted distance to the seed. Solver-agnostic: it only sees
 *          IkPort, so it sits on MoveIt, cuRobo or anything else.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "motion_planner/core/planners/ik_port.hpp"

namespace motion_planner
{

struct SeedPreferringIkConfig
{
  int extra_seeds{8};                 ///< Perturbed seeds tried besides the caller's own.
  double perturbation{0.3};           ///< Half-width of the uniform seed noise [rad], on wrappable joints only.
  double accept_distance{0.0};        ///< Stop early once a solution is this close; 0 = try all.
  uint32_t rng_seed{1};               ///< Fixed so a query gives the same answer every time.
  std::vector<double> lower_limits;   ///< Optional; with upper_limits, solutions outside are dropped.
  std::vector<double> upper_limits;
  std::vector<double> weights;        ///< Optional per-joint weight in the distance; empty = 1.
  std::vector<bool> wrap_joint;       ///< Optional; true for joints that may be shifted by 2*pi
                                      ///< (revolute). Empty = every joint (all-revolute arm).
};

class SeedPreferringIk : public IkPort
{
public:
  SeedPreferringIk(std::shared_ptr<IkPort> inner, SeedPreferringIkConfig config);

  IkResult ik(const IkRequest & request) override;

  /// Shift each wrappable joint of `solution` by whole turns to be nearest `seed`; if limits are
  /// set, prefer the nearest shift that stays inside them. Returns false when a joint cannot be
  /// brought inside its limits.
  bool wrapToSeed(std::vector<double> & solution, const std::vector<double> & seed) const;

  double distance(const std::vector<double> & a, const std::vector<double> & b) const;

private:
  std::shared_ptr<IkPort> inner_;
  SeedPreferringIkConfig config_;
};

}  // namespace motion_planner
