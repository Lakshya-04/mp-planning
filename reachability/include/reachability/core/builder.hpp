#pragma once

#include <cstddef>
#include <cstdint>

#include "reachability/core/arm_model.hpp"
#include "reachability/core/map4d.hpp"

namespace reachability
{

struct BuildOptions
{
  size_t samples{2'000'000};  ///< Random joint configurations to evaluate.
  int threads{0};             ///< 0 = hardware concurrency.
  uint32_t seed{1};
};

struct BuildStats
{
  size_t samples{0};
  size_t valid{0};       ///< Samples the model accepted.
  size_t in_window{0};   ///< Valid samples inside the map window.
  size_t nonzero_cells{0};
  size_t saturated_cells{0};  ///< Cells at the 255 cap: more samples would not add information there.
  double seconds{0.0};
};

/// Fills a map by sampling joint configurations uniformly within the joint limits. Shards run on
/// separate threads and are merged; the result is deterministic for a given seed and thread count.
Map4D buildMap(const ArmModel & arm, const MapParams & params, const BuildOptions & options,
  BuildStats * stats = nullptr);

}  // namespace reachability
