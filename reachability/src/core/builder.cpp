#include "reachability/core/builder.hpp"

#include <algorithm>
#include <chrono>
#include <random>
#include <thread>
#include <vector>

namespace reachability
{

Map4D buildMap(
  const ArmModel & arm, const MapParams & params, const BuildOptions & options, BuildStats * stats)
{
  options.redundancy.validate(arm);
  const auto t0 = std::chrono::steady_clock::now();
  int threads = options.threads > 0 ? options.threads :
    static_cast<int>(std::max(1U, std::thread::hardware_concurrency()));
  threads = std::max(1, std::min<int>(threads, static_cast<int>(options.samples)));

  std::vector<Map4D> shards(static_cast<size_t>(threads), Map4D(params));
  std::vector<size_t> valid(static_cast<size_t>(threads), 0), in_window(static_cast<size_t>(threads), 0);

  auto work = [&](int t) {
      std::mt19937_64 rng(options.seed * 1000003ULL + static_cast<uint64_t>(t));
      const size_t share = options.samples / static_cast<size_t>(threads) +
        (static_cast<size_t>(t) < options.samples % static_cast<size_t>(threads) ? 1 : 0);
      std::vector<double> q(static_cast<size_t>(arm.dof()));
      Pose tool;
      for (size_t n = 0; n < share; ++n) {
        for (int j = 0; j < arm.dof(); ++j) {
          if (j == options.redundancy.joint) {
            q[static_cast<size_t>(j)] = options.redundancy.draw(rng, arm.lower(j), arm.upper(j));
            continue;
          }
          std::uniform_real_distribution<double> u(arm.lower(j), arm.upper(j));
          q[static_cast<size_t>(j)] = u(rng);
        }
        if (!arm.evaluate(q, tool)) {
          continue;
        }
        ++valid[static_cast<size_t>(t)];
        const CellKey key = shards[static_cast<size_t>(t)].keyOf(tool);
        if (key.valid) {
          ++in_window[static_cast<size_t>(t)];
          shards[static_cast<size_t>(t)].recordKey(key);
        }
      }
    };

  std::vector<std::thread> pool;
  for (int t = 1; t < threads; ++t) {
    pool.emplace_back(work, t);
  }
  work(0);
  for (auto & th : pool) {
    th.join();
  }

  Map4D map(params);
  for (const auto & s : shards) {
    map.mergeFrom(s);
  }
  if (stats) {
    stats->samples = options.samples;
    stats->valid = 0;
    stats->in_window = 0;
    for (int t = 0; t < threads; ++t) {
      stats->valid += valid[static_cast<size_t>(t)];
      stats->in_window += in_window[static_cast<size_t>(t)];
    }
    stats->nonzero_cells = map.nonzeroCells();
    stats->saturated_cells = static_cast<size_t>(
      std::count(map.raw().begin(), map.raw().end(), static_cast<uint8_t>(255)));
    stats->seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  }
  return map;
}

}  // namespace reachability
