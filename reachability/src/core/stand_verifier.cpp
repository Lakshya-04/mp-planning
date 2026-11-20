// Stand verification bookkeeping: see core/stand_verifier.hpp.
#include "reachability/core/stand_verifier.hpp"

#include <algorithm>
#include <cmath>
#include <map>

namespace reachability
{
namespace
{
int rank(VerifyReason r)
{
  switch (r) {
    case VerifyReason::kInCollision: return 3;
    case VerifyReason::kOutOfLimits: return 2;
    case VerifyReason::kIkFailed: return 1;
    default: return 0;
  }
}

Stand shifted(const Stand & s, double forward, double left)
{
  Stand o = s;
  o.arm_xy += Eigen::Vector2d(
    forward * std::cos(s.yaw) - left * std::sin(s.yaw), forward * std::sin(s.yaw) + left * std::cos(s.yaw));
  return o;
}
}  // namespace

const char * toString(VerifyReason reason)
{
  switch (reason) {
    case VerifyReason::kOk: return "ok";
    case VerifyReason::kIkFailed: return "ik_failed";
    case VerifyReason::kInCollision: return "in_collision";
    case VerifyReason::kOutOfLimits: return "out_of_limits";
    case VerifyReason::kError: return "error";
  }
  return "unknown";
}

std::vector<double> pickProbeHeights(const std::vector<double> & heights, size_t max_probes)
{
  if (max_probes == 0 || heights.size() <= max_probes) {return heights;}
  if (max_probes == 1) {return {heights[heights.size() / 2]};}
  std::vector<double> out;
  for (size_t i = 0; i < max_probes; ++i) {
    const size_t k = static_cast<size_t>(std::llround(
        static_cast<double>(i) * static_cast<double>(heights.size() - 1) / static_cast<double>(max_probes - 1)));
    out.push_back(heights[k]);
  }
  return out;
}

StandVerification verifyStand(
  StandVerifier & verifier, const Stand & stand, const std::vector<Pose> & targets_world,
  const std::vector<size_t> & indices, const std::vector<std::vector<double>> & heights, const VerifyOptions & opt)
{
  StandVerification out;
  out.stand = stand;
  const size_t n = indices.size();
  out.targets.resize(n);

  // Nominal probes, batched per distinct height: verdicts[i][k] for heights[i][k].
  std::vector<std::vector<TargetVerdict>> verdicts(n);
  std::map<double, std::vector<std::pair<size_t, size_t>>> by_height;   // height -> (i, k)
  for (size_t i = 0; i < n; ++i) {
    const auto probes = pickProbeHeights(i < heights.size() ? heights[i] : std::vector<double>{}, opt.max_lift_probes);
    verdicts[i].resize(probes.size());
    for (size_t k = 0; k < probes.size(); ++k) {by_height[probes[k]].push_back({i, k});}
  }
  auto call = [&](const Stand & s, double h, const std::vector<size_t> & ts) {
      std::vector<Pose> poses;
      for (const size_t t : ts) {poses.push_back(targets_world[t]);}
      ++out.verifier_calls;
      out.verifier_poses += ts.size();
      auto v = verifier.verify(s, h, poses);
      v.resize(ts.size(), TargetVerdict{VerifyReason::kError, {}});   // a short answer is an error, never an ok
      return v;
    };
  // Heights ascend, so with stop_at_first_ok each target meets its lowest height first.
  std::vector<bool> solved(n, false);
  std::vector<size_t> tried(n, 0);
  for (const auto & [h, who] : by_height) {
    std::vector<size_t> ts;
    std::vector<std::pair<size_t, size_t>> sent;
    for (const auto & ik : who) {
      if (opt.stop_at_first_ok && solved[ik.first]) {continue;}
      ts.push_back(indices[ik.first]);
      sent.push_back(ik);
    }
    if (ts.empty()) {continue;}
    const auto v = call(stand, h, ts);
    for (size_t j = 0; j < sent.size(); ++j) {
      verdicts[sent[j].first][sent[j].second] = v[j];
      ++tried[sent[j].first];
      if (v[j].ok()) {solved[sent[j].first] = true;}
    }
  }

  // Per target: confirmed if any nominal height solved; the best one is the one the jittered base keeps.
  std::vector<double> best_h(n, 0.0);
  std::vector<int> best_k(n, -1);
  for (size_t i = 0; i < n; ++i) {
    auto & r = out.targets[i];
    r.target = indices[i];
    const auto probes = pickProbeHeights(i < heights.size() ? heights[i] : std::vector<double>{}, opt.max_lift_probes);
    r.height_probes = opt.stop_at_first_ok ? tried[i] : probes.size();
    int worst = -1;
    for (size_t k = 0; k < probes.size(); ++k) {
      if (opt.stop_at_first_ok && k >= tried[i]) {break;}   // never sent
      const auto & v = verdicts[i][k];
      if (v.ok()) {
        ++r.height_ok;
      } else if (rank(v.reason) > worst) {
        worst = rank(v.reason);
        r.reason = v.reason;
      }
    }
    if (probes.empty()) {r.reason = VerifyReason::kIkFailed;}
    r.lift_margin = r.height_probes ? static_cast<double>(r.height_ok) / static_cast<double>(r.height_probes) : 0.0;
    r.confirmed = r.height_ok > 0;
    if (r.confirmed) {
      r.reason = VerifyReason::kOk;
      for (size_t k = 0; k < probes.size(); ++k) {
        if (verdicts[i][k].ok()) {best_k[i] = static_cast<int>(k); best_h[i] = probes[k]; break;}   // lowest first
      }
    }
  }

  // Offset probes at the best height: 4 shifted stands, batched per (height, offset).
  const bool jitter = opt.jitter_m > 0.0;
  const double offsets[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  std::vector<size_t> jitter_ok(n, 0);
  if (jitter) {
    // The best height is the lowest ok one by default; prefer, per target, the ok height with the most ok offsets.
    std::vector<std::vector<size_t>> ok_offsets(n);
    for (size_t i = 0; i < n; ++i) {
      if (!out.targets[i].confirmed) {continue;}
      const auto probes = pickProbeHeights(heights[i], opt.max_lift_probes);
      size_t best_count = 0;
      bool first = true;
      for (size_t k = 0; k < probes.size(); ++k) {
        if (!verdicts[i][k].ok()) {continue;}
        size_t cnt = 0;
        for (const auto & o : offsets) {
          const auto v = call(shifted(stand, o[0] * opt.jitter_m, o[1] * opt.jitter_m), probes[k], {indices[i]});
          cnt += v[0].ok() ? 1 : 0;
        }
        if (first || cnt > best_count) {first = false; best_count = cnt; best_k[i] = static_cast<int>(k); best_h[i] = probes[k];}
      }
      jitter_ok[i] = best_count;
    }
  }

  double msum = 0.0;
  for (size_t i = 0; i < n; ++i) {
    auto & r = out.targets[i];
    if (!r.confirmed) {continue;}
    r.offset_margin = jitter ? static_cast<double>(jitter_ok[i]) / 4.0 : 1.0;
    r.margin = r.lift_margin * r.offset_margin;
    // a target that solves nominally but at no jittered offset still counts as confirmed, with a tiny margin
    if (r.margin <= 0.0) {r.margin = 1e-3 * r.lift_margin;}
    r.best_height = best_h[i];
    r.joints = verdicts[i][static_cast<size_t>(best_k[i])].joints;
    ++out.confirmed;
    msum += r.margin;
  }
  out.coverage = n ? static_cast<double>(out.confirmed) / static_cast<double>(n) : 1.0;
  out.mean_margin = out.confirmed ? msum / static_cast<double>(out.confirmed) : 0.0;

  // Bands: greedy cover over the ok nominal heights (most newly covered first, ties to the lower height).
  std::map<double, std::vector<size_t>> ok_at;   // height -> positions i
  for (size_t i = 0; i < n; ++i) {
    const auto probes = pickProbeHeights(i < heights.size() ? heights[i] : std::vector<double>{}, opt.max_lift_probes);
    for (size_t k = 0; k < probes.size(); ++k) {
      if (verdicts[i][k].ok()) {ok_at[probes[k]].push_back(i);}
    }
  }
  std::vector<bool> done(n, false);
  for (;;) {
    double best_h_val = 0.0;
    size_t best_gain = 0;
    for (const auto & [h, ids] : ok_at) {
      size_t gain = 0;
      for (const size_t i : ids) {gain += done[i] ? 0 : 1;}
      if (gain > best_gain) {best_gain = gain; best_h_val = h;}
    }
    if (best_gain == 0) {break;}
    LiftBand band;
    band.lift_height = best_h_val;
    for (const size_t i : ok_at[best_h_val]) {
      if (!done[i]) {done[i] = true; band.pose_indices.push_back(indices[i]);}
    }
    out.bands.push_back(std::move(band));
  }
  return out;
}

}  // namespace reachability
