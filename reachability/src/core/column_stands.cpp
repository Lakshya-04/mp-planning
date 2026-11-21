#include "reachability/core/column_stands.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "reachability/core/global_stands.hpp"
#include "reachability/core/lift_bands.hpp"

namespace reachability
{
namespace
{

// One (candidate, column) evaluation, kept small: the full check is only kept for the best one per column.
struct Cell
{
  ColumnStatus status{ColumnStatus::kUnreachable};
  bool ends_usable{false};
  double height{0.0};
  double min_score{0.0};
  std::vector<size_t> usable;   // usable stroke indices at `height`
};

bool betterFeas(const ColumnFeasibility & a, const ColumnFeasibility & b)
{
  const SequenceCheck & x = a.best;
  const SequenceCheck & y = b.best;
  if (x.whole() != y.whole()) {return x.whole();}
  if (x.ends_usable != y.ends_usable) {return x.ends_usable;}
  if (x.strokes_usable != y.strokes_usable) {return x.strokes_usable > y.strokes_usable;}
  return x.min_score > y.min_score + 1e-12;
}

double stepCost(
  const StationRequest & q, const Stand & a, const Stand & b)
{
  if (q.base) {return travelTime(*q.base, a, b);}
  return (a.arm_xy - b.arm_xy).norm();
}

}  // namespace

ColumnStandPlan planColumnStands(
  const Map4D & map, const StationRequest & station_in, const std::vector<ColumnPlan> & columns,
  const std::optional<Stand> & start, const ColumnStandOptions & opt)
{
  ColumnStandPlan out;
  const size_t nc = columns.size();
  out.columns.resize(nc);
  for (size_t k = 0; k < nc; ++k) {
    out.columns[k].column = k;
    out.columns[k].label = columns[k].label;
    out.columns[k].strokes_total = columns[k].strokes_world.size();
    out.strokes_total += columns[k].strokes_world.size();
  }
  if (out.strokes_total == 0) {
    out.success = true;
    out.message = "no strokes to cover";
    return out;
  }
  if (!(opt.margin_penalty >= 0.0) || !std::isfinite(opt.margin_penalty)) {
    out.message = "margin_penalty must be finite and >= 0";
    return out;
  }

  StationRequest station = station_in;
  station.targets_world.clear();
  for (const auto & c : columns) {
    station.targets_world.insert(station.targets_world.end(), c.strokes_world.begin(), c.strokes_world.end());
  }
  if (const std::string problem = stationGridProblem(station); !problem.empty()) {
    out.message = problem;
    return out;
  }
  const std::vector<CandidateStand> cands = opt.candidates ? *opt.candidates : makeCandidateStands(station);
  out.candidates = cands.size();
  if (cands.empty()) {
    out.message = "no candidate stand fits the wall (clearance, footprint or wall normal)";
    for (auto & r : out.columns) {r.limited = false;}
    out.unreachable = nc;
    return out;
  }
  const std::vector<double> heights = heightRange(station.lift_min, station.lift_max, station.lift_step);
  if (heights.empty()) {
    out.message = "lift range is not usable";
    return out;
  }

  SequenceParams sp = opt.sequence;
  sp.wall_normal = station.wall_normal;
  std::vector<ColumnPoseSequence> seqs(nc);
  for (size_t k = 0; k < nc; ++k) {seqs[k] = buildColumnSequence(columns[k].strokes_world, sp);}

  // 1. Evaluate every (candidate, column).
  std::vector<std::vector<Cell>> cell(cands.size(), std::vector<Cell>(nc));
  std::vector<ColumnFeasibility> best_feas(nc);
  std::vector<bool> have_best(nc, false);
  for (size_t c = 0; c < cands.size(); ++c) {
    for (size_t k = 0; k < nc; ++k) {
      if (seqs[k].poses.empty()) {continue;}
      ColumnFeasibility f = evaluateColumn(map, seqs[k], cands[c].stand.arm_xy, heights, station.arm_mount_offset_z, opt.margin);
      Cell & e = cell[c][k];
      e.status = f.status;
      e.ends_usable = f.best.ends_usable;
      e.height = f.best.lift_height;
      e.min_score = f.best.min_score;
      for (const auto & p : f.best.poses) {
        if (p.role == PoseRole::kStroke && p.usable) {e.usable.push_back(p.stroke);}
      }
      if (!have_best[k] || betterFeas(f, best_feas[k])) {
        best_feas[k] = std::move(f);
        have_best[k] = true;
      }
    }
  }

  // Baseline: stroke-by-stroke cover (columns may be cut anywhere, end poses ignored).
  {
    std::vector<std::vector<bool>> done(nc);
    for (size_t k = 0; k < nc; ++k) {done[k].assign(columns[k].strokes_world.size(), false);}
    size_t used = 0;
    while (true) {
      size_t best_gain = 0, best_c = 0;
      for (size_t c = 0; c < cands.size(); ++c) {
        size_t gain = 0;
        for (size_t k = 0; k < nc; ++k) {
          for (const size_t s : cell[c][k].usable) {if (!done[k][s]) {++gain;}}
        }
        if (gain > best_gain) {best_gain = gain; best_c = c;}
      }
      if (best_gain == 0) {break;}
      for (size_t k = 0; k < nc; ++k) {for (const size_t s : cell[best_c][k].usable) {done[k][s] = true;}}
      ++used;
    }
    out.stands_if_splitting_allowed = used;
  }

  // 2. Cover whole columns.
  struct Pick
  {
    size_t cand;
    std::vector<StandColumn> cols;
  };
  std::vector<Pick> picks;
  std::vector<bool> placed(nc, false);
  auto whole_at = [&](size_t c, size_t k) {return cell[c][k].status == ColumnStatus::kWhole;};
  while (picks.size() < opt.max_stands) {
    double best_ratio = 0.0;
    size_t best_c = cands.size();
    for (size_t c = 0; c < cands.size(); ++c) {
      size_t gain = 0;
      double margin_sum = 0.0;
      for (size_t k = 0; k < nc; ++k) {
        if (!placed[k] && whole_at(c, k)) {++gain; margin_sum += cell[c][k].min_score;}
      }
      if (gain == 0) {continue;}
      const double cost = 1.0 + opt.margin_penalty * (1.0 - margin_sum / static_cast<double>(gain));
      const double ratio = cost / static_cast<double>(gain);
      if (best_c == cands.size() || ratio < best_ratio - 1e-12) {best_ratio = ratio; best_c = c;}
    }
    if (best_c == cands.size()) {break;}
    Pick p;
    p.cand = best_c;
    for (size_t k = 0; k < nc; ++k) {
      if (!placed[k] && whole_at(best_c, k)) {
        placed[k] = true;
        StandColumn sc;
        sc.column = k;
        sc.lift_height = cell[best_c][k].height;
        sc.min_score = cell[best_c][k].min_score;
        sc.whole = true;
        sc.strokes.resize(columns[k].strokes_world.size());
        std::iota(sc.strokes.begin(), sc.strokes.end(), 0);
        p.cols.push_back(std::move(sc));
      }
    }
    picks.push_back(std::move(p));
  }
  // Drop redundant stands: every whole column it holds is whole at another pick (in the reverse of pick order).
  for (size_t i = picks.size(); i-- > 0;) {
    bool redundant = !picks[i].cols.empty();
    for (const auto & sc : picks[i].cols) {
      bool elsewhere = false;
      for (size_t j = 0; j < picks.size() && !elsewhere; ++j) {
        if (j != i && whole_at(picks[j].cand, sc.column)) {elsewhere = true;}
      }
      redundant = redundant && elsewhere;
    }
    if (redundant) {picks.erase(picks.begin() + static_cast<long>(i));}
  }
  // Reassign whole columns among the kept stands: best margin, then the earlier pick.
  for (auto & p : picks) {p.cols.clear();}
  for (size_t k = 0; k < nc; ++k) {
    size_t best_i = picks.size();
    for (size_t i = 0; i < picks.size(); ++i) {
      if (!whole_at(picks[i].cand, k)) {continue;}
      if (best_i == picks.size() || cell[picks[i].cand][k].min_score > cell[picks[best_i].cand][k].min_score + 1e-12) {
        best_i = i;
      }
    }
    if (best_i == picks.size()) {continue;}
    StandColumn sc;
    sc.column = k;
    sc.lift_height = cell[picks[best_i].cand][k].height;
    sc.min_score = cell[picks[best_i].cand][k].min_score;
    sc.whole = true;
    sc.strokes.resize(columns[k].strokes_world.size());
    std::iota(sc.strokes.begin(), sc.strokes.end(), 0);
    picks[best_i].cols.push_back(std::move(sc));
    placed[k] = true;
  }

  // 3. Leftover columns: split on purpose, fewest pieces. A piece is entered and left on its own: its approach
  // and retract are built from its own strokes and have to be usable at the stand, like a column's.
  auto subset = [&](size_t k, const std::vector<size_t> & idx) {
      std::vector<Pose> v;
      v.reserve(idx.size());
      for (const size_t i : idx) {v.push_back(columns[k].strokes_world[i]);}
      return v;
    };
  for (size_t k = 0; k < nc; ++k) {
    if (placed[k] || columns[k].strokes_world.empty()) {continue;}
    if (!opt.allow_splits) {out.columns[k].skipped_to_avoid_split = true; continue;}
    std::vector<bool> covered(columns[k].strokes_world.size(), false);
    while (true) {
      struct Option
      {
        size_t cand, gain;
        bool chosen;
      };
      std::vector<Option> options;
      for (size_t c = 0; c < cands.size(); ++c) {
        size_t gain = 0;
        for (const size_t s : cell[c][k].usable) {if (!covered[s]) {++gain;}}
        if (gain == 0) {continue;}
        bool chosen = false;
        for (const auto & p : picks) {if (p.cand == c) {chosen = true;}}
        if (!chosen && picks.size() >= opt.max_stands) {continue;}
        options.push_back({c, gain, chosen});
      }
      std::sort(options.begin(), options.end(), [&](const Option & a, const Option & b) {
          if (a.gain != b.gain) {return a.gain > b.gain;}
          if (a.chosen != b.chosen) {return a.chosen;}
          if (std::fabs(cell[a.cand][k].min_score - cell[b.cand][k].min_score) > 1e-12) {
            return cell[a.cand][k].min_score > cell[b.cand][k].min_score;
          }
          return a.cand < b.cand;
        });
      bool progressed = false;
      for (const Option & op : options) {
        std::vector<size_t> want;
        for (const size_t s : cell[op.cand][k].usable) {if (!covered[s]) {want.push_back(s);}}
        const ColumnPoseSequence ps = buildColumnSequence(subset(k, want), sp);
        const ColumnFeasibility f = evaluateColumn(map, ps, cands[op.cand].stand.arm_xy, heights, station.arm_mount_offset_z, opt.margin);
        if (!f.best.ends_usable) {continue;}
        StandColumn sc;
        sc.column = k;
        sc.lift_height = f.best.lift_height;
        sc.min_score = f.best.min_score;
        for (const auto & pc : f.best.poses) {
          if (pc.role == PoseRole::kStroke && pc.usable) {sc.strokes.push_back(want[pc.stroke]);}
        }
        if (sc.strokes.empty()) {continue;}
        for (const size_t s : sc.strokes) {covered[s] = true;}
        size_t pi = picks.size();
        for (size_t i = 0; i < picks.size(); ++i) {if (picks[i].cand == op.cand) {pi = i;}}
        if (pi == picks.size()) {picks.push_back(Pick{op.cand, {}});}
        picks[pi].cols.push_back(std::move(sc));
        progressed = true;
        break;
      }
      if (!progressed) {break;}
    }
  }
  // A split piece set can leave a stand with no columns only if nothing was added: drop empties.
  picks.erase(
    std::remove_if(picks.begin(), picks.end(), [](const Pick & p) {return p.cols.empty();}), picks.end());

  // 4. Order the stands.
  std::vector<size_t> order;
  {
    std::vector<size_t> left(picks.size());
    std::iota(left.begin(), left.end(), 0);
    const Vec3 u = surfaceFrame(station.wall_normal).u;
    auto along = [&](size_t i) {
        const auto & xy = cands[picks[i].cand].stand.arm_xy;
        return Vec3(xy.x(), xy.y(), 0.0).dot(u);
      };
    std::optional<Stand> at = start;
    while (!left.empty()) {
      size_t pick = 0;
      if (!at) {
        for (size_t i = 1; i < left.size(); ++i) {if (along(left[i]) < along(left[pick]) - 1e-9) {pick = i;}}
      } else {
        double best = stepCost(station, *at, cands[picks[left[0]].cand].stand);
        for (size_t i = 1; i < left.size(); ++i) {
          const double d = stepCost(station, *at, cands[picks[left[i]].cand].stand);
          if (d < best - 1e-9) {best = d; pick = i;}
        }
      }
      order.push_back(left[pick]);
      at = cands[picks[left[pick]].cand].stand;
      left.erase(left.begin() + static_cast<long>(pick));
    }
  }
  std::optional<Stand> at = start;
  for (const size_t i : order) {
    PlannedColumnStand ps;
    ps.stand = cands[picks[i].cand].stand;
    ps.standoff = cands[picks[i].cand].standoff;
    ps.columns = picks[i].cols;
    std::sort(ps.columns.begin(), ps.columns.end(), [](const StandColumn & a, const StandColumn & b) {
        return a.column < b.column;
      });
    ps.travel_cost = at ? stepCost(station, *at, ps.stand) : 0.0;
    out.total_travel += ps.travel_cost;
    at = ps.stand;
    out.stands.push_back(std::move(ps));
  }

  // 5. Optional confirmation: a refused pose shrinks what the stand does.
  if (opt.verifier) {
    out.verified = true;
    for (auto & ps : out.stands) {
      for (auto & sc : ps.columns) {
        const ColumnPoseSequence own = buildColumnSequence(subset(sc.column, sc.strokes), sp);
        const ColumnFeasibility f = evaluateColumn(map, own, ps.stand.arm_xy, {sc.lift_height}, station.arm_mount_offset_z, opt.margin);
        const ColumnFeasibility v = confirmColumnWithVerifier(*opt.verifier, opt.arm, ps.stand, own, f, opt.margin);
        std::vector<size_t> kept;
        if (v.best.ends_usable) {
          for (const auto & p : v.best.poses) {
            if (p.role == PoseRole::kStroke && p.usable) {kept.push_back(sc.strokes[p.stroke]);}
          }
        }
        const std::vector<size_t> before = sc.strokes;
        if (kept.size() != sc.strokes.size()) {out.refused.push_back({ps.stand, sc.column});}
        sc.whole = sc.whole && kept.size() == sc.strokes.size() && !kept.empty();
        sc.strokes = kept;
        sc.min_score = v.best.min_score;
        if (!v.best.poses.empty() && !sc.whole) {
          ColumnResult & r = out.columns[sc.column];
          if (v.best.limiting && !r.limited) {
            const PoseCheck & lp = v.best.poses[*v.best.limiting];
            r.limited = true;
            r.limiting_role = lp.role;
            r.limiting_stroke = lp.stroke == kNoStroke ? kNoStroke : before[lp.stroke];
            r.limiting_failure = lp.failure;
            r.limiting_score = lp.score.score;
          }
        }
      }
      ps.columns.erase(
        std::remove_if(ps.columns.begin(), ps.columns.end(), [](const StandColumn & c) {return c.strokes.empty();}),
        ps.columns.end());
    }
    // Whole columns the verifier cut are no longer assigned as whole: the tally below reads strokes.
  }

  // 6. Tally.
  for (size_t si = 0; si < out.stands.size(); ++si) {
    for (const auto & sc : out.stands[si].columns) {
      ColumnResult & r = out.columns[sc.column];
      ColumnPiece piece;
      piece.stand = si;
      piece.lift_height = sc.lift_height;
      piece.strokes = sc.strokes;
      r.strokes_covered += sc.strokes.size();
      r.pieces.push_back(std::move(piece));
    }
  }
  for (size_t k = 0; k < nc; ++k) {
    ColumnResult & r = out.columns[k];
    if (r.strokes_total == 0) {   // nothing to do counts as done
      r.status = ColumnStatus::kWhole;
      ++out.whole;
      continue;
    }
    // Several visits that together cover every stroke are a split column, not a whole one.
    const bool whole = r.pieces.size() == 1 && r.strokes_covered == r.strokes_total;
    r.status = whole ? ColumnStatus::kWhole : (r.strokes_covered > 0 ? ColumnStatus::kSplit : ColumnStatus::kUnreachable);
    if (!whole && !r.limited && have_best[k] && best_feas[k].best.limiting) {
      const PoseCheck & lp = best_feas[k].best.poses[*best_feas[k].best.limiting];
      r.limited = true;
      r.limiting_role = lp.role;
      r.limiting_stroke = lp.stroke;
      r.limiting_failure = lp.failure;
      r.limiting_score = lp.score.score;
    }
    out.strokes_covered += r.strokes_covered;
    if (r.status == ColumnStatus::kWhole) {++out.whole;}
    else if (r.status == ColumnStatus::kSplit) {++out.split;}
    else {++out.unreachable;}
  }
  out.success = out.whole == nc;
  out.message = out.success ? "every column whole" :
    (std::to_string(out.split) + " split, " + std::to_string(out.unreachable) + " unreachable");
  return out;
}

}  // namespace reachability
