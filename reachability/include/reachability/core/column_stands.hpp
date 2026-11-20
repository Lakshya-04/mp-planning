// Stands for a plan made of columns, keeping each column whole where the arm can.
//
// planStands / selectStands cover target poses one by one: a column of strokes can end up shared by two
// stands (the first strokes from one, the rest from the next), and nothing checks the approach, the
// contact pose or the retract. planColumnStands works per column:
//   1. every candidate stand (the grid of makeCandidateStands) is evaluated per column with
//      evaluateColumn: the whole pose sequence (approach, contact, strokes, retract) at one lift height,
//      scored against MarginConfig. Per (stand, column) the result is whole, split or unreachable;
//   2. a weighted greedy set cover over WHOLE columns picks the stands (cost 1 + margin_penalty * (1 - mean
//      margin of the columns it newly takes); redundant stands are dropped). A column is only ever
//      broken up when no candidate stand does it whole;
//   3. those leftover columns are split on purpose and reported: their strokes are cut into the fewest
//      pieces by greedy cover over the stands whose end poses are usable (a piece needs its own approach and
//      retract), preferring stands already chosen; each split is listed with its pieces. With
//      allow_splits = false they are left out instead (reason "would_split");
//   4. the stands are put in an order (travelTime with a base model, else distance) from `start`.
// ColumnStandPlan::stands_if_splitting_allowed is the stand count of the same candidates when strokes may
// be taken one by one (no column kept whole, end poses ignored): the price of keeping columns whole.
// Optional: a StandVerifier re-checks each chosen (stand, column) and the totals follow (no re-planning:
// a refused column is reported, not moved to another stand).
//
// Estimates: RM4D has no IK or collision, so "whole" means "the map says so"; see docs/reachability.md.
#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "reachability/core/column_sequence.hpp"
#include "reachability/core/station.hpp"

namespace reachability
{

struct ColumnPlan
{
  std::string label;
  std::vector<Pose> strokes_world;   ///< The column's plan poses in stroke order (tool z = approach axis).
};

struct ColumnStandOptions
{
  MarginConfig margin;
  SequenceParams sequence;       ///< `wall_normal` is overwritten by the station's.
  double margin_penalty{0.5};    ///< >= 0. 0 = plain greedy by column count.
  bool allow_splits{true};
  size_t max_stands{32};
  StandVerifier * verifier{nullptr};   ///< Optional confirmation of the chosen (stand, column) pairs.
  const ArmModel * arm{nullptr};       ///< With a verifier: singular value and joint-limit margins of its solutions.
};

/// One column visit from a stand: its lift height and the strokes done in it.
struct StandColumn
{
  size_t column{0};
  double lift_height{0.0};
  std::vector<size_t> strokes;   ///< Indices into the column's strokes (all of them when `whole`).
  bool whole{false};
  double min_score{0.0};
};

struct PlannedColumnStand
{
  Stand stand;
  double standoff{0.0};
  std::vector<StandColumn> columns;   ///< Ascending column index.
  double travel_cost{0.0};            ///< From the previous stop (or from `start`).
};

struct ColumnPiece
{
  size_t stand{0};                    ///< Index into ColumnStandPlan::stands.
  double lift_height{0.0};
  std::vector<size_t> strokes;
};

struct ColumnResult
{
  size_t column{0};
  std::string label;
  ColumnStatus status{ColumnStatus::kUnreachable};   ///< kWhole: one stand does all of it.
  size_t strokes_total{0};
  size_t strokes_covered{0};
  std::vector<ColumnPiece> pieces;    ///< One for a whole column; the cut for a split one.
  bool skipped_to_avoid_split{false};
  /// Why the column is not whole (the pose of the best-evaluated stand that limits it); unset for a whole column.
  bool limited{false};
  PoseRole limiting_role{PoseRole::kStroke};
  size_t limiting_stroke{kNoStroke};
  PoseFailure limiting_failure{PoseFailure::kNone};
  double limiting_score{0.0};
};

struct ColumnStandPlan
{
  bool success{false};                ///< Every column whole.
  std::string message;
  std::vector<PlannedColumnStand> stands;   ///< In visiting order.
  std::vector<ColumnResult> columns;        ///< One per input column, in input order.
  size_t whole{0}, split{0}, unreachable{0};
  size_t strokes_total{0}, strokes_covered{0};
  size_t candidates{0};
  size_t stands_if_splitting_allowed{0};
  bool verified{false};
  double total_travel{0.0};
};

/// Plans the stands. `start` is where the base is now (with request.station.base it drives the order by
/// travelTime). Deterministic. An invalid grid or empty plan gives success = false and a message.
ColumnStandPlan planColumnStands(
  const Map4D & map, const StationRequest & station, const std::vector<ColumnPlan> & columns,
  const std::optional<Stand> & start = std::nullopt, const ColumnStandOptions & options = {});

}  // namespace reachability
