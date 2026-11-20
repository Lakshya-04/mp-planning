// "How much of the plan can we do": the column plan's stands reduced to numbers and a JSON report.
//
// analyzeColumnCoverage runs planColumnStands and reports, per stand and overall, how many columns are
// whole, split (with each piece) and unreachable, the share of plan points (stroke poses) covered, the
// number of stands (and how many it would take if columns could be split freely), and for every column that
// is not whole the pose that limits it: approach, contact, a stroke or retract, and why (out of reach, low
// margin, refused by the verifier, singular, joint limit). Estimate level of the inputs: see column_stands.hpp.
#pragma once

#include <string>
#include <vector>

#include "reachability/core/column_stands.hpp"

namespace reachability
{

struct StandAnalysis
{
  size_t index{0};
  double x{0.0}, y{0.0}, yaw{0.0};
  size_t columns_whole{0};
  size_t columns_piece{0};       ///< Columns of which this stand does only a piece.
  size_t points_covered{0};
  std::vector<double> lift_heights;   ///< Distinct, ascending.
  double travel_cost{0.0};
};

struct CoverageAnalysis
{
  size_t columns_total{0}, columns_whole{0}, columns_split{0}, columns_unreachable{0};
  size_t points_total{0}, points_covered{0};
  double covered_fraction{0.0};       ///< points_covered / points_total (1 for no points).
  double whole_fraction{0.0};         ///< columns_whole / columns_total (1 for none).
  size_t stands{0};
  size_t stands_if_splitting_allowed{0};
  bool verified{false};
  std::vector<StandAnalysis> per_stand;
  ColumnStandPlan plan;               ///< The plan behind the numbers (columns carry pieces and limiting poses).
};

CoverageAnalysis analyzeColumnCoverage(
  const Map4D & map, const StationRequest & station, const std::vector<ColumnPlan> & columns,
  const std::optional<Stand> & start = std::nullopt, const ColumnStandOptions & options = {});

/// The analysis of an existing plan (no planning).
CoverageAnalysis summarizePlan(const ColumnStandPlan & plan);

/// JSON text: {"summary": {...}, "stands": [...], "columns": [...]}. Numbers rounded to 6 digits; strings escaped.
std::string toJson(const CoverageAnalysis & analysis);

}  // namespace reachability
