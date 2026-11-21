/**
 * @file base_placement.hpp
 * @brief Base placement optimiser (BPO): where the base stands, decided from the reachability of the WHOLE
 *        column sequence (approach, contact, strokes, retract), not from single targets.
 * @details planBasePlacement composes the existing pieces instead of reimplementing them:
 *          - the candidate stands of the surface frame (`makeCandidateStands`, footprint, work-plane clearance
 *            and the FreeSpace of obstacles and other walls),
 *          - a navigation filter: a `NavFeasibility` (grid connectivity on a footprint-inflated free space, a
 *            port a Nav2 costmap can replace) from the start pose to each candidate, and a yaw window
 *            (a diff-drive base prefers to face the surface; an omni base is free). A rejected candidate is
 *            never offered to the cover, and the reason is kept,
 *          - `planColumnStands` (whole-column weighted set cover over the candidates; escalation: another lift
 *            height of the same column, another stand that does it whole, and only then a split, reported),
 *          - the verifier loop of `planStandsVerified`: a stand whose (stand, column) pair the verifier cut is
 *            avoided and the columns are planned again (bounded rounds), the best round is kept.
 *          The result lists, per column, whole / split / unreachable with the limiting pose and a stable reason
 *          code, the stands in visiting order with their yaw, the covered fraction, the stand count against
 *          the exact-search minimum when the problem is small, the splits and the travel.
 *
 *          RM4D level: no IK, no collision of the arm or tool against adjacent walls, 2D obstacles, no
 *          costmap. Deterministic. See docs/reachability.md, "Base placement optimiser".
 *
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "reachability/core/column_stands.hpp"
#include "reachability/core/free_space.hpp"
#include "reachability/core/surface_frame.hpp"

namespace reachability
{

// ---------------------------------------------------------------- navigation feasibility port

enum class NavReason
{
  kOk,
  kStartBlocked,    ///< The start pose is in an inflated obstacle (or outside the grid): nothing is reachable.
  kGoalBlocked,     ///< The goal is in an inflated obstacle.
  kDisconnected,    ///< Free, but no free path from the start.
  kOutsideMap,      ///< The goal is outside the navigation grid.
};

const char * toString(NavReason reason);

struct NavVerdict
{
  bool ok{false};
  NavReason reason{NavReason::kDisconnected};
};

/// The port. A Nav2 costmap implementation can replace the grid one without touching the planner.
class NavFeasibility
{
public:
  virtual ~NavFeasibility() = default;
  /// Can the base drive from `start` to `goal`? Poses are the arm base position and yaw of a Stand.
  virtual NavVerdict feasible(const Stand & start, const Stand & goal) const = 0;
};

struct GridNavParams
{
  double resolution{0.1};        ///< Cell size (m).
  double margin{0.05};           ///< Added to the footprint radius when inflating.
  /// Goal and start may sit this much inside an inflated obstacle (m) and still connect: the circle
  /// inflation is bigger than a rectangular footprint that was already tested for the stand. Negative =
  /// the circumscribed minus the inscribed radius plus one cell.
  double snap_m{-1.0};
};

/// Connectivity on a grid over [lo, hi]: a cell is free when a disc of the footprint's circumscribed radius
/// (plus margin) around it is clear of `free_space`; 8-connected flood from the start. Not thread-safe
/// (caches the flood of the last start).
class GridNavFeasibility : public NavFeasibility
{
public:
  GridNavFeasibility(
    const FreeSpace & free_space, const std::optional<BaseModel> & base, const Eigen::Vector2d & lo,
    const Eigen::Vector2d & hi, const GridNavParams & params = {});
  NavVerdict feasible(const Stand & start, const Stand & goal) const override;
  size_t cells() const {return nx_ * ny_;}

private:
  bool inside(const Eigen::Vector2d & p) const;
  Eigen::Vector2d centreOf(const Stand & s) const;
  void flood(const Eigen::Vector2d & start_c) const;
  bool nearComponent(const Eigen::Vector2d & p) const;

  std::optional<BaseModel> base_;
  Eigen::Vector2d lo_;
  double res_{0.1};
  double snap_{0.1};
  size_t nx_{0}, ny_{0};
  std::vector<unsigned char> free_;
  mutable std::vector<unsigned char> reached_;
  mutable bool have_flood_{false};
  mutable Eigen::Vector2d flood_start_{0.0, 0.0};
};

// ---------------------------------------------------------------- planner

/// Stable codes: why a column is not whole (or "ok").
enum class BpoReason
{
  kOk,                    ///< Whole at one stand.
  kSplitNoWholeStand,     ///< No candidate does it whole; split on purpose (reported).
  kSkippedToAvoidSplit,   ///< allow_splits = false and no whole stand: left out.
  kOutOfReach,            ///< The limiting pose has no map cell at any usable stand.
  kLowMargin,             ///< Reachable but under the margin threshold.
  kRefusedByVerifier,     ///< The verifier refused the limiting pose.
  kNearSingular,
  kNearJointLimit,
  kUnreachableByNav,      ///< A candidate that reaches it was removed: the base cannot drive there.
  kYawWindow,             ///< ... or no yaw inside the yaw window fits the footprint.
  kNoFreeStand,           ///< No candidate stand is left (clearance, footprint, free space).
  kMaxStands,             ///< A whole stand exists but the stand cap is reached.
};

const char * toString(BpoReason reason);

/// What the final plan needed for the column: the cheapest step at which a whole column became whole, or
/// kSplit / kSkipped / kUnreachable when the final status is not whole (also when the stand cap or the
/// verifier cut a column that a whole stand could serve; BpoReason says why).
enum class BpoEscalation
{
  kNone,          ///< Whole at the stand nearest to it, at the nominal lift height.
  kLiftHeight,    ///< Whole there after changing the lift height.
  kOtherStand,    ///< Whole at some other stand.
  kSplit,         ///< Not whole anywhere: split on purpose.
  kSkipped,       ///< Not whole anywhere and left out (allow_splits = false).
  kUnreachable,   ///< No stand reaches it.
};

const char * toString(BpoEscalation escalation);

struct BasePlacementOptions
{
  /// Search grid, lift range, base model, margins. `targets_world`, `wall_point`, `wall_normal`,
  /// `free_space` are set from the arguments of planBasePlacement; `base` and `free_space_margin` are used.
  StationRequest search;
  MarginConfig margin;
  SequenceParams sequence;       ///< `wall_normal` is overwritten by the frame.
  double margin_penalty{0.5};
  bool allow_splits{true};
  size_t max_stands{32};
  StandVerifier * verifier{nullptr};
  const ArmModel * arm{nullptr};
  size_t verifier_rounds{4};     ///< Re-plans after the verifier cut a (stand, column) pair.

  /// Navigation: a feasibility port (null = the grid one over the free space plus the work wall). Needs a
  /// start pose; without one nothing is filtered.
  const NavFeasibility * nav{nullptr};
  bool check_navigation{true};
  GridNavParams nav_grid;
  double nav_padding_m{1.5};

  /// Half-width (rad) around "facing the surface" the base yaw must stay in. Negative = by drive type: free
  /// (pi) for a base that can strafe, `diff_yaw_window_rad` otherwise. >= pi = no restriction.
  double yaw_window_rad{-1.0};
  double diff_yaw_window_rad{0.7};
  int yaw_steps{24};

  /// Exact whole-cover search (the lower bound on the stand count): runs when the whole-coverable columns
  /// number at most `exact_max_columns` (<= 62); node budget in `exact_node_budget`.
  size_t exact_max_columns{24};
  size_t exact_node_budget{200000};
};

struct BpoColumn
{
  size_t column{0};
  std::string label;
  ColumnStatus status{ColumnStatus::kUnreachable};
  BpoReason reason{BpoReason::kNoFreeStand};
  BpoEscalation escalation{BpoEscalation::kUnreachable};
  size_t strokes_total{0};
  size_t strokes_covered{0};
  std::vector<ColumnPiece> pieces;   ///< `stand` indexes BasePlacement::stands.
  bool limited{false};               ///< The limiting pose below is set (a column that is not whole).
  PoseRole limiting_role{PoseRole::kStroke};
  size_t limiting_stroke{kNoStroke};
  PoseFailure limiting_failure{PoseFailure::kNone};
  double limiting_score{0.0};
};

struct BpoStand
{
  Stand stand;                       ///< Arm base position and the yaw inside the window.
  double standoff{0.0};
  std::vector<StandColumn> columns;
  double travel_cost{0.0};           ///< From the previous stop (travelTime with a base model, else metres).
  double travel_m{0.0};              ///< Straight distance from the previous stop (or the start).
  double clearance_m{0.0};           ///< Footprint to the nearest obstacle / wall (capped at 2 m).
};

struct BpoRejected
{
  Stand stand;
  BpoReason reason{BpoReason::kNoFreeStand};   ///< kUnreachableByNav or kYawWindow.
  NavReason nav{NavReason::kOk};
};

struct BasePlacement
{
  bool success{false};               ///< Every column whole.
  std::string message;
  std::vector<BpoStand> stands;      ///< Visiting order.
  std::vector<BpoColumn> columns;    ///< Input order.
  size_t whole{0}, split{0}, unreachable{0};
  size_t strokes_total{0}, strokes_covered{0};
  double covered_fraction{0.0};      ///< strokes_covered / strokes_total (1 for none).
  /// Strokes usable at some navigable stand whose column's end poses are usable there: what the arm and the
  /// navigable stands allow at best (one stand per stroke, no stand cap).
  size_t reachable_strokes{0};
  double reachable_fraction{0.0};    ///< reachable_strokes / strokes_total.
  size_t stand_count{0};
  std::optional<size_t> exact_min_stands;   ///< Fewest stands covering every whole-coverable column whole.
  size_t whole_coverable{0};                ///< Columns some stand does whole.
  size_t split_count{0};
  double travel_cost{0.0};
  double travel_m{0.0};
  double min_clearance_m{0.0};
  size_t candidates{0};              ///< After the footprint filter, before navigation and yaw.
  size_t nav_rejected{0}, yaw_rejected{0};
  std::vector<BpoRejected> rejected;
  std::vector<Stand> refused_stands; ///< Stands avoided after the verifier cut a column.
  size_t rounds{1};
  bool verified{false};
  size_t stands_if_splitting_allowed{0};
};

/// Plans the base placement for `columns` on the surface `frame` (`frame.origin` is a point on it, `frame.n`
/// points into the free side). `free_space` holds obstacles and the other walls; `start` is where the base
/// is now (navigation and order). Deterministic. An invalid frame or an empty grid gives success = false and
/// a message; no column is dropped silently.
BasePlacement planBasePlacement(
  const Map4D & map, const SurfaceFrame & frame, const std::vector<ColumnPlan> & columns,
  const FreeSpace & free_space, const std::optional<Stand> & start, const BasePlacementOptions & options = {});

}  // namespace reachability
