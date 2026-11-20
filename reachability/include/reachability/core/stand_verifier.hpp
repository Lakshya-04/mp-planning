// Stand verification: confirm what the RM4D map promised with inverse kinematics and collision checking.
//
// RM4D is a cheap prefilter. It ignores self-collision and obstacles and only says a tool pose is
// kinematically reachable from some base position. A StandVerifier is the port that answers the real
// question for one stand: does a collision-free joint solution exist for this tool pose, with the arm base
// at the stand and the lift at a given height? The core defines the port and the bookkeeping (verifyStand);
// implementations live next to the IK / collision backends (reachability_moveit: MoveIt through the
// motion_planner IkPort / CollisionPort). FunctionStandVerifier is the fake for unit tests.
//
// Results are typed (VerifyReason) so a refusal is never just "false": ik_failed, in_collision,
// out_of_limits, or error (the backend could not answer).
#pragma once

#include <cstddef>
#include <functional>
#include <vector>

#include "reachability/core/base_model.hpp"
#include "reachability/core/lift_bands.hpp"
#include "reachability/core/map4d.hpp"

namespace reachability
{

enum class VerifyReason
{
  kOk,            ///< A collision-free solution within the limits exists.
  kIkFailed,      ///< The solver found no solution.
  kInCollision,   ///< A solution exists but every one tried is in self-collision or hits the scene.
  kOutOfLimits,   ///< The solution violates a joint or lift limit.
  kError,         ///< The backend could not answer (bad request, no scene, no service).
};

const char * toString(VerifyReason reason);

struct TargetVerdict
{
  VerifyReason reason{VerifyReason::kIkFailed};
  std::vector<double> joints;   ///< The arm joint solution (backend order) when reason == kOk.
  bool ok() const {return reason == VerifyReason::kOk;}
};

/// The port. `stand.arm_xy` is the arm base position and `stand.yaw` the base yaw, `lift_height` the lift
/// position (the arm base is at lift_height + the mount offset above the floor), `targets_world` tool poses
/// (tool z = approach axis) in the world frame. One verdict per target, same order. The scene (obstacles,
/// wall) belongs to the implementation, which must pose it relative to the stand.
class StandVerifier
{
public:
  virtual ~StandVerifier() = default;
  virtual std::vector<TargetVerdict> verify(
    const Stand & stand, double lift_height, const std::vector<Pose> & targets_world) = 0;
};

/// A verifier from a function of one target: the fake for tests, and a thin wrapper for per-pose backends.
class FunctionStandVerifier : public StandVerifier
{
public:
  using Fn = std::function<TargetVerdict(const Stand &, double, const Pose &)>;
  explicit FunctionStandVerifier(Fn fn)
  : fn_(std::move(fn)) {}
  std::vector<TargetVerdict> verify(
    const Stand & stand, double lift_height, const std::vector<Pose> & targets_world) override
  {
    ++calls_;
    std::vector<TargetVerdict> out;
    out.reserve(targets_world.size());
    for (const auto & t : targets_world) {
      out.push_back(fn_(stand, lift_height, t));
    }
    return out;
  }
  size_t calls() const {return calls_;}   ///< verify() calls so far (batches, not poses).

private:
  Fn fn_;
  size_t calls_{0};
};

struct VerifyOptions
{
  /// Lift heights tried per target, at most (evenly picked from the heights RM4D says reach it). >= 1.
  size_t max_lift_probes{4};
  /// Base offset (m) for the margin: the best height is re-tried with the base shifted by +-jitter_m along and
  /// across its heading (4 probes). 0 = no offset probes (offset margin 1).
  double jitter_m{0.0};
  /// selectStands only: verify, re-rank and re-cover at most this many rounds. A round verifies the stands the
  /// current cover chose, so a wall whose every candidate is refused needs about one round per candidate.
  size_t max_rounds{64};
  /// Try each target's heights lowest first and stop at the first that solves, instead of probing all of them.
  /// The confirmed set and each target's best height (the lowest ok one) are unchanged; the cost is that
  /// `height_ok` is 1 and `lift_margin` = 1 / (heights tried) (a lowest-first ordinal, not a share of ok
  /// heights), and with `jitter_m` > 0 the offset probes only run at that first ok height. Refused targets
  /// are still probed at every height. Default false (every probe is made).
  bool stop_at_first_ok{false};
  /// Upper bound on verifier batches (`verify()` calls) one selectStands / planStandsVerified run may make;
  /// 0 = unlimited. When reached, no further stand is verified and the result says `converged = false`.
  size_t max_verifier_calls{0};
};

struct TargetReport
{
  size_t target{0};                       ///< Index into the caller's target list.
  bool confirmed{false};                  ///< At least one lift height gives a valid solution at the stand itself.
  VerifyReason reason{VerifyReason::kIkFailed};   ///< kOk when confirmed, else the failure that got furthest
                                                  ///< (in_collision > out_of_limits > ik_failed > error).
  double best_height{0.0};                ///< Lift height of `joints` (confirmed only).
  std::vector<double> joints;             ///< The solution at best_height (confirmed only).
  size_t height_probes{0};                ///< Lift heights tried (with stop_at_first_ok: up to and including the first ok).
  size_t height_ok{0};                    ///< ... that solved.
  double lift_margin{0.0};                ///< height_ok / height_probes.
  double offset_margin{0.0};              ///< Share of the jittered base offsets (at best_height) that solve; 1 with no jitter.
  double margin{0.0};                     ///< lift_margin * offset_margin, in [0, 1]; > 0 exactly when confirmed.
};

struct StandVerification
{
  Stand stand;
  std::vector<TargetReport> targets;      ///< In the order of the request's indices.
  size_t confirmed{0};
  double coverage{0.0};                   ///< confirmed / targets (1 for none).
  double mean_margin{0.0};                ///< Mean margin of the confirmed targets (0 when none).
  std::vector<LiftBand> bands;            ///< Greedy lift bands over the confirmed (target, height) pairs.
  size_t verifier_calls{0};              ///< verify() batches sent.
  size_t verifier_poses{0};              ///< Target poses sent in those batches (the real IK cost).
};

/// Verifies `indices` (into targets_world) at `stand`. `heights[i]` are the lift heights to try for
/// `indices[i]` (typically those RM4D reaches, see verifiedHeights); an empty list makes that target
/// unconfirmed with ik_failed. Deterministic. Targets are batched per height (and per jitter offset).
StandVerification verifyStand(
  StandVerifier & verifier, const Stand & stand, const std::vector<Pose> & targets_world,
  const std::vector<size_t> & indices, const std::vector<std::vector<double>> & heights,
  const VerifyOptions & options = {});

/// At most `max_probes` heights picked evenly (both ends kept) from `heights`.
std::vector<double> pickProbeHeights(const std::vector<double> & heights, size_t max_probes);

}  // namespace reachability
