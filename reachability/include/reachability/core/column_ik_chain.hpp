// IK along a column's pose sequence with a continuity check.
//
// A column can be "whole" in the reachability map (every pose has a joint solution) and still have no
// continuous joint path: IK solved one pose at a time may land on a different branch (elbow flip, wrist
// flip, a lift jump) for the next pose, and the arm cannot get there without leaving the stroke. This core
// solves the poses of a ColumnPoseSequence (approach, contact, strokes, retract; see column_sequence.hpp) in
// order, each seeded from the previous solution plus a few caller-supplied "folded" seeds, and measures
// every step:
//
//   continuity ratio  max_i |dq_i| / jump        jump = Cartesian distance of the two poses (position
//                                                plus rotation_weight_m per radian, floored at min_jump_m)
//   per-joint limits  a step is continuous only when, for every joint, |dq_i| / jump is under the limit of
//                     that joint's type (continuous / revolute in rad per m, prismatic lift in m per m);
//                     the dq of a continuous joint is wrapped to [-pi, pi]
//   branch signature  the signs of `branch_joints` (a joint within `branch_deadband_rad` of zero counts as
//                     either sign); a signature change is a branch change and is also a break
//
// A step whose seeded solution is not continuous is retried from the other seeds (a "reseed"); the lowest
// normalised ratio among the continuous ones wins. When none is continuous the best available solution is
// kept, the step is flagged PoseFailure::kBranchBreak and counted in `split_count`: the column would have
// to be run in pieces there. A pose with no IK solution at all stops the chain (kRefused).
//
// The IK is injected (`IkSolveFn`), so the core has no ROS, MoveIt or arm model dependency and is tested
// with a fake. Not here (planned): changing the lift height or the stand on a break, re-ordering columns,
// the manager gate and the execution-time split count. Everything here is a property of the IK callable
// given, not of a real arm until an adapter supplies it.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <vector>

#include "reachability/core/column_sequence.hpp"

namespace reachability
{

using JointVec = Eigen::VectorXd;

enum class JointKind
{
  kContinuous,   ///< Unbounded revolute: dq wrapped to [-pi, pi], limit in rad per m.
  kRevolute,     ///< Limited revolute: limit in rad per m.
  kPrismatic,    ///< Prismatic (the lift): limit in m per m; its travel is summed in `lift_travel_m`.
};

/// Joint solution for `target` starting the search at `seed`, or nullopt when there is none. Must return
/// a vector of the chain's size.
using IkSolveFn = std::function<std::optional<JointVec>(const Pose & target, const JointVec & seed)>;

struct ColumnIkChainParams
{
  std::vector<JointKind> joint_kinds;   ///< One per joint; its size is the chain's size.
  double ratio_limit_continuous{6.0};   ///< rad per m of Cartesian jump.
  double ratio_limit_revolute{6.0};     ///< rad per m.
  double ratio_limit_prismatic{2.0};    ///< m per m.
  double rotation_weight_m{0.1};        ///< Metres of jump per radian of tool rotation.
  double min_jump_m{1e-3};              ///< Floor of the jump, so near-identical poses do not blow up the ratio.
  std::vector<size_t> branch_joints;    ///< Joints whose sign is the branch signature (e.g. the elbow). Empty: not tracked.
  double branch_deadband_rad{0.05};
  JointVec initial_seed;                ///< Seed of the first pose; empty = zeros.
  std::vector<JointVec> folded_seeds;   ///< Extra seeds tried when the previous-solution seed breaks (or fails).
};

struct ChainStep
{
  size_t pose{0};                 ///< Index into the sequence.
  bool solved{false};
  JointVec q;                     ///< The accepted solution (empty when not solved).
  double jump_m{0.0};             ///< Distance from the previous pose (0 for the first).
  double ratio{0.0};              ///< max_i |dq_i| / jump (raw, rad/m and m/m mixed as the joints are).
  double normalized_ratio{0.0};   ///< max_i (|dq_i| / jump) / limit of its type; over 1 is a break.
  size_t worst_joint{0};          ///< The joint with the largest normalised ratio.
  bool branch_changed{false};     ///< The signature differs from the previous step's.
  bool reseeded{false};           ///< The previous-solution seed was not used for this step's solution.
  bool branch_break{false};       ///< The accepted solution is not continuous with the previous one.
  PoseFailure failure{PoseFailure::kNone};   ///< kBranchBreak, kRefused, or kNone.
};

struct ColumnIkChainResult
{
  bool complete{false};           ///< Every pose has a solution.
  std::vector<ChainStep> steps;   ///< One per pose up to and including the one that stopped the chain.
  size_t split_count{0};          ///< Steps flagged kBranchBreak: the column breaks into split_count + 1 pieces.
  size_t branch_changes{0};       ///< Steps whose branch signature changed.
  size_t reseeds{0};              ///< Steps rescued or changed by a seed other than the previous solution.
  double max_continuity_ratio{0.0};   ///< Largest raw ratio over the steps.
  double lift_travel_m{0.0};      ///< Sum of |dq| of the prismatic joints.
  PoseFailure failure{PoseFailure::kNone};   ///< First failure (kRefused or kBranchBreak), else kNone.
  std::optional<size_t> failure_pose;        ///< Its pose index.
  /// Solved, no IK failure and no break.
  bool continuous() const {return complete && split_count == 0;}
};

class ColumnIkChain
{
public:
  ColumnIkChain(ColumnIkChainParams params, IkSolveFn ik);

  /// Solves `poses` in order (a ColumnPoseSequence's `poses`).
  ColumnIkChainResult solve(const std::vector<Pose> & poses) const;
  ColumnIkChainResult solve(const ColumnPoseSequence & seq) const {return solve(seq.poses);}

  /// Signs of the branch joints of `q` (+1, -1, 0 inside the deadband), empty without branch joints.
  std::vector<int8_t> branchSignature(const JointVec & q) const;

  const ColumnIkChainParams & params() const {return params_;}

private:
  ColumnIkChainParams params_;
  IkSolveFn ik_;
};

/// Two signatures differ when some entry is non-zero in both and the signs are opposite.
bool signaturesDiffer(const std::vector<int8_t> & a, const std::vector<int8_t> & b);

}  // namespace reachability
