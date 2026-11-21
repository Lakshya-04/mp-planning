#include "reachability/core/column_ik_chain.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/Geometry>

namespace reachability
{

namespace
{
double limitOf(const ColumnIkChainParams & p, JointKind k)
{
  switch (k) {
    case JointKind::kContinuous: return p.ratio_limit_continuous;
    case JointKind::kRevolute: return p.ratio_limit_revolute;
    case JointKind::kPrismatic: return p.ratio_limit_prismatic;
  }
  return p.ratio_limit_revolute;
}

double poseJump(const Pose & a, const Pose & b, const ColumnIkChainParams & p)
{
  const double dp = (b.translation() - a.translation()).norm();
  const Eigen::AngleAxisd rel(a.linear().transpose() * b.linear());
  const double jump = dp + p.rotation_weight_m * std::abs(rel.angle());
  return std::max(jump, p.min_jump_m);
}

struct Candidate
{
  JointVec q;
  double ratio{0.0};
  double normalized{0.0};
  size_t worst{0};
  double lift{0.0};
  bool signature_differs{false};
  bool ok() const {return normalized <= 1.0 && !signature_differs;}
};
}  // namespace

bool signaturesDiffer(const std::vector<int8_t> & a, const std::vector<int8_t> & b)
{
  const size_t n = std::min(a.size(), b.size());
  for (size_t i = 0; i < n; ++i) {
    if (a[i] != 0 && b[i] != 0 && a[i] != b[i]) {return true;}
  }
  return false;
}

ColumnIkChain::ColumnIkChain(ColumnIkChainParams params, IkSolveFn ik)
: params_(std::move(params)), ik_(std::move(ik)) {}

std::vector<int8_t> ColumnIkChain::branchSignature(const JointVec & q) const
{
  std::vector<int8_t> sig;
  sig.reserve(params_.branch_joints.size());
  for (const size_t j : params_.branch_joints) {
    if (static_cast<Eigen::Index>(j) >= q.size()) {sig.push_back(0); continue;}
    const double v = q[static_cast<Eigen::Index>(j)];
    sig.push_back(std::abs(v) <= params_.branch_deadband_rad ? 0 : (v > 0 ? 1 : -1));
  }
  return sig;
}

ColumnIkChainResult ColumnIkChain::solve(const std::vector<Pose> & poses) const
{
  ColumnIkChainResult out;
  const Eigen::Index dof = static_cast<Eigen::Index>(params_.joint_kinds.size());
  if (poses.empty() || dof == 0 || !ik_) {return out;}

  const JointVec zero = JointVec::Zero(dof);
  const JointVec start = params_.initial_seed.size() == dof ? params_.initial_seed : zero;

  // Calls the IK and drops answers of the wrong size or with non-finite values.
  auto solveFrom = [&](const Pose & target, const JointVec & seed) -> std::optional<JointVec> {
      auto q = ik_(target, seed);
      if (!q || q->size() != dof || !q->allFinite()) {return std::nullopt;}
      return q;
    };

  // Measures the step from `prev` (at `prev_pose`) to `q` at `pose`.
  auto measure = [&](const JointVec & prev, const Pose & prev_pose, const JointVec & q, const Pose & pose) {
      Candidate c;
      c.q = q;
      const double jump = poseJump(prev_pose, pose, params_);
      for (Eigen::Index i = 0; i < dof; ++i) {
        const JointKind kind = params_.joint_kinds[static_cast<size_t>(i)];
        double dq = q[i] - prev[i];
        if (kind == JointKind::kContinuous) {dq = std::remainder(dq, 2.0 * M_PI);}
        dq = std::abs(dq);
        if (kind == JointKind::kPrismatic) {c.lift += dq;}
        const double r = dq / jump;
        c.ratio = std::max(c.ratio, r);
        const double lim = limitOf(params_, kind);
        const double n = lim > 0.0 ? r / lim : (r > 0.0 ? std::numeric_limits<double>::infinity() : 0.0);
        if (n > c.normalized) {c.normalized = n; c.worst = static_cast<size_t>(i);}
      }
      c.signature_differs = signaturesDiffer(branchSignature(prev), branchSignature(q));
      return c;
    };

  JointVec prev;
  for (size_t i = 0; i < poses.size(); ++i) {
    ChainStep step;
    step.pose = i;

    if (i == 0) {
      // First pose: the initial seed, then the folded ones; the first solution wins.
      std::optional<JointVec> q = solveFrom(poses[0], start);
      for (size_t s = 0; !q && s < params_.folded_seeds.size(); ++s) {
        if (params_.folded_seeds[s].size() != dof) {continue;}
        q = solveFrom(poses[0], params_.folded_seeds[s]);
        step.reseeded = q.has_value();
      }
      if (!q) {
        step.failure = PoseFailure::kRefused;
        out.failure = step.failure;
        out.failure_pose = i;
        out.steps.push_back(std::move(step));
        return out;
      }
      step.solved = true;
      step.q = *q;
      if (step.reseeded) {++out.reseeds;}
      prev = *q;
      out.steps.push_back(std::move(step));
      continue;
    }

    // Later poses: the previous solution first, then the folded seeds.
    std::vector<Candidate> cands;
    std::vector<bool> from_prev;
    auto tryOne = [&](const JointVec & seed, bool is_prev) {
        if (auto q = solveFrom(poses[i], seed)) {
          cands.push_back(measure(prev, poses[i - 1], *q, poses[i]));
          from_prev.push_back(is_prev);
        }
      };
    tryOne(prev, true);
    if (cands.empty() || !cands.front().ok()) {
      for (const auto & seed : params_.folded_seeds) {
        if (seed.size() == dof) {tryOne(seed, false);}
      }
    }
    if (cands.empty()) {
      step.failure = PoseFailure::kRefused;
      out.failure = step.failure;
      out.failure_pose = i;
      out.steps.push_back(std::move(step));
      return out;
    }

    // The continuous candidate with the smallest ratio (the previous-seed one when it is continuous);
    // none continuous: the smallest ratio overall.
    size_t best = 0;
    for (size_t c = 1; c < cands.size(); ++c) {
      const bool better_class = cands[c].ok() && !cands[best].ok();
      const bool same_class = cands[c].ok() == cands[best].ok();
      if (better_class || (same_class && cands[c].normalized < cands[best].normalized)) {best = c;}
    }
    if (cands.front().ok()) {best = 0;}

    const Candidate & c = cands[best];
    step.solved = true;
    step.q = c.q;
    step.jump_m = poseJump(poses[i - 1], poses[i], params_);
    step.ratio = c.ratio;
    step.normalized_ratio = c.normalized;
    step.worst_joint = c.worst;
    step.branch_changed = c.signature_differs;
    step.reseeded = !from_prev[best];
    step.branch_break = !c.ok();
    if (step.branch_break) {
      step.failure = PoseFailure::kBranchBreak;
      ++out.split_count;
      if (out.failure == PoseFailure::kNone) {
        out.failure = step.failure;
        out.failure_pose = i;
      }
    }
    if (step.branch_changed) {++out.branch_changes;}
    if (step.reseeded) {++out.reseeds;}
    out.max_continuity_ratio = std::max(out.max_continuity_ratio, c.ratio);
    out.lift_travel_m += c.lift;
    prev = c.q;
    out.steps.push_back(std::move(step));
  }
  out.complete = true;
  return out;
}

}  // namespace reachability
