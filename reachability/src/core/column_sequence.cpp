#include "reachability/core/column_sequence.hpp"

#include <algorithm>
#include <cmath>

namespace reachability
{

const char * toString(PoseRole r)
{
  switch (r) {
    case PoseRole::kApproach: return "approach";
    case PoseRole::kContact: return "contact";
    case PoseRole::kStroke: return "stroke";
    case PoseRole::kRetract: return "retract";
  }
  return "stroke";
}

const char * toString(PoseFailure f)
{
  switch (f) {
    case PoseFailure::kNone: return "none";
    case PoseFailure::kOutOfReach: return "out_of_reach";
    case PoseFailure::kLowMargin: return "low_margin";
    case PoseFailure::kRefused: return "refused";
    case PoseFailure::kSingular: return "singular";
    case PoseFailure::kJointLimit: return "joint_limit";
    case PoseFailure::kBranchBreak: return "branch_break";
  }
  return "none";
}

const char * toString(ColumnStatus s)
{
  switch (s) {
    case ColumnStatus::kWhole: return "whole";
    case ColumnStatus::kSplit: return "split";
    case ColumnStatus::kUnreachable: return "unreachable";
  }
  return "unreachable";
}

namespace
{
Pose shifted(const Pose & p, const Vec3 & n, double d)
{
  Pose q = p;
  q.translation() += n * d;
  return q;
}
}  // namespace

ColumnPoseSequence buildColumnSequence(const std::vector<Pose> & strokes, const SequenceParams & params)
{
  ColumnPoseSequence seq;
  if (strokes.empty()) {return seq;}
  Vec3 n = params.wall_normal;
  n = n.norm() > 1e-9 ? Vec3(n.normalized()) : Vec3(-Vec3::UnitX());
  const size_t dock = params.dock_at_first_stroke ? 0 : strokes.size() / 2;

  auto add = [&](const Pose & p, PoseRole role, size_t stroke) {
      seq.poses.push_back(p);
      seq.roles.push_back(role);
      seq.stroke_index.push_back(stroke);
    };
  const double out = std::max(0.0, params.standoff_m) + std::max(0.0, params.approach_extra_m);
  if (out > 1e-9) {add(shifted(strokes[dock], n, out), PoseRole::kApproach, kNoStroke);}
  if (params.press_depth_m > 1e-9) {
    add(shifted(strokes[dock], n, -params.press_depth_m), PoseRole::kContact, kNoStroke);
  }
  for (size_t i = 0; i < strokes.size(); ++i) {add(strokes[i], PoseRole::kStroke, i);}
  const double back = params.retract_m < 0.0 ? out : params.retract_m;
  if (back > 1e-9) {add(shifted(strokes.back(), n, back), PoseRole::kRetract, kNoStroke);}
  seq.strokes = strokes.size();
  return seq;
}

void summarize(SequenceCheck * c)
{
  c->strokes_total = 0;
  c->strokes_usable = 0;
  c->ends_usable = true;
  c->min_score = 0.0;
  c->limiting.reset();
  bool any = false;
  std::optional<size_t> first_end, first_stroke, lowest;
  for (size_t i = 0; i < c->poses.size(); ++i) {
    const PoseCheck & p = c->poses[i];
    const bool stroke = p.role == PoseRole::kStroke;
    if (stroke) {
      ++c->strokes_total;
      if (p.usable) {++c->strokes_usable;}
    }
    if (p.usable) {
      c->min_score = any ? std::min(c->min_score, p.score.score) : p.score.score;
      any = true;
      if (!lowest || p.score.score < c->poses[*lowest].score.score) {lowest = i;}
    } else if (stroke) {
      if (!first_stroke) {first_stroke = i;}
    } else {
      c->ends_usable = false;
      if (!first_end) {first_end = i;}
    }
  }
  c->limiting = first_end ? first_end : (first_stroke ? first_stroke : lowest);
}

SequenceCheck checkSequenceAt(
  const Map4D & map, const ColumnPoseSequence & seq, const Eigen::Vector2d & base_xy, double lift_height,
  double arm_mount_offset_z, const MarginConfig & margin)
{
  SequenceCheck c;
  c.lift_height = lift_height;
  c.poses.reserve(seq.poses.size());
  const Vec3 base(base_xy.x(), base_xy.y(), lift_height + arm_mount_offset_z);
  for (size_t i = 0; i < seq.poses.size(); ++i) {
    PoseCheck p;
    p.role = seq.roles[i];
    p.stroke = seq.stroke_index[i];
    Pose in_base = seq.poses[i];
    in_base.translation() -= base;
    p.score = scorePose(map, in_base, margin);
    if (!p.score.reachable) {
      p.failure = PoseFailure::kOutOfReach;
    } else if (p.score.score < margin.min_score) {
      p.failure = PoseFailure::kLowMargin;
    } else {
      p.failure = PoseFailure::kNone;
      p.usable = true;
    }
    c.poses.push_back(p);
  }
  summarize(&c);
  return c;
}

ColumnStatus statusOf(const SequenceCheck & c)
{
  if (c.whole()) {return ColumnStatus::kWhole;}
  if (c.ends_usable && c.strokes_usable > 0) {return ColumnStatus::kSplit;}
  return ColumnStatus::kUnreachable;
}

namespace
{
// Higher is better.
bool better(const SequenceCheck & a, const SequenceCheck & b)
{
  if (a.whole() != b.whole()) {return a.whole();}
  if (a.ends_usable != b.ends_usable) {return a.ends_usable;}
  if (a.strokes_usable != b.strokes_usable) {return a.strokes_usable > b.strokes_usable;}
  return a.min_score > b.min_score + 1e-12;
}
}  // namespace

ColumnFeasibility evaluateColumn(
  const Map4D & map, const ColumnPoseSequence & seq, const Eigen::Vector2d & base_xy,
  const std::vector<double> & heights, double arm_mount_offset_z, const MarginConfig & margin)
{
  ColumnFeasibility f;
  bool have = false;
  for (const double h : heights) {
    SequenceCheck c = checkSequenceAt(map, seq, base_xy, h, arm_mount_offset_z, margin);
    if (!have || better(c, f.best)) {   // strict: a tie keeps the lower height (heights ascend)
      f.best = std::move(c);
      have = true;
    }
  }
  f.status = have ? statusOf(f.best) : ColumnStatus::kUnreachable;
  return f;
}

ColumnFeasibility confirmColumnWithVerifier(
  StandVerifier & verifier, const ArmModel * arm, const Stand & stand, const ColumnPoseSequence & seq,
  const ColumnFeasibility & feas, const MarginConfig & margin)
{
  ColumnFeasibility out = feas;
  out.verified = true;
  std::vector<size_t> idx;
  std::vector<Pose> poses;
  for (size_t i = 0; i < out.best.poses.size(); ++i) {
    if (out.best.poses[i].usable) {
      idx.push_back(i);
      poses.push_back(seq.poses[i]);
    }
  }
  if (!poses.empty()) {
    const auto verdicts = verifier.verify(stand, out.best.lift_height, poses);
    for (size_t k = 0; k < idx.size(); ++k) {
      PoseCheck & p = out.best.poses[idx[k]];
      if (k >= verdicts.size() || !verdicts[k].ok()) {
        p.usable = false;
        p.failure = PoseFailure::kRefused;
        continue;
      }
      if (arm && !verdicts[k].joints.empty()) {
        p.singular_value = minSingularValue(*arm, verdicts[k].joints);
        p.limit_margin = jointLimitMargin(*arm, verdicts[k].joints);
        if (margin.min_singular_value > 0.0 && p.singular_value >= 0.0 && p.singular_value < margin.min_singular_value) {
          p.usable = false;
          p.failure = PoseFailure::kSingular;
        } else if (margin.min_limit_margin > 0.0 && p.limit_margin >= 0.0 && p.limit_margin < margin.min_limit_margin) {
          p.usable = false;
          p.failure = PoseFailure::kJointLimit;
        }
      }
    }
  }
  summarize(&out.best);
  out.status = statusOf(out.best);
  return out;
}

}  // namespace reachability
