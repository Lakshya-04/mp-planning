#include "reachability/core/column_analysis.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>
#include <sstream>

namespace reachability
{

CoverageAnalysis summarizePlan(const ColumnStandPlan & plan)
{
  CoverageAnalysis a;
  a.plan = plan;
  a.columns_total = plan.columns.size();
  a.columns_whole = plan.whole;
  a.columns_split = plan.split;
  a.columns_unreachable = plan.unreachable;
  a.points_total = plan.strokes_total;
  a.points_covered = plan.strokes_covered;
  a.covered_fraction = a.points_total ? static_cast<double>(a.points_covered) / static_cast<double>(a.points_total) : 1.0;
  a.whole_fraction = a.columns_total ? static_cast<double>(a.columns_whole) / static_cast<double>(a.columns_total) : 1.0;
  a.stands = plan.stands.size();
  a.stands_if_splitting_allowed = plan.stands_if_splitting_allowed;
  a.verified = plan.verified;
  for (size_t i = 0; i < plan.stands.size(); ++i) {
    const PlannedColumnStand & ps = plan.stands[i];
    StandAnalysis s;
    s.index = i;
    s.x = ps.stand.arm_xy.x();
    s.y = ps.stand.arm_xy.y();
    s.yaw = ps.stand.yaw;
    s.travel_cost = ps.travel_cost;
    std::set<double> heights;
    for (const auto & c : ps.columns) {
      if (c.whole) {++s.columns_whole;} else {++s.columns_piece;}
      s.points_covered += c.strokes.size();
      heights.insert(c.lift_height);
    }
    s.lift_heights.assign(heights.begin(), heights.end());
    a.per_stand.push_back(std::move(s));
  }
  return a;
}

CoverageAnalysis analyzeColumnCoverage(
  const Map4D & map, const StationRequest & station, const std::vector<ColumnPlan> & columns,
  const std::optional<Stand> & start, const ColumnStandOptions & options)
{
  return summarizePlan(planColumnStands(map, station, columns, start, options));
}

namespace
{
std::string num(double v)
{
  if (!std::isfinite(v)) {return "null";}
  char buf[48];
  std::snprintf(buf, sizeof buf, "%.6g", v);
  return buf;
}

std::string str(const std::string & s)
{
  std::string o = "\"";
  for (const char c : s) {
    switch (c) {
      case '"': o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\n': o += "\\n"; break;
      case '\t': o += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char b[8];
          std::snprintf(b, sizeof b, "\\u%04x", c);
          o += b;
        } else {
          o += c;
        }
    }
  }
  return o + "\"";
}

std::string idxList(const std::vector<size_t> & v)
{
  std::string o = "[";
  for (size_t i = 0; i < v.size(); ++i) {o += (i ? "," : "") + std::to_string(v[i]);}
  return o + "]";
}
}  // namespace

std::string toJson(const CoverageAnalysis & a)
{
  std::ostringstream o;
  o << "{\n  \"summary\": {"
    << "\"columns_total\": " << a.columns_total << ", \"columns_whole\": " << a.columns_whole
    << ", \"columns_split\": " << a.columns_split << ", \"columns_unreachable\": " << a.columns_unreachable
    << ", \"points_total\": " << a.points_total << ", \"points_covered\": " << a.points_covered
    << ", \"covered_fraction\": " << num(a.covered_fraction) << ", \"whole_fraction\": " << num(a.whole_fraction)
    << ", \"stands\": " << a.stands << ", \"stands_if_splitting_allowed\": " << a.stands_if_splitting_allowed
    << ", \"verified\": " << (a.verified ? "true" : "false") << ", \"success\": " << (a.plan.success ? "true" : "false")
    << ", \"total_travel\": " << num(a.plan.total_travel) << ", \"candidates\": " << a.plan.candidates
    << ", \"message\": " << str(a.plan.message) << ", \"estimate\": \"RM4D map only; no IK or collision unless verified\"},\n";
  o << "  \"stands\": [";
  for (size_t i = 0; i < a.per_stand.size(); ++i) {
    const StandAnalysis & s = a.per_stand[i];
    o << (i ? ",\n    " : "\n    ") << "{\"index\": " << s.index << ", \"x\": " << num(s.x) << ", \"y\": " << num(s.y)
      << ", \"yaw\": " << num(s.yaw) << ", \"columns_whole\": " << s.columns_whole << ", \"columns_piece\": "
      << s.columns_piece << ", \"points_covered\": " << s.points_covered << ", \"travel_cost\": " << num(s.travel_cost)
      << ", \"lift_heights\": [";
    for (size_t h = 0; h < s.lift_heights.size(); ++h) {o << (h ? "," : "") << num(s.lift_heights[h]);}
    o << "], \"columns\": [";
    const auto & cols = a.plan.stands[i].columns;
    for (size_t c = 0; c < cols.size(); ++c) {
      o << (c ? "," : "") << "{\"column\": " << cols[c].column << ", \"whole\": " << (cols[c].whole ? "true" : "false")
        << ", \"lift\": " << num(cols[c].lift_height) << ", \"min_score\": " << num(cols[c].min_score)
        << ", \"strokes\": " << idxList(cols[c].strokes) << "}";
    }
    o << "]}";
  }
  o << (a.per_stand.empty() ? "" : "\n  ") << "],\n  \"columns\": [";
  for (size_t i = 0; i < a.plan.columns.size(); ++i) {
    const ColumnResult & r = a.plan.columns[i];
    o << (i ? ",\n    " : "\n    ") << "{\"column\": " << r.column << ", \"label\": " << str(r.label)
      << ", \"status\": \"" << toString(r.status) << "\", \"strokes_total\": " << r.strokes_total
      << ", \"strokes_covered\": " << r.strokes_covered << ", \"pieces\": [";
    for (size_t p = 0; p < r.pieces.size(); ++p) {
      o << (p ? "," : "") << "{\"stand\": " << r.pieces[p].stand << ", \"lift\": " << num(r.pieces[p].lift_height)
        << ", \"strokes\": " << idxList(r.pieces[p].strokes) << "}";
    }
    o << "]";
    if (r.skipped_to_avoid_split) {o << ", \"reason\": \"would_split\"";}
    if (r.limited) {
      o << ", \"limiting\": {\"pose\": \"" << toString(r.limiting_role) << "\"";
      if (r.limiting_stroke != kNoStroke) {o << ", \"stroke\": " << r.limiting_stroke;}
      o << ", \"why\": \"" << toString(r.limiting_failure) << "\", \"score\": " << num(r.limiting_score) << "}";
    }
    o << "}";
  }
  o << (a.plan.columns.empty() ? "" : "\n  ") << "]\n}\n";
  return o.str();
}

}  // namespace reachability
