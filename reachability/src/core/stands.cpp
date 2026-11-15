#include "reachability/core/stands.hpp"

#include <algorithm>

namespace reachability
{

StandPlan planStands(
  const Map4D & map, const StationRequest & request, const std::optional<Stand> & start,
  size_t max_stands)
{
  StandPlan plan;
  std::vector<size_t> left(request.targets_world.size());
  for (size_t i = 0; i < left.size(); ++i) {left[i] = i;}
  std::optional<Stand> at = request.base ? start : std::nullopt;

  while (!left.empty() && plan.stands.size() < max_stands) {
    StationRequest r = request;
    r.targets_world.clear();
    for (const size_t i : left) {r.targets_world.push_back(request.targets_world[i]);}
    r.from = at;
    const Station st = chooseStation(map, r);
    if (!st.found) {break;}

    StandAssignment a;
    a.station = st;
    for (const auto & band : st.plan.bands) {
      for (const size_t k : band.pose_indices) {a.targets.push_back(left[k]);}
    }
    if (a.targets.empty()) {break;}
    std::sort(a.targets.begin(), a.targets.end());
    // chooseStation numbered targets within the remaining set; make every index global
    for (auto & band : a.station.plan.bands) {
      for (auto & k : band.pose_indices) {k = left[k];}
    }
    for (auto & k : a.station.plan.unreachable) {k = left[k];}
    a.stand = Stand{Eigen::Vector2d(st.x, st.y), st.yaw};
    if (request.base && at) {
      a.travel_s = travelTime(*request.base, *at, a.stand);
      plan.total_travel_s += a.travel_s;
    }
    if (request.base) {at = a.stand;}

    left.erase(std::remove_if(left.begin(), left.end(), [&](size_t i) {
        return std::binary_search(a.targets.begin(), a.targets.end(), i);
      }), left.end());
    plan.stands.push_back(std::move(a));
  }
  // left over because the cap stopped the loop: not known to be unreachable
  (plan.stands.size() >= max_stands ? plan.not_planned : plan.unreachable) = left;
  return plan;
}

}  // namespace reachability
