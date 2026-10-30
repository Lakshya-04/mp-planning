// Compiled-in VAMP robots. Adding a robot = include its generated header and add one line to the
// table at the bottom; nothing else in the package changes.
#include <algorithm>
#include <memory>

#include <vamp/planning/rrtc.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/planning/validate.hh>
#include <vamp/random/halton.hh>
#include <vamp/robots/ur5.hh>
#include <vamp/vector.hh>

#include "robot_backend.hpp"

// Cricket-generated robots found at configure time (see CMakeLists.txt); empty when none.
#include "robots_includes.inc"

namespace vamp_planner
{
namespace
{
constexpr std::size_t kRake = vamp::FloatVectorWidth;

template<typename Robot>
class VampRobot : public RobotBackend
{
  using Configuration = typename Robot::Configuration;
  using ConfigurationArray = typename Robot::ConfigurationArray;
  using EnvVector = vamp::collision::Environment<vamp::FloatVector<kRake>>;

  static Configuration toConfiguration(const std::vector<double> & q)
  {
    ConfigurationArray a{};
    for (std::size_t i = 0; i < Robot::dimension && i < q.size(); ++i) {
      a[i] = static_cast<float>(q[i]);
    }
    return Configuration(a);
  }

public:
  size_t dimension() const override {return Robot::dimension;}

  std::vector<std::string> jointNames() const override
  {
    std::vector<std::string> out;
    for (auto n : Robot::joint_names) {
      out.emplace_back(n);
    }
    return out;
  }

  float minRadius() const override {return Robot::min_radius;}
  float maxRadius() const override {return Robot::max_radius;}

  bool stateValid(const std::vector<double> & q, const vamp::collision::Environment<float> & env) const override
  {
    const auto c = toConfiguration(q);
    return vamp::planning::validate_motion<Robot, kRake, 1>(c, c, EnvVector(env));
  }

  bool motionValid(
    const std::vector<double> & a, const std::vector<double> & b,
    const vamp::collision::Environment<float> & env) const override
  {
    return vamp::planning::validate_motion<Robot, kRake, Robot::resolution>(
      toConfiguration(a), toConfiguration(b), EnvVector(env));
  }

  RawPath plan(
    const std::vector<double> & start, const std::vector<double> & goal,
    const vamp::collision::Environment<float> & env, const PlanSettings & s) const override
  {
    using Planner = vamp::planning::RRTC<Robot, kRake, Robot::resolution>;
    vamp::planning::RRTCSettings settings;
    settings.max_iterations = s.max_iterations;
    settings.max_samples = s.max_iterations;
    settings.range = s.range;

    const EnvVector env_v(env);
    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();
    auto result = Planner::solve(toConfiguration(start), toConfiguration(goal), env_v, settings, rng);
    RawPath out;
    if (result.path.empty()) {
      return out;
    }
    if (s.simplify) {
      rng->reset();
      auto simplified = vamp::planning::simplify<Robot, kRake, Robot::resolution>(
        result.path, env_v, vamp::planning::SimplifySettings{}, rng);
      if (!simplified.path.empty()) {
        result.path = std::move(simplified.path);
      }
    }
    for (const auto & c : result.path) {
      const auto arr = c.to_array();
      out.waypoints.emplace_back(arr.begin(), arr.begin() + Robot::dimension);
    }
    out.found = true;
    return out;
  }
};

struct Entry
{
  const char * name;
  std::unique_ptr<RobotBackend> (*make)();
};

template<typename Robot>
std::unique_ptr<RobotBackend> make()
{
  return std::make_unique<VampRobot<Robot>>();
}

const Entry kRobots[] = {
  {"ur5", &make<vamp::robots::UR5>},
#include "robots_table.inc"
};
}  // namespace

std::unique_ptr<RobotBackend> makeBackend(const std::string & robot)
{
  for (const auto & e : kRobots) {
    if (robot == e.name) {
      return e.make();
    }
  }
  return nullptr;
}

std::vector<std::string> backendNames()
{
  std::vector<std::string> out;
  for (const auto & e : kRobots) {
    out.emplace_back(e.name);
  }
  return out;
}

}  // namespace vamp_planner
