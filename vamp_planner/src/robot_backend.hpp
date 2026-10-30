// Internal: the non-template face of a compiled-in VAMP robot. Robot templates and VAMP types stay
// in robots.cpp; vamp_planner.cpp only sees this interface and VAMP's Environment.
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <vamp/collision/environment.hh>

namespace vamp_planner
{

struct PlanSettings
{
  uint64_t max_iterations{100000};
  float range{2.0F};
  bool simplify{true};
};

struct RawPath
{
  bool found{false};
  std::vector<std::vector<double>> waypoints;
};

class RobotBackend
{
public:
  virtual ~RobotBackend() = default;
  virtual size_t dimension() const = 0;
  virtual std::vector<std::string> jointNames() const = 0;
  virtual float minRadius() const = 0;  ///< Smallest / largest collision sphere: sizes the point-cloud tree.
  virtual float maxRadius() const = 0;
  virtual bool stateValid(const std::vector<double> & q, const vamp::collision::Environment<float> & env) const = 0;
  virtual bool motionValid(
    const std::vector<double> & a, const std::vector<double> & b,
    const vamp::collision::Environment<float> & env) const = 0;
  virtual RawPath plan(
    const std::vector<double> & start, const std::vector<double> & goal,
    const vamp::collision::Environment<float> & env, const PlanSettings & settings) const = 0;
};

std::unique_ptr<RobotBackend> makeBackend(const std::string & robot);
std::vector<std::string> backendNames();

}  // namespace vamp_planner
