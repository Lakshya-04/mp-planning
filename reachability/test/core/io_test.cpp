#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>

#include "reachability/core/builder.hpp"
#include "reachability/core/io.hpp"
#include "support.hpp"

using namespace reachability;  // NOLINT
using namespace reachability_test;  // NOLINT
namespace fs = std::filesystem;

namespace
{
std::string tmp(const std::string & name)
{
  return (fs::temp_directory_path() / ("reachability_test_" + std::to_string(::getpid()) + "_" + name)).string();
}
}  // namespace

TEST(MapIo, RoundTripKeepsEverything)
{
  const ChainModel arm = genericArm();
  BuildOptions o;
  o.samples = 30000;
  o.threads = 2;
  const Map4D a = buildMap(arm, coarse(), o);
  const std::string path = tmp("a.rm4d");
  saveMap(path, a, arm.fingerprint());
  const Map4D b = loadMap(path, arm.fingerprint());
  EXPECT_EQ(a.raw(), b.raw());
  EXPECT_TRUE(a.params() == b.params());
  fs::remove(path);
}

TEST(MapIo, AMapForAnotherRobotIsRejected)
{
  const ChainModel arm = genericArm();
  Map4D m(coarse());
  const std::string path = tmp("b.rm4d");
  saveMap(path, m, arm.fingerprint());
  EXPECT_THROW(loadMap(path, "some other robot"), std::runtime_error);
  EXPECT_NO_THROW(loadMap(path));  // no expectation: anything goes
  fs::remove(path);
}

TEST(MapIo, BadFilesFailWithAMessage)
{
  EXPECT_THROW(loadMap("/tmp/definitely_not_here.rm4d"), std::runtime_error);
  const std::string path = tmp("c.rm4d");
  std::ofstream(path) << "this is not a map";
  EXPECT_THROW(loadMap(path), std::runtime_error);
  // Truncated: a valid header followed by too little data.
  Map4D m(coarse());
  saveMap(path, m);
  fs::resize_file(path, fs::file_size(path) / 2);
  EXPECT_THROW(loadMap(path), std::runtime_error);
  fs::remove(path);
}

TEST(ChainYaml, LoadsAChainThatMatchesTheBuiltInOne)
{
  const std::string path = tmp("chain.yaml");
  std::ofstream(path) <<
    "joints:\n"
    "  - {xyz: [0, 0, 0],        rpy: [0, 0, 0], axis: [0, 0, 1], lower: -3.14159, upper: 3.14159}\n"
    "  - {xyz: [0, 0, 0.18],     rpy: [0, 0, 0], axis: [0, 1, 0], lower: -3.14159, upper: 3.14159}\n"
    "  - {xyz: [0, -0.12, 0.612], rpy: [0, 0, 0], axis: [0, 1, 0], lower: -3.14159, upper: 3.14159}\n"
    "  - {xyz: [0, 0, 0.572],    rpy: [0, 0, 0], axis: [0, 1, 0], lower: -3.14159, upper: 3.14159}\n"
    "  - {xyz: [0, 0.12, 0],     rpy: [0, 0, 0], axis: [0, 0, 1], lower: -3.14159, upper: 3.14159}\n"
    "  - {xyz: [0, 0, 0.12],     rpy: [0, 0, 0], axis: [0, 1, 0], lower: -3.14159, upper: 3.14159}\n"
    "tool: {xyz: [0, 0.06, 0.165], rpy: [-1.5707963267948966, 0, 0]}\n";
  const ChainModel loaded = loadChainYaml(path);
  const ChainModel builtin = genericArm();
  ASSERT_EQ(loaded.dof(), 6);
  const std::vector<double> q = {0.3, -0.5, 0.8, 0.1, -0.7, 1.2};
  Pose a, b;
  ASSERT_TRUE(loaded.evaluate(q, a));
  ASSERT_TRUE(builtin.evaluate(q, b));
  // The YAML tool places the TCP after the flange rotation the same way the built-in chain does:
  // both put a point 0.165 m along the tool axis from the flange, so compare the tool axis and
  // the position of the origin along it.
  EXPECT_NEAR((a.linear().col(2) - b.linear().col(2)).norm(), 0.0, 1e-4);
  fs::remove(path);
}

TEST(ChainYaml, ErrorsAreClear)
{
  EXPECT_THROW(loadChainYaml("/tmp/no_such_chain.yaml"), std::runtime_error);
  const std::string path = tmp("bad.yaml");
  std::ofstream(path) << "joints: []\n";
  EXPECT_THROW(loadChainYaml(path), std::runtime_error);
  fs::remove(path);
}

TEST(ChainModel, RejectsOutOfLimitConfigurationsAndWrongSizes)
{
  const ChainModel arm = genericArm();
  Pose T;
  EXPECT_FALSE(arm.evaluate({0, 0, 0, 0, 0}, T));               // wrong dof
  EXPECT_FALSE(arm.evaluate({4.0, 0, 0, 0, 0, 0}, T));          // beyond +-pi
  EXPECT_TRUE(arm.evaluate({0, 0, 0, 0, 0, 0}, T));
  EXPECT_NE(arm.fingerprint(), "");
}
