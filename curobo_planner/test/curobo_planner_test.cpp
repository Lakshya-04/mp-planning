// A real client talking to a stand-in cuRobo planner node (same process, real ROS services).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <sstream>
#include <thread>

#include <curobo_bridge_msgs/srv/check_motion.hpp>
#include <curobo_bridge_msgs/srv/plan_joint.hpp>
#include <curobo_bridge_msgs/srv/plan_pose.hpp>
#include <curobo_bridge_msgs/srv/set_esdf.hpp>
#include <curobo_bridge_msgs/srv/set_world.hpp>
#include <curobo_bridge_msgs/srv/solve_ik.hpp>
#include <rclcpp/rclcpp.hpp>

#include "curobo_planner/curobo_planner.hpp"
#include "scene_model/core/voxel.hpp"

using curobo_bridge_msgs::srv::CheckMotion;
using curobo_bridge_msgs::srv::PlanJoint;
using curobo_bridge_msgs::srv::PlanPose;
using curobo_bridge_msgs::srv::SetEsdf;
using curobo_bridge_msgs::srv::SetWorld;
using curobo_bridge_msgs::srv::SolveIk;
namespace mp = motion_planner;

namespace
{
class StubServer
{
public:
  StubServer()
  {
    node = std::make_shared<rclcpp::Node>("curobo_planner");
    joint = node->create_service<PlanJoint>(
      "~/plan_joint", [this](const PlanJoint::Request::SharedPtr q, PlanJoint::Response::SharedPtr r) {
        last_joint = *q;
        if (q->goal.size() != 2) {
          r->failure_reason_code = "invalid_request";
          r->failure_stage = "precheck";
          r->message = "bad goal";
          return;
        }
        if (q->goal[0] > 5.0) {
          r->failure_reason_code = "no_path";
          r->failure_stage = "planning";
          return;
        }
        r->success = true;
        r->failure_reason_code = "none";
        r->planning_time_s = 0.5;
        r->trajectory.joint_names = {"j1", "j2"};
        for (int i = 0; i < 3; ++i) {
          trajectory_msgs::msg::JointTrajectoryPoint p;
          p.positions = {q->start[0] + i * 0.1, q->start[1]};
          p.time_from_start.nanosec = static_cast<uint32_t>(i * 100000000);
          r->trajectory.points.push_back(p);
        }
      });
    pose = node->create_service<PlanPose>(
      "~/plan_pose", [this](const PlanPose::Request::SharedPtr q, PlanPose::Response::SharedPtr r) {
        last_pose = *q;
        r->success = true;
        r->trajectory.joint_names = {"j1", "j2"};
        r->trajectory.points.resize(1);
      });
    esdf_srv = node->create_service<SetEsdf>(
      "~/set_esdf", [this](const SetEsdf::Request::SharedPtr q, SetEsdf::Response::SharedPtr r) {
        last_esdf = *q;
        r->success = q->nx != 99;
        if (!r->success) {r->message = "shape mismatch";}
      });
    ik_srv = node->create_service<SolveIk>(
      "~/solve_ik", [this](const SolveIk::Request::SharedPtr q, SolveIk::Response::SharedPtr r) {
        last_ik = *q;
        if (q->pose.position.x > 5.0) {
          r->failure_reason_code = "no_solution";
          r->failure_stage = "planning";
          r->message = "none";
          return;
        }
        r->success = true;
        r->failure_reason_code = "none";
        r->joints = {0.1, 0.2};
      });
    check_srv = node->create_service<CheckMotion>(
      "~/check_motion", [this](const CheckMotion::Request::SharedPtr q, CheckMotion::Response::SharedPtr r) {
        last_check = *q;
        r->success = true;
        r->collision_free = true;
        for (const auto & p : q->states.points) {
          const bool ok = p.positions[0] < 1.0;
          r->state_valid.push_back(ok);
          r->collision_free = r->collision_free && ok;
        }
      });
    world = node->create_service<SetWorld>(
      "~/set_world", [this](const SetWorld::Request::SharedPtr q, SetWorld::Response::SharedPtr r) {
        last_world = *q;
        r->success = q->boxes.size() < 5;
        r->box_count = static_cast<uint32_t>(q->boxes.size());
        if (!r->success) {r->message = "too many boxes";}
      });
  }

  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Service<SetEsdf>::SharedPtr esdf_srv;
  SetEsdf::Request last_esdf;
  rclcpp::Service<SolveIk>::SharedPtr ik_srv;
  rclcpp::Service<CheckMotion>::SharedPtr check_srv;
  SolveIk::Request last_ik;
  CheckMotion::Request last_check;
  rclcpp::Service<PlanJoint>::SharedPtr joint;
  rclcpp::Service<PlanPose>::SharedPtr pose;
  rclcpp::Service<SetWorld>::SharedPtr world;
  PlanJoint::Request last_joint;
  PlanPose::Request last_pose;
  SetWorld::Request last_world;
};

struct Fixture
{
  Fixture()
  {
    server = std::make_unique<StubServer>();
    client_node = std::make_shared<rclcpp::Node>("curobo_client");
    exec.add_node(server->node);
    exec.add_node(client_node);
    spinner = std::thread([this] {exec.spin();});
    curobo_planner::Options o;
    o.wait_for_service_sec = 5.0;
    o.timeout_sec = 5.0;
    planner = std::make_unique<curobo_planner::CuroboClientPlanner>(client_node, o);
  }
  ~Fixture()
  {
    exec.cancel();
    spinner.join();
  }

  std::unique_ptr<StubServer> server;
  std::shared_ptr<rclcpp::Node> client_node;
  rclcpp::executors::SingleThreadedExecutor exec;
  std::thread spinner;
  std::unique_ptr<curobo_planner::CuroboClientPlanner> planner;
};

mp::PlanningRequest jointRequest(double goal0)
{
  mp::PlanningRequest r;
  r.start_joint_state = {0.0, 0.2};
  r.target_joint_state = {goal0, 0.2};
  return r;
}
}  // namespace

TEST(CuroboClient, JointPlanBecomesATimedTrajectory)
{
  Fixture f;
  const auto r = f.planner->plan(jointRequest(1.0));
  ASSERT_TRUE(r.success) << r.message;
  EXPECT_EQ(f.planner->name(), "curobo");
  EXPECT_EQ(r.trajectory.joint_names, (std::vector<std::string>{"j1", "j2"}));
  ASSERT_EQ(r.trajectory.points.size(), 3U);
  EXPECT_NEAR(r.trajectory.points[2].time_from_start_sec, 0.2, 1e-9);
  EXPECT_NEAR(r.trajectory.points[2].positions[0], 0.2, 1e-9);
  EXPECT_EQ(r.planning_time_ms, 500U);
  EXPECT_DOUBLE_EQ(r.path_fraction, 1.0);
}

TEST(CuroboClient, ServerFailuresKeepCodeAndStage)
{
  Fixture f;
  auto r = f.planner->plan(jointRequest(9.0));
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.failure_reason_code, "no_path");
  EXPECT_EQ(r.failure_stage, mp::FailureStage::kPlanning);

  mp::PlanningRequest bad = jointRequest(1.0);
  bad.target_joint_state = {1.0, 2.0, 3.0};
  r = f.planner->plan(bad);
  EXPECT_EQ(r.failure_reason_code, "invalid_request");
  EXPECT_EQ(r.failure_stage, mp::FailureStage::kPrecheck);
}

TEST(CuroboClient, PoseGoalSendsPositionAndQuaternion)
{
  Fixture f;
  mp::PlanningRequest r;
  r.start_joint_state = {0.0, 0.0};
  r.target_poses = {core_types::Pose{0.4, 0.1, 0.3, 0.1, 0.2, 0.3, 0.9}};
  const auto res = f.planner->plan(r);
  ASSERT_TRUE(res.success) << res.message;
  const auto & g = f.server->last_pose.goal;
  EXPECT_DOUBLE_EQ(g.position.x, 0.4);
  EXPECT_DOUBLE_EQ(g.orientation.x, 0.1);
  EXPECT_DOUBLE_EQ(g.orientation.w, 0.9);
}

TEST(CuroboClient, RejectsRequestsItCannotServe)
{
  Fixture f;
  mp::PlanningRequest none;
  EXPECT_EQ(f.planner->plan(none).failure_reason_code, "invalid_request");
  mp::PlanningRequest ambiguous;
  ambiguous.start_joint_state = {0.0, 0.0};
  ambiguous.target_poses = {core_types::Pose{}, core_types::Pose{}};
  EXPECT_EQ(f.planner->plan(ambiguous).failure_reason_code, "invalid_request");
}

TEST(CuroboClient, IkConvertsPosesAndReturnsJoints)
{
  Fixture f;
  mp::IkRequest r;
  r.target_pose = {0.4, 0.0, 0.3, 0.1, 0.2, 0.3, 0.9};
  r.joint_seed = {0.5, 0.5};
  auto res = f.planner->ik(r);
  ASSERT_TRUE(res.success) << res.message;
  EXPECT_EQ(res.joint_solution, (std::vector<double>{0.1, 0.2}));
  EXPECT_DOUBLE_EQ(f.server->last_ik.pose.orientation.w, 0.9);
  EXPECT_EQ(f.server->last_ik.seed, (std::vector<double>{0.5, 0.5}));

  r.target_pose = {0.4, 0.0, 0.3, 0.0, 0.0, M_PI / 2};   // rpy: yaw 90 degrees
  ASSERT_TRUE(f.planner->ik(r).success);
  EXPECT_NEAR(f.server->last_ik.pose.orientation.z, std::sqrt(0.5), 1e-9);
  EXPECT_NEAR(f.server->last_ik.pose.orientation.w, std::sqrt(0.5), 1e-9);

  r.target_pose = {9.0, 0.0, 0.3, 0, 0, 0, 1};
  res = f.planner->ik(r);
  EXPECT_FALSE(res.success);
  EXPECT_EQ(res.failure_reason_code, "no_solution");
  EXPECT_EQ(res.failure_stage, mp::FailureStage::kPlanning);

  r.target_pose = {1.0, 2.0};
  EXPECT_EQ(f.planner->ik(r).failure_reason_code, "invalid_request");
}

TEST(CuroboClient, CollisionChecksStatesAndMotions)
{
  Fixture f;
  mp::CollisionRequest c;
  c.joint_values = {0.5, 0.0};
  auto res = f.planner->collision(c);
  ASSERT_TRUE(res.success) << res.message;
  EXPECT_TRUE(res.collision_free);
  EXPECT_DOUBLE_EQ(f.server->last_check.max_step, 0.0);

  c.joint_values = {2.0, 0.0};
  EXPECT_FALSE(f.planner->collision(c).collision_free);

  mp::CollisionRequest t;
  t.trajectory_joint_values = {{0.0, 0.0}, {0.5, 0.0}, {0.9, 0.0}};
  res = f.planner->collision(t);
  EXPECT_TRUE(res.collision_free);
  EXPECT_GT(f.server->last_check.max_step, 0.0);   // a trajectory also asks for the motions
  EXPECT_EQ(f.server->last_check.states.points.size(), 3U);

  EXPECT_EQ(f.planner->collision(mp::CollisionRequest{}).failure_reason_code, "invalid_request");
  mp::CollisionRequest pose_based;
  pose_based.target_pose = {0, 0, 0, 0, 0, 0};
  EXPECT_EQ(f.planner->collision(pose_based).failure_reason_code, "unsupported");
}

TEST(CuroboClient, WorldCarriesBoxesAndTransformedPoints)
{
  Fixture f;
  mp::WorldGeometryUpdate w;
  w.frame_id = "base";
  mp::WorldGeometryBox b;
  b.id = "wall";
  b.center_x = 1.0;
  b.size_y = 2.0;
  b.orientation_w = 1.0;
  w.boxes.push_back(b);
  w.points = {{1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  w.points_origin.x = 10.0;   // points are in a frame whose origin is at x = 10 in `base`
  std::string message;
  ASSERT_TRUE(f.planner->setWorld(w, &message)) << message;
  const auto & got = f.server->last_world;
  ASSERT_EQ(got.boxes.size(), 1U);
  EXPECT_EQ(got.boxes[0].id, "wall");
  EXPECT_DOUBLE_EQ(got.boxes[0].size[1], 2.0);
  ASSERT_EQ(got.points.width, 2U);
  float xyz[3];
  std::memcpy(xyz, &got.points.data[12], 12);
  EXPECT_NEAR(xyz[0], 12.0F, 1e-5);   // second point (2,0,0) + origin (10,0,0)

  EXPECT_TRUE(f.planner->clearWorld());
  EXPECT_TRUE(f.server->last_world.boxes.empty());

  mp::WorldGeometryUpdate many;
  many.boxes.assign(6, b);
  EXPECT_FALSE(f.planner->setWorld(many, &message));
  EXPECT_EQ(message, "too many boxes");
}

TEST(CuroboClient, RejectsAWorldInAnotherFrame)
{
  Fixture f;
  curobo_planner::Options o;
  o.base_frame = "base";
  o.timeout_sec = 5.0;
  curobo_planner::CuroboClientPlanner planner(f.client_node, o);
  mp::WorldGeometryUpdate w;
  w.frame_id = "map";
  mp::WorldGeometryBox b;
  b.id = "crate";
  b.orientation_w = 1.0;
  w.boxes.push_back(b);
  std::string error;
  EXPECT_FALSE(planner.updateWorldGeometry(w, error));
  EXPECT_NE(error.find("map"), std::string::npos);
  EXPECT_TRUE(f.server->last_world.boxes.empty());   // nothing reached the node
  w.frame_id = "base";
  EXPECT_TRUE(planner.updateWorldGeometry(w, error)) << error;
  EXPECT_EQ(f.server->last_world.boxes.size(), 1U);
}

TEST(CuroboClient, IsAScenePortToo)
{
  Fixture f;
  mp::ScenePort & port = *f.planner;
  mp::WorldGeometryUpdate w;
  mp::WorldGeometryBox b;
  b.id = "crate";
  b.orientation_w = 1.0;
  w.boxes.push_back(b);
  std::string error;
  ASSERT_TRUE(port.updateWorldGeometry(w, error)) << error;
  ASSERT_EQ(f.server->last_world.boxes.size(), 1U);
  EXPECT_EQ(f.server->last_world.boxes[0].id, "crate");
  ASSERT_TRUE(port.clearWorldGeometry(error)) << error;
  EXPECT_TRUE(f.server->last_world.boxes.empty());
  w.boxes.assign(6, b);                       // the stub refuses 5 or more
  EXPECT_FALSE(port.updateWorldGeometry(w, error));
  EXPECT_EQ(error, "too many boxes");
}

TEST(CuroboClient, EsdfIsReorderedToXMajorWithTheGridCentre)
{
  Fixture f;
  scene_model::EsdfGrid g;
  g.nx = 2;
  g.ny = 3;
  g.nz = 4;
  g.voxel_size = 0.1;
  g.origin = scene_model::Vec3(1.0, 2.0, 3.0);   // centre of voxel (0,0,0)
  g.distance.resize(24);
  for (int k = 0; k < 4; ++k) {
    for (int j = 0; j < 3; ++j) {
      for (int i = 0; i < 2; ++i) {g.distance[g.index(i, j, k)] = 100.0F * i + 10.0F * j + k;}
    }
  }
  std::string message;
  ASSERT_TRUE(f.planner->setEsdf(g, &message)) << message;
  const auto & q = f.server->last_esdf;
  EXPECT_EQ(q.nx, 2U);
  EXPECT_EQ(q.nz, 4U);
  EXPECT_NEAR(q.center[0], 1.05, 1e-12);        // origin + (n - 1) / 2 voxels
  EXPECT_NEAR(q.center[1], 2.10, 1e-12);
  EXPECT_NEAR(q.center[2], 3.15, 1e-12);
  ASSERT_EQ(q.distance.size(), 24U);
  // x-major: index = (ix * ny + iy) * nz + iz
  EXPECT_FLOAT_EQ(q.distance[(1 * 3 + 2) * 4 + 3], 100.0F + 20.0F + 3.0F);
  EXPECT_FLOAT_EQ(q.distance[(0 * 3 + 1) * 4 + 2], 12.0F);

  g.distance.pop_back();
  EXPECT_FALSE(f.planner->setEsdf(g, &message));
  EXPECT_NE(message.find("inconsistent"), std::string::npos);
  g.distance.push_back(0.0F);
  g.nx = 99;                                      // the stub node refuses this shape
  g.distance.resize(static_cast<size_t>(99) * 3 * 4);
  EXPECT_FALSE(f.planner->setEsdf(g, &message));
  EXPECT_EQ(message, "shape mismatch");
}

TEST(CuroboClient, MissingServerIsReportedNotHung)
{
  auto node = std::make_shared<rclcpp::Node>("lonely_client");
  curobo_planner::Options o;
  o.node_name = "/nobody_home";
  o.wait_for_service_sec = 0.3;
  curobo_planner::CuroboClientPlanner planner(node, o);
  const auto r = planner.plan(jointRequest(1.0));
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.failure_reason_code, "unavailable");
}

// End to end against a real `curobo_planner_node` running cuRobo on the GPU. Only runs when
// CUROBO_START holds the robot's start joint values (space separated) and the node is up.
TEST(CuroboClientIntegration, PlansOnTheRealNode)
{
  const char * env = std::getenv("CUROBO_START");
  if (!env) {
    GTEST_SKIP() << "CUROBO_START not set";
  }
  std::vector<double> start;
  std::istringstream in(env);
  for (double v; in >> v;) {start.push_back(v);}
  auto node = std::make_shared<rclcpp::Node>("integration_client");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec] {exec.spin();});
  curobo_planner::Options o;
  o.wait_for_service_sec = 120.0;   // the node compiles CUDA kernels on first start
  o.timeout_sec = 120.0;
  curobo_planner::CuroboClientPlanner planner(node, o);

  mp::PlanningRequest r;
  r.start_joint_state = start;
  r.target_joint_state = start;
  for (auto & v : r.target_joint_state) {v += 0.2;}
  const auto res = planner.plan(r);
  mp::PlanningResult blocked;
  std::string message;
  mp::WorldGeometryUpdate w;
  mp::WorldGeometryBox slab;
  slab.id = "slab";
  slab.center_z = 0.4;
  slab.size_x = slab.size_y = slab.size_z = 4.0;   // swallows the robot
  slab.orientation_w = 1.0;
  w.boxes.push_back(slab);
  const bool world_ok = planner.setWorld(w, &message);
  blocked = planner.plan(r);
  planner.clearWorld();
  exec.cancel();
  spinner.join();

  ASSERT_TRUE(res.success) << res.failure_reason_code << ": " << res.message;
  EXPECT_EQ(res.trajectory.joint_names.size(), start.size());
  EXPECT_GT(res.trajectory.points.size(), 2U);
  for (size_t j = 0; j < start.size(); ++j) {
    EXPECT_NEAR(res.trajectory.points.back().positions[j], r.target_joint_state[j], 2e-2);
  }
  ASSERT_TRUE(world_ok) << message;
  EXPECT_FALSE(blocked.success);
  EXPECT_EQ(blocked.failure_reason_code, "no_path");
}

// IK and validity checks against the same real node. CUROBO_TOOL holds the tool pose at CUROBO_START
// as "x y z qx qy qz qw".
TEST(CuroboClientIntegration, IkAndCollisionOnTheRealNode)
{
  const char * start_env = std::getenv("CUROBO_START");
  const char * tool_env = std::getenv("CUROBO_TOOL");
  if (!start_env || !tool_env) {
    GTEST_SKIP() << "CUROBO_START / CUROBO_TOOL not set";
  }
  std::vector<double> start, tool;
  {std::istringstream in(start_env); for (double v; in >> v;) {start.push_back(v);}}
  {std::istringstream in(tool_env); for (double v; in >> v;) {tool.push_back(v);}}
  ASSERT_EQ(tool.size(), 7U);
  auto node = std::make_shared<rclcpp::Node>("integration_client_ik");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec] {exec.spin();});
  curobo_planner::Options o;
  o.wait_for_service_sec = 120.0;
  o.timeout_sec = 120.0;
  curobo_planner::CuroboClientPlanner planner(node, o);

  mp::IkRequest ik;
  ik.target_pose = tool;
  ik.joint_seed = start;
  const auto solved = planner.ik(ik);
  mp::CollisionRequest ok;
  ok.joint_values = solved.joint_solution;
  const auto ok_res = planner.collision(ok);
  mp::CollisionRequest out_of_limits;
  out_of_limits.joint_values = start;
  out_of_limits.joint_values[0] = 10.0;
  const auto bad_res = planner.collision(out_of_limits);
  mp::CollisionRequest path;
  path.trajectory_joint_values = {start, start};
  for (auto & v : path.trajectory_joint_values.back()) {v += 0.1;}
  const auto path_res = planner.collision(path);
  exec.cancel();
  spinner.join();

  ASSERT_TRUE(solved.success) << solved.failure_reason_code << ": " << solved.message;
  ASSERT_EQ(solved.joint_solution.size(), start.size());
  ASSERT_TRUE(ok_res.success) << ok_res.message;
  EXPECT_TRUE(ok_res.collision_free);
  ASSERT_TRUE(bad_res.success);
  EXPECT_FALSE(bad_res.collision_free);
  ASSERT_TRUE(path_res.success) << path_res.message;
  EXPECT_TRUE(path_res.collision_free);
}

// Our EsdfGrid through the real node (started with esdf_dims 2x2x2, voxel 0.05, centre 0.3 0 0.5) to
// cuRobo on the GPU: an obstacle at the tool blocks the move, a free field does not.
TEST(CuroboClientIntegration, EsdfFromOurGridBlocksAndFrees)
{
  const char * start_env = std::getenv("CUROBO_START");
  const char * tool_env = std::getenv("CUROBO_TOOL");
  if (!start_env || !tool_env || !std::getenv("CUROBO_ESDF")) {
    GTEST_SKIP() << "CUROBO_START / CUROBO_TOOL / CUROBO_ESDF not set";
  }
  std::vector<double> start, tool;
  {std::istringstream in(start_env); for (double v; in >> v;) {start.push_back(v);}}
  {std::istringstream in(tool_env); for (double v; in >> v;) {tool.push_back(v);}}

  const double v = 0.05;
  const int n = 40;
  const scene_model::Vec3 centre(0.3, 0.0, 0.5);
  scene_model::VoxelGrid vox;
  vox.voxel_size = v;
  vox.nx = vox.ny = vox.nz = n;
  vox.origin = centre - scene_model::Vec3::Constant(0.5 * n * v) + scene_model::Vec3::Constant(0.5 * v);
  vox.occupied.assign(static_cast<size_t>(n) * n * n, scene_model::kVoxelFree);
  const scene_model::Vec3 blob(tool[0], tool[1], tool[2]);
  for (int k = 0; k < n; ++k) {
    for (int j = 0; j < n; ++j) {
      for (int i = 0; i < n; ++i) {
        if ((vox.origin + v * scene_model::Vec3(i, j, k) - blob).norm() < 0.08) {
          vox.occupied[vox.index(i, j, k)] = scene_model::kVoxelOccupied;
        }
      }
    }
  }
  const scene_model::EsdfGrid esdf = scene_model::computeEsdf(vox);
  scene_model::EsdfGrid free_field = esdf;
  free_field.distance.assign(esdf.distance.size(), 1.0F);

  auto node = std::make_shared<rclcpp::Node>("integration_client_esdf");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec] {exec.spin();});
  curobo_planner::Options o;
  o.wait_for_service_sec = 120.0;
  o.timeout_sec = 120.0;
  curobo_planner::CuroboClientPlanner planner(node, o);

  mp::PlanningRequest r;
  r.start_joint_state = start;
  r.target_joint_state = start;
  for (auto & x : r.target_joint_state) {x += 0.15;}
  std::string m1, m2;
  const bool set_blocking = planner.setEsdf(esdf, &m1);
  const auto blocked = planner.plan(r);
  const bool set_free = planner.setEsdf(free_field, &m2);
  const auto freed = planner.plan(r);
  planner.clearWorld();
  exec.cancel();
  spinner.join();

  ASSERT_TRUE(set_blocking) << m1;
  EXPECT_FALSE(blocked.success) << "an obstacle at the tool must block the plan";
  ASSERT_TRUE(set_free) << m2;
  EXPECT_TRUE(freed.success) << freed.failure_reason_code << ": " << freed.message;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
