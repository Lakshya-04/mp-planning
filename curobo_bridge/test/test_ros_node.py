"""ROS shell tests: a real node and real services in one process. The planner is a stand-in for the
logic tests (no GPU needed); one test runs the real cuRobo planner when CUDA is available."""
import threading

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
msgs = pytest.importorskip('curobo_bridge_msgs.srv')
Box = pytest.importorskip('curobo_bridge_msgs.msg').Box

from curobo_bridge.planner import IkResult, PlanResult  # noqa: E402
from curobo_bridge.ros_node import CuroboPlannerNode  # noqa: E402
from geometry_msgs.msg import Pose  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from sensor_msgs_py import point_cloud2  # noqa: E402
from std_msgs.msg import Header  # noqa: E402


class FakePlanner:
    joint_names = ['j1', 'j2']
    dimension = 2
    tool_frames = ['tool']
    max_boxes = 10

    def __init__(self):
        self.world = None
        self.cleared = 0
        self.pose_args = None

    def plan_joint(self, start, goal):
        if len(start) != 2:
            return PlanResult(failure_reason_code='invalid_request', failure_stage='precheck',
                              message='bad length')
        return PlanResult(success=True, joint_names=self.joint_names,
                          positions=[list(start), [0.5, 0.5], list(goal)], dt=0.1, planning_time_s=0.25)

    def plan_pose(self, start, position, quat_wxyz):
        self.pose_args = (tuple(position), tuple(quat_wxyz))
        return PlanResult(success=True, joint_names=self.joint_names, positions=[list(start)], dt=0.1)

    def solve_ik(self, position, quat_wxyz, seed=None):
        self.ik_args = (tuple(position), tuple(quat_wxyz), list(seed or []))
        if position[0] > 5.0:
            return IkResult(failure_reason_code='no_solution', failure_stage='planning', message='none')
        return IkResult(success=True, joint_names=self.joint_names, joints=[0.1, 0.2], position_error=1e-4)

    def check_states(self, states):
        for q in states:
            if len(q) != 2:
                raise ValueError('wrong joint count')
        return [q[0] < 1.0 for q in states]

    def check_motion(self, waypoints, max_step=0.05):
        self.motion_args = (len(waypoints), max_step)
        return waypoints[-1][1] < 1.0

    class esdf:   # the configured grid, as the node reads it
        voxel_size = 0.1

        @staticmethod
        def shape():
            return (2, 2, 2)

    def set_esdf(self, field, center=None):
        self.esdf_args = (field.copy(), tuple(center))
        return 1

    def clear_world(self):
        self.cleared += 1
        self.world = None

    def set_world(self, boxes=(), points=None, point_voxel=0.05):
        if len(boxes) > self.max_boxes:
            raise ValueError('too many boxes')
        self.world = (list(boxes), points, point_voxel)
        return len(boxes) + (0 if points is None else len(points))


@pytest.fixture()
def harness():
    rclpy.init()
    fake = FakePlanner()
    node = CuroboPlannerNode(planner=fake)
    client_node = rclpy.create_node('client')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def call(srv, name, request):
        client = client_node.create_client(srv, f'/curobo_planner/{name}')
        assert client.wait_for_service(5.0)
        future = client.call_async(request)
        deadline = 10.0
        while not future.done() and deadline > 0:
            threading.Event().wait(0.02)
            deadline -= 0.02
        assert future.done(), f'{name} timed out'
        return future.result()

    yield fake, call
    executor.shutdown()
    node.destroy_node()
    client_node.destroy_node()
    rclpy.shutdown()


def test_plan_joint_returns_a_timed_trajectory(harness):
    _, call = harness
    r = call(msgs.PlanJoint, 'plan_joint', msgs.PlanJoint.Request(start=[0.0, 0.0], goal=[1.0, 1.0]))
    assert r.success and r.failure_reason_code == 'none'
    assert list(r.trajectory.joint_names) == ['j1', 'j2']
    assert len(r.trajectory.points) == 3
    times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in r.trajectory.points]
    assert times == pytest.approx([0.0, 0.1, 0.2])
    assert list(r.trajectory.points[-1].positions) == [1.0, 1.0]
    assert r.planning_time_s == pytest.approx(0.25)


def test_failures_keep_their_structure(harness):
    _, call = harness
    r = call(msgs.PlanJoint, 'plan_joint', msgs.PlanJoint.Request(start=[0.0], goal=[1.0, 1.0]))
    assert not r.success
    assert (r.failure_reason_code, r.failure_stage) == ('invalid_request', 'precheck')
    assert len(r.trajectory.points) == 0


def test_pose_goal_converts_quaternion_order(harness):
    fake, call = harness
    goal = Pose()
    goal.position.x, goal.position.y, goal.position.z = 0.4, 0.1, 0.3
    goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w = 0.1, 0.2, 0.3, 0.9
    r = call(msgs.PlanPose, 'plan_pose', msgs.PlanPose.Request(start=[0.0, 0.0], goal=goal))
    assert r.success
    assert fake.pose_args == ((0.4, 0.1, 0.3), (0.9, 0.1, 0.2, 0.3))   # ROS xyzw in, cuRobo wxyz out


def test_set_world_takes_boxes_and_points(harness):
    fake, call = harness
    req = msgs.SetWorld.Request()
    box = Box()
    box.id = 'wall'
    box.pose.position.x = 1.0
    box.pose.orientation.w = 1.0
    box.size = [0.1, 2.0, 2.0]
    req.boxes = [box]
    header = Header(frame_id='base')
    req.points = point_cloud2.create_cloud_xyz32(header, [[0.5, 0.0, 0.1], [0.6, 0.0, 0.1]])
    req.point_voxel = 0.1
    r = call(msgs.SetWorld, 'set_world', req)
    assert r.success and r.box_count == 3
    boxes, points, voxel = fake.world
    assert boxes[0].id == 'wall' and boxes[0].center == (1.0, 0.0, 0.0) and boxes[0].size == (0.1, 2.0, 2.0)
    assert np.allclose(points, [[0.5, 0.0, 0.1], [0.6, 0.0, 0.1]])
    assert voxel == pytest.approx(0.1)


def test_empty_request_clears_and_errors_are_reported(harness):
    fake, call = harness
    r = call(msgs.SetWorld, 'set_world', msgs.SetWorld.Request())
    assert r.success and r.box_count == 0
    assert fake.cleared == 1 and fake.world is None   # an empty world is a clear, not a set_world
    req = msgs.SetWorld.Request()
    req.boxes = [Box() for _ in range(11)]
    r = call(msgs.SetWorld, 'set_world', req)
    assert not r.success and 'too many' in r.message


def test_missing_robot_config_answers_unavailable():
    from rclpy.parameter import Parameter
    rclpy.init()
    node = CuroboPlannerNode(parameter_overrides=[Parameter('robot', value='/nonexistent.yml')])
    client_node = rclpy.create_node('client')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(client_node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        client = client_node.create_client(msgs.PlanJoint, '/curobo_planner/plan_joint')
        assert client.wait_for_service(5.0)
        future = client.call_async(msgs.PlanJoint.Request(start=[0.0], goal=[0.0]))
        for _ in range(500):
            if future.done():
                break
            threading.Event().wait(0.02)
        r = future.result()
        assert not r.success
        assert (r.failure_reason_code, r.failure_stage) == ('unavailable', 'precheck')
    finally:
        executor.shutdown()
        node.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()


def test_real_planner_through_the_node():
    """cuRobo on the GPU, behind the real services (skipped without CUDA)."""
    torch = pytest.importorskip('torch')
    if not torch.cuda.is_available():
        pytest.skip('no CUDA device')
    from rclpy.parameter import Parameter
    rclpy.init()
    node = CuroboPlannerNode(parameter_overrides=[Parameter('robot', value='franka.yml')])
    client_node = rclpy.create_node('client')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(client_node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        assert node._planner is not None
        start = node._planner.default_joint_state
        goal = [v + 0.2 for v in start]
        client = client_node.create_client(msgs.PlanJoint, '/curobo_planner/plan_joint')
        assert client.wait_for_service(5.0)
        future = client.call_async(msgs.PlanJoint.Request(start=start, goal=goal))
        for _ in range(1500):
            if future.done():
                break
            threading.Event().wait(0.02)
        r = future.result()
        assert r.success, r.message
        assert len(r.trajectory.joint_names) == 7 and len(r.trajectory.points) > 2
        assert list(r.trajectory.points[-1].positions) == pytest.approx(goal, abs=2e-2)
    finally:
        executor.shutdown()
        node.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()


def test_esdf_parameters():
    from curobo_bridge.ros_node import esdf_from_parameters
    assert esdf_from_parameters([], 0.04, []) is None
    spec = esdf_from_parameters([2.0, 2.0, 1.0], 0.05, [0.3, 0.0, 0.5])
    assert spec.dims == (2.0, 2.0, 1.0) and spec.voxel_size == 0.05 and spec.center == (0.3, 0.0, 0.5)
    assert esdf_from_parameters([2.0, 2.0, 1.0], 0.05, []).center == (0.0, 0.0, 0.0)
    with pytest.raises(ValueError):
        esdf_from_parameters([2.0, 2.0], 0.05, [])


def _states(*rows):
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    return JointTrajectory(points=[JointTrajectoryPoint(positions=list(r)) for r in rows])


def test_solve_ik_converts_the_pose_and_returns_joints(harness):
    fake, call = harness
    pose = Pose()
    pose.position.x = 0.4
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = 0.1, 0.2, 0.3, 0.9
    r = call(msgs.SolveIk, 'solve_ik', msgs.SolveIk.Request(pose=pose, seed=[0.5, 0.5]))
    assert r.success and list(r.joints) == [0.1, 0.2] and list(r.joint_names) == ['j1', 'j2']
    assert r.position_error == pytest.approx(1e-4)
    assert fake.ik_args == ((0.4, 0.0, 0.0), (0.9, 0.1, 0.2, 0.3), [0.5, 0.5])   # xyzw in, wxyz out
    pose.position.x = 9.0
    r = call(msgs.SolveIk, 'solve_ik', msgs.SolveIk.Request(pose=pose))
    assert not r.success and (r.failure_reason_code, r.failure_stage) == ('no_solution', 'planning')


def test_check_motion_reports_per_state_and_sampled_motion(harness):
    fake, call = harness
    r = call(msgs.CheckMotion, 'check_motion',
             msgs.CheckMotion.Request(states=_states([0.0, 0.0], [2.0, 0.0])))
    assert r.success and list(r.state_valid) == [True, False] and not r.collision_free
    r = call(msgs.CheckMotion, 'check_motion',
             msgs.CheckMotion.Request(states=_states([0.0, 0.0], [0.5, 0.5]), max_step=0.1))
    assert r.success and r.collision_free and fake.motion_args == (2, pytest.approx(0.1))
    r = call(msgs.CheckMotion, 'check_motion',
             msgs.CheckMotion.Request(states=_states([0.0, 0.0], [2.0, 0.5]), max_step=0.1))
    assert r.success and list(r.state_valid) == [True, False] and not r.collision_free
    r = call(msgs.CheckMotion, 'check_motion', msgs.CheckMotion.Request(states=_states([0.0])))
    assert not r.success and 'wrong joint count' in r.message


def test_set_esdf_reshapes_x_major_and_checks_the_grid(harness):
    fake, call = harness
    req = msgs.SetEsdf.Request(nx=2, ny=2, nz=2, voxel_size=0.1, center=[1.0, 2.0, 3.0],
                               distance=[float(i) for i in range(8)])
    r = call(msgs.SetEsdf, 'set_esdf', req)
    assert r.success, r.message
    field, center = fake.esdf_args
    assert center == (1.0, 2.0, 3.0)
    assert field.shape == (2, 2, 2) and field[1, 0, 1] == 5.0 and field[0, 1, 0] == 2.0   # ix slowest

    bad = msgs.SetEsdf.Request(nx=2, ny=2, nz=4, voxel_size=0.1, center=[0.0] * 3, distance=[0.0] * 16)
    assert not call(msgs.SetEsdf, 'set_esdf', bad).success                    # wrong shape
    bad = msgs.SetEsdf.Request(nx=2, ny=2, nz=2, voxel_size=0.1, center=[0.0] * 3, distance=[0.0] * 7)
    assert 'expected 8' in call(msgs.SetEsdf, 'set_esdf', bad).message        # wrong length
    bad = msgs.SetEsdf.Request(nx=2, ny=2, nz=2, voxel_size=0.2, center=[0.0] * 3, distance=[0.0] * 8)
    assert not call(msgs.SetEsdf, 'set_esdf', bad).success                    # wrong voxel size
