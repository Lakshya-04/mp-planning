"""ROS 2 shell over CuroboPlanner: services for joint / pose planning and the world.

Parameters: `robot` (cuRobo robot config: a bundled name like franka.yml or an absolute path to one
made by docker/tools/generate_curobo_robot.py), `max_boxes` (obstacle cache size), and optionally
`esdf_dims` (3 values, m), `esdf_voxel_size`, `esdf_center` to reserve an ESDF grid: set_world points
then become a distance field instead of cubes. IK picks the solution nearest the seed: name the
prismatic joints in `prismatic_joints` (a list of joint names; 1 m then weighs `ik_prismatic_factor`
rad, default 6) or give `ik_weights` (one per joint, on the squared difference) to override. Services (under
the node name): ~/plan_joint, ~/plan_pose, ~/set_world. Everything is in the robot base frame.

Run it with the Python that has torch and cuRobo (in the gpu image that is the venv, not the system
Python `ros2 run` picks): `/opt/venv/bin/python -m curobo_bridge.ros_node --ros-args -p robot:=...`.

The planner is built lazily-but-once at start-up (CUDA kernels compile on the first warmup, which
can take a minute); until then the services answer `unavailable` instead of blocking callers.
"""
import functools

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from curobo_bridge_msgs.srv import CheckMotion, PlanJoint, PlanPose, SetEsdf, SetWorld, SolveIk
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs_py import point_cloud2
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from curobo_bridge.planner import PRISMATIC_IK_FACTOR, Box, CuroboPlanner, EsdfSpec, PlanResult


def _duration(seconds: float) -> Duration:
    sec = int(seconds)
    return Duration(sec=sec, nanosec=int(round((seconds - sec) * 1e9)))


def esdf_from_parameters(dims, voxel_size, center):
    """EsdfSpec from node parameters; an empty `dims` means no ESDF grid (points become cubes)."""
    if not len(dims):
        return None
    if len(dims) != 3:
        raise ValueError(f'esdf_dims needs 3 values, got {len(dims)}')
    return EsdfSpec(tuple(float(d) for d in dims), float(voxel_size),
                    tuple(float(c) for c in center) if len(center) else (0.0, 0.0, 0.0))


def ik_weights_from_parameter(value):
    """The `ik_weights` parameter as a list, or None when it is unset or empty (the planner then derives
    the weights from the joint types). Zeros are kept: a zero weight makes IK ignore that joint, and
    the planner checks the length against the joint count."""
    return [float(w) for w in value] if value is not None and len(value) else None


def _guarded(handler):
    """A service callback must never raise: rclpy would take the whole node down with it. Any
    exception (a cuRobo / CUDA error, a bad weight vector) becomes a failed response."""
    @functools.wraps(handler)
    def wrapper(self, request, response):
        try:
            return handler(self, request, response)
        except Exception as e:  # noqa: BLE001 - the contract is "answer, never die"
            self.get_logger().error(f'{handler.__name__} failed: {type(e).__name__}: {e}')
            response.success = False
            response.message = f'{type(e).__name__}: {e}'
            if hasattr(response, 'failure_reason_code'):
                response.failure_reason_code = 'internal_error'
                response.failure_stage = 'planning'
            return response
    return wrapper


def to_trajectory(result: PlanResult) -> JointTrajectory:
    traj = JointTrajectory(joint_names=list(result.joint_names))
    for i, row in enumerate(result.positions):
        traj.points.append(JointTrajectoryPoint(positions=[float(v) for v in row],
                                                time_from_start=_duration(i * result.dt)))
    return traj


def fill(response, result: PlanResult):
    response.success = result.success
    response.failure_reason_code = result.failure_reason_code
    response.failure_stage = result.failure_stage
    response.message = result.message
    response.route = result.route
    response.trajectory = to_trajectory(result)
    response.planning_time_s = result.planning_time_s
    return response


class CuroboPlannerNode(Node):

    def __init__(self, planner=None, **node_kwargs):
        super().__init__('curobo_planner', **node_kwargs)
        robot = self.declare_parameter('robot', 'franka.yml').value
        max_boxes = int(self.declare_parameter('max_boxes', 256).value)
        esdf_dims = self.declare_parameter('esdf_dims', [0.0]).value
        esdf_dims = [d for d in esdf_dims if d > 0.0]
        esdf_voxel = float(self.declare_parameter('esdf_voxel_size', 0.04).value)
        esdf_center = self.declare_parameter('esdf_center', [0.0, 0.0, 0.0]).value
        prismatic = [n for n in self.declare_parameter('prismatic_joints', ['']).value if n]
        ik_prismatic_factor = float(self.declare_parameter('ik_prismatic_factor', PRISMATIC_IK_FACTOR).value)
        ik_weights = ik_weights_from_parameter(
            self.declare_parameter('ik_weights', Parameter.Type.DOUBLE_ARRAY).value)
        seed_fallback = bool(self.declare_parameter('seed_fallback', True).value)
        fallback_attempts = int(self.declare_parameter('fallback_attempts', 6).value)
        max_attempts = int(self.declare_parameter('max_attempts', 20).value)
        self._planner = planner
        if self._planner is None:
            try:
                self._planner = CuroboPlanner(
                    robot, max_boxes=max_boxes, prismatic_joints=prismatic,
                    ik_prismatic_factor=ik_prismatic_factor, ik_weights=ik_weights,
                    seed_fallback=seed_fallback, fallback_attempts=fallback_attempts, max_attempts=max_attempts,
                    esdf=esdf_from_parameters(esdf_dims, esdf_voxel, esdf_center))
                self.get_logger().info(
                    f'cuRobo ready: {self._planner.dimension} joints, tool {self._planner.tool_frames}')
            except Exception as e:  # noqa: BLE001 - report, keep serving "unavailable"
                self.get_logger().error(f'cuRobo planner unavailable: {e}')
        self.create_service(PlanJoint, '~/plan_joint', self._plan_joint)
        self.create_service(PlanPose, '~/plan_pose', self._plan_pose)
        self.create_service(SetWorld, '~/set_world', self._set_world)
        self.create_service(SetEsdf, '~/set_esdf', self._set_esdf)
        self.create_service(SolveIk, '~/solve_ik', self._solve_ik)
        self.create_service(CheckMotion, '~/check_motion', self._check_motion)

    def _unavailable(self, response):
        return fill(response, PlanResult(failure_reason_code='unavailable', failure_stage='precheck',
                                         message='cuRobo planner is not running'))

    @_guarded
    def _plan_joint(self, request, response):
        if self._planner is None:
            return self._unavailable(response)
        return fill(response, self._planner.plan_joint(list(request.start), list(request.goal)))

    @_guarded
    def _plan_pose(self, request, response):
        if self._planner is None:
            return self._unavailable(response)
        p, q = request.goal.position, request.goal.orientation
        return fill(response, self._planner.plan_pose(
            list(request.start), (p.x, p.y, p.z), (q.w, q.x, q.y, q.z)))   # ROS xyzw -> cuRobo wxyz

    @_guarded
    def _set_esdf(self, request, response):
        if self._planner is None:
            response.message = 'cuRobo planner is not running'
            return response
        shape = (request.nx, request.ny, request.nz)
        grid = getattr(self._planner, 'esdf', None)
        if grid is None:
            response.message = 'no ESDF grid configured: start the node with esdf_dims'
            return response
        if len(request.distance) != request.nx * request.ny * request.nz:
            response.message = f'distance has {len(request.distance)} values, expected {shape[0] * shape[1] * shape[2]}'
            return response
        if shape != grid.shape() or abs(request.voxel_size - grid.voxel_size) > 1e-9:
            response.message = (f'grid {shape} at {request.voxel_size} m does not match the planner\'s '
                                f'{grid.shape()} at {grid.voxel_size} m')
            return response
        field = np.asarray(request.distance, dtype=np.float32).reshape(shape)
        try:
            self._planner.set_esdf(field, tuple(request.center))
            response.success = True
        except ValueError as e:
            response.message = str(e)
        return response

    @_guarded
    def _solve_ik(self, request, response):
        if self._planner is None:
            response.failure_reason_code = 'unavailable'
            response.failure_stage = 'precheck'
            response.message = 'cuRobo planner is not running'
            return response
        p, q = request.pose.position, request.pose.orientation
        result = self._planner.solve_ik((p.x, p.y, p.z), (q.w, q.x, q.y, q.z), list(request.seed))
        response.success = result.success
        response.failure_reason_code = result.failure_reason_code
        response.failure_stage = result.failure_stage
        response.message = result.message
        response.joint_names = list(result.joint_names)
        response.joints = [float(v) for v in result.joints]
        response.position_error = result.position_error
        response.rotation_error = result.rotation_error
        return response

    @_guarded
    def _check_motion(self, request, response):
        if self._planner is None:
            response.message = 'cuRobo planner is not running'
            return response
        states = [list(pt.positions) for pt in request.states.points]
        try:
            valid = self._planner.check_states(states)
            ok = all(valid)
            if ok and request.max_step > 0.0 and len(states) > 1:
                ok = self._planner.check_motion(states, request.max_step)
        except ValueError as e:
            response.message = str(e)
            return response
        response.success = True
        response.state_valid = valid
        response.collision_free = ok
        return response

    @_guarded
    def _set_world(self, request, response):
        if self._planner is None:
            response.message = 'cuRobo planner is not running'
            return response
        boxes = [Box(b.id, (b.pose.position.x, b.pose.position.y, b.pose.position.z), tuple(b.size),
                     (b.pose.orientation.w, b.pose.orientation.x, b.pose.orientation.y,
                      b.pose.orientation.z)) for b in request.boxes]
        points = None
        if request.points.width * request.points.height > 0:
            raw = point_cloud2.read_points(request.points, field_names=('x', 'y', 'z'), skip_nans=True)
            points = np.column_stack([raw['x'], raw['y'], raw['z']]) if len(raw) else None
        try:
            if not boxes and points is None:      # an empty world is a clear (ClearWorld sends exactly this)
                self._planner.clear_world()
                response.box_count = 0
            else:
                response.box_count = self._planner.set_world(boxes, points, request.point_voxel or 0.05)
            response.success = True
        except ValueError as e:
            response.message = str(e)
        return response


def main():
    rclpy.init()
    node = CuroboPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
