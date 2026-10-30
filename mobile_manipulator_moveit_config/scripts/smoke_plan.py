#!/usr/bin/env python3
"""End-to-end smoke test for the MoveIt config: needs `demo.launch.py` running.

For the 7-DOF `lift_arm` group and the 6-DOF `arm` group: FK of the current state, IK for a pose
8 cm away, then a collision-free plan (plan only) to that joint solution through move_group.
With `--execute` the `lift_arm` plan is also executed through the controller, and the script then
checks that the joints actually arrived at the solution (works for either controller). The 6-joint
`arm` group stays plan-only: the single controller takes all seven joints, no partial goals. Exits non-zero
with a message if any step fails.
"""
import sys

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions
from moveit_msgs.srv import GetPositionFK, GetPositionIK
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

TIMEOUT = 60.0
EXECUTE = '--execute' in sys.argv
ARRIVAL_TOLERANCE = 0.03  # rad / m


class Smoke(Node):

    def __init__(self):
        super().__init__('smoke_plan')
        self.fk = self.create_client(GetPositionFK, '/compute_fk')
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.move = ActionClient(self, MoveGroup, '/move_action')
        self.joint_state = None
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)

    def _on_joint_state(self, msg):
        self.joint_state = msg

    def wait(self):
        for client, name in ((self.fk, 'compute_fk'), (self.ik, 'compute_ik')):
            if not client.wait_for_service(timeout_sec=TIMEOUT):
                raise RuntimeError(f'{name} is not available (is demo.launch.py running?)')
        if not self.move.wait_for_server(timeout_sec=TIMEOUT):
            raise RuntimeError('move_action is not available')
        deadline = self.get_clock().now().nanoseconds + int(TIMEOUT * 1e9)
        while self.joint_state is None and self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.joint_state is None:
            raise RuntimeError('no /joint_states received')

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=TIMEOUT)
        if future.result() is None:
            raise RuntimeError('service call timed out')
        return future.result()

    def check_group(self, group, tip='tcp_link', base='base_link'):
        fk_req = GetPositionFK.Request()
        fk_req.header.frame_id = base
        fk_req.fk_link_names = [tip]
        fk_req.robot_state.joint_state = self.joint_state  # the real current state, not a default
        fk = self.call(self.fk, fk_req)
        if fk.error_code.val != 1 or not fk.pose_stamped:
            raise RuntimeError(f'[{group}] FK failed: {fk.error_code.val}')
        home_pose = fk.pose_stamped[0]

        # A reachable nearby target that is also collision free: try small offsets in turn and
        # accept the first IK solution that stays close to the current state. (The home pose can
        # sit next to a collision boundary, which makes the nearest solution in one direction
        # invalid and sends a numerical solver to a distant branch.)
        current = dict(zip(self.joint_state.name, self.joint_state.position))
        ik = None
        biggest = None
        tried = []
        for axis, delta in (('z', 0.05), ('z', -0.05), ('x', 0.05), ('x', -0.05), ('y', 0.05),
                            ('y', -0.05)):
            pose = type(home_pose)()
            pose.header = home_pose.header
            pose.pose.orientation = home_pose.pose.orientation
            pose.pose.position.x = home_pose.pose.position.x + (delta if axis == 'x' else 0.0)
            pose.pose.position.y = home_pose.pose.position.y + (delta if axis == 'y' else 0.0)
            pose.pose.position.z = home_pose.pose.position.z + (delta if axis == 'z' else 0.0)

            ik_req = GetPositionIK.Request()
            ik_req.ik_request.group_name = group
            ik_req.ik_request.robot_state.joint_state = self.joint_state  # seed: nearby solution
            ik_req.ik_request.pose_stamped = pose
            ik_req.ik_request.ik_link_name = tip
            ik_req.ik_request.avoid_collisions = True
            ik_req.ik_request.timeout.sec = 1
            candidate = self.call(self.ik, ik_req)
            if candidate.error_code.val != 1:
                tried.append(f'{axis}{delta:+.2f}: IK error {candidate.error_code.val}')
                continue
            moved = max(abs(v - current[n]) for n, v in zip(
                candidate.solution.joint_state.name, candidate.solution.joint_state.position)
                if n in current)
            if moved > 1.0:
                tried.append(f'{axis}{delta:+.2f}: far solution ({moved:.2f} rad)')
                continue
            ik, biggest = candidate, moved
            break
        if ik is None:
            raise RuntimeError(f'[{group}] no nearby collision-free IK solution: ' + '; '.join(tried))

        names = ik.solution.joint_state.name
        values = ik.solution.joint_state.position
        constraints = Constraints()
        group_joints = self.group_joints(group)
        for n, v in zip(names, values):
            if n in group_joints:
                c = JointConstraint(joint_name=n, position=v, tolerance_above=1e-3,
                                    tolerance_below=1e-3, weight=1.0)
                constraints.joint_constraints.append(c)

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = group
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        goal.request.goal_constraints = [constraints]
        execute = EXECUTE and group == 'lift_arm'
        goal.planning_options = PlanningOptions(plan_only=not execute)
        send = self.move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=TIMEOUT)
        handle = send.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f'[{group}] plan goal rejected')
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=TIMEOUT)
        result = result_future.result().result
        if result.error_code.val != 1:
            raise RuntimeError(f'[{group}] planning failed: {result.error_code.val}')
        points = result.planned_trajectory.joint_trajectory.points
        if len(points) < 2:
            raise RuntimeError(f'[{group}] empty trajectory')
        duration = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1e-9
        if execute:
            self.expect_arrival(group, dict(zip(names, values)), group_joints)
        print(f'[{group}] OK: IK solved (largest joint move {biggest:.2f}), '
              f'{len(points)} waypoints, {duration:.2f} s, '
              f'{len(result.planned_trajectory.joint_trajectory.joint_names)} joints')

    def expect_arrival(self, group, target, group_joints):
        """After an executed plan: the live joint state must be at the solution."""
        deadline = self.get_clock().now().nanoseconds + int(10e9)
        worst = None
        while self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            now = dict(zip(self.joint_state.name, self.joint_state.position))
            worst = max(abs(now[n] - v) for n, v in target.items() if n in group_joints and n in now)
            if worst < ARRIVAL_TOLERANCE:
                return
        raise RuntimeError(f'[{group}] executed, but the joints are still {worst:.3f} from the goal')

    @staticmethod
    def group_joints(group):
        lift = ['lift_joint']
        return set(lift + [f'arm_{n}' for n in (
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
            'wrist_2_joint', 'wrist_3_joint', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5',
            'joint6')]) - (set() if group == 'lift_arm' else {'lift_joint'})


def main():
    rclpy.init()
    node = Smoke()
    try:
        node.wait()
        node.check_group('lift_arm')
        node.check_group('arm')
    except RuntimeError as error:
        print(f'FAIL: {error}', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print('SMOKE OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
