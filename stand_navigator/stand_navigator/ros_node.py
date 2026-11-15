"""NavigateToStand action server: drives the arm base to a stand pose with /cmd_vel, reading /odom.

Parameters: `base_profile` (robot profile YAML with a `base:` section, required), `odom_topic`,
`cmd_vel_topic`, `tolerance_xy_m`, `tolerance_yaw_rad`, `timeout_s`, `rate_hz`, `odom_timeout_s`.
The goal frame must equal the odom frame id; nothing is transformed (no localisation here).
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from stand_navigator_msgs.action import NavigateToStand

from stand_navigator.core import (
    StandController, Tolerance, arm_to_footprint, footprint_to_arm, load_profile)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class StandNavigator(Node):
    def __init__(self):
        super().__init__('stand_navigator')
        path = self.declare_parameter('base_profile', '').value
        if not path:
            raise RuntimeError('parameter base_profile (robot profile YAML) is required')
        self.profile = load_profile(path)
        self.tol = Tolerance(self.declare_parameter('tolerance_xy_m', 0.02).value,
                             self.declare_parameter('tolerance_yaw_rad', 0.02).value)
        self.timeout = self.declare_parameter('timeout_s', 120.0).value
        self.rate = self.declare_parameter('rate_hz', 50.0).value
        self.odom_timeout = self.declare_parameter('odom_timeout_s', 1.0).value
        self.odom = None
        self.odom_stamp = 0.0
        cb = ReentrantCallbackGroup()
        self.create_subscription(Odometry, self.declare_parameter('odom_topic', 'odom').value,
                                 self._on_odom, 10, callback_group=cb)
        self.pub = self.create_publisher(Twist, self.declare_parameter('cmd_vel_topic', 'cmd_vel').value, 10)
        self.server = ActionServer(
            self, NavigateToStand, 'navigate_to_stand', self._execute, callback_group=cb,
            goal_callback=self._goal, cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.get_logger().info(
            f'navigate_to_stand ready: drive {self.profile.drive}, max {self.profile.max_lin} m/s '
            f'{self.profile.max_ang} rad/s, arm mount {self.profile.arm_mount}')

    def _on_odom(self, msg):
        self.odom = msg
        self.odom_stamp = time.monotonic()

    def _goal(self, req):
        p = req.pose.pose.position
        ok = all(map(math.isfinite, (p.x, p.y, req.pose.pose.orientation.z, req.pose.pose.orientation.w)))
        return GoalResponse.ACCEPT if ok else GoalResponse.REJECT

    def _stop(self):
        self.pub.publish(Twist())

    def _pose(self):
        o = self.odom.pose.pose
        return o.position.x, o.position.y, yaw_of(o.orientation)

    def _result(self, handle, ok, code, msg, ctl=None, arm_pose=None):
        res = NavigateToStand.Result()
        res.error_code, res.error_msg = code, msg
        if ctl is not None and self.odom is not None:
            res.position_error_m, res.yaw_error_rad = ctl.errors(self._pose())
        (handle.succeed if ok else handle.abort)()
        return res

    def _execute(self, handle):
        goal = handle.request.pose
        res = NavigateToStand.Result
        t0 = time.monotonic()
        while self.odom is None and time.monotonic() - t0 < 2.0:
            time.sleep(0.05)
        if self.odom is None:
            return self._result(handle, False, res.NO_ODOM, 'no odometry')
        if goal.header.frame_id and goal.header.frame_id != self.odom.header.frame_id:
            return self._result(handle, False, res.INVALID_GOAL,
                                f'goal frame {goal.header.frame_id} != odom frame {self.odom.header.frame_id}')
        gp = goal.pose
        gyaw = yaw_of(gp.orientation)
        fx, fy, fyaw = arm_to_footprint(gp.position.x, gp.position.y, gyaw, self.profile.arm_mount)
        ctl = StandController(self.profile, (fx, fy, fyaw), self.tol)
        dt = 1.0 / self.rate
        t0 = time.monotonic()
        fb = NavigateToStand.Feedback()
        last = time.monotonic()
        while rclpy.ok():
            if handle.is_cancel_requested:
                self._stop()
                handle.canceled()
                r = NavigateToStand.Result()
                r.error_code, r.error_msg = res.CANCELED, 'canceled'
                return r
            now = time.monotonic()
            if now - self.odom_stamp > self.odom_timeout:
                self._stop()
                return self._result(handle, False, res.NO_ODOM, 'odometry went stale', ctl)
            if now - t0 > self.timeout:
                self._stop()
                return self._result(handle, False, res.TIMEOUT, f'not there after {self.timeout:.0f} s', ctl)
            pose = self._pose()
            tw = self.odom.twist.twist
            if ctl.arrived(pose, math.hypot(tw.linear.x, tw.linear.y), abs(tw.angular.z)):
                self._stop()
                return self._result(handle, True, res.NONE, 'arrived', ctl)
            vx, vy, wz = ctl.step(pose, max(now - last, 1e-3))
            last = now
            cmd = Twist()
            cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz
            self.pub.publish(cmd)
            ax, ay, ayaw = footprint_to_arm(*pose, self.profile.arm_mount)
            fb.distance_remaining, fb.yaw_remaining = ctl.errors(pose)
            fb.current_pose = PoseStamped()
            fb.current_pose.header = self.odom.header
            fb.current_pose.pose.position.x, fb.current_pose.pose.position.y = ax, ay
            fb.current_pose.pose.orientation.z, fb.current_pose.pose.orientation.w = (
                math.sin(ayaw / 2), math.cos(ayaw / 2))
            handle.publish_feedback(fb)
            time.sleep(dt)
        self._stop()
        return self._result(handle, False, res.UNKNOWN, 'shutdown', ctl)


def main():
    rclpy.init()
    node = StandNavigator()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()


if __name__ == '__main__':
    main()
