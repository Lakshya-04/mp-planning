"""WallDock action server: servo the base to a standoff and square to the wall plane from wall_detector.

Reads the plane from `wall_scene_topic` (std_msgs/String JSON, `/wall_detector/wall_scene`: `plane_abcd`,
`normal_outward`, `frame_id`) and `/odom`, publishes `Twist` on `cmd_vel`. Best with the detector's
`target_frame` set to the odom frame: the plane is then a fixed world plane and the base moving while the
scans accumulate does not smear it. A plane in another frame is taken as a base-frame plane of the moment it
arrived and is moved into the odom frame with the odometry then (stop-and-look quality).

Parameters: `base_profile` (required, robot profile YAML with a `base:` section), `ref_xy` (reference point in
base_footprint; default the profile's `arm_mount_xy`), `facing_rad` (base axis that faces the wall, 0 = +x),
`tolerance_standoff_m` (0.005), `tolerance_yaw_rad`, `timeout_s`, `max_linear_mps`, `max_angular_radps` (docking caps),
`plane_max_age_s`, `wall_timeout_s`, `odom_timeout_s`, `rate_hz`, `odom_topic`, `cmd_vel_topic`,
`wall_scene_topic`.
"""
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from stand_navigator_msgs.action import WallDock

from stand_navigator.core import load_profile
from stand_navigator.wall_docking import (
    DockStatus, DockTolerance, WallDockController, measure_wall, plane_to_base, plane_to_world)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def plane_from_scene(scene):
    """(a, b, c, d) with (a, b, c) along `normal_outward`, and the scene's frame id, or None."""
    walls = scene.get('walls') or []
    if not walls:
        return None
    w = walls[0]
    a, b, c, d = (float(v) for v in w['plane_abcd'])
    n = w.get('normal_outward')
    if n is not None and a * n[0] + b * n[1] + c * n[2] < 0.0:
        a, b, c, d = -a, -b, -c, -d
    return (a, b, c, d), scene.get('frame_id', '')


class WallDocking(Node):
    def __init__(self):
        super().__init__('wall_docking')
        path = self.declare_parameter('base_profile', '').value
        if not path:
            raise RuntimeError('parameter base_profile (robot profile YAML) is required')
        self.profile = load_profile(path)
        ref = self.declare_parameter('ref_xy', [math.nan, math.nan]).value
        self.ref_xy = tuple(self.profile.arm_mount) if math.isnan(ref[0]) else (float(ref[0]), float(ref[1]))
        self.facing = self.declare_parameter('facing_rad', 0.0).value
        self.tol = DockTolerance(self.declare_parameter('tolerance_standoff_m', 0.005).value,
                                 self.declare_parameter('tolerance_yaw_rad', 0.02).value)
        self.timeout = self.declare_parameter('timeout_s', 60.0).value
        self.max_lin = self.declare_parameter('max_linear_mps', 0.15).value
        self.max_ang = self.declare_parameter('max_angular_radps', 0.3).value
        self.max_age = self.declare_parameter('plane_max_age_s', 3.0).value
        self.wall_timeout = self.declare_parameter('wall_timeout_s', 3.0).value
        self.odom_timeout = self.declare_parameter('odom_timeout_s', 1.0).value
        self.rate = self.declare_parameter('rate_hz', 20.0).value
        self.odom, self.odom_stamp = None, 0.0
        self.world_plane, self.plane_stamp = None, 0.0
        cb = ReentrantCallbackGroup()
        self.create_subscription(Odometry, self.declare_parameter('odom_topic', 'odom').value,
                                 self._on_odom, 10, callback_group=cb)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, self.declare_parameter(
            'wall_scene_topic', '/wall_detector/wall_scene').value, self._on_scene, latched, callback_group=cb)
        self.pub = self.create_publisher(Twist, self.declare_parameter('cmd_vel_topic', 'cmd_vel').value, 10)
        self.server = ActionServer(
            self, WallDock, 'wall_dock', self._execute, callback_group=cb,
            goal_callback=self._goal, cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.get_logger().info(
            f'wall_dock ready: drive {self.profile.drive}, ref {self.ref_xy}, facing {self.facing:.2f} rad, '
            f'caps {self.max_lin} m/s {self.max_ang} rad/s')

    def _pose(self):
        o = self.odom.pose.pose
        return o.position.x, o.position.y, yaw_of(o.orientation)

    def _on_odom(self, msg):
        self.odom, self.odom_stamp = msg, time.monotonic()

    def _on_scene(self, msg):
        try:
            got = plane_from_scene(json.loads(msg.data))
        except (ValueError, KeyError, TypeError) as e:
            self.get_logger().warn(f'unreadable wall scene: {e}')
            return
        if got is None or self.odom is None:
            return
        plane, frame = got
        # a plane in the odom frame is already a world plane; anything else is a base-frame plane of now
        self.world_plane = plane if frame == self.odom.header.frame_id else plane_to_world(plane, self._pose())
        self.plane_stamp = time.monotonic()

    def _goal(self, req):
        ok = math.isfinite(req.standoff_m) and req.standoff_m > 0.0 and math.isfinite(req.timeout_s)
        return GoalResponse.ACCEPT if ok else GoalResponse.REJECT

    def _stop(self):
        self.pub.publish(Twist())

    def _finish(self, handle, code, msg, m=None, ctl=None):
        res = WallDock.Result()
        res.error_code, res.error_msg = code, msg
        if m is not None and ctl is not None:
            res.standoff_error_m, res.yaw_error_rad = m.standoff - ctl.standoff, m.yaw_error
        if code == WallDock.Result.NONE:
            handle.succeed()
        elif code == WallDock.Result.CANCELED:
            handle.canceled()
        else:
            handle.abort()
        return res

    def _execute(self, handle):
        R = WallDock.Result
        goal = handle.request
        t0 = time.monotonic()
        while self.odom is None and time.monotonic() - t0 < 2.0:
            time.sleep(0.05)
        if self.odom is None:
            return self._finish(handle, R.NO_ODOM, 'no odometry')
        timeout = goal.timeout_s if goal.timeout_s > 0.0 else self.timeout
        ctl = WallDockController(
            self.profile, goal.standoff_m, self.tol, timeout, self.wall_timeout, self.facing,
            max_lin=self.max_lin, max_ang=self.max_ang)
        fb = WallDock.Feedback()
        m, last = None, time.monotonic()
        while rclpy.ok():
            now = time.monotonic()
            dt = max(now - last, 1e-3)
            last = now
            if handle.is_cancel_requested:
                self._stop()
                return self._finish(handle, R.CANCELED, 'canceled', m, ctl)
            if now - self.odom_stamp > self.odom_timeout:
                self._stop()
                return self._finish(handle, R.NO_ODOM, 'odometry went stale', m, ctl)
            pose, tw = self._pose(), self.odom.twist.twist
            fresh = self.world_plane is not None and now - self.plane_stamp <= self.max_age
            m = measure_wall(plane_to_base(self.world_plane, pose), self.ref_xy, self.facing) if fresh else None
            step = ctl.step(m, dt, math.hypot(tw.linear.x, tw.linear.y), abs(tw.angular.z))
            cmd = Twist()
            cmd.linear.x, cmd.linear.y, cmd.angular.z = step.cmd
            self.pub.publish(cmd)
            if step.status == DockStatus.SUCCESS:
                self._stop()
                return self._finish(handle, R.NONE, 'docked', m, ctl)
            if step.status == DockStatus.TIMEOUT:
                self._stop()
                return self._finish(handle, R.TIMEOUT, f'not docked after {timeout:.0f} s', m, ctl)
            if step.status == DockStatus.NO_WALL:
                self._stop()
                return self._finish(handle, R.NO_WALL, 'no fresh usable wall plane', m, ctl)
            fb.standoff_error_m, fb.yaw_error_rad = step.standoff_error, step.yaw_error
            handle.publish_feedback(fb)
            time.sleep(1.0 / self.rate)
        self._stop()
        return self._finish(handle, R.UNKNOWN, 'shutdown', m, ctl)


def main():
    rclpy.init()
    node = WallDocking()
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
