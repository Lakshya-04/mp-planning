"""ROS-free core of the stand navigator: frame conversion, the base profile and the go-to-pose controller.

Everything is in the odometry frame (x, y, yaw). A stand is where the ARM BASE goes (reachability
`Stand.x/.y`); the base is driven by `base_footprint`, so the goal is shifted by the profile's
`arm_mount_xy` rotated by the yaw.
"""
import math
from dataclasses import dataclass

import yaml


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lim):
    return max(-lim, min(lim, v))


@dataclass
class BaseProfile:
    drive: str = 'omni'                 # omni | diff | skid | 4ws
    max_lin: float = 0.5                # m/s
    max_ang: float = 1.0                # rad/s
    max_lin_acc: float = 0.5            # m/s^2
    max_ang_acc: float = 1.0            # rad/s^2
    arm_mount: tuple = (0.0, 0.0)       # arm base in base_footprint

    @property
    def holonomic(self):
        return self.drive in ('omni', '4ws')


def load_profile(path):
    """Read the `base:` section of a robot profile YAML (the file reachability reads too)."""
    with open(path) as f:
        b = yaml.safe_load(f)['base']
    d = BaseProfile()
    mount = b.get('arm_mount_xy') or [0.0, 0.0]
    return BaseProfile(
        drive=str(b.get('drive_type', d.drive)),
        max_lin=float(b.get('max_linear_velocity_mps', d.max_lin)),
        max_ang=float(b.get('max_angular_velocity_radps', d.max_ang)),
        max_lin_acc=float(b.get('max_linear_acceleration_mps2', d.max_lin_acc)),
        max_ang_acc=float(b.get('max_angular_acceleration_radps2', d.max_ang_acc)),
        arm_mount=(float(mount[0]), float(mount[1])))


def arm_to_footprint(arm_x, arm_y, yaw, mount):
    """base_footprint pose (x, y, yaw) that puts the arm base at (arm_x, arm_y) with base yaw `yaw`."""
    c, s = math.cos(yaw), math.sin(yaw)
    return arm_x - (c * mount[0] - s * mount[1]), arm_y - (s * mount[0] + c * mount[1]), yaw


def footprint_to_arm(x, y, yaw, mount):
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * mount[0] - s * mount[1], y + s * mount[0] + c * mount[1], yaw


@dataclass
class Tolerance:
    xy: float = 0.02
    yaw: float = 0.02
    speed: float = 0.02      # m/s and rad/s the base must be below to count as arrived


def _approach_speed(dist, vmax, acc):
    """Fastest speed from which `acc` can still stop within `dist` (square-root braking profile)."""
    return min(vmax, math.sqrt(2.0 * acc * max(dist, 0.0)))


class StandController:
    """Proportional go-to-pose with braking ramps and acceleration limits.

    `step(pose, dt)` -> (vx, vy, wz) in the base frame for a base_footprint pose; the goal is a
    base_footprint pose too. Omni: translate and turn together. Diff: turn toward the goal, drive
    forward steering on the heading error (reversing when the goal is behind), then turn to the
    final yaw.
    """

    def __init__(self, profile, goal, tol=None, kp_lin=1.0, kp_ang=1.5, turn_first_rad=0.6):
        self.p, self.goal = profile, goal
        self.tol = tol or Tolerance()
        self.kp_lin, self.kp_ang, self.turn_first = kp_lin, kp_ang, turn_first_rad
        self.cmd = (0.0, 0.0, 0.0)
        self.phase = 'drive'

    def errors(self, pose):
        gx, gy, gyaw = self.goal
        return math.hypot(gx - pose[0], gy - pose[1]), wrap(gyaw - pose[2])

    def arrived(self, pose, lin_speed=0.0, ang_speed=0.0):
        d, e = self.errors(pose)
        return (d <= self.tol.xy and abs(e) <= self.tol.yaw
                and lin_speed <= self.tol.speed and ang_speed <= self.tol.speed)

    def _target(self, pose):
        p, (gx, gy, gyaw) = self.p, self.goal
        dx, dy = gx - pose[0], gy - pose[1]
        dist = math.hypot(dx, dy)
        yaw_err = wrap(gyaw - pose[2])
        c, s = math.cos(pose[2]), math.sin(pose[2])
        bx, by = c * dx + s * dy, -s * dx + c * dy      # goal in the base frame
        wz_final = clamp(self.kp_ang * yaw_err,
                         _approach_speed(abs(yaw_err), p.max_ang, p.max_ang_acc))
        if p.holonomic:
            if dist < 1e-6:
                return 0.0, 0.0, wz_final
            v = min(self.kp_lin * dist, _approach_speed(dist, p.max_lin, p.max_lin_acc))
            return v * bx / dist, v * by / dist, wz_final
        # non-holonomic: no lateral velocity; near the goal only the final turn is left
        if dist <= self.tol.xy:
            self.phase = 'rotate'
            return 0.0, 0.0, wz_final
        head = math.atan2(by, bx)
        reverse = abs(head) > math.pi / 2
        if reverse:
            head = wrap(head + math.pi)
        if self.phase == 'rotate':
            self.phase = 'drive'    # drifted off the goal during the final turn
        if abs(head) > self.turn_first and dist > 3 * self.tol.xy:
            self.phase = 'turn'
        elif self.phase == 'turn' and abs(head) < 0.1:
            self.phase = 'drive'
        if self.phase == 'turn':
            return 0.0, 0.0, clamp(self.kp_ang * head, p.max_ang)
        v = min(self.kp_lin * dist, _approach_speed(dist, p.max_lin, p.max_lin_acc))
        v *= max(0.0, math.cos(head)) * (-1.0 if reverse else 1.0)
        # steer on the heading error, fading out near the goal so it cannot spin about it
        w = clamp(self.kp_ang * head, p.max_ang) * min(1.0, dist / 0.2)
        return v, 0.0, w

    def step(self, pose, dt):
        tx, ty, tw = self._target(pose)
        p = self.p
        n = math.hypot(tx, ty)
        if n > p.max_lin:
            tx, ty = tx * p.max_lin / n, ty * p.max_lin / n
        tw = clamp(tw, p.max_ang)
        dv = p.max_lin_acc * dt
        ex, ey = tx - self.cmd[0], ty - self.cmd[1]
        en = math.hypot(ex, ey)
        if en > dv:
            ex, ey = ex * dv / en, ey * dv / en
        wz = self.cmd[2] + clamp(tw - self.cmd[2], p.max_ang_acc * dt)
        self.cmd = (self.cmd[0] + ex, self.cmd[1] + ey, wz)
        return self.cmd
