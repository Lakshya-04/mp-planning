"""ROS-free core of the wall docking servo: from the fitted wall plane, drive the base to a standoff and square to the wall.

The plane is `n . p + d = 0` with `n` the outward normal (pointing from the wall into the free space, as
`wall_detector` writes it), in the base frame (x forward, y left, z up). Only the horizontal part of the
normal matters: the base cannot tilt, and a plane whose normal is mostly vertical is a floor or a ceiling
(no measurement).

Two numbers drive the base:

- standoff: distance from a reference point of the base (`ref_xy`, the arm base in practice) to the plane;
- yaw error: the rotation that makes the base's facing axis (`facing_rad` from +x: 0 = the front, pi = the
  back) point straight into the wall.

Omni (and 4ws): translate along the wall normal and turn together. Diff and skid: no sideways velocity, so
turn first until the yaw error is small, then drive along the facing axis (turning on the yaw error), so the
motion is along the wall normal (the facing axis must be the drive axis: 0 or pi). Velocity and acceleration
limits come from the same `BaseProfile` as the stand navigator; docked is standoff and yaw within tolerance
with the base nearly at rest.

Optional lateral term (off by default, `LateralConfig`): the plane does not fix the position ALONG the wall, so a
third error can be servoed, the signed distance the base must slide along the wall tangent (`+` toward the left when
facing the wall) to reach a target. It comes from a `LateralSource` (planned stand against odom, or the distance to
a visible wall edge) and is put in `WallMeasurement.lateral`. Omni bases slide; a base without lateral velocity cannot,
so the controller reports `NEEDS_REAPPROACH` (outcome `lateral_unreachable`) and `plan_lateral_manoeuvre` gives the
back-off, turn, drive, turn, approach primitives the caller runs before docking again.
"""
import math
from dataclasses import dataclass, replace
from enum import Enum
from typing import Optional

from stand_navigator.core import clamp, wrap


class DockStatus(Enum):
    RUNNING = 'running'
    SUCCESS = 'success'
    TIMEOUT = 'timeout'
    NO_WALL = 'no_wall'
    NEEDS_REAPPROACH = 'needs_reapproach'   # lateral error that a sliding correction cannot fix (see LateralConfig)
    BLOCKED = 'blocked'                     # commanding motion but the base is not moving (needs blocked_timeout_s)


class DockOutcome(Enum):
    """Typed result of a dock attempt; same lower_snake naming as the manager's stand_check reason codes."""
    DOCKED = 'docked'
    TIMEOUT = 'timeout'
    LOST_WALL = 'lost_wall'
    LATERAL_UNREACHABLE = 'lateral_unreachable'
    BLOCKED = 'blocked'


_OUTCOME = {DockStatus.SUCCESS: DockOutcome.DOCKED, DockStatus.TIMEOUT: DockOutcome.TIMEOUT,
            DockStatus.NO_WALL: DockOutcome.LOST_WALL, DockStatus.NEEDS_REAPPROACH: DockOutcome.LATERAL_UNREACHABLE,
            DockStatus.BLOCKED: DockOutcome.BLOCKED}


def outcome_of(status):
    """DockOutcome of a terminal DockStatus (None while RUNNING)."""
    return _OUTCOME.get(status)


@dataclass
class DockTolerance:
    standoff: float = 0.005     # m
    yaw: float = 0.02           # rad
    speed: float = 0.02         # m/s and rad/s the base must be below to count as docked


@dataclass
class WallMeasurement:
    standoff: float             # m, reference point to plane along the normal (positive on the free side)
    yaw_error: float            # rad, rotation that squares the facing axis to the wall
    into_wall: float            # rad, direction of the wall normal (into the wall) in the base frame
    tilt: float                 # rad, angle of the normal from the horizontal plane
    lateral: Optional[float] = None     # m, slide still needed along the wall tangent (see LateralSource); None = unknown


@dataclass
class LateralConfig:
    """Lateral-along-the-wall term. Off by default: with `enabled` False the controller is unchanged."""
    enabled: bool = False
    tol: float = 0.01               # m, docked needs |lateral error| within this
    max_speed: float = 0.05         # m/s cap of the sliding speed
    kp: float = 1.0
    max_correctable: float = 1.0    # m, a larger error is not slid out: NEEDS_REAPPROACH (lateral_unreachable)

    def __post_init__(self):
        for name in ('tol', 'max_speed', 'kp', 'max_correctable'):
            v = getattr(self, name)
            if not (math.isfinite(v) and v > 0.0):
                raise ValueError(f'LateralConfig.{name} must be finite and > 0, got {v}')


@dataclass
class LateralContext:
    """What a lateral source may look at. `ref_world_xy`/`base_yaw` are the odom pose of the reference point and
    the base; `u_range` is the visible wall extent (u_min, u_max) along the tangent measured from the reference point
    (u > 0 toward the left when facing the wall), from the fitted plane's inlier extent when the detector gives it."""
    m: WallMeasurement
    ref_world_xy: Optional[tuple] = None
    base_yaw: float = 0.0
    u_range: Optional[tuple] = None


def tangent_angle(m):
    """Direction of the wall tangent (to the left when facing the wall) in the base frame."""
    return wrap(m.into_wall + 0.5 * math.pi)


class LateralSource:
    """Interface: `error(ctx)` -> the signed slide along the tangent still needed (m), or None when this source cannot say."""

    def error(self, ctx):
        raise NotImplementedError


class PlannedStandLateral(LateralSource):
    """(a) Planned stand against odom: the part of (planned - current) along the wall tangent. It is only as good as the
    odometry (drift is invisible to it), the plane does not enter except for the tangent direction."""

    def __init__(self, planned_xy):
        self.planned = planned_xy

    def error(self, ctx):
        if ctx.ref_world_xy is None or not all(map(math.isfinite, (*ctx.ref_world_xy, *self.planned))):
            return None
        a = ctx.base_yaw + tangent_angle(ctx.m)
        e = (self.planned[0] - ctx.ref_world_xy[0]) * math.cos(a) + (self.planned[1] - ctx.ref_world_xy[1]) * math.sin(a)
        return e if math.isfinite(e) else None


class WallEdgeLateral(LateralSource):
    """(b) Wall feature: the visible wall extent `u_range` against a wanted distance to an edge.
    `edge` 'left': keep `target` m from the left edge; 'right': from the right edge; 'center': be `target` m
    left of the middle of the visible extent (0 = centred). Independent of odometry, but only as good as the edge
    detection (the visible extent can be cut by occlusion or the field of view: then it is wrong, not missing)."""

    def __init__(self, target, edge='left'):
        if edge not in ('left', 'right', 'center'):
            raise ValueError('edge must be left, right or center')
        self.target, self.edge = target, edge

    def error(self, ctx):
        if ctx.u_range is None or not all(map(math.isfinite, ctx.u_range)):
            return None
        u_min, u_max = ctx.u_range
        e = edge_lateral_error(u_min, u_max, self.target, self.edge)
        return e if math.isfinite(e) else None


def edge_lateral_error(u_min, u_max, target, edge='left'):
    """Slide s along +tangent that puts the base `target` m from `edge` of the wall [u_min, u_max] (base at u = 0)."""
    if edge == 'left':
        return u_max - target            # distance to the left edge is u_max, sliding left shortens it
    if edge == 'right':
        return target + u_min            # distance to the right edge is -u_min, sliding left lengthens it
    return 0.5 * (u_min + u_max) - target


def with_lateral(m, source, ctx):
    """Copy of the measurement with the source's lateral error filled in (None when the source has none)."""
    return replace(m, lateral=source.error(replace(ctx, m=m)))


def plan_lateral_manoeuvre(lateral_error, backoff=0.3, facing_rad=0.0, yaw_error=0.0):
    """Back-off, turn, slide, turn back, approach for a base that cannot move sideways, as primitives
    [('drive', m), ('turn', rad), ...] in the base frame (drive: + along +x; turn: + counterclockwise). The caller runs
    them on odom (open or closed loop) and docks again; the last drive returns to about the original standoff and the
    dock servo trims it. The tangent is 90 deg left of the direction into the wall, so with the front to the wall
    (`facing_rad` 0) a positive error turns left and with the rear to the wall (pi) it turns right, and the
    back-off and approach drives are reversed. A nonzero `yaw_error` (WallMeasurement.yaw_error) is turned out first,
    so the manoeuvre starts square to the wall; the slide is along the wall, so the lateral error stays valid."""
    fwd = 1.0 if math.cos(facing_rad) >= 0.0 else -1.0
    turn = (1.0 if lateral_error >= 0.0 else -1.0) * fwd * 0.5 * math.pi
    prims = [('drive', -fwd * backoff), ('turn', turn), ('drive', abs(lateral_error)),
             ('turn', -turn), ('drive', fwd * backoff)]
    return ([('turn', yaw_error)] + prims) if yaw_error else prims


@dataclass
class DockResult:
    """Final numbers of a dock attempt (plain dataclass; the WallDock action keeps its standoff and yaw fields,
    the lateral error and the outcome are the follow-up for the .action file)."""
    outcome: Optional[DockOutcome]       # None while the dock is still running
    standoff_error: float
    yaw_error: float
    lateral_error: Optional[float]
    elapsed_s: float


def plane_to_world(plane, pose):
    """Plane (a, b, c, d) in the base frame -> in the odom frame, for the base pose (x, y, yaw)."""
    a, b, c, d = plane
    cy, sy = math.cos(pose[2]), math.sin(pose[2])
    nx, ny = cy * a - sy * b, sy * a + cy * b
    return nx, ny, c, d - (nx * pose[0] + ny * pose[1])


def plane_to_base(plane, pose):
    """Plane in the odom frame -> in the base frame, for the base pose."""
    a, b, c, d = plane
    cy, sy = math.cos(pose[2]), math.sin(pose[2])
    return cy * a + sy * b, -sy * a + cy * b, c, d + a * pose[0] + b * pose[1]


def measure_wall(plane, ref_xy=(0.0, 0.0), facing_rad=0.0, max_tilt_rad=0.5):
    """Standoff and yaw error from a base-frame plane, or None when it is not a usable wall
    (non-finite, normal too vertical, reference point behind the plane)."""
    if not all(map(math.isfinite, plane)):
        return None
    a, b, c, d = plane
    n = math.sqrt(a * a + b * b + c * c)
    if n < 1e-9:
        return None
    a, b, c, d = a / n, b / n, c / n, d / n
    tilt = math.asin(min(1.0, abs(c)))
    if tilt > max_tilt_rad:
        return None
    standoff = (a * ref_xy[0] + b * ref_xy[1] + d) / max(math.hypot(a, b), 1e-9)   # along the horizontal normal
    if standoff < 0.0:
        return None
    into = math.atan2(-b, -a)
    return WallMeasurement(standoff, wrap(into - facing_rad), into, tilt)


@dataclass
class DockStep:
    cmd: tuple                  # (vx, vy, wz) in the base frame
    status: DockStatus
    standoff_error: float = 0.0
    yaw_error: float = 0.0
    lateral_error: Optional[float] = None
    outcome: Optional[DockOutcome] = None
    reapproach: bool = False


class WallDockController:
    """Proportional standoff and yaw servo with velocity and acceleration limits, tolerances and a timeout.

    `step(measurement, dt, lin_speed, ang_speed)` returns a `DockStep`. Pass `None` when there is no fresh
    plane: the command decays to zero, and after `wall_timeout_s` without one the status is NO_WALL.
    """

    def __init__(self, profile, standoff, tol=None, timeout_s=60.0, wall_timeout_s=2.0, facing_rad=0.0,
                 kp_lin=1.0, kp_ang=1.5, turn_first_rad=0.15, max_lin=None, max_ang=None, creep=0.01,
                 lateral=None, blocked_timeout_s=0.0, blocked_cmd_min=0.03):
        self.p, self.standoff, self.facing = profile, standoff, facing_rad
        self.tol = tol or DockTolerance()
        self.timeout_s, self.wall_timeout_s = timeout_s, wall_timeout_s
        self.kp_lin, self.kp_ang, self.turn_first = kp_lin, kp_ang, turn_first_rad
        # docking is close to a wall: the caller may cap the speeds below the profile's travel speeds
        self.max_lin = min(max_lin, profile.max_lin) if max_lin else profile.max_lin
        self.max_ang = min(max_ang, profile.max_ang) if max_ang else profile.max_ang
        self.creep = creep          # m/s and rad/s floor so a small error still closes
        self.cmd = (0.0, 0.0, 0.0)
        self.elapsed = 0.0
        self.since_wall = 0.0
        self.turning = False
        self.lat = lateral or LateralConfig()
        self.blocked_timeout_s, self.blocked_cmd_min = blocked_timeout_s, blocked_cmd_min   # 0 = no stall check
        self.stalled = 0.0
        self.last = DockStep(self.cmd, DockStatus.RUNNING)

    def result(self, step=None):
        """DockResult of a step (default: the last one)."""
        st = step or self.last
        return DockResult(st.outcome, st.standoff_error, st.yaw_error, st.lateral_error, self.elapsed)

    def _lateral_velocity(self, m):
        """(vx, vy) of the sliding correction in the base frame (holonomic bases)."""
        e = m.lateral
        if not self.lat.enabled or e is None or abs(e) <= 0.5 * self.lat.tol or abs(e) > self.lat.max_correctable:
            return 0.0, 0.0
        v = min(self.lat.kp * abs(e), self.lat.max_speed,
                math.sqrt(2.0 * self.p.max_lin_acc * abs(e)) + self.creep)
        a = tangent_angle(m)
        sign = 1.0 if e > 0 else -1.0
        return sign * v * math.cos(a), sign * v * math.sin(a)

    def _target(self, m):
        p = self.p
        e = m.standoff - self.standoff               # positive: too far, drive toward the wall
        yaw = m.yaw_error
        wz = clamp(self.kp_ang * yaw,
                   min(self.max_ang, math.sqrt(2.0 * p.max_ang_acc * abs(yaw)) + self.creep))
        speed = 0.0
        if abs(e) > 0.5 * self.tol.standoff:
            speed = min(self.kp_lin * abs(e), self.max_lin,
                        math.sqrt(2.0 * p.max_lin_acc * abs(e)) + self.creep)
        sign = 1.0 if e > 0 else -1.0
        if p.holonomic:
            lx, ly = self._lateral_velocity(m)
            return (sign * speed * math.cos(m.into_wall) + lx, sign * speed * math.sin(m.into_wall) + ly, wz)
        if abs(yaw) > self.turn_first:
            self.turning = True
        elif abs(yaw) < 0.5 * self.turn_first:
            self.turning = False
        if self.turning:
            return 0.0, 0.0, wz
        # facing axis forward (facing 0) or backward (facing pi); the cosine drops the speed while the yaw is off
        fwd = 1.0 if math.cos(self.facing) >= 0.0 else -1.0
        return sign * fwd * speed * max(0.0, math.cos(yaw)), 0.0, wz

    def step(self, m, dt, lin_speed=0.0, ang_speed=0.0):
        p = self.p
        self.elapsed += dt
        e = (0.0, 0.0)
        lat = None
        reapproach = False
        if m is None:
            self.since_wall += dt
            tx = ty = tw = 0.0
            status = DockStatus.NO_WALL if self.since_wall > self.wall_timeout_s else DockStatus.RUNNING
        else:
            self.since_wall = 0.0
            if m.lateral is not None and not math.isfinite(m.lateral):
                m = replace(m, lateral=None)          # a NaN reading is no reading
            tx, ty, tw = self._target(m)
            e = (m.standoff - self.standoff, m.yaw_error)
            lat = m.lateral if self.lat.enabled else None
            if lat is not None and not math.isfinite(lat):
                lat = None
            status = DockStatus.RUNNING
            at_rest = lin_speed <= self.tol.speed and ang_speed <= self.tol.speed
            plane_ok = abs(e[0]) <= self.tol.standoff and abs(e[1]) <= self.tol.yaw
            lat_ok = not self.lat.enabled or (lat is not None and abs(lat) <= self.lat.tol)
            if plane_ok and at_rest and lat_ok:
                status = DockStatus.SUCCESS
            elif self.lat.enabled and lat is not None and abs(lat) > self.lat.tol:
                too_big = abs(lat) > self.lat.max_correctable
                if too_big or (not p.holonomic and plane_ok and at_rest):
                    status = DockStatus.NEEDS_REAPPROACH
                    reapproach = True
            if status == DockStatus.RUNNING and self.blocked_timeout_s > 0.0:
                cmd_n = math.hypot(self.cmd[0], self.cmd[1])
                if cmd_n > self.blocked_cmd_min and lin_speed < 0.25 * cmd_n:
                    self.stalled += dt
                    if self.stalled > self.blocked_timeout_s:
                        status = DockStatus.BLOCKED
                else:
                    self.stalled = 0.0
        if status == DockStatus.RUNNING and self.elapsed > self.timeout_s:
            status = DockStatus.TIMEOUT
        if status != DockStatus.RUNNING and status != DockStatus.NO_WALL:
            tx = ty = tw = 0.0
        n = math.hypot(tx, ty)
        if n > self.max_lin:
            tx, ty = tx * self.max_lin / n, ty * self.max_lin / n
        tw = clamp(tw, self.max_ang)
        dv = p.max_lin_acc * dt
        ex, ey = tx - self.cmd[0], ty - self.cmd[1]
        en = math.hypot(ex, ey)
        if en > dv:
            ex, ey = ex * dv / en, ey * dv / en
        wz = self.cmd[2] + clamp(tw - self.cmd[2], p.max_ang_acc * dt)
        self.cmd = (self.cmd[0] + ex, self.cmd[1] + ey, wz)
        self.last = DockStep(self.cmd, status, e[0], e[1], lat, outcome_of(status), reapproach)
        return self.last
