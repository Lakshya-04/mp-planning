import math

import pytest

from stand_navigator.core import BaseProfile, wrap
from stand_navigator.wall_docking import (
    DockStatus, DockTolerance, WallDockController, measure_wall, plane_to_base, plane_to_world)

OMNI = BaseProfile(drive='omni', max_lin=0.5, max_ang=1.0, max_lin_acc=0.5, max_ang_acc=1.0)
DIFF = BaseProfile(drive='diff', max_lin=0.5, max_ang=1.0, max_lin_acc=0.5, max_ang_acc=1.0)


def wall_plane(normal_angle, dist_from_origin):
    """World plane of a vertical wall: outward normal at `normal_angle`, passing `dist_from_origin` behind the world origin."""
    return math.cos(normal_angle), math.sin(normal_angle), 0.0, dist_from_origin


def run(profile, start, plane_w, standoff, ref=(0.0, 0.0), facing=0.0, dt=0.02, t_max=60.0, **kw):
    ctl = WallDockController(profile, standoff, timeout_s=t_max, facing_rad=facing, **kw)
    x, y, yaw = start
    prev, t, trace, status, m = (0.0, 0.0, 0.0), 0.0, [], DockStatus.RUNNING, None
    while t < t_max:
        m = measure_wall(plane_to_base(plane_w, (x, y, yaw)), ref, facing)
        step = ctl.step(m, dt, math.hypot(prev[0], prev[1]), abs(prev[2]))
        status = step.status
        trace.append((t, step.cmd, ctl.turning))
        if status != DockStatus.RUNNING:
            break
        vx, vy, wz = step.cmd
        c, s = math.cos(yaw), math.sin(yaw)
        x += (c * vx - s * vy) * dt
        y += (s * vx + c * vy) * dt
        yaw = wrap(yaw + wz * dt)
        prev = step.cmd
        t += dt
    return (x, y, yaw), status, t, trace, ctl, m


def test_plane_frame_roundtrip():
    plane = (0.6, 0.8, 0.0, -1.3)
    pose = (1.2, -0.7, 0.9)
    back = plane_to_base(plane_to_world(plane, pose), pose)
    assert back == pytest.approx(plane)


def test_measurement_on_a_known_wall():
    # wall at x = 0.98, normal -x (toward the robot at the origin): distance 0.98, facing it
    m = measure_wall((-1.0, 0.0, 0.0, 0.98))
    assert m.standoff == pytest.approx(0.98) and m.yaw_error == pytest.approx(0.0)
    # base yawed +15 deg: the wall is seen rotated -15 deg, the base must turn -15 deg
    yawed = plane_to_base((-1.0, 0.0, 0.0, 0.98), (0.0, 0.0, math.radians(15)))
    m = measure_wall(yawed)
    assert m.yaw_error == pytest.approx(-math.radians(15))
    assert m.standoff == pytest.approx(0.98)      # the distance to the plane does not depend on the yaw
    # reference point shifted ahead by 0.2 m shortens the standoff by 0.2
    assert measure_wall((-1.0, 0.0, 0.0, 0.98), ref_xy=(0.2, 0.0)).standoff == pytest.approx(0.78)


def test_rejects_floor_and_wall_behind_the_reference():
    assert measure_wall((0.0, 0.0, 1.0, 0.5)) is None
    assert measure_wall((-1.0, 0.0, 0.0, -0.1)) is None
    assert measure_wall((math.nan, 0.0, 0.0, 0.1)) is None
    assert measure_wall((0.0, 0.0, 0.0, 0.1)) is None


@pytest.mark.parametrize('profile', [OMNI, DIFF], ids=['omni', 'diff'])
@pytest.mark.parametrize('start,normal_angle', [
    ((0.0, 0.0, math.radians(15)), math.pi),              # 15 deg yawed, 0.4 m too far
    ((0.0, 0.0, -math.radians(30)), math.pi),
    ((0.5, 0.2, math.radians(10)), math.pi),              # too close, offset along the wall
    ((0.0, 0.0, math.radians(100)), math.pi),             # nearly sideways
    ((1.0, 1.0, 0.0), math.pi * 0.75),                    # wall at an angle in the world
])
def test_converges_to_standoff_and_square(profile, start, normal_angle):
    standoff = 0.55
    plane = wall_plane(normal_angle, 0.98 if abs(normal_angle - math.pi) < 1e-9 else 1.7)
    pose, status, t, _, ctl, m = run(profile, start, plane, standoff)
    assert status == DockStatus.SUCCESS, (status, t)
    # truth, independent of the measurement: signed distance of the final base position to the world plane, and the heading
    truth = plane[0] * pose[0] + plane[1] * pose[1] + plane[3]
    assert truth == pytest.approx(standoff, abs=0.006)
    assert abs(wrap(pose[2] - (normal_angle + math.pi))) <= 0.021


def test_limits_respected():
    _, _, _, trace, _, _ = run(DIFF, (0.0, 0.0, math.radians(40)), wall_plane(math.pi, 1.5), 0.5,
                               max_lin=0.2, max_ang=0.4)
    dt = 0.02
    prev = (0.0, 0.0, 0.0)
    for _, cmd, _ in trace:
        assert math.hypot(cmd[0], cmd[1]) <= 0.2 + 1e-9
        assert abs(cmd[2]) <= 0.4 + 1e-9
        assert math.hypot(cmd[0] - prev[0], cmd[1] - prev[1]) <= DIFF.max_lin_acc * dt + 1e-9
        assert abs(cmd[2] - prev[2]) <= DIFF.max_ang_acc * dt + 1e-9
        prev = cmd


def test_diff_never_strafes_and_turns_first():
    _, status, _, trace, _, _ = run(DIFF, (0.0, 0.0, math.radians(40)), wall_plane(math.pi, 1.5), 0.5)
    assert status == DockStatus.SUCCESS
    assert all(abs(cmd[1]) < 1e-12 for _, cmd, _ in trace)
    # while the yaw error is above the turn-first threshold there is no forward motion
    assert all(abs(cmd[0]) < 1e-12 for _, cmd, turning in trace if turning)
    assert any(turning for _, _, turning in trace)


def test_omni_translates_and_turns_together():
    _, _, _, trace, _, _ = run(OMNI, (0.0, 0.0, math.radians(40)), wall_plane(math.pi, 1.5), 0.5)
    both = [c for _, c, _ in trace if abs(c[2]) > 0.05 and math.hypot(c[0], c[1]) > 0.05]
    assert both
    # with the base yawed the body-frame velocity has a lateral part
    assert any(abs(c[1]) > 0.02 for _, c, _ in trace)


def test_diff_backs_up_when_too_close():
    pose, status, _, trace, _, _ = run(DIFF, (0.7, 0.0, 0.0), wall_plane(math.pi, 0.98), 0.55)
    assert status == DockStatus.SUCCESS
    assert any(c[0] < -0.01 for _, c, _ in trace)


def test_facing_the_rear_drives_backward_to_the_wall():
    # wall behind the base (normal +x... the wall at x = -1, robot at origin): the back faces it
    plane = wall_plane(0.0, 1.0)
    pose, status, _, trace, _, _ = run(DIFF, (0.0, 0.0, 0.0), plane, 0.5, facing=math.pi)
    assert status == DockStatus.SUCCESS
    assert pose[0] == pytest.approx(-0.5, abs=0.006)
    assert all(c[0] <= 1e-9 for _, c, _ in trace)


def test_timeout():
    _, status, t, _, _, _ = run(DIFF, (0.0, 0.0, 0.0), wall_plane(math.pi, 5.0), 0.5, t_max=3.0)
    assert status == DockStatus.TIMEOUT and t == pytest.approx(3.0, abs=0.1)


def test_no_wall_after_the_timeout_and_the_command_decays():
    ctl = WallDockController(OMNI, 0.5, wall_timeout_s=1.0)
    m = measure_wall((-1.0, 0.0, 0.0, 2.0))
    for _ in range(50):
        ctl.step(m, 0.02)
    assert ctl.cmd[0] > 0.1
    status = DockStatus.RUNNING
    for _ in range(80):
        s = ctl.step(None, 0.02)
        status = s.status
    assert status == DockStatus.NO_WALL
    assert math.hypot(*ctl.cmd[:2]) < 1e-9


def test_tolerance_reported_and_not_docked_while_moving():
    ctl = WallDockController(OMNI, 0.55, tol=DockTolerance(0.01, 0.02, 0.02))
    m = measure_wall((-1.0, 0.0, 0.0, 0.55))
    assert ctl.step(m, 0.02, 0.1, 0.0).status == DockStatus.RUNNING
    s = ctl.step(m, 0.02, 0.0, 0.0)
    assert s.status == DockStatus.SUCCESS and s.standoff_error == pytest.approx(0.0)
