import math
import os
import tempfile

import pytest

from stand_navigator.core import (
    BaseProfile, StandController, Tolerance, arm_to_footprint, footprint_to_arm, load_profile, wrap)


def simulate(profile, start, goal, dt=0.02, t_max=60.0, ctl_kwargs=None):
    """Integrate the commanded body velocity exactly (ideal base) and return (pose, time, trace)."""
    ctl = StandController(profile, goal, **(ctl_kwargs or {}))
    x, y, yaw = start
    t, trace, speed = 0.0, [], 0.0
    while t < t_max:
        vx, vy, wz = ctl.step((x, y, yaw), dt)
        speed = math.hypot(vx, vy)
        c, s = math.cos(yaw), math.sin(yaw)
        x += (c * vx - s * vy) * dt
        y += (s * vx + c * vy) * dt
        yaw = wrap(yaw + wz * dt)
        t += dt
        trace.append((t, vx, vy, wz, ctl.phase))
        if ctl.arrived((x, y, yaw), speed, abs(wz)):
            break
    return (x, y, yaw), t, trace, ctl


def test_arm_footprint_roundtrip():
    mount = (0.2, -0.1)
    for yaw in (0.0, 1.0, -2.5, math.pi):
        fx, fy, _ = arm_to_footprint(3.0, -1.0, yaw, mount)
        ax, ay, _ = footprint_to_arm(fx, fy, yaw, mount)
        assert (ax, ay) == pytest.approx((3.0, -1.0))


def test_arm_mount_shift_is_rotated_by_yaw():
    fx, fy, _ = arm_to_footprint(0.0, 0.0, math.pi / 2, (0.5, 0.0))
    assert (fx, fy) == pytest.approx((0.0, -0.5))   # arm 0.5 m ahead of the footprint, facing +y


def test_zero_mount_is_identity():
    assert arm_to_footprint(1.0, 2.0, 0.7, (0.0, 0.0)) == (1.0, 2.0, 0.7)


def test_load_profile_matches_repo_profile():
    here = os.path.dirname(os.path.abspath(__file__))
    for up in range(2, 8):
        root = os.path.abspath(os.path.join(here, *(['..'] * up)))
        path = os.path.join(root, 'description', 'mobile_manipulator_description', 'config', 'profiles',
                            'ur10e_telescopic.yaml')
        if os.path.exists(path):
            break
    else:
        pytest.skip('profile not found (installed package)')
    p = load_profile(path)
    assert p.drive == 'omni' and p.max_lin == 0.5 and p.max_lin_acc == 0.5 and p.arm_mount == (0.0, 0.0)


def test_load_profile_arm_mount_and_diff(tmp_path):
    f = tmp_path / 'p.yaml'
    f.write_text('base:\n  drive_type: diff\n  max_linear_velocity_mps: 0.3\n  arm_mount_xy: [0.25, 0.05]\n')
    p = load_profile(str(f))
    assert p.drive == 'diff' and not p.holonomic and p.max_lin == 0.3 and p.arm_mount == (0.25, 0.05)


@pytest.mark.parametrize('goal', [(1.0, 0.0, 0.0), (-0.8, 0.6, 1.2), (0.0, -1.5, -3.0), (0.3, 0.2, 0.0)])
def test_omni_arrives_within_tolerance(goal):
    prof = BaseProfile(drive='omni')
    pose, t, _, ctl = simulate(prof, (0.0, 0.0, 0.0), goal)
    d, e = ctl.errors(pose)
    assert d <= 0.02 and abs(e) <= 0.02 and t < 30.0


@pytest.mark.parametrize('goal', [(1.0, 0.0, 0.0), (-0.8, 0.6, 1.2), (0.0, -1.5, -3.0), (-1.0, 0.0, 0.0),
                                  (0.3, 0.2, 2.0)])
def test_diff_arrives_within_tolerance(goal):
    prof = BaseProfile(drive='diff', max_lin=0.4, max_ang=0.8)
    pose, t, trace, ctl = simulate(prof, (0.0, 0.0, 0.0), goal)
    d, e = ctl.errors(pose)
    assert d <= 0.02 and abs(e) <= 0.02 and t < 59.0
    assert all(abs(vy) < 1e-9 for _, _, vy, _, _ in trace)   # a diff base never strafes


def test_limits_respected_incl_acceleration():
    prof = BaseProfile(drive='omni', max_lin=0.3, max_ang=0.5, max_lin_acc=0.2, max_ang_acc=0.4)
    dt = 0.02
    _, _, trace, _ = simulate(prof, (0.0, 0.0, 0.0), (2.0, 1.0, 2.0), dt=dt)
    prev = (0.0, 0.0, 0.0)
    for _, vx, vy, wz, _ in trace:
        assert math.hypot(vx, vy) <= 0.3 + 1e-9 and abs(wz) <= 0.5 + 1e-9
        assert math.hypot(vx - prev[0], vy - prev[1]) <= 0.2 * dt + 1e-9
        assert abs(wz - prev[2]) <= 0.4 * dt + 1e-9
        prev = (vx, vy, wz)


def test_no_overshoot_on_final_approach():
    prof = BaseProfile(drive='omni')
    _, _, trace, _ = simulate(prof, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    # integrate forward distance: never beyond the goal by more than the tolerance
    x = 0.0
    for _, vx, _, _, _ in trace:
        x += vx * 0.02
        assert x <= 1.0 + 0.02


def test_diff_turns_first_when_goal_behind_side():
    prof = BaseProfile(drive='diff')
    _, _, trace, _ = simulate(prof, (0.0, 0.0, 0.0), (0.0, 2.0, 0.0))
    assert trace[0][4] == 'turn' or abs(trace[3][3]) > 0     # starts by turning toward +y
    assert abs(trace[5][1]) < 0.05                           # little forward speed during the turn


def test_diff_reverses_for_goal_behind_when_close():
    prof = BaseProfile(drive='diff')
    _, _, trace, _ = simulate(prof, (0.0, 0.0, 0.0), (-0.3, 0.0, 0.0))
    assert min(vx for _, vx, _, _, _ in trace) < -0.05


def test_already_there_stops_immediately():
    ctl = StandController(BaseProfile(), (1.0, 1.0, 0.5))
    assert ctl.arrived((1.0, 1.0, 0.5))
    assert ctl.step((1.0, 1.0, 0.5), 0.02) == (0.0, 0.0, 0.0)


def test_arrival_requires_speed_and_yaw():
    ctl = StandController(BaseProfile(), (0.0, 0.0, 0.0), tol=Tolerance(xy=0.02, yaw=0.02, speed=0.02))
    assert not ctl.arrived((0.0, 0.0, 0.1))
    assert not ctl.arrived((0.0, 0.0, 0.0), lin_speed=0.2)
    assert not ctl.arrived((0.1, 0.0, 0.0))


def test_yaw_wraps_the_short_way():
    ctl = StandController(BaseProfile(), (0.0, 0.0, -3.0))
    _, _, wz = ctl._target((0.0, 0.0, 3.0))
    assert wz > 0   # 3.0 -> -3.0 is a short +0.28 turn through pi, not -6
