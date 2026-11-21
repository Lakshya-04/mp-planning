import math

import pytest

from stand_navigator.core import BaseProfile, wrap
from stand_navigator.wall_docking import (
    DockOutcome, DockResult, DockStatus, LateralConfig, LateralContext, PlannedStandLateral, WallDockController,
    WallEdgeLateral, edge_lateral_error, measure_wall, outcome_of, plan_lateral_manoeuvre, plane_to_base,
    tangent_angle, with_lateral)

OMNI = BaseProfile(drive='omni', max_lin=0.5, max_ang=1.0, max_lin_acc=0.5, max_ang_acc=1.0)
DIFF = BaseProfile(drive='diff', max_lin=0.5, max_ang=1.0, max_lin_acc=0.5, max_ang_acc=1.0)
PLANE = (-1.0, 0.0, 0.0, 0.55)       # wall in front, base at 0.55 m: docked in standoff and yaw
LAT = LateralConfig(enabled=True, tol=0.01, max_speed=0.05)


def meas(lateral=None, standoff_plane=PLANE):
    m = measure_wall(standoff_plane)
    m.lateral = lateral
    return m


def settle(ctl, m, n=100, dt=0.02):
    """Step with a fixed measurement until the command has ramped to its target."""
    s = None
    for _ in range(n):
        s = ctl.step(m, dt, 0.5, 0.5)       # moving fast: never docked while settling
    return s


# --- lateral off keeps the old behaviour ---------------------------------------------------------------------

def test_off_ignores_the_lateral_field():
    a = WallDockController(OMNI, 0.5)
    b = WallDockController(OMNI, 0.5, lateral=LateralConfig(enabled=False))
    for lat in (None, 0.3, -0.7):
        m0 = measure_wall((-1.0, 0.0, 0.0, 1.0))
        m1 = measure_wall((-1.0, 0.0, 0.0, 1.0))
        m1.lateral = lat
        sa, sb = a.step(m0, 0.02), b.step(m1, 0.02)
        assert sa.cmd == sb.cmd and sa.status == sb.status


def test_off_is_docked_without_a_lateral_measurement():
    ctl = WallDockController(OMNI, 0.55)
    assert ctl.step(meas(None), 0.02).status == DockStatus.SUCCESS


# --- measurement math -----------------------------------------------------------------------------------------

def test_edge_error_signs():
    # wall from 0.4 m right to 1.2 m left of the base; keep 0.5 m from the left edge: slide right (negative) by 0.7
    assert edge_lateral_error(-0.4, 1.2, 0.5, 'left') == pytest.approx(0.7)
    assert edge_lateral_error(-0.4, 1.2, 0.5, 'right') == pytest.approx(0.1)
    assert edge_lateral_error(-0.4, 1.2, 0.0, 'center') == pytest.approx(0.4)
    # already 0.5 m from the left edge
    assert edge_lateral_error(-2.0, 0.5, 0.5, 'left') == pytest.approx(0.0)
    assert edge_lateral_error(-0.5, 2.0, 0.5, 'right') == pytest.approx(0.0)


def test_edge_error_is_a_slide_that_reaches_the_target():
    u_min, u_max, target = -0.3, 1.5, 0.6
    s = edge_lateral_error(u_min, u_max, target, 'left')
    assert (u_max - s) == pytest.approx(target)            # after sliding by s left, the left edge is `target` away
    s = edge_lateral_error(u_min, u_max, target, 'right')
    assert (-(u_min - s)) == pytest.approx(target)


def test_edge_source_needs_a_valid_extent():
    src = WallEdgeLateral(0.5)
    m = meas()
    assert src.error(LateralContext(m)) is None
    assert src.error(LateralContext(m, u_range=(-1.0, math.nan))) is None
    assert src.error(LateralContext(m, u_range=(-0.4, 1.2))) == pytest.approx(0.7)
    with pytest.raises(ValueError):
        WallEdgeLateral(0.5, edge='top')


def test_planned_source_projects_on_the_tangent():
    m = meas()                                       # wall straight ahead: tangent is +y (left) in the base frame
    assert tangent_angle(m) == pytest.approx(math.pi / 2)
    src = PlannedStandLateral((1.0, 0.25))
    # base at the origin facing +x: planned stand 0.25 m to the left
    assert src.error(LateralContext(m, (0.0, 0.0), 0.0)) == pytest.approx(0.25)
    # the along-wall coordinate does not count
    assert src.error(LateralContext(m, (0.4, 0.25), 0.0)) == pytest.approx(0.0)
    # base yawed 90 deg (facing +y): the wall normal is seen rotated, the tangent in the world is -x
    m90 = measure_wall(plane_to_base((0.0, -1.0, 0.0, 1.0), (0.0, 0.0, math.pi / 2)))
    assert src.error(LateralContext(m90, (0.0, 0.0), math.pi / 2)) == pytest.approx(-1.0)


def test_planned_source_needs_odom():
    assert PlannedStandLateral((0.0, 0.0)).error(LateralContext(meas())) is None
    assert PlannedStandLateral((math.nan, 0.0)).error(LateralContext(meas(), (0.0, 0.0), 0.0)) is None


def test_with_lateral_fills_a_copy():
    m = meas()
    out = with_lateral(m, WallEdgeLateral(0.5), LateralContext(m, u_range=(-0.4, 1.2)))
    assert out.lateral == pytest.approx(0.7) and m.lateral is None
    assert with_lateral(m, WallEdgeLateral(0.5), LateralContext(m)).lateral is None


# --- controller -----------------------------------------------------------------------------------------------

def test_omni_slides_toward_the_error_sign():
    for lat, sign in ((0.1, 1.0), (-0.1, -1.0)):
        s = settle(WallDockController(OMNI, 0.55, lateral=LAT), meas(lat))
        assert s.cmd[1] * sign > 0.02 and abs(s.cmd[0]) < 1e-9


def test_lateral_speed_cap_and_gain():
    s = settle(WallDockController(OMNI, 0.55, lateral=LateralConfig(enabled=True, max_speed=0.03)), meas(0.5))
    assert abs(s.cmd[1]) == pytest.approx(0.03)
    s = settle(WallDockController(OMNI, 0.55, lateral=LateralConfig(enabled=True, kp=0.5, max_speed=0.5)), meas(0.04))
    assert abs(s.cmd[1]) == pytest.approx(0.02, abs=1e-6)


def test_lateral_deadband_inside_half_tolerance():
    s = settle(WallDockController(OMNI, 0.55, lateral=LAT), meas(0.004))
    assert abs(s.cmd[1]) < 1e-9


def test_docked_needs_the_lateral_within_tolerance():
    ctl = WallDockController(OMNI, 0.55, lateral=LAT)
    assert ctl.step(meas(0.05), 0.02).status == DockStatus.RUNNING
    s = ctl.step(meas(0.008), 0.02)
    assert s.status == DockStatus.SUCCESS and s.lateral_error == pytest.approx(0.008)
    assert s.outcome == DockOutcome.DOCKED


def test_docked_needs_a_lateral_measurement_when_enabled():
    s = WallDockController(OMNI, 0.55, lateral=LAT).step(meas(None), 0.02)
    assert s.status == DockStatus.RUNNING


def test_lateral_too_large_is_unreachable_for_omni():
    s = WallDockController(OMNI, 0.55, lateral=LateralConfig(enabled=True, max_correctable=0.5)).step(meas(0.9), 0.02)
    assert s.status == DockStatus.NEEDS_REAPPROACH and s.reapproach
    assert s.outcome == DockOutcome.LATERAL_UNREACHABLE


def test_diff_never_strafes_and_asks_for_a_reapproach_once_square():
    ctl = WallDockController(DIFF, 0.55, lateral=LAT)
    # not square yet (standoff 0.4 m too far): keeps docking, no lateral motion, no reapproach request
    far = meas(0.2, (-1.0, 0.0, 0.0, 0.95))
    for _ in range(50):
        s = ctl.step(far, 0.02, 0.1, 0.0)
        assert s.status == DockStatus.RUNNING and abs(s.cmd[1]) < 1e-12
    prev_cmd_x = s.cmd[0]
    # square, at rest, lateral off by 0.2: needs the back-off / turn / approach manoeuvre
    s = ctl.step(meas(0.2), 0.02, 0.0, 0.0)
    assert s.status == DockStatus.NEEDS_REAPPROACH and s.outcome == DockOutcome.LATERAL_UNREACHABLE
    assert s.cmd[0] < prev_cmd_x          # the target is zero: the command decays under the acceleration limit


def test_diff_within_lateral_tolerance_docks():
    s = WallDockController(DIFF, 0.55, lateral=LAT).step(meas(0.005), 0.02)
    assert s.status == DockStatus.SUCCESS


def test_omni_closed_loop_slides_onto_the_planned_stand():
    source = PlannedStandLateral((0.0, 0.3))
    ctl = WallDockController(OMNI, 0.55, lateral=LAT)
    x, y, yaw, prev = 0.0, 0.0, 0.0, (0.0, 0.0, 0.0)
    plane_w = (-1.0, 0.0, 0.0, 0.55)
    s = None
    for _ in range(3000):
        m = measure_wall(plane_to_base(plane_w, (x, y, yaw)))
        m = with_lateral(m, source, LateralContext(m, (x, y), yaw))
        s = ctl.step(m, 0.02, math.hypot(prev[0], prev[1]), abs(prev[2]))
        if s.status != DockStatus.RUNNING:
            break
        c, sn = math.cos(yaw), math.sin(yaw)
        x += (c * s.cmd[0] - sn * s.cmd[1]) * 0.02
        y += (sn * s.cmd[0] + c * s.cmd[1]) * 0.02
        yaw = wrap(yaw + s.cmd[2] * 0.02)
        prev = s.cmd
    assert s.status == DockStatus.SUCCESS
    assert y == pytest.approx(0.3, abs=0.011)


# --- typed outcomes -------------------------------------------------------------------------------------------

def test_outcome_names_and_mapping():
    assert [o.value for o in DockOutcome] == ['docked', 'timeout', 'lost_wall', 'lateral_unreachable', 'blocked']
    assert outcome_of(DockStatus.SUCCESS) == DockOutcome.DOCKED
    assert outcome_of(DockStatus.TIMEOUT) == DockOutcome.TIMEOUT
    assert outcome_of(DockStatus.NO_WALL) == DockOutcome.LOST_WALL
    assert outcome_of(DockStatus.NEEDS_REAPPROACH) == DockOutcome.LATERAL_UNREACHABLE
    assert outcome_of(DockStatus.BLOCKED) == DockOutcome.BLOCKED
    assert outcome_of(DockStatus.RUNNING) is None


def test_lost_wall_and_timeout_outcomes():
    ctl = WallDockController(OMNI, 0.5, wall_timeout_s=0.5, lateral=LAT)
    s = None
    for _ in range(40):
        s = ctl.step(None, 0.02)
    assert s.status == DockStatus.NO_WALL and s.outcome == DockOutcome.LOST_WALL
    ctl = WallDockController(OMNI, 0.5, timeout_s=0.1)
    for _ in range(10):
        s = ctl.step(measure_wall((-1.0, 0.0, 0.0, 2.0)), 0.02, 0.5, 0.5)
    assert s.outcome == DockOutcome.TIMEOUT


def test_a_short_dropout_is_ridden_out():
    ctl = WallDockController(OMNI, 0.5, wall_timeout_s=1.0, lateral=LAT)
    m = meas(0.1, (-1.0, 0.0, 0.0, 1.0))
    settle(ctl, m, 30)
    for _ in range(20):                       # 0.4 s without a plane: still running, command decaying
        assert ctl.step(None, 0.02).status == DockStatus.RUNNING
    assert ctl.step(m, 0.02).status == DockStatus.RUNNING
    assert ctl.since_wall == 0.0


def test_blocked_is_detected_only_when_asked_for():
    far = measure_wall((-1.0, 0.0, 0.0, 1.5))
    off = WallDockController(OMNI, 0.5)
    on = WallDockController(OMNI, 0.5, blocked_timeout_s=1.0)
    s_off = s_on = None
    for _ in range(300):                      # commanding forward, measured speed stays 0
        s_off = off.step(far, 0.02, 0.0, 0.0)
        s_on = on.step(far, 0.02, 0.0, 0.0)
        if s_on.status != DockStatus.RUNNING:
            break                             # terminal: the node stops the base here
    assert s_off.status == DockStatus.RUNNING
    assert s_on.status == DockStatus.BLOCKED and s_on.outcome == DockOutcome.BLOCKED


def test_blocked_not_raised_when_the_base_moves():
    ctl = WallDockController(OMNI, 0.5, blocked_timeout_s=1.0)
    far = measure_wall((-1.0, 0.0, 0.0, 1.5))
    s = None
    for _ in range(300):
        s = ctl.step(far, 0.02, math.hypot(*ctl.cmd[:2]), 0.0)
    assert s.status == DockStatus.RUNNING


def test_result_dataclass():
    ctl = WallDockController(OMNI, 0.55, lateral=LAT)
    s = ctl.step(meas(0.004, (-1.0, 0.0, 0.0, 0.552)), 0.02)
    r = ctl.result(s)
    assert isinstance(r, DockResult) and r.outcome == DockOutcome.DOCKED
    assert r.standoff_error == pytest.approx(0.002) and r.yaw_error == pytest.approx(0.0)
    assert r.lateral_error == pytest.approx(0.004) and r.elapsed_s == pytest.approx(0.02)


# --- the manoeuvre for a base that cannot slide ---------------------------------------------------------------

def _run_prims(prims, pose):
    x, y, yaw = pose
    for kind, a in prims:
        if kind == 'drive':
            x, y = x + a * math.cos(yaw), y + a * math.sin(yaw)
        else:
            yaw = wrap(yaw + a)
    return x, y, yaw


@pytest.mark.parametrize('lat', [0.25, -0.25])
def test_manoeuvre_slides_along_the_wall_tangent(lat):
    # wall at x = 1 straight ahead: the tangent (left) is +y; the manoeuvre returns to the same x, y moved by lat
    x, y, yaw = _run_prims(plan_lateral_manoeuvre(lat, backoff=0.4), (0.0, 0.0, 0.0))
    assert (x, y) == pytest.approx((0.0, lat), abs=1e-9) and yaw == pytest.approx(0.0, abs=1e-9)


def test_manoeuvre_with_the_rear_to_the_wall_is_mirrored():
    # rear to the wall at x = -1: into the wall is -x, the tangent (90 deg left of that) is -y
    x, y, yaw = _run_prims(plan_lateral_manoeuvre(0.25, backoff=0.4, facing_rad=math.pi), (0.0, 0.0, 0.0))
    assert (x, y) == pytest.approx((0.0, -0.25), abs=1e-9)
    prims = plan_lateral_manoeuvre(0.25, backoff=0.4, facing_rad=math.pi)
    assert prims[0] == ('drive', 0.4)           # backing off from a rear wall is driving forward


# --- review fixes ---------------------------------------------------------------------------------------------

@pytest.mark.parametrize('profile', [OMNI, DIFF], ids=['omni', 'diff'])
def test_nan_lateral_reading_is_no_reading(profile):
    ctl = WallDockController(profile, 0.55, lateral=LAT)
    ctl.step(meas(math.nan, (-1.0, 0.0, 0.0, 0.8)), 0.05)
    s = None
    for _ in range(100):
        s = ctl.step(meas(0.0, (-1.0, 0.0, 0.0, 0.8)), 0.05)
    assert all(map(math.isfinite, s.cmd))
    assert s.cmd[0] > 0.0 or profile is DIFF
    # a NaN reading never counts as docked
    s = WallDockController(profile, 0.55, lateral=LAT).step(meas(math.nan), 0.02)
    assert s.status == DockStatus.RUNNING and s.lateral_error is None


def test_sources_do_not_return_nan():
    m = meas()
    assert PlannedStandLateral((1.0, 1.0)).error(LateralContext(m, (0.0, 0.0), math.nan)) is None
    assert WallEdgeLateral(math.nan).error(LateralContext(m, u_range=(-1.0, 1.0))) is None


def test_manoeuvre_squares_up_first_for_a_yawed_base():
    # diff base 1.2 m from the wall at x = 1.2, yawed 0.4 rad, lateral error -1.2 m: ends square, lateral and standoff right
    yaw0 = 0.4
    m = measure_wall(plane_to_base((-1.0, 0.0, 0.0, 1.2), (0.0, 0.0, yaw0)))
    assert m.yaw_error == pytest.approx(-yaw0)
    prims = plan_lateral_manoeuvre(-1.2, backoff=0.3, yaw_error=m.yaw_error)
    x, y, yaw = _run_prims(prims, (0.0, 0.0, yaw0))
    assert yaw == pytest.approx(0.0, abs=1e-9)
    assert (x, y) == pytest.approx((0.0, -1.2), abs=0.01)
    # without the yaw the slide is off
    x, y, _ = _run_prims(plan_lateral_manoeuvre(-1.2, backoff=0.3), (0.0, 0.0, yaw0))
    assert abs(y + 1.2) > 0.05


@pytest.mark.parametrize('kw', [dict(kp=0.0), dict(kp=-1.0), dict(tol=0.0), dict(max_speed=0.0),
                                dict(tol=math.nan), dict(max_speed=math.inf), dict(max_correctable=-1.0)])
def test_lateral_config_is_validated(kw):
    with pytest.raises(ValueError):
        LateralConfig(enabled=True, **kw)


def test_result_while_running_has_no_outcome():
    ctl = WallDockController(OMNI, 0.55, lateral=LAT)
    s = ctl.step(meas(0.2, (-1.0, 0.0, 0.0, 1.0)), 0.02)
    assert s.status == DockStatus.RUNNING and ctl.result(s).outcome is None
