import io
import contextlib
import math
from dataclasses import replace

import pytest

from stand_navigator import dock_montecarlo as mc
from stand_navigator.wall_docking import DockOutcome


def quiet(**kw):
    n = mc.NoiseConfig(odom_step_m=0.0, odom_step_rad=0.0, odom_scale=0.0, drift_mps=0.0, drift_walk=0.0,
                       drift_yaw=0.0, standoff_sigma=0.0, yaw_sigma=0.0, edge_sigma=0.0, dropout_prob=0.0)
    return replace(n, **kw)


def test_percentile_known_distribution():
    v = list(range(101))                       # 0..100: percentile q is q
    assert mc.percentile(v, 0) == 0 and mc.percentile(v, 50) == 50 and mc.percentile(v, 95) == 95
    assert mc.percentile(v, 100) == 100
    assert mc.percentile([1.0, 2.0, 3.0, 4.0], 50) == pytest.approx(2.5)      # interpolates (numpy default)
    assert mc.percentile([3.0, 1.0, 2.0], 50) == 2.0                           # order does not matter
    assert mc.percentile([7.0], 95) == 7.0
    assert math.isnan(mc.percentile([], 50))


def test_summarize():
    s = mc.summarize([1.0, 2.0, 3.0, 4.0, 10.0])
    assert s['mean'] == pytest.approx(4.0) and s['p50'] == 3.0 and s['max'] == 10.0
    assert s['p95'] == pytest.approx(3.0 + 0.8 * 0 + (10.0 - 4.0) * 0.8 + 1.0)    # between the 4th and 5th value
    assert math.isnan(mc.summarize([])['mean'])


def test_deterministic_for_a_seed_and_different_across_seeds():
    scn = mc.Scenario(lateral='edge')
    a = [mc.run_once(scn, 5, i) for i in range(5)]
    b = [mc.run_once(scn, 5, i) for i in range(5)]
    c = [mc.run_once(scn, 6, i) for i in range(5)]
    assert a == b
    assert a != c
    assert mc.run_many(scn, 4, 5).results == mc.run_many(scn, 4, 5).results


@pytest.mark.parametrize('seed', [1, 2, 3, 4, 5, 6])
def test_no_noise_docks_precisely_in_every_mode_and_base(seed):
    for base in ('omni', 'diff'):
        for lat in mc.LATERAL_MODES:
            scn = mc.Scenario(base=base, lateral=lat, noise=quiet())
            rep = mc.run_many(scn, 10, seed)
            assert rep.docked_rate == 1.0, (seed, base, lat, rep.failures())
            assert rep.stats()['standoff']['max'] <= 0.005 + 1e-6
            assert rep.stats()['yaw']['max'] <= 0.02 + 1e-6


def test_manoeuvre_terminates_when_the_remaining_goal_is_below_float_resolution():
    w = mc._World((0.0, 0.0, 0.0), quiet(), mc.random.Random(1), mc.random.Random(2))
    t = mc._manoeuvre(w, [('drive', 0.3), ('turn', math.pi / 2), ('drive', 1.0 / 3.0)], 0.05)
    assert t < 20.0


def test_edge_source_cancels_odom_drift_that_planned_cannot_see():
    noise = quiet(drift_mps=0.01)
    off = mc.run_many(mc.Scenario(lateral='off', noise=noise), 60, 2).stats()['lateral']['mean']
    planned = mc.run_many(mc.Scenario(lateral='planned', noise=noise), 60, 2).stats()['lateral']['mean']
    edge = mc.run_many(mc.Scenario(lateral='edge', noise=noise), 60, 2).stats()['lateral']['mean']
    assert edge < 0.011                      # within the lateral tolerance up to the reading
    assert edge < 0.5 * off
    assert planned > 0.5 * off              # the planned-vs-odom source is blind to the drift


def test_sweep_more_drift_never_improves_lateral_error():
    for lat in ('off', 'planned'):
        rows = mc.sweep_drift(mc.Scenario(lateral=lat), 100, 1, [0.0, 0.002, 0.005, 0.01])
        means = [r[1] for r in rows]
        assert means == sorted(means), (lat, means)
        assert means[-1] > 2 * means[0]


def test_dropouts_are_survived_or_typed():
    noise = replace(quiet(), dropout_prob=0.05, dropout_max_steps=10)       # up to 0.5 s, wall timeout is 2 s
    rep = mc.run_many(mc.Scenario(lateral='edge', noise=noise), 30, 4)
    assert rep.docked_rate >= 0.9
    # dropouts longer than the wall timeout end as lost_wall
    long = replace(quiet(), dropout_prob=0.2, dropout_max_steps=100)
    rep = mc.run_many(mc.Scenario(noise=long), 20, 4)
    assert all(isinstance(r.outcome, DockOutcome) for r in rep.results)
    assert 'lost_wall' in rep.failures()


def test_diff_base_does_the_reapproach_manoeuvre():
    noise = quiet(drift_mps=0.002)
    edge = mc.run_many(mc.Scenario(base='diff', lateral='edge', noise=noise), 30, 1)
    off = mc.run_many(mc.Scenario(base='diff', lateral='off', noise=noise), 30, 1)
    assert sum(r.reapproaches for r in edge.results) > 0
    assert sum(r.reapproaches for r in off.results) == 0
    assert edge.stats()['lateral']['mean'] < 0.5 * off.stats()['lateral']['mean']


def test_report_and_cli_output():
    rep = mc.run_many(mc.Scenario(), 5, 1)
    text = mc.format_report(rep)
    assert 'docked=' in text and 'position [mm]' in text and 'p95' in text
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        mc.main(['--runs', '3', '--seed', '1', '--lateral', 'edge', '--sweep', '--drifts', '0', '0.005'])
    out = buf.getvalue()
    assert 'not a simulator' in out and 'drift sweep' in out and 'lateral=edge' in out
