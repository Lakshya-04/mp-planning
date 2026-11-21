"""Monte Carlo of the stand-approach and wall-dock CONTROLLER logic against synthetic noise.

This runs `StandController` (go to the planned stand on odom) and then `WallDockController` (standoff, yaw and the
optional lateral term from the wall plane) on a kinematic planar base with injected odometry error (Gaussian per
step, a slowly random-walking drift velocity, a scale error) and plane measurement noise (standoff, yaw, wall-edge
distance, dropouts). The base moves exactly as commanded (no wheels, no slip, no actuation lag, no obstacles) and the
wall is a perfect plane of known extent. It therefore measures what the controller logic does with noisy inputs; it is
NOT a simulator or hardware result, and the noise levels are assumptions, not measurements.

Pure python (math and random only), deterministic for a given seed: every run draws from three streams derived from
(seed, run index) (start and error draws, odometry noise, plane noise) so that changing one noise level keeps the other
draws (a sweep compares the same random starts).

    python3 -m stand_navigator.dock_montecarlo --runs 200 --seed 1
    python3 -m stand_navigator.dock_montecarlo --runs 200 --seed 1 --base diff --sweep
"""
import argparse
import math
import random
from collections import Counter
from dataclasses import dataclass, field, replace

from stand_navigator.core import BaseProfile, StandController, Tolerance, wrap
from stand_navigator.wall_docking import (
    DockOutcome, DockStatus, LateralConfig, LateralContext, PlannedStandLateral, WallDockController,
    WallEdgeLateral, measure_wall, plan_lateral_manoeuvre, plane_to_base, with_lateral)

WALL_X = 3.0                    # the wall is the plane x = WALL_X, normal -x (toward the robot)
WALL_Y = (-1.5, 1.5)            # visible extent along the wall
LATERAL_MODES = ('off', 'planned', 'edge')


@dataclass
class NoiseConfig:
    odom_step_m: float = 0.0003     # sigma per step of translation noise (m), per axis
    odom_step_rad: float = 0.0003   # sigma per step of yaw noise (rad)
    odom_scale: float = 0.01        # sigma of the per-run scale error of the odometry (fraction)
    drift_mps: float = 0.002        # sigma of the per-run world-frame drift velocity (m/s, per axis), applies while the base moves
    drift_walk: float = 0.0005      # random-walk sigma of the drift velocity (m/s per sqrt(s))
    drift_yaw: float = 0.002        # sigma of the per-run yaw drift (rad/s)
    standoff_sigma: float = 0.002   # plane standoff noise (m)
    yaw_sigma: float = 0.003        # plane yaw noise (rad)
    edge_sigma: float = 0.005       # wall edge distance noise (m)
    dropout_prob: float = 0.005     # per step chance that a plane dropout starts
    dropout_max_steps: int = 20     # a dropout lasts 1..this many steps


@dataclass
class Scenario:
    base: str = 'omni'              # omni | diff
    lateral: str = 'off'            # off | planned | edge
    standoff: float = 0.55
    dt: float = 0.05
    stand_timeout_s: float = 90.0
    dock_timeout_s: float = 90.0
    max_reapproach: int = 2
    lateral_tol: float = 0.01
    backoff: float = 0.3
    noise: NoiseConfig = field(default_factory=NoiseConfig)


@dataclass
class RunResult:
    outcome: DockOutcome
    position: float                 # m, |final true footprint position - planned stand|
    standoff: float                 # m, |true standoff - wanted|
    yaw: float                      # rad, |true yaw error|
    lateral: float                  # m, |true position error along the wall|
    time_s: float
    reapproaches: int = 0
    stand_arrived: bool = True      # the approach reached the stand tolerance on odom before its timeout


def percentile(values, q):
    """q-th percentile (0..100) with linear interpolation between order statistics (numpy's default method)."""
    if not values:
        return float('nan')
    v = sorted(values)
    k = (len(v) - 1) * q / 100.0
    lo = int(math.floor(k))
    hi = min(lo + 1, len(v) - 1)
    return v[lo] + (v[hi] - v[lo]) * (k - lo)


def summarize(values):
    """mean, p50, p95, max of a list."""
    if not values:
        return {'mean': float('nan'), 'p50': float('nan'), 'p95': float('nan'), 'max': float('nan')}
    return {'mean': sum(values) / len(values), 'p50': percentile(values, 50),
            'p95': percentile(values, 95), 'max': max(values)}


class _World:
    """True pose and the odometry estimate of the same base."""

    def __init__(self, pose, noise, rng_init, rng_odom):
        self.true = list(pose)
        self.est = list(pose)               # localised at the start, then dead reckoning
        self.n, self.rng = noise, rng_odom
        self.scale = rng_init.gauss(0.0, noise.odom_scale)
        self.drift = [rng_init.gauss(0.0, noise.drift_mps), rng_init.gauss(0.0, noise.drift_mps),
                      rng_init.gauss(0.0, noise.drift_yaw)]
        self.travelled = 0.0

    def move(self, cmd, dt):
        """Advance the truth by the command and the estimate by the same motion with the odometry errors."""
        for pose, scale, noisy in ((self.true, 0.0, False), (self.est, self.scale, True)):
            vx, vy, wz = cmd
            dx, dy, dyaw = vx * dt * (1 + scale), vy * dt * (1 + scale), wz * dt * (1 + scale)
            if noisy:
                dx += self.rng.gauss(0.0, self.n.odom_step_m)
                dy += self.rng.gauss(0.0, self.n.odom_step_m)
                dyaw += self.rng.gauss(0.0, self.n.odom_step_rad)
            c, s = math.cos(pose[2]), math.sin(pose[2])
            pose[0] += c * dx - s * dy
            pose[1] += s * dx + c * dy
            pose[2] = wrap(pose[2] + dyaw)
        for i in range(2):
            self.drift[i] += self.rng.gauss(0.0, self.n.drift_walk * math.sqrt(dt))
        if math.hypot(cmd[0], cmd[1]) > 1e-3 or abs(cmd[2]) > 1e-3:      # wheel odometry does not drift at rest
            self.est[0] += self.drift[0] * dt
            self.est[1] += self.drift[1] * dt
            self.est[2] = wrap(self.est[2] + self.drift[2] * dt)
        self.travelled += math.hypot(cmd[0], cmd[1]) * dt


class _Sensor:
    """Noisy plane and wall-edge measurements with dropouts."""

    def __init__(self, noise, rng, facing=0.0):
        self.n, self.rng, self.left = noise, rng, 0
        self.facing = facing

    def dropped(self):
        if self.left > 0:
            self.left -= 1
            return True
        if self.rng.random() < self.n.dropout_prob:
            self.left = self.rng.randint(1, self.n.dropout_max_steps) - 1
            return True
        return False

    def read(self, true_pose):
        """(WallMeasurement, u_range) from the true pose, or (None, None) in a dropout."""
        if self.dropped():
            return None, None
        plane = (-1.0, 0.0, 0.0, WALL_X)
        m = measure_wall(plane_to_base(plane, true_pose), (0.0, 0.0), self.facing)
        if m is None:               # base past the wall plane or tilted view: no usable plane
            return None, None
        g = self.rng.gauss
        yn = g(0.0, self.n.yaw_sigma)
        m = replace(m, standoff=m.standoff + g(0.0, self.n.standoff_sigma), yaw_error=m.yaw_error + yn,
                    into_wall=m.into_wall + yn)
        # the wall tangent is +y in the world for this wall (90 deg left of the direction into the wall, +x)
        u_range = (WALL_Y[0] - true_pose[1] + g(0.0, self.n.edge_sigma), WALL_Y[1] - true_pose[1] + g(0.0, self.n.edge_sigma))
        return m, u_range


def _manoeuvre(world, prims, dt, lin=0.1, ang=0.3):
    """Run back-off/turn/slide primitives closed loop on the ODOMETRY estimate (the truth moves as commanded).
    Returns the time used."""
    t = 0.0
    for kind, amount in prims:
        goal = abs(amount)
        sign = 1.0 if amount >= 0 else -1.0
        done = 0.0
        for _ in range(100000):
            if goal - done < 1e-9:
                break
            rate = lin if kind == 'drive' else ang
            step = min(rate * dt, goal - done)
            cmd = (sign * step / dt, 0.0, 0.0) if kind == 'drive' else (0.0, 0.0, sign * step / dt)
            before = list(world.est)
            world.move(cmd, dt)
            done += (math.hypot(world.est[0] - before[0], world.est[1] - before[1]) if kind == 'drive'
                     else abs(wrap(world.est[2] - before[2])))
            t += dt
            if t > 120.0:
                return t
    return t


def run_once(scn, seed, index):
    """One approach + dock; returns a RunResult with the TRUE final errors."""
    base = int(seed) * 1_000_003 + index
    rng_init, rng_odom, rng_meas = (random.Random(base * 3 + k) for k in range(3))
    profile = BaseProfile(drive=scn.base, max_lin=0.5, max_ang=1.0, max_lin_acc=0.5, max_ang_acc=1.0)
    y_goal = rng_init.uniform(-0.5, 0.5)
    goal = (WALL_X - scn.standoff, y_goal, 0.0)
    start = (goal[0] - rng_init.uniform(0.8, 2.0), y_goal + rng_init.uniform(-0.8, 0.8), rng_init.uniform(-0.5, 0.5))
    world = _World(start, scn.noise, rng_init, rng_odom)
    sensor = _Sensor(scn.noise, rng_meas)
    dt = scn.dt

    stand = StandController(profile, goal, Tolerance())
    prev, t = (0.0, 0.0, 0.0), 0.0
    while t < scn.stand_timeout_s and not stand.arrived(tuple(world.est), math.hypot(prev[0], prev[1]), abs(prev[2])):
        prev = stand.step(tuple(world.est), dt)
        world.move(prev, dt)
        t += dt
    stand_arrived = t < scn.stand_timeout_s
    dock_t = 0.0

    if scn.lateral == 'planned':
        source = PlannedStandLateral((goal[0], goal[1]))
    elif scn.lateral == 'edge':
        source = WallEdgeLateral(target=WALL_Y[1] - y_goal, edge='left')
    else:
        source = None
    lat_cfg = LateralConfig(enabled=source is not None, tol=scn.lateral_tol)

    outcome, reapproaches = DockOutcome.TIMEOUT, 0
    while True:
        ctl = WallDockController(profile, scn.standoff, timeout_s=max(scn.dock_timeout_s - dock_t, 1.0), lateral=lat_cfg)
        prev = (0.0, 0.0, 0.0)
        step = None
        while True:
            m, u_range = sensor.read(tuple(world.true))
            if m is not None and source is not None:
                ctx = LateralContext(m, (world.est[0], world.est[1]), world.est[2], u_range)
                m = with_lateral(m, source, ctx)
            step = ctl.step(m, dt, math.hypot(prev[0], prev[1]), abs(prev[2]))
            t += dt
            dock_t += dt
            if step.status != DockStatus.RUNNING and step.status != DockStatus.NO_WALL:
                break
            if step.status == DockStatus.NO_WALL:
                break
            prev = step.cmd
            world.move(prev, dt)
        outcome = step.outcome
        if step.reapproach and reapproaches < scn.max_reapproach:
            reapproaches += 1
            tm = _manoeuvre(world, plan_lateral_manoeuvre(step.lateral_error, scn.backoff, yaw_error=step.yaw_error), dt)
            t += tm
            dock_t += tm
            continue
        break
    sx = WALL_X - world.true[0]
    lateral = world.true[1] - y_goal
    standoff = sx - scn.standoff
    return RunResult(outcome, math.hypot(standoff, lateral), abs(standoff), abs(wrap(world.true[2])), abs(lateral), t,
                     reapproaches, stand_arrived)


@dataclass
class Report:
    scenario: Scenario
    results: list

    @property
    def docked_rate(self):
        return sum(r.outcome == DockOutcome.DOCKED for r in self.results) / max(len(self.results), 1)

    def failures(self):
        return Counter(r.outcome.value for r in self.results if r.outcome != DockOutcome.DOCKED)

    def stats(self):
        return {name: summarize([getattr(r, name) for r in self.results]) for name in ('position', 'standoff', 'yaw', 'lateral')}


def run_many(scn, runs, seed):
    return Report(scn, [run_once(scn, seed, i) for i in range(runs)])


def format_report(rep):
    scn, n = rep.scenario, len(rep.results)
    out = [f'base={scn.base} lateral={scn.lateral} runs={n} docked={rep.docked_rate * 100:.1f}% '
           f'stand_timeouts={sum(not r.stand_arrived for r in rep.results)} reapproaches={sum(r.reapproaches for r in rep.results)} '
           f'failures={dict(rep.failures()) or "none"}',
           f'{"final error (true)":<22}{"mean":>9}{"p50":>9}{"p95":>9}{"max":>9}']
    for name, unit, k in (('position', 'mm', 1000.0), ('standoff', 'mm', 1000.0), ('lateral', 'mm', 1000.0),
                          ('yaw', 'deg', 180.0 / math.pi)):
        s = rep.stats()[name]
        out.append(f'{name + " [" + unit + "]":<22}' + ''.join(f'{s[c] * k:>9.2f}' for c in ('mean', 'p50', 'p95', 'max')))
    times = summarize([r.time_s for r in rep.results])
    out.append(f'{"time [s]":<22}' + ''.join(f'{times[c]:>9.1f}' for c in ('mean', 'p50', 'p95', 'max')))
    return '\n'.join(out)


def sweep_drift(scn, runs, seed, drifts):
    """Lateral error against injected odometry drift (drift sigma per axis, m/s); the same starts for every value."""
    rows = []
    for d in drifts:
        s = replace(scn, noise=replace(scn.noise, drift_mps=d))
        rep = run_many(s, runs, seed)
        lat = rep.stats()['lateral']
        rows.append((d, lat['mean'], lat['p95'], rep.docked_rate))
    return rows


def format_sweep(scn, rows):
    out = [f'drift sweep: base={scn.base} lateral={scn.lateral}',
           f'{"drift [mm/s]":>13}{"lat mean [mm]":>15}{"lat p95 [mm]":>14}{"docked [%]":>12}']
    out += [f'{d * 1000:>13.1f}{m * 1000:>15.2f}{p * 1000:>14.2f}{r * 100:>12.1f}' for d, m, p, r in rows]
    return '\n'.join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--runs', type=int, default=200)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--base', choices=('omni', 'diff'), default='omni')
    ap.add_argument('--lateral', choices=LATERAL_MODES + ('all',), default='all')
    ap.add_argument('--sweep', action='store_true', help='also sweep the injected odometry drift')
    ap.add_argument('--drifts', type=float, nargs='+', default=[0.0, 0.001, 0.002, 0.005, 0.01])
    ap.add_argument('--drift', type=float, help='odometry drift sigma, m/s per axis')
    ap.add_argument('--dropout', type=float, help='per-step dropout start probability')
    args = ap.parse_args(argv)
    noise = NoiseConfig()
    if args.drift is not None:
        noise = replace(noise, drift_mps=args.drift)
    if args.dropout is not None:
        noise = replace(noise, dropout_prob=args.dropout)
    modes = LATERAL_MODES if args.lateral == 'all' else (args.lateral,)
    print('Controller logic against synthetic noise; not a simulator or hardware result.\n')
    for mode in modes:
        scn = Scenario(base=args.base, lateral=mode, noise=noise)
        print(format_report(run_many(scn, args.runs, args.seed)) + '\n')
        if args.sweep:
            print(format_sweep(scn, sweep_drift(scn, args.runs, args.seed, args.drifts)) + '\n')


if __name__ == '__main__':
    main()
