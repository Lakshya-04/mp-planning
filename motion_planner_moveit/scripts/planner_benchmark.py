#!/usr/bin/env python3
"""Shared planner benchmark: the same joint-space free-space transitions, planned by every backend.

Problems are ordered pairs between stand-, stow- and dock-like configurations of an arm (the UR10e
profile), in three scenes: empty, a wall slab, and a wall with a box in front of it. Every backend gets
the same start/goal pairs and the same boxes; paths are then judged by one planner-independent model
(a sphere set on a numpy FK), which gives path length and clearance. Method, results and what could not
run: docs/planner_benchmark.md.

  prepare      CPU, host python (numpy, pyyaml): configurations, scenes, valid pairs
               -> problems.json, vamp_problems.txt
  run-vamp     host, docker (motion-planner:full): VAMP RRT-Connect + simplify (docker/tools/vamp_bench.cpp)
  run-moveit   docker (motion-planner:build with the workspace installed): OMPL RRT-Connect on a MoveIt
               PlanningScene, the `planner_benchmark` executable of motion_planner_moveit
  run-curobo   inside motion-planner:gpu with --gpus all: cuRobo v0.9.0 in process (CuroboPlanner), one
               --mode per run: plan_cspace | plan_cspace_fallback | bi_rrt_trajopt | bi_rrt_seed; first call, steady state, VRAM
  refload      a fixed single-thread numpy workload, repeated: the host-load yardstick (run it under taskset)
  report       CPU: path length, clearance and a markdown table from whichever result files exist; result
               files of the same backend/set are passes of one benchmark and give a spread

Reuses docker/tools/bench_spheres.py (FK, sphere model, box distance, VAMP / cuRobo runners). Frames: the
URDF root frame, metres, radians.
"""
import argparse
import json
import math
import pathlib
import os
import subprocess
import sys
import threading
import time

import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / 'docker' / 'tools'))
import bench_spheres as bs  # noqa: E402

# Named configurations of the UR10e (pan, shoulder lift, elbow, wrist 1..3). Stand-like: the arm raised
# in front of the base (the profile's home and variations of it). Stow-like: folded over the base.
# Dock-like ones are not listed: they are sampled so that the tool tip is next to the wall (see prepare).
STAND = {
    'stand_home': [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0],
    'stand_left': [0.6, -1.3, 1.3, -1.5708, -1.5708, 0.0],
    'stand_right': [-0.6, -1.3, 1.3, -1.5708, -1.5708, 0.0],
    'stand_high': [0.0, -1.0, 0.7, -1.2, -1.5708, 0.0],
}
STOW = {
    'stow_fold': [0.0, -2.6, 2.7, -1.6, -1.5708, 0.0],
    'stow_side': [1.5708, -2.4, 2.5, -1.6, -1.5708, 0.0],
}
WALL_X = 0.95          # wall face of the slab, from the root frame
SCENES = [
    ('empty', []),
    ('wall', [{'center': [WALL_X + 0.03, 0.0, 1.0], 'size': [0.06, 2.4, 2.2], 'yaw': 0.0}]),
    ('wall_box', [{'center': [WALL_X + 0.03, 0.0, 1.0], 'size': [0.06, 2.4, 2.2], 'yaw': 0.0},
                  {'center': [0.6, -0.3, 0.9], 'size': [0.25, 0.25, 0.4], 'yaw': 0.4}]),
]


class RevoluteArm(bs.Arm):
    """bs.Arm whose chain depth counts movable joints only: links welded together (base, lift, tool
    mount) are one body, so the 'links up to HOPS joints apart are not self-checked' rule means the same
    as the rigid-group SRDF the VAMP header was generated with."""

    def chain_index(self, links):
        def depth(l):
            d = 0
            while l in self.by_child:
                j = self.by_child[l]
                d += j.get('type') == 'revolute'
                l = j.find('parent').get('link')
            return d
        return {l: depth(l) for l in links}


def _write_vamp_text(path, scenes):
    lines = []
    for sc in scenes:
        lines.append(f"S {sc['id']} {len(sc['boxes'])} {len(sc['problems'])} 0")
        for b in sc['boxes']:
            c, h, R = bs.box_frame(b)
            lines.append('B ' + ' '.join(f'{v:.7g}' for v in [*c, *R[:, 0], *R[:, 1], *R[:, 2], *h]))
        for p in sc['problems']:
            lines.append(f"P {p['id']} " + ' '.join(f'{v:.7g}' for v in p['start'] + p['goal']))
    pathlib.Path(path).write_text('\n'.join(lines) + '\n')


def cmd_prepare(a):
    work = pathlib.Path(a.work)
    work.mkdir(parents=True, exist_ok=True)
    arm = RevoluteArm(a.urdf)
    assert arm.dof == 6, f'expected a 6-joint arm, got {arm.dof}'
    spheres, buf, hops = bs.parse_sets([f'ref={a.spheres}'])['ref']
    ref = bs.SphereModel(arm, spheres, buf, hops, 'ref')
    rng = np.random.default_rng(a.seed)

    # Dock-like: tool tip 8 to 15 cm from the wall face, tool axis within 20 degrees of the wall normal,
    # free of the wall and the arm itself under the reference model. Sampled once, same in every scene.
    wall = SCENES[1][1]
    cand = rng.uniform(-math.pi, math.pi, (200000, arm.dof))
    tf = arm.link_tfs(cand)[a.tool_link]
    axis, tip = tf[:, :3, 2], tf[:, :3, 3] + a.tool_length * tf[:, :3, 2]
    ok = (axis[:, 0] > math.cos(math.radians(20))) & (tip[:, 0] > WALL_X - 0.15) & (tip[:, 0] < WALL_X - 0.08)
    ok &= np.abs(tip[:, 1]) < 0.5
    ok &= (tip[:, 2] > 0.5) & (tip[:, 2] < 1.5)
    cand = cand[ok]
    free = ~ref.in_collision(cand, wall)
    cand = cand[free]
    docks = {}
    for q in cand:
        if all(np.abs(q - o).max() > 0.6 for o in docks.values()):
            docks[f'dock_{len(docks)}'] = q
        if len(docks) == a.docks:
            break
    names = {**{k: np.array(v) for k, v in STAND.items()}, **{k: np.array(v) for k, v in STOW.items()}, **docks}

    scenes, pid, dropped = [], 0, {}
    for si, (family, boxes) in enumerate(SCENES):
        valid = [n for n, q in names.items() if not ref.in_collision(q[None], boxes)[0]]
        dropped[family] = [n for n in names if n not in valid]
        probs = []
        for s in valid:
            for g in valid:
                if s != g:
                    probs.append({'id': pid, 'start': names[s].round(6).tolist(), 'goal': names[g].round(6).tolist(),
                                  'pair': f'{s}>{g}', 'kind': f"{s.split('_')[0]}>{g.split('_')[0]}"})
                    pid += 1
        scenes.append({'id': si, 'family': family, 'boxes': boxes, 'problems': probs, 'probe_states': []})
        print(f'scene {family}: {len(valid)} valid configurations, {len(probs)} problems; dropped {dropped[family]}')
    doc = {'robot': {'urdf': str(a.urdf), 'joint_names': arm.joint_names}, 'seed': a.seed, 'scenes': scenes,
           'configs': {k: v.round(5).tolist() for k, v in names.items()}, 'dropped': dropped}
    (work / 'problems.json').write_text(json.dumps(doc))
    _write_vamp_text(work / 'vamp_problems.txt', scenes)
    print(f'wrote {work}/problems.json and vamp_problems.txt ({pid} problems, docks found: {len(docks)})')


def cmd_run_vamp(a):
    bs.cmd_run_vamp(argparse.Namespace(
        work=a.work, header=a.header, out=a.out, name=a.name, struct=a.struct, image=a.image,
        skip_compile=a.skip_compile, compile_only=False, cpuset=a.cpuset))


# cuRobo v0.9.0 routes through MotionPlanner.plan_cspace (see docs/planner_benchmark.md):
#   plan_cspace      defaults: attempt 0 is a straight-line seed + trajectory optimisation, graph (PRM) seeding
#                    from attempt 1 on; cuRobo's own behaviour (CuroboPlanner seed_fallback=False)
#   plan_cspace_fallback  the same, then, if no trajectory came back, the validated graph seed
#                    (CuroboPlanner default, what the ROS node uses)
#   bi_rrt_trajopt   graph_planner_algorithm='rrt' (GPU bidirectional RRT), seeded from attempt 0, then
#                    trajectory optimisation
#   bi_rrt_seed      the same RRT seed, retimed and validated, trajectory optimisation skipped
CUROBO_MODES = {
    'plan_cspace': {'seed_fallback': False},
    'plan_cspace_fallback': {},
    'bi_rrt_trajopt': {'graph_planner_algorithm': 'rrt', 'enable_graph_attempt': 0},
    'bi_rrt_seed': {'graph_planner_algorithm': 'rrt', 'enable_graph_attempt': 0, 'enable_trajopt': False},
}


class VramPoll(threading.Thread):
    """Device-wide used memory (torch.cuda.mem_get_info, the nvidia-smi counter) polled every 20 ms, as in
    docker/tools/eval_curobo_mapper.py: Warp allocates outside torch, so torch's own counters under-report."""

    def __init__(self, torch):
        super().__init__(daemon=True)
        self.torch, self.stop_flag = torch, False
        self.base = self.used()
        self.peak = self.base

    def used(self):
        free, total = self.torch.cuda.mem_get_info()
        return (total - free) / 2**20

    def run(self):
        while not self.stop_flag:
            self.peak = max(self.peak, self.used())
            time.sleep(0.02)


def cmd_run_curobo(a):
    """One pass of one cuRobo mode over every problem, in process. Timing: wall clock around
    CuroboPlanner.plan_joint (precheck, host to device, plan_cspace, trajectory back to the host), successful
    and failed plans both recorded. The first plan_joint after construction + warmup is timed on its own."""
    import torch
    sys.path.insert(0, a.bridge)
    from curobo_bridge.planner import Box, CuroboPlanner
    kw = CUROBO_MODES[a.mode]
    doc = json.loads((pathlib.Path(a.work) / 'problems.json').read_text())
    if a.limit:
        for sc in doc['scenes']:
            sc['problems'] = sc['problems'][:a.limit]
    poll = VramPoll(torch)
    poll.start()
    t0 = time.perf_counter()
    planner = CuroboPlanner(robot=a.cfg, max_boxes=16, warmup=True, **kw)
    construct_s = time.perf_counter() - t0
    assert planner.joint_names == doc['robot']['joint_names'], (planner.joint_names, doc['robot']['joint_names'])

    def world(sc):
        planner.set_world([Box(f'b{i}', tuple(b['center']), tuple(b['size']),
                               (math.cos(b['yaw'] / 2), 0.0, 0.0, math.sin(b['yaw'] / 2)))
                           for i, b in enumerate(sc['boxes'])])

    sc0 = doc['scenes'][0]
    world(sc0)
    p0 = sc0['problems'][0]
    t = time.perf_counter()
    first = planner.plan_joint(p0['start'], p0['goal'])
    first_call_s = time.perf_counter() - t
    for pi in range(a.passes):
        index = a.pass_index + pi
        out = {'backend': 'curobo', 'set': a.mode, 'mode': a.mode, 'cfg': a.cfg, 'pass': index,
               'passes_in_this_process': a.passes, 'construct_warmup_s': construct_s, 'first_call_s': first_call_s,
               'first_call_ok': bool(first.success), 'loadavg_start': os.getloadavg(), 'time_start': time.strftime('%H:%M:%S'),
               'scenes': []}
        for sc in doc['scenes']:
            world(sc)
            rec = {'id': sc['id'], 'problems': []}
            probe = 0
            for p in sc['problems']:
                valid = [bool(v) for v in planner.check_states([p['start'], p['goal']])]
                row = {'id': p['id'], 'endpoints_valid': valid}
                if not all(valid) and not a.plan_invalid:
                    # cuRobo's own model rejects an endpoint: counted as a failure without calling the planner
                    # (a plan from or to a state it considers in collision cannot succeed; --verify-invalid N
                    # plans N such problems per scene to show that, and what they cost)
                    row.update(success=False, wall_s=0.0, solver_s=0.0, traj_s=0.0, reason='endpoint_invalid',
                               path=[], planned=False)
                    if probe < a.verify_invalid:
                        probe += 1
                        t = time.perf_counter()
                        res = planner.plan_joint(p['start'], p['goal'])
                        row.update(probe_success=bool(res.success), probe_wall_s=time.perf_counter() - t)
                    rec['problems'].append(row)
                    continue
                t = time.perf_counter()
                res = planner.plan_joint(p['start'], p['goal'])
                wall = time.perf_counter() - t
                row.update(success=bool(res.success), wall_s=wall, solver_s=res.planning_time_s, traj_s=res.duration,
                           reason=res.failure_reason_code, planned=True, route=res.route,
                           path=np.asarray(res.positions).round(5).tolist() if res.success else [])
                rec['problems'].append(row)
            out['scenes'].append(rec)
            print(f'pass {index} scene {sc["id"]} {sc["family"]}: '
                  f'{sum(p["success"] for p in rec["problems"])}/{len(rec["problems"])} ok', flush=True)
        out['loadavg_end'] = os.getloadavg()
        out['time_end'] = time.strftime('%H:%M:%S')
        out['vram_base_mib'] = poll.base
        out['vram_peak_mib'] = poll.peak          # the process so far (construct, warmup and the passes up to this one)
        out['vram_torch_max_allocated_mib'] = torch.cuda.max_memory_allocated() / 2**20
        path = pathlib.Path(a.out)
        if a.passes > 1 or a.pass_index:
            path = path.with_name(f'{path.stem}_p{index}{path.suffix}')
        path.write_text(json.dumps(out))
    poll.stop_flag = True
    poll.join()
    planner.close()
    print(f'{a.mode}: construct+warmup {construct_s:.1f} s, first call {first_call_s:.2f} s, '
          f'VRAM peak {poll.peak:.0f} MiB (base {poll.base:.0f})')


def cmd_refload(a):
    """Host-load yardstick: a fixed single-thread numpy workload (float64 matmuls, fixed seed), timed `n`
    times. Run it pinned to the cores the benchmark uses (taskset -c 10-11). Prints one JSON line."""
    rng = np.random.default_rng(0)
    x = rng.standard_normal((384, 384))
    y = rng.standard_normal((384, 384))
    ms = []
    for _ in range(a.n):
        t = time.perf_counter()
        z = x
        for _ in range(40):
            z = (z @ y) * 1e-2
        ms.append((time.perf_counter() - t) * 1e3)
    rec = {'label': a.label, 'median_ms': float(np.median(ms)), 'min_ms': min(ms), 'max_ms': max(ms),
           'loadavg': os.getloadavg(), 'time': time.strftime('%H:%M:%S')}
    print(json.dumps(rec))
    if a.out:
        with open(a.out, 'a') as f:
            f.write(json.dumps(rec) + '\n')


def cmd_run_moveit(a):
    work = pathlib.Path(a.work).resolve()
    out = pathlib.Path(a.out).resolve()
    urdf, srdf = pathlib.Path(a.urdf).resolve(), pathlib.Path(a.srdf).resolve()
    inner = (f'source /opt/ros/jazzy/setup.bash && source /install/setup.bash && '
             f'/install/motion_planner_moveit/lib/motion_planner_moveit/planner_benchmark /robot/{urdf.name} '
             f'/srdf/{srdf.name} {a.group} /work/vamp_problems.txt /out/{out.name} {a.name} {a.timeout}')
    cmd = ['docker', 'run', '--rm', *(['--cpuset-cpus', a.cpuset] if a.cpuset else []),
           '--user', f'{os.getuid()}:{os.getgid()}',
           '-v', f'{work}:/work', '-v', f'{urdf.parent}:/robot:ro', '-v', f'{srdf.parent}:/srdf:ro',
           '-v', f'{pathlib.Path(a.install).resolve()}:/install:ro', '-v', f'{out.parent}:/out',
           a.image, 'bash', '-c', inner]
    print(' '.join(cmd))
    subprocess.run(cmd, check=True)


def _load_jsonl(path):
    run = {'problems': {}, 'file': str(path)}
    for ln in pathlib.Path(path).read_text().splitlines():
        o = json.loads(ln)
        if o['type'] == 'header':
            run['backend'], run['set'] = o['backend'], o['set']
        elif o['type'] == 'first_call':
            run['first_call_s'] = o['us'] * 1e-6
        elif o['type'] == 'plan':
            o['wall_s'] = o['us'] * 1e-6
            o['success'] = o.get('success', o.get('ok'))
            run['problems'][o['id']] = o
    return run


def _load_curobo(path):
    o = json.loads(pathlib.Path(path).read_text())
    run = {'backend': 'curobo', 'set': o['set'], 'problems': {}, 'file': str(path),
           **{k: o[k] for k in o if k not in ('scenes',)}}
    for rs in o['scenes']:
        for p in rs['problems']:
            run['problems'][p['id']] = p
    return run


def _label(run):
    return f"cuRobo {run['set']}" if run['backend'] == 'curobo' else (
        'cuRobo via node + C++ client' if run['backend'] == 'curobo_node' else run['backend'])


def _spread(vals, d=1, unit=''):
    """'median' or 'median (min-max)' of a list of per-pass numbers."""
    vals = [v for v in vals if v == v]
    if not vals:
        return '-'
    m = float(np.median(vals))
    if len(vals) == 1 or max(vals) - min(vals) < 10 ** -d / 2:
        return f'{m:.{d}f}{unit}'
    return f'{m:.{d}f} ({min(vals):.{d}f}-{max(vals):.{d}f}){unit}'


def cmd_report(a):
    work = pathlib.Path(a.work)
    doc = json.loads((work / 'problems.json').read_text())
    arm = RevoluteArm(a.urdf)
    spheres, buf, hops = bs.parse_sets([f'ref={a.spheres}'])['ref']
    ref = bs.SphereModel(arm, spheres, buf, hops, 'ref')
    runs = [_load_curobo(f) if f.endswith('.json') else _load_jsonl(f) for f in a.results]
    notes = dict(n.split('=', 1) for n in a.note)
    scenes = {sc['family']: sc for sc in doc['scenes']}
    groups = {}                      # label -> passes (files of one backend/set are repeats of one benchmark)
    for r in runs:
        groups.setdefault(_label(r), []).append(r)

    def clearance(path, boxes):
        """Smallest gap (m) between any reference sphere and any box along the dense path; negative = hit."""
        if not boxes:
            return float('nan')
        dense = bs.densify(path, 0.05)
        best = float('inf')
        for i in range(0, len(dense), 200):
            ws = ref.world_spheres(arm.link_tfs(dense[i:i + 200]))
            for w in ws.values():
                for b in boxes:
                    d = bs.box_distance(w[..., :3], b) - w[..., 3]
                    best = min(best, float(d.min()))
        return best

    results = []
    for label, passes in groups.items():
        if passes[0]['backend'] == 'curobo_node':
            continue
        for fam in [*scenes, 'all']:
            sps = [p for f, sc in scenes.items() if fam in (f, 'all') for p in sc['problems']]
            per_pass = []
            for run in passes:
                ps = [run['problems'].get(p['id']) for p in sps]
                ps = [p for p in ps if p is not None]
                ok = [p for p in ps if p['success']]
                per_pass.append({'n': len(ps), 'ok': ok, 'valid': sum(all(p['endpoints_valid']) for p in ps),
                                 'ok_valid': sum(all(p['endpoints_valid']) for p in ok),
                                 'ms': [p['wall_s'] * 1e3 for p in ok]})
            if not per_pass[0]['n']:
                continue
            first = per_pass[0]
            pooled = [t for pp in per_pass for t in pp['ms']]
            boxes_of = {p['id']: sc['boxes'] for sc in doc['scenes'] for p in sc['problems']}
            lengths = [bs.path_length(p['path']) for p in first['ok']]
            clear = [clearance(p['path'], boxes_of[p['id']]) for p in first['ok']] if fam != 'all' else []
            clear = [c for c in clear if not math.isnan(c)]
            r = {'backend': label, 'scene': fam, 'passes': len(passes), 'n': first['n'],
                 'ok_per_pass': [len(pp['ok']) for pp in per_pass],
                 'ok_valid_per_pass': [pp['ok_valid'] for pp in per_pass],
                 'valid_endpoints': first['valid'],
                 'time_ms_median_pooled': bs.pct(pooled, 50), 'time_ms_p95_pooled': bs.pct(pooled, 95),
                 'time_ms_median_per_pass': [bs.pct(pp['ms'], 50) for pp in per_pass],
                 'time_ms_p95_per_pass': [bs.pct(pp['ms'], 95) for pp in per_pass],
                 'path_rad_median': bs.pct(lengths, 50),
                 'traj_s_median': bs.pct([p['traj_s'] for p in first['ok'] if 'traj_s' in p], 50),
                 'clearance_cm_median': bs.pct(clear, 50) * 100 if clear else float('nan'),
                 'clearance_cm_min': min(clear) * 100 if clear else float('nan'),
                 'ref_hits': sum(c < 0 for c in clear)}
            results.append(r)
    (work / 'results.json').write_text(json.dumps(results, indent=1))

    f = lambda v, d=1: '-' if v != v else f'{v:.{d}f}'  # noqa: E731
    fmt_ok = lambda xs, n: f'{min(xs)}/{n}' if min(xs) == max(xs) else f'{min(xs)}-{max(xs)}/{n}'  # noqa: E731
    out = ['| backend | scene | passes | success | success, valid endpoints | median ms (pass medians min-max) | p95 ms (pass p95s min-max) | path rad (median) | clearance cm (median / min) | hits in reference model | trajectory duration s (median, cuRobo only) |',
           '|---|---|---|---|---|---|---|---|---|---|---|']
    for r in results:
        out.append(
            f"| {r['backend']} | {r['scene']} | {r['passes']} | {fmt_ok(r['ok_per_pass'], r['n'])} | "
            f"{fmt_ok(r['ok_valid_per_pass'], r['valid_endpoints'])} | {_spread(r['time_ms_median_per_pass'])} | "
            f"{_spread(r['time_ms_p95_per_pass'])} | {f(r['path_rad_median'], 2)} | "
            f"{f(r['clearance_cm_median'])} / {f(r['clearance_cm_min'])} | "
            f"{'-' if r['scene'] == 'all' else r['ref_hits']} | {f(r['traj_s_median'], 2)} |")
    for k, v in notes.items():
        out.append(f'| {k} | not run | - | - | - | - | - | - | - | {v} | - |')

    # cuRobo: first call, warmup, VRAM, per pass
    cur = [r for r in runs if r['backend'] == 'curobo']
    if cur:
        out += ['', '| cuRobo mode | pass | construct + warmup s | first call s | VRAM peak MiB (device, above base) | torch max allocated MiB | loadavg start / end |',
                '|---|---|---|---|---|---|---|']
        for r in cur:
            out.append(f"| {r['set']} | {r['pass']} | {r['construct_warmup_s']:.1f} | {r['first_call_s']:.2f} | "
                       f"{r['vram_peak_mib']:.0f} ({r['vram_peak_mib'] - r['vram_base_mib']:.0f}) | "
                       f"{r['vram_torch_max_allocated_mib']:.0f} | {r['loadavg_start'][0]:.1f} / {r['loadavg_end'][0]:.1f} |")

    # node + C++ client against the in-process runs, same problem ids (passes of the client pooled)
    for label, nruns in groups.items():
        if nruns[0]['backend'] != 'curobo_node':
            continue
        ids = sorted(set().union(*[set(r['problems']) for r in nruns]))
        times = [r['problems'][i]['wall_s'] * 1e3 for r in nruns for i in ids if i in r['problems'] and r['problems'][i]['success']]
        total = sum(len(r['problems']) for r in nruns)
        node_ms = [r['problems'][i]['node_ms'] for r in nruns for i in ids if i in r['problems'] and r['problems'][i]['success']]
        traj = [r['problems'][i]['traj_s'] for r in nruns for i in ids if i in r['problems'] and r['problems'][i]['success']]
        out += ['', f"cuRobo through the ROS node and C++ client: {len(times)}/{total} planned ({len(nruns)} client passes over "
                    f"{len(ids)} problems); the same problem ids in process, per mode and pass:",
                '', '| path | n | median ms | p95 ms | min ms | max ms |', '|---|---|---|---|---|---|']
        out.append(f"| node + client (wall around `plan()`) | {len(times)} | {f(bs.pct(times, 50))} | {f(bs.pct(times, 95))} | "
                   f"{f(min(times) if times else float('nan'))} | {f(max(times) if times else float('nan'))} |")
        out.append(f"| node's own `planning_time_ms` (cuRobo `total_time`, whole ms) | {len(node_ms)} | {f(bs.pct(node_ms, 50))} | "
                   f"{f(bs.pct(node_ms, 95))} | {f(min(node_ms) if node_ms else float('nan'))} | {f(max(node_ms) if node_ms else float('nan'))} |")
        for r in cur:
            ts = [r['problems'][i]['wall_s'] * 1e3 for i in ids if i in r['problems'] and r['problems'][i]['success']]
            out.append(f"| in process, {r['set']} pass {r['pass']} | {len(ts)} | {f(bs.pct(ts, 50))} | "
                       f"{f(bs.pct(ts, 95))} | {f(min(ts) if ts else float('nan'))} | {f(max(ts) if ts else float('nan'))} |")
    (work / 'results.txt').write_text('\n'.join(out) + '\n')
    print('\n'.join(out))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('prepare')
    p.add_argument('--urdf', required=True, help='arm-only URDF (revolute joints only), meshes resolved')
    p.add_argument('--spheres', required=True, help='FOAM spheres (spherized URDF or yml): the reference model')
    p.add_argument('--work', required=True)
    p.add_argument('--tool-link', default='tcp_link')
    p.add_argument('--tool-length', type=float, default=0.0, help='extra tool length along the link z axis, m')
    p.add_argument('--docks', type=int, default=4)
    p.add_argument('--seed', type=int, default=0)
    p.set_defaults(fn=cmd_prepare)
    p = sub.add_parser('run-vamp')
    p.add_argument('--work', required=True)
    p.add_argument('--header', required=True)
    p.add_argument('--struct', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--name', default='foam')
    p.add_argument('--image', default='motion-planner:full')
    p.add_argument('--skip-compile', action='store_true')
    p.add_argument('--cpuset', default=None, help='docker --cpuset-cpus, e.g. 10-11')
    p.set_defaults(fn=cmd_run_vamp)
    p = sub.add_parser('run-moveit')
    p.add_argument('--work', required=True)
    p.add_argument('--urdf', required=True)
    p.add_argument('--srdf', required=True)
    p.add_argument('--group', default='arm')
    p.add_argument('--install', required=True, help='colcon install dir containing motion_planner_moveit')
    p.add_argument('--out', required=True)
    p.add_argument('--name', default='urdf_meshes')
    p.add_argument('--timeout', type=float, default=5.0)
    p.add_argument('--image', default='motion-planner:build')
    p.add_argument('--cpuset', default=None, help='docker --cpuset-cpus, e.g. 10-11')
    p.set_defaults(fn=cmd_run_moveit)
    p = sub.add_parser('run-curobo')
    p.add_argument('--work', required=True)
    p.add_argument('--cfg', required=True)
    p.add_argument('--bridge', default='/ws/src/manipulation/curobo_bridge')
    p.add_argument('--out', required=True)
    p.add_argument('--mode', choices=sorted(CUROBO_MODES), default='plan_cspace')
    p.add_argument('--pass-index', type=int, default=0, help='number of the first pass')
    p.add_argument('--passes', type=int, default=1, help='repeat the whole problem set this many times in one process (one warmup)')
    p.add_argument('--plan-invalid', action='store_true', help='call the planner even when cuRobo rejects an endpoint')
    p.add_argument('--verify-invalid', type=int, default=0, help='plan N endpoint-invalid problems per scene anyway, record the outcome')
    p.add_argument('--limit', type=int, default=0, help='only the first N problems of each scene (smoke test)')
    p.set_defaults(fn=cmd_run_curobo)
    p = sub.add_parser('refload')
    p.add_argument('--label', default='')
    p.add_argument('--n', type=int, default=15)
    p.add_argument('--out', default=None, help='append the JSON line to this file')
    p.set_defaults(fn=cmd_refload)
    p = sub.add_parser('report')
    p.add_argument('--work', required=True)
    p.add_argument('--urdf', required=True)
    p.add_argument('--spheres', required=True)
    p.add_argument('--note', action='append', default=[], help='BACKEND=reason, for a backend that could not run')
    p.add_argument('results', nargs='*')
    p.set_defaults(fn=cmd_report)
    a = ap.parse_args()
    a.fn(a)


if __name__ == '__main__':
    main()
