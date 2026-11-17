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
  run-curobo   inside motion-planner:gpu with --gpus all (needs a working NVIDIA driver)
  report       CPU: path length, clearance and a markdown table from whichever result files exist

Reuses docker/tools/bench_spheres.py (FK, sphere model, box distance, VAMP / cuRobo runners). Frames: the
URDF root frame, metres, radians.
"""
import argparse
import json
import math
import pathlib
import subprocess
import sys

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
        skip_compile=a.skip_compile, compile_only=False))


def cmd_run_curobo(a):
    bs.cmd_run_curobo(argparse.Namespace(
        bridge=a.bridge, problems=str(pathlib.Path(a.work) / 'problems.json'), limit=0, cfg=a.cfg, name=a.name,
        out=a.out))


def cmd_run_moveit(a):
    work = pathlib.Path(a.work).resolve()
    out = pathlib.Path(a.out).resolve()
    urdf, srdf = pathlib.Path(a.urdf).resolve(), pathlib.Path(a.srdf).resolve()
    inner = (f'source /opt/ros/jazzy/setup.bash && source /install/setup.bash && '
             f'/install/motion_planner_moveit/lib/motion_planner_moveit/planner_benchmark /robot/{urdf.name} '
             f'/srdf/{srdf.name} {a.group} /work/vamp_problems.txt /out/{out.name} {a.name} {a.timeout}')
    cmd = ['docker', 'run', '--rm', '--user', f'{__import__("os").getuid()}:{__import__("os").getgid()}',
           '-v', f'{work}:/work', '-v', f'{urdf.parent}:/robot:ro', '-v', f'{srdf.parent}:/srdf:ro',
           '-v', f'{pathlib.Path(a.install).resolve()}:/install:ro', '-v', f'{out.parent}:/out',
           a.image, 'bash', '-c', inner]
    print(' '.join(cmd))
    subprocess.run(cmd, check=True)


def _load_jsonl(path):
    run = {'problems': {}}
    for ln in pathlib.Path(path).read_text().splitlines():
        o = json.loads(ln)
        if o['type'] == 'header':
            run['backend'], run['set'] = o['backend'], o['set']
        elif o['type'] == 'plan':
            o['wall_s'] = o['us'] * 1e-6
            run['problems'][o['id']] = o
    return run


def _load_curobo(path):
    o = json.loads(pathlib.Path(path).read_text())
    run = {'backend': 'curobo', 'set': o['set'], 'problems': {}}
    for rs in o['scenes']:
        for p in rs['problems']:
            run['problems'][p['id']] = p
    return run


def cmd_report(a):
    work = pathlib.Path(a.work)
    doc = json.loads((work / 'problems.json').read_text())
    arm = RevoluteArm(a.urdf)
    spheres, buf, hops = bs.parse_sets([f'ref={a.spheres}'])['ref']
    ref = bs.SphereModel(arm, spheres, buf, hops, 'ref')
    runs = []
    for f in a.results:
        runs.append(_load_curobo(f) if f.endswith('.json') else _load_jsonl(f))
    notes = dict(n.split('=', 1) for n in a.note)
    scenes = {sc['family']: sc for sc in doc['scenes']}

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

    rows, results = [], []
    for run in runs:
        for fam, sc in scenes.items():
            ps = [run['problems'].get(p['id']) for p in sc['problems']]
            ps = [p for p in ps if p is not None]
            if not ps:
                continue
            ok = [p for p in ps if p['success']]
            times = [p['wall_s'] * 1e3 for p in ok]
            lengths = [bs.path_length(p['path']) for p in ok]
            clear = [clearance(p['path'], sc['boxes']) for p in ok]
            clear = [c for c in clear if not math.isnan(c)]
            hits = sum(c < 0 for c in clear)
            r = {'backend': run['backend'], 'set': run['set'], 'scene': fam, 'n': len(ps), 'ok': len(ok),
                 'time_ms_median': bs.pct(times, 50), 'time_ms_p95': bs.pct(times, 95),
                 'path_rad_median': bs.pct(lengths, 50), 'clearance_cm_median': bs.pct(clear, 50) * 100 if clear else float('nan'),
                 'clearance_cm_min': min(clear) * 100 if clear else float('nan'), 'ref_hits': hits,
                 'endpoints_invalid': sum(not all(p['endpoints_valid']) for p in ps)}
            results.append(r)
    (work / 'results.json').write_text(json.dumps(results, indent=1))
    f = lambda v, d=1: '-' if v != v else f'{v:.{d}f}'  # noqa: E731
    out = ['| backend | scene | success | success, valid endpoints | median ms | p95 ms | path rad (median) | clearance cm (median / min) | hits in reference model |',
           '|---|---|---|---|---|---|---|---|---|']
    for r in results:
        out.append(f"| {r['backend']} | {r['scene']} | {r['ok']}/{r['n']} | {r['ok']}/{r['n'] - r['endpoints_invalid']} | {f(r['time_ms_median'])} | {f(r['time_ms_p95'])} | "
                   f"{f(r['path_rad_median'], 2)} | {f(r['clearance_cm_median'])} / {f(r['clearance_cm_min'])} | {r['ref_hits']} |")
    for k, v in notes.items():
        out.append(f'| {k} | not run | - | - | - | - | - | - | {v} |')
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
    p.set_defaults(fn=cmd_run_moveit)
    p = sub.add_parser('run-curobo')
    p.add_argument('--work', required=True)
    p.add_argument('--cfg', required=True)
    p.add_argument('--bridge', default='/ws/src/manipulation/curobo_bridge')
    p.add_argument('--out', required=True)
    p.add_argument('--name', default='foam')
    p.set_defaults(fn=cmd_run_curobo)
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
