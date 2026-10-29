#!/usr/bin/env python3
"""Write a reachability chain description (YAML) from a URDF.

Walks the kinematic chain from --base to --tip, folding fixed joints into the next moving joint's
origin (and the trailing ones into the tool transform), and records each revolute joint's axis and
limits. The result is what `loadChainYaml` reads and what the map is fingerprinted with.

  # from a xacro (render first), for the arm of mobile_manipulator_description
  xacro urdf/mobile_manipulator.urdf.xacro arm:=ur10e -o /tmp/robot.urdf
  urdf_to_chain.py /tmp/robot.urdf --base arm_base_link --tip tcp_link -o ur10e_chain.yaml
"""
import argparse
import math
import sys

import numpy as np
import yaml
from urdf_parser_py.urdf import URDF


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def matrix_rpy(R):
    sp = -R[2, 0]
    p = math.asin(max(-1.0, min(1.0, sp)))
    if abs(sp) < 0.999999:
        return [math.atan2(R[2, 1], R[2, 2]), p, math.atan2(R[1, 0], R[0, 0])]
    return [math.atan2(-R[1, 2], R[1, 1]), p, 0.0]  # gimbal lock: fold yaw into roll


def transform(joint):
    T = np.eye(4)
    if joint.origin is not None:
        xyz = joint.origin.xyz or [0, 0, 0]
        rpy = joint.origin.rpy or [0, 0, 0]
        T[:3, :3] = rpy_matrix(*rpy)
        T[:3, 3] = xyz
    return T


def entry(T, extra=None):
    d = {'xyz': [float(v) for v in T[:3, 3]], 'rpy': [float(v) for v in matrix_rpy(T[:3, :3])]}
    if extra:
        d.update(extra)
    return d


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('urdf')
    ap.add_argument('--base', required=True)
    ap.add_argument('--tip', required=True)
    ap.add_argument('-o', '--out', required=True)
    args = ap.parse_args()

    robot = URDF.from_xml_file(args.urdf)
    joint_names = robot.get_chain(args.base, args.tip, links=False, fixed=True)
    pending = np.eye(4)
    joints = []
    for name in joint_names:
        j = robot.joint_map[name]
        T = transform(j)
        if j.type in ('revolute', 'continuous'):
            lo = j.limit.lower if (j.limit and j.type == 'revolute') else -math.pi
            hi = j.limit.upper if (j.limit and j.type == 'revolute') else math.pi
            axis = j.axis or [1, 0, 0]
            joints.append(entry(pending @ T, {'axis': [float(a) for a in axis], 'lower': float(lo),
                                              'upper': float(hi)}))
            pending = np.eye(4)
        elif j.type == 'fixed':
            pending = pending @ T
        else:
            sys.exit(f'joint {name} has type {j.type}: only revolute chains are supported')
    if not joints:
        sys.exit('no moving joints between the given links')
    with open(args.out, 'w') as f:
        yaml.safe_dump({'joints': joints, 'tool': entry(pending)}, f, sort_keys=False)
    print(f'{args.out}: {len(joints)} joints, base={args.base}, tip={args.tip}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
