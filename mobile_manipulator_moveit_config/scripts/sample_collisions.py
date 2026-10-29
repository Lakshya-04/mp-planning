#!/usr/bin/env python3
"""Sample random joint states against a running move_group and report collision statistics.

Needs `demo.launch.py` running. For N random states inside the joint limits it records which link
pairs collide, then prints:

  * the fraction of states that are valid (a very low value means the allowed-collision matrix or
    the geometry blocks the workspace, and sampling planners will struggle);
  * pairs that collide in at least `--always` of the states: these are almost certainly structural
    overlaps and belong in the allowed-collision matrix (printed as SRDF lines to paste or feed
    into the generator).

  ros2 run mobile_manipulator_moveit_config sample_collisions.py --group lift_arm --samples 300
"""
import argparse
import collections
import random
import sys

from moveit_msgs.srv import GetStateValidity
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--group', default='lift_arm')
    ap.add_argument('--samples', type=int, default=300)
    ap.add_argument('--always', type=float, default=0.9)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--limits', default='', help='joint=lo:hi,... overrides')
    args = ap.parse_args()

    rclpy.init()
    node = Node('sample_collisions')
    state = {}
    node.create_subscription(JointState, '/joint_states', lambda m: state.update(msg=m), 10)
    client = node.create_client(GetStateValidity, '/check_state_validity')
    if not client.wait_for_service(timeout_sec=60):
        print('check_state_validity is not available (is demo.launch.py running?)', file=sys.stderr)
        return 1
    while 'msg' not in state:
        rclpy.spin_once(node, timeout_sec=0.2)
    base = state['msg']

    # Joint ranges: lift 0..1.5, everything else +-pi unless overridden.
    ranges = {n: (0.0, 1.5) if n == 'lift_joint' else (-3.14, 3.14) for n in base.name}
    for item in filter(None, args.limits.split(',')):
        joint, span = item.split('=')
        lo, hi = span.split(':')
        ranges[joint] = (float(lo), float(hi))

    rng = random.Random(args.seed)
    pairs = collections.Counter()
    valid = 0
    for _ in range(args.samples):
        js = JointState()
        js.name = list(base.name)
        js.position = [rng.uniform(*ranges[n]) for n in base.name]
        request = GetStateValidity.Request()
        request.group_name = args.group
        request.robot_state.joint_state = js
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=30)
        result = future.result()
        if result is None:
            continue
        if result.valid:
            valid += 1
        seen = {tuple(sorted((c.contact_body_1, c.contact_body_2))) for c in result.contacts}
        pairs.update(seen)

    print(f'group={args.group} samples={args.samples} valid={valid} '
          f'({100.0 * valid / args.samples:.1f}%)')
    for (a, b), n in pairs.most_common(15):
        frac = n / args.samples
        flag = '  <-- almost always: add to the allowed-collision matrix?' if frac >= args.always else ''
        print(f'  {a:28s} {b:28s} {100 * frac:5.1f}%{flag}')
    print('\nSRDF lines for the almost-always pairs:')
    for (a, b), n in pairs.most_common():
        if n / args.samples >= args.always:
            print(f'  <disable_collisions link1="{a}" link2="{b}" reason="Always"/>')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
