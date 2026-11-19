"""Planner tests. Pure-Python parts always run; the GPU parts need CUDA + cuRobo (the gpu image)."""
import numpy as np
import pytest

from curobo_bridge.planner import Box, PlanResult, voxelize_points


def test_voxelize_groups_points_into_cubes():
    pts = np.array([[0.01, 0.01, 0.01], [0.02, 0.03, 0.04], [0.31, 0.0, 0.0]])
    boxes = voxelize_points(pts, voxel=0.1)
    assert len(boxes) == 2
    assert all(b.size == (0.1, 0.1, 0.1) for b in boxes)
    assert boxes[0].center == pytest.approx((0.05, 0.05, 0.05))


def test_voxelize_only_grows_when_capped():
    pts = np.random.default_rng(0).uniform(0, 1, size=(5000, 3))
    boxes = voxelize_points(pts, voxel=0.02, max_boxes=100)
    assert 0 < len(boxes) <= 100
    edge = boxes[0].size[0]
    assert edge > 0.02
    for p in pts:     # every input point stays inside some box: the obstacle never shrinks
        assert any(np.all(np.abs(p - np.array(b.center)) <= edge / 2 + 1e-9) for b in boxes)


def test_voxelize_with_no_room_is_an_error_not_a_giant_cube():
    pts = np.random.default_rng(0).uniform(0, 1, size=(50, 3))
    with pytest.raises(ValueError):
        voxelize_points(pts, voxel=0.05, max_boxes=0)       # the explicit boxes already fill the cache
    with pytest.raises(ValueError):
        voxelize_points(pts, voxel=0.05, max_boxes=-3)


def test_voxelize_never_returns_more_than_the_cap_or_an_oversized_cube():
    # spread over a range no 12 doublings of 0.05 m can cover with one cell per point
    pts = np.array([[0.0, 0.0, 0.0], [1e6, 0.0, 0.0], [-1e6, 0.0, 0.0]])
    with pytest.raises(ValueError):
        voxelize_points(pts, voxel=0.05, max_boxes=1)
    boxes = voxelize_points(np.random.default_rng(1).uniform(0, 1, size=(500, 3)), voxel=0.05, max_boxes=1)
    assert len(boxes) == 1 and boxes[0].size[0] <= 0.05 * 2 ** 12    # fits at a size whose cell count was checked
    assert np.all(np.abs(np.array(boxes[0].center) - 0.5) <= boxes[0].size[0])


def test_voxelize_empty():
    assert voxelize_points(np.zeros((0, 3))) == []


def test_resolve_ik_weights_validates_length_and_keeps_zeros():
    from curobo_bridge.planner import resolve_ik_weights
    names = ['lift', 'a', 'b']
    assert resolve_ik_weights(names, [36.0, 1.0, 0.0]) == [36.0, 1.0, 0.0]     # zero = ignore the joint
    assert resolve_ik_weights(names, None, ['lift'], 6.0) == [36.0, 1.0, 1.0]
    assert resolve_ik_weights(names, []) == [1.0, 1.0, 1.0]
    with pytest.raises(ValueError, match='3 joints'):
        resolve_ik_weights(names, [1.0, 1.0])                                  # the old node dropped zeros: 2 for 3
    with pytest.raises(ValueError, match='negative'):
        resolve_ik_weights(names, [1.0, -1.0, 1.0])
    with pytest.raises(ValueError, match='not joints'):
        resolve_ik_weights(names, None, ['lft'])                               # a typo must not be ignored silently


def test_route_is_explicit():
    from curobo_bridge.planner import route_from_source
    assert route_from_source('trajopt') == 'trajopt' and route_from_source(None) == 'trajopt'
    assert route_from_source('graph_fit') == 'graph_seed' and route_from_source('graph_seed') == 'graph_seed'


def test_plan_result_duration():
    r = PlanResult(success=True, positions=[[0.0], [1.0], [2.0]], dt=0.5)
    assert r.duration == pytest.approx(1.0)


# ---- GPU ------------------------------------------------------------------------------------
torch = pytest.importorskip('torch')
pytest.importorskip('curobo')
needs_gpu = pytest.mark.skipif(not torch.cuda.is_available(), reason='no CUDA device')


@pytest.fixture(scope='module')
def planner():
    if not torch.cuda.is_available():
        pytest.skip('no CUDA device')
    from curobo_bridge import CuroboPlanner
    p = CuroboPlanner('franka.yml')
    yield p
    p.close()


@needs_gpu
def test_robot_info(planner):
    assert planner.dimension == 7
    assert len(planner.joint_names) == 7
    assert len(planner.default_joint_state) == 7


@needs_gpu
def test_joint_plan_reaches_the_goal(planner):
    start = planner.default_joint_state
    goal = list(np.array(start) + np.array([0.4, -0.2, 0.2, 0.1, 0.0, 0.2, 0.0]))
    r = planner.plan_joint(start, goal)
    assert r.success, r.message
    assert r.joint_names == planner.joint_names
    assert len(r.positions) > 2 and r.dt > 0
    assert np.allclose(r.positions[0], start, atol=1e-2)
    assert np.allclose(r.positions[-1], goal, atol=2e-2)


@needs_gpu
def test_pose_plan_succeeds_with_a_box_in_the_world(planner):
    planner.set_world([Box('b', (0.5, 0.35, 0.3), (0.1, 0.1, 0.4))])
    r = planner.plan_pose(planner.default_joint_state, (0.5, 0.0, 0.3))
    planner.clear_world()
    assert r.success, r.message
    assert r.duration > 0


@needs_gpu
def test_start_inside_an_obstacle_is_refused_structurally(planner):
    n = planner.set_world([Box('swallow', (0.0, 0.0, 0.4), (3.0, 3.0, 3.0))])
    assert n == 1
    r = planner.plan_joint(planner.default_joint_state, list(np.array(planner.default_joint_state) + 0.1))
    planner.clear_world()
    assert not r.success
    assert r.failure_reason_code == 'no_path' and r.failure_stage == 'planning'


@needs_gpu
def test_wrong_dimension_is_an_invalid_request(planner):
    r = planner.plan_joint([0.0, 0.0], [0.1, 0.1])
    assert not r.success
    assert r.failure_reason_code == 'invalid_request' and r.failure_stage == 'precheck'


@needs_gpu
def test_points_become_obstacles_and_too_many_boxes_are_an_error(planner):
    pts = np.random.default_rng(1).uniform([0.4, -0.5, 0.0], [0.6, 0.5, 0.2], size=(3000, 3))
    n = planner.set_world(points=pts, point_voxel=0.02)
    assert 0 < n <= planner.max_boxes
    too_many = [Box(f'b{i}', (2.0, 2.0, 2.0), (0.01, 0.01, 0.01)) for i in range(planner.max_boxes + 1)]
    with pytest.raises(ValueError):
        planner.set_world(too_many)
    planner.clear_world()


def _goal_near(planner):
    return list(np.array(planner.default_joint_state) + np.array([0.4, -0.2, 0.2, 0.1, 0.0, 0.2, 0.0]))


@needs_gpu
def test_clear_world_really_removes_the_cuboids(planner):
    """cuRobo's cache keeps the previous voxel layer when a scene omits it (see _voxel_scene); cuboids must
    not behave the same way: after clear_world a blocking box may not linger."""
    start = planner.default_joint_state
    goal = _goal_near(planner)
    assert planner.check_states([start]) == [True] and planner.plan_joint(start, goal).success
    planner.set_world([Box('swallow', (0.0, 0.0, 0.4), (3.0, 3.0, 3.0))])
    assert planner.check_states([start]) == [False]
    assert not planner.plan_joint(start, goal).success
    planner.clear_world()
    assert planner.check_states([start]) == [True]
    r = planner.plan_joint(start, goal)
    assert r.success, r.message


@needs_gpu
@pytest.mark.parametrize('blocker_first', [True, False])
def test_a_smaller_world_drops_the_cuboids_it_omits(planner, blocker_first):
    start = planner.default_joint_state
    goal = _goal_near(planner)
    blocker = Box('swallow', (0.0, 0.0, 0.4), (3.0, 3.0, 3.0))
    far = Box('far', (3.0, 3.0, 3.0), (0.1, 0.1, 0.1))        # nowhere near the arm
    planner.set_world([blocker, far] if blocker_first else [far, blocker])
    assert planner.check_states([start]) == [False]
    assert planner.set_world([far]) == 1                       # the blocker is no longer in the scene
    assert planner.check_states([start]) == [True]
    r = planner.plan_joint(start, goal)
    assert r.success, r.message
    planner.clear_world()


# Our own arm: only runs when a config generated by docker/tools/generate_curobo_robot.py is given
# (CUROBO_TEST_ROBOT=/robots/mm_arm.yml, with its URDF and meshes mounted at the same path).
@needs_gpu
def test_generated_robot_config_plans_and_respects_obstacles():
    import os
    path = os.environ.get('CUROBO_TEST_ROBOT')
    if not path:
        pytest.skip('CUROBO_TEST_ROBOT not set')
    from curobo_bridge import CuroboPlanner
    p = CuroboPlanner(path)
    assert p.dimension == 6
    start = [-0.8, -0.5, 0.8, 0.0, 0.5, 0.0]
    goal = [0.8, -0.5, 0.8, 0.0, 0.5, 0.0]
    assert p.plan_joint(start, goal).success
    p.set_world([Box('slab', (0.0, 0.0, 1.9), (8.0, 8.0, 0.1))])   # through the arm
    r = p.plan_joint(start, goal)
    assert not r.success and r.failure_reason_code == 'no_path'
    p.clear_world()
    assert p.plan_joint(start, goal).success


@needs_gpu
def test_ik_recovers_the_default_pose_and_respects_obstacles(planner):
    from curobo_bridge import Box
    ks = planner._planner.compute_kinematics(planner._state(planner.default_joint_state))
    pos = ks.tool_poses.position.reshape(-1, 3)[0].tolist()
    quat = ks.tool_poses.quaternion.reshape(-1, 4)[0].tolist()
    r = planner.solve_ik(pos, quat)
    assert r.success, r.message
    assert len(r.joints) == planner.dimension and r.joint_names == planner.joint_names
    assert r.position_error < 5e-3
    assert planner.check_states([r.joints]) == [True]
    # an obstacle sitting on the target pose leaves no collision-free solution
    planner.set_world([Box('on_target', tuple(pos), (0.3, 0.3, 0.3))])
    blocked = planner.solve_ik(pos, quat)
    planner.clear_world()
    assert not blocked.success and blocked.failure_reason_code == 'no_solution'


@needs_gpu
def test_ik_prefers_the_seed_branch(planner):
    import numpy as np
    # target = FK of a configuration near a seed; the solution must stay on the seed's branch
    seed = [v + d for v, d in zip(planner.default_joint_state, [0.3, -0.2, 0.25, 0.2, -0.3, 0.2, 0.1])]
    ks = planner._planner.compute_kinematics(planner._state(seed))
    pos = ks.tool_poses.position.reshape(-1, 3)[0].tolist()
    quat = ks.tool_poses.quaternion.reshape(-1, 4)[0].tolist()
    r = planner.solve_ik(pos, quat, seed=seed)
    assert r.success, r.message
    assert float(np.linalg.norm(np.array(r.joints) - np.array(seed))) < 1.5


@needs_gpu
def test_ik_rejects_a_wrong_sized_seed(planner):
    r = planner.solve_ik((0.4, 0.0, 0.4), seed=[0.0, 0.0])
    assert not r.success and r.failure_reason_code == 'invalid_request'


@needs_gpu
def test_state_validity_covers_limits_self_collision_and_world(planner):
    from curobo_bridge import Box
    d = planner.default_joint_state
    out_of_limits = list(d)
    out_of_limits[0] = 10.0
    folded = [0.0, 1.7, 0.0, -3.0, 0.0, 3.5, 0.0]
    assert planner.check_states([d, out_of_limits, folded]) == [True, False, False]
    planner.set_world([Box('swallow', (0.0, 0.0, 0.4), (3.0, 3.0, 3.0))])
    assert planner.check_states([d]) == [False]
    planner.clear_world()
    assert planner.check_states([d]) == [True]
    assert planner.check_states([]) == []
    with pytest.raises(ValueError):
        planner.check_states([[0.0, 0.0]])


@needs_gpu
def test_motion_check_samples_between_waypoints(planner):
    d = np.array(planner.default_joint_state)
    assert planner.check_motion([list(d), list(d + 0.1)])
    # a straight joint-space motion into a joint limit must be reported invalid
    beyond = list(d)
    beyond[0] = 10.0
    assert not planner.check_motion([list(d), beyond])
    assert planner.check_motion([])
    with pytest.raises(ValueError):
        planner.check_motion([list(d)], max_step=0.0)


@needs_gpu
def test_esdf_needs_a_configured_grid(planner):
    with pytest.raises(ValueError):
        planner.set_world(esdf=np.zeros((4, 4, 4), dtype=np.float32))



# ---- trajectory-optimisation fallback ---------------------------------------------------------
class _FakeCfg:
    always_run_trajopt = True
    run_trajopt_on_graph_seed = True


class _FakeResult:
    def __init__(self, ok, source='trajopt'):
        self.success = torch.tensor([[ok]])
        self.trajectory_source = source
        self.total_time = 0.25
        self._plan = type('P', (), {'position': torch.zeros(1, 1, 3, 2), 'joint_names': ['a', 'b']})()

    def get_interpolated_plan(self):
        return self._plan


class _FakeMotionPlanner:
    """plan_cspace that 'fails trajectory optimisation' (the default config) and succeeds from a graph seed
    when the two bypass switches are off; records the calls and the config seen."""
    joint_names = ['a', 'b']
    config = _FakeCfg()

    def __init__(self, seed_ok=True):
        self.calls, self.seed_ok = [], seed_ok
        self.trajopt_solver = type('T', (), {'config': type('C', (), {'interpolation_dt': 0.025})()})()

    def plan_cspace(self, goal, cur, max_attempts=20, enable_graph_attempt=1, finetune_attempts=1):
        bypass = not self.config.always_run_trajopt and not self.config.run_trajopt_on_graph_seed
        self.calls.append((max_attempts, enable_graph_attempt, bypass))
        if bypass and self.seed_ok:
            return _FakeResult(True, 'graph_fit')
        return _FakeResult(False)


def _fake_planner(seed_fallback, seed_ok=True, enable_trajopt=True):
    from curobo_bridge.planner import CuroboPlanner
    p = CuroboPlanner.__new__(CuroboPlanner)
    p._planner = _FakeMotionPlanner(seed_ok)
    _FakeMotionPlanner.config = _FakeCfg()
    p._seed_fallback = seed_fallback and enable_trajopt
    p._fallback_attempts = 6
    p._max_attempts = 20
    p._enable_graph_attempt = 1
    p._torch = torch
    p._check_state = lambda q, what: None
    p._state = lambda q: None
    return p


def test_trajopt_failure_falls_back_to_the_validated_graph_seed():
    p = _fake_planner(seed_fallback=True)
    r = p.plan_joint([0.0, 0.0], [1.0, 1.0])
    assert r.success and r.route == 'graph_seed'
    assert 'not optimised' in r.message
    assert r.planning_time_s == pytest.approx(0.5)            # the failed pass and the fallback both count
    assert p._planner.calls == [(20, 1, False), (6, 0, True)]
    cfg = p._planner.config                                     # the switches are put back
    assert cfg.always_run_trajopt and cfg.run_trajopt_on_graph_seed


def test_fallback_can_be_turned_off_and_failure_stays_a_failure():
    off = _fake_planner(seed_fallback=False)
    r = off.plan_joint([0.0, 0.0], [1.0, 1.0])
    assert not r.success and r.route == '' and len(off._planner.calls) == 1
    no_seed = _fake_planner(seed_fallback=True, seed_ok=False)
    r = no_seed.plan_joint([0.0, 0.0], [1.0, 1.0])
    assert not r.success and r.failure_reason_code == 'no_path' and len(no_seed._planner.calls) == 2
    assert no_seed._planner.config.always_run_trajopt          # restored on the failing path too
    seed_only = _fake_planner(seed_fallback=True, enable_trajopt=False)
    assert len(seed_only._planner.calls) == 0 and not seed_only.plan_joint([0.0, 0.0], [1.0, 1.0]).success
    assert len(seed_only._planner.calls) == 1                  # enable_trajopt=False is already the seed route


@needs_gpu
def test_fallback_returns_a_valid_seed_when_trajopt_fails_on_the_real_planner(planner):
    """Force 'trajectory optimisation fails' (every default-config plan_cspace returns no trajectory) and
    check the real graph-seed route gives a trajectory that cuRobo's own checker accepts."""
    mp = planner._planner
    real = mp.plan_cspace

    def failing_when_default(goal, cur, **kw):
        if mp.config.always_run_trajopt:
            return None
        return real(goal, cur, **kw)

    planner._seed_fallback, mp.plan_cspace = True, failing_when_default
    try:
        start = planner.default_joint_state
        goal = list(np.array(start) + np.array([0.4, -0.2, 0.2, 0.1, 0.0, 0.2, 0.0]))
        r = planner.plan_joint(start, goal)
    finally:
        mp.plan_cspace = real
    assert r.success, r.message
    assert r.route == 'graph_seed'
    assert np.allclose(r.positions[-1], goal, atol=2e-2)
    assert planner.check_motion(r.positions, max_step=0.1)
    assert mp.config.always_run_trajopt and mp.config.run_trajopt_on_graph_seed
    normal = planner.plan_joint(start, goal)
    assert normal.success and normal.route == 'trajopt'


def test_check_motion_with_a_tiny_step_is_refused_not_unbounded():
    from curobo_bridge.planner import CuroboPlanner
    p = CuroboPlanner.__new__(CuroboPlanner)
    p.check_states = lambda states: [True] * len(states)
    with pytest.raises(ValueError, match='max_step'):
        p.check_motion([[0.0, 0.0], [1.0, 1.0]], max_step=1e-9)
    assert p.check_motion([[0.0, 0.0], [1.0, 1.0]], max_step=0.1)
