"""ESDF world tests. Their own module so the plain planner of test_planner.py is closed before a
planner with a voxel cache is built (see CuroboPlanner.close)."""
import numpy as np
import pytest

torch = pytest.importorskip('torch')
pytest.importorskip('curobo')
pytest.importorskip('scipy')
needs_gpu = pytest.mark.skipif(not torch.cuda.is_available(), reason='no CUDA device')

from curobo_bridge.planner import EsdfSpec, points_to_esdf  # noqa: E402


def test_esdf_spec_needs_even_voxel_counts():
    assert EsdfSpec((2.0, 2.0, 2.0), 0.1).shape() == (20, 20, 20)
    with pytest.raises(ValueError):
        EsdfSpec((1.0, 2.0, 2.0), 0.4).shape()     # 1.0 / 0.4 -> 2 (ok) but 2.0/0.4 = 5 (odd)


def test_esdf_signs_and_values_match_a_brute_force_distance():
    spec = EsdfSpec((1.0, 1.0, 1.0), 0.1)           # 10^3 voxels, cell centre = (i + 0.5 - 5) * 0.1
    pts = np.array([[0.05, 0.05, 0.05]])            # inside cell (5, 5, 5)
    f = points_to_esdf(pts, spec)
    assert f.shape == (10, 10, 10)
    assert f[5, 5, 5] < 0                            # occupied voxel is inside
    assert f[5, 5, 6] == pytest.approx(0.1, abs=1e-6)   # next cell along z is one voxel away
    assert f[5, 8, 5] == pytest.approx(0.3, abs=1e-6)   # three voxels along y
    assert f[0, 0, 0] == pytest.approx(0.1 * np.sqrt(75), abs=1e-5)   # Euclidean, not city-block
    assert np.all(f[np.arange(10) != 5, 0, 0] > 0)


def test_esdf_ignores_points_outside_the_grid_and_clamps():
    spec = EsdfSpec((1.0, 1.0, 1.0), 0.1)
    far = points_to_esdf(np.array([[5.0, 5.0, 5.0]]), spec)
    assert np.all(far == 1.0)                       # nothing inside: everything is far from obstacles
    assert np.max(points_to_esdf(np.array([[0.05, 0.05, 0.05]]), spec, max_distance=0.25)) <= 0.25


def test_esdf_grid_follows_its_centre():
    a = points_to_esdf(np.array([[0.05, 0.05, 0.05]]), EsdfSpec((1.0, 1.0, 1.0), 0.1))
    b = points_to_esdf(np.array([[1.05, 0.05, 0.05]]), EsdfSpec((1.0, 1.0, 1.0), 0.1, (1.0, 0.0, 0.0)))
    assert np.array_equal(a, b)                     # same point relative to the grid centre


@pytest.fixture(scope='module')
def esdf_planner():
    if not torch.cuda.is_available():
        pytest.skip('no CUDA device')
    from curobo_bridge import CuroboPlanner
    p = CuroboPlanner('franka.yml', esdf=EsdfSpec((2.0, 2.0, 2.0), 0.05, (0.3, 0.0, 0.5)))
    yield p
    p.close()


def _tool_position(planner):
    ks = planner._planner.compute_kinematics(planner._state(planner.default_joint_state))
    return ks.tool_poses.position.reshape(-1, 3)[0].detach().cpu().numpy()


def _blob(center, half=0.06, step=0.02):
    g = np.arange(-half, half + 1e-9, step)
    return np.array([[center[0] + x, center[1] + y, center[2] + z] for x in g for y in g for z in g])


@needs_gpu
def test_esdf_obstacle_at_the_tool_blocks_and_axes_are_right(esdf_planner):
    p = esdf_planner
    tool = _tool_position(p)
    assert abs(tool[0] - tool[2]) > 0.1, 'test needs distinct x and z to catch swapped axes'
    goal = list(np.array(p.default_joint_state) + 0.15)

    # a blob around the tool at the start pose: the start is in collision
    assert p.set_world(points=_blob(tool)) == 1
    blocked = p.plan_joint(p.default_joint_state, goal)
    # the same blob with x and z swapped is somewhere else: not touching the robot
    p.set_world(points=_blob(tool[[2, 1, 0]]))
    elsewhere = p.plan_joint(p.default_joint_state, goal)
    p.clear_world()
    assert not blocked.success, 'an ESDF obstacle at the tool position must block the start'
    assert elsewhere.success, f'swapped-axes blob should be free: {elsewhere.message}'


@needs_gpu
def test_clear_world_really_removes_the_esdf(esdf_planner):
    p = esdf_planner
    tool = _tool_position(p)
    goal = list(np.array(p.default_joint_state) + 0.15)
    p.set_world(points=_blob(tool))
    assert not p.plan_joint(p.default_joint_state, goal).success
    p.clear_world()
    assert p.plan_joint(p.default_joint_state, goal).success


@needs_gpu
def test_ready_made_esdf_and_shape_checks(esdf_planner):
    p = esdf_planner
    field = points_to_esdf(_blob(_tool_position(p)), p.esdf)
    assert p.set_world(esdf=field) == 1
    assert not p.plan_joint(p.default_joint_state, list(np.array(p.default_joint_state) + 0.15)).success
    p.clear_world()
    with pytest.raises(ValueError):
        p.set_world(esdf=np.zeros((4, 4, 4), dtype=np.float32))


@needs_gpu
def test_set_esdf_keeps_the_boxes_and_clear_drops_everything(esdf_planner):
    from curobo_bridge import Box
    p = esdf_planner
    goal = list(np.array(p.default_joint_state) + 0.15)
    far_box = Box('far', (1.5, 1.5, 1.5), (0.1, 0.1, 0.1))
    assert p.set_world([far_box]) == 1
    field = points_to_esdf(_blob(_tool_position(p)), p.esdf)
    assert p.set_esdf(field) == 2                    # the box stays, the field is added
    assert [b.id for b in p._boxes] == ['far']
    assert not p.plan_joint(p.default_joint_state, goal).success
    p.set_esdf(np.full(p.esdf.shape(), 1.0, dtype=np.float32))   # an all-free field replaces it
    assert p.plan_joint(p.default_joint_state, goal).success
    p.clear_world()
    assert p._boxes == []
