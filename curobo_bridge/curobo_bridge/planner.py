"""A ROS-free planner over cuRobo v2.

Mirrors the structured results of the motion_planner ports (success, failure code, failure stage,
message) so a C++ client or a ROS node can adapt it without parsing text. cuRobo itself is imported
lazily: importing this module works without a GPU stack, constructing a CuroboPlanner needs one.

Frames: everything is in the robot base frame, the same convention as the VAMP adapter. Poses are
[x, y, z] + quaternion [w, x, y, z] (cuRobo's order).
"""
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class Box:
    """An oriented box obstacle; `size` is the full extent along each local axis."""
    id: str
    center: Tuple[float, float, float]
    size: Tuple[float, float, float]
    quat_wxyz: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)


@dataclass
class PlanResult:
    success: bool = False
    failure_reason_code: str = 'none'
    failure_stage: str = 'none'          # none | precheck | planning
    message: str = ''
    joint_names: List[str] = field(default_factory=list)
    positions: List[List[float]] = field(default_factory=list)   # one row per waypoint
    dt: float = 0.0                       # seconds between rows (cuRobo's interpolation step)
    planning_time_s: float = 0.0

    @property
    def duration(self) -> float:
        return self.dt * max(0, len(self.positions) - 1)


@dataclass
class IkResult:
    success: bool = False
    failure_reason_code: str = 'none'
    failure_stage: str = 'none'
    message: str = ''
    joint_names: List[str] = field(default_factory=list)
    joints: List[float] = field(default_factory=list)
    position_error: float = 0.0           # m
    rotation_error: float = 0.0           # rad


def voxelize_points(points: np.ndarray, voxel: float = 0.05, max_boxes: int = 2000) -> List[Box]:
    """Turn a point cloud into occupied-voxel boxes (cuRobo's scene has no raw point cloud type).

    Each occupied voxel becomes one cube of edge `voxel`. If that exceeds `max_boxes` the voxel size
    is doubled until it fits, so the obstacle only grows (conservative), never shrinks. For dense
    sensing use cuRobo's own ESDF mapper (see docs/curobo_v2_notes.md); this is the bridge for the
    sparse, already-filtered clouds the scene model hands over.
    """
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if len(pts) == 0:
        return []
    size = float(voxel)
    for _ in range(12):
        cells = np.unique(np.floor(pts / size).astype(np.int64), axis=0)
        if len(cells) <= max_boxes:
            break
        size *= 2.0
    return [Box(f'pc_{i}', tuple(((c + 0.5) * size).tolist()), (size, size, size)) for i, c in enumerate(cells)]


@dataclass
class EsdfSpec:
    """The fixed voxel grid the planner reserves for an ESDF (cuRobo allocates it at construction).

    `dims` are the full extents (m) and must be an even number of voxels per axis. The grid is
    centred on `center` in the robot base frame; the centre can move per update, the size cannot.
    """
    dims: Tuple[float, float, float] = (2.0, 2.0, 2.0)
    voxel_size: float = 0.04
    center: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    def shape(self) -> Tuple[int, int, int]:
        n = tuple(int(round(d / self.voxel_size)) for d in self.dims)
        if any(c < 2 or c % 2 for c in n):
            raise ValueError(f'ESDF grid {self.dims} at {self.voxel_size} m needs an even voxel count '
                             f'per axis, got {n}')
        return n


def points_to_esdf(points: np.ndarray, spec: EsdfSpec, max_distance: float = 1.0) -> np.ndarray:
    """Signed distance field (m, positive OUTSIDE obstacles) of a point cloud on `spec`'s grid.

    Each point occupies the voxel it falls in; points outside the grid are ignored. The result has
    shape (nx, ny, nz) with cell (i, j, k) centred at `center + ((i, j, k) + 0.5 - n/2) * voxel`.
    Distances are clamped to +-max_distance. Needs scipy.
    """
    from scipy.ndimage import distance_transform_edt

    nx, ny, nz = spec.shape()
    occupied = np.zeros((nx, ny, nz), dtype=bool)
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if len(pts):
        idx = np.floor((pts - np.asarray(spec.center)) / spec.voxel_size + np.array([nx, ny, nz]) / 2.0).astype(int)
        inside = np.all((idx >= 0) & (idx < np.array([nx, ny, nz])), axis=1)
        idx = idx[inside]
        occupied[idx[:, 0], idx[:, 1], idx[:, 2]] = True
    if not occupied.any():
        return np.full((nx, ny, nz), max_distance, dtype=np.float32)
    outside = distance_transform_edt(~occupied) * spec.voxel_size
    inside_d = distance_transform_edt(occupied) * spec.voxel_size
    return np.clip(outside - inside_d, -max_distance, max_distance).astype(np.float32)


class CuroboPlanner:
    """Joint-goal and pose-goal planning with cuRobo v2 on the GPU."""

    def __init__(self, robot: str = 'franka.yml', max_goalset: int = 1, max_boxes: int = 256,
                 esdf: Optional[EsdfSpec] = None, warmup: bool = True):
        import torch
        from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
        from curobo.types import JointState

        if not torch.cuda.is_available():
            raise RuntimeError('cuRobo needs a CUDA device')
        self._torch = torch
        self._JointState = JointState
        # cuRobo allocates the obstacle cache up front; update_world can then swap scenes up to this size.
        self.max_boxes = max_boxes
        self.esdf = esdf
        cache = {'cuboid': max_boxes}
        if esdf is not None:
            esdf.shape()   # validate before allocating anything on the GPU
            cache['voxel'] = {'layers': 1, 'dims': list(esdf.dims), 'voxel_size': esdf.voxel_size}
        self._planner = MotionPlanner(MotionPlannerCfg.create(
            robot=robot, max_goalset=max_goalset, collision_cache=cache))
        if warmup:
            self._planner.warmup(enable_graph=True, num_warmup_iterations=3)
        self._world_set = False

    def close(self) -> None:
        """Releases the GPU resources. Use one planner per process, or close one before building
        another with a different collision-cache layout (an ESDF grid): Warp specialises its collision
        kernels on that layout, and a planner built earlier then fails with illegal memory accesses."""
        self._planner.destroy()

    # ---- robot info -------------------------------------------------------------------------
    @property
    def joint_names(self) -> List[str]:
        return list(self._planner.joint_names)

    @property
    def dimension(self) -> int:
        return len(self.joint_names)

    @property
    def tool_frames(self) -> List[str]:
        return list(self._planner.tool_frames)

    @property
    def default_joint_state(self) -> List[float]:
        return self._planner.default_joint_state.position.detach().cpu().numpy().tolist()

    # ---- world ------------------------------------------------------------------------------
    def set_world(self, boxes: Sequence[Box] = (), points: Optional[np.ndarray] = None,
                  point_voxel: float = 0.05, esdf: Optional[np.ndarray] = None,
                  esdf_center: Optional[Sequence[float]] = None) -> int:
        """Replaces the world. Returns the number of obstacles handed to cuRobo (boxes + 1 per ESDF).

        Points: with an ESDF grid configured (`esdf=EsdfSpec(...)` at construction) they become that
        grid's distance field; otherwise they are voxelized into cubes (capped by the obstacle
        cache, conservative). `esdf` passes a ready-made field instead (nx, ny, nz; metres, positive
        outside obstacles, e.g. from the scene model or a mapper) on the configured grid, centred
        at `esdf_center`. Explicit boxes beyond the cache are an error, not silently dropped.
        """
        from curobo.scene import Cuboid, Scene

        all_boxes = list(boxes)
        if len(all_boxes) > self.max_boxes:
            raise ValueError(f'{len(all_boxes)} boxes exceed the obstacle cache ({self.max_boxes})')
        field = None
        if esdf is not None:
            if self.esdf is None:
                raise ValueError('no ESDF grid configured: construct the planner with esdf=EsdfSpec(...)')
            if tuple(np.shape(esdf)) != self.esdf.shape():
                raise ValueError(f'ESDF shape {tuple(np.shape(esdf))} != grid {self.esdf.shape()}')
            field = np.asarray(esdf, dtype=np.float32)
        elif points is not None and len(points) > 0:
            if self.esdf is not None:
                field = points_to_esdf(points, self._grid(esdf_center))
            else:
                all_boxes += voxelize_points(points, point_voxel, self.max_boxes - len(all_boxes))
        cuboids = [Cuboid(name=b.id, dims=list(b.size), pose=[*b.center, *b.quat_wxyz]) for b in all_boxes]
        voxels = self._voxel_scene(field, esdf_center) if self.esdf is not None else []
        self._planner.update_world(Scene(cuboid=cuboids, voxel=voxels))
        self._world_set = True
        return len(cuboids) + (1 if field is not None else 0)

    def clear_world(self) -> None:
        from curobo.scene import Scene
        voxels = self._voxel_scene(None, None) if self.esdf is not None else []
        self._planner.update_world(Scene(voxel=voxels))
        self._world_set = False

    # ---- ESDF helpers -----------------------------------------------------------------------
    def _grid(self, center: Optional[Sequence[float]]) -> EsdfSpec:
        c = tuple(center) if center is not None else self.esdf.center
        return EsdfSpec(self.esdf.dims, self.esdf.voxel_size, c)

    def _voxel_scene(self, field: Optional[np.ndarray], center: Optional[Sequence[float]]):
        """The voxel layer for update_world. With no field it is an all-free grid, because the cache
        keeps the previous layer otherwise (a stale ESDF would silently stay in the world)."""
        from curobo.scene import VoxelGrid
        t = self._torch
        spec = self._grid(center)
        if field is None:
            field = np.full(spec.shape(), 1.0, dtype=np.float32)    # far outside everything
        # Same convention as ours (positive outside; cuRobo initialises empty voxels to +10000) and
        # x-major flattening, so the field goes in as is.
        feature = t.as_tensor(field.reshape(-1), device='cuda', dtype=t.float32)
        return [VoxelGrid(name='esdf', pose=[*spec.center, 1.0, 0.0, 0.0, 0.0], dims=list(spec.dims),
                          voxel_size=spec.voxel_size, feature_tensor=feature)]

    # ---- planning ---------------------------------------------------------------------------
    def plan_joint(self, start: Sequence[float], goal: Sequence[float]) -> PlanResult:
        """Collision-free joint-space move start -> goal."""
        bad = self._check_state(start, 'start') or self._check_state(goal, 'goal')
        if bad:
            return bad
        cur, tgt = self._state(start), self._state(goal)
        result = self._planner.plan_cspace(tgt, cur)
        return self._wrap(result, last=goal)

    def plan_pose(self, start: Sequence[float], position: Sequence[float],
                  quat_wxyz: Sequence[float] = (1.0, 0.0, 0.0, 0.0)) -> PlanResult:
        """Collision-free move to a tool pose (position + quaternion wxyz, robot base frame)."""
        bad = self._check_state(start, 'start')
        if bad:
            return bad
        from curobo.types import GoalToolPose
        t = self._torch
        goal = GoalToolPose(
            tool_frames=self._planner.tool_frames,
            position=t.tensor([[[[list(position)]]]], device='cuda', dtype=t.float32),
            quaternion=t.tensor([[[[list(quat_wxyz)]]]], device='cuda', dtype=t.float32))
        return self._wrap(self._planner.plan_pose(goal, self._state(start)))

    # ---- IK and validity ---------------------------------------------------------------------
    def solve_ik(self, position: Sequence[float], quat_wxyz: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
                 seed: Optional[Sequence[float]] = None) -> IkResult:
        """Collision-free joint solution for a tool pose (robot base frame). `seed` biases the
        solution toward a configuration (default: the robot's default state)."""
        seed = list(seed) if seed is not None and len(seed) else self.default_joint_state
        bad = self._check_state(seed, 'seed')
        if bad:
            return IkResult(failure_reason_code=bad.failure_reason_code, failure_stage='precheck',
                            message=bad.message)
        from curobo.types import GoalToolPose
        t = self._torch
        goal = GoalToolPose(
            tool_frames=self._planner.tool_frames,
            position=t.tensor([[[[list(position)]]]], device='cuda', dtype=t.float32),
            quaternion=t.tensor([[[[list(quat_wxyz)]]]], device='cuda', dtype=t.float32))
        res = self._planner.ik_solver.solve_pose(goal, current_state=self._state(seed))
        out = IkResult(joint_names=self.joint_names)
        if not bool(res.success.any()):
            out.failure_reason_code = 'no_solution'
            out.failure_stage = 'planning'
            out.message = 'cuRobo found no collision-free IK solution for this pose'
            return out
        out.success = True
        out.joints = res.solution.reshape(-1, self.dimension)[0].detach().cpu().numpy().tolist()
        for name, attr in (('position_error', 'position_error'), ('rotation_error', 'rotation_error')):
            value = getattr(res, attr, None)
            if value is not None:
                setattr(out, name, float(value.reshape(-1)[0]))
        return out

    def check_states(self, states: Sequence[Sequence[float]]) -> List[bool]:
        """Per-state validity: inside joint limits, no self-collision, clear of the world."""
        for i, q in enumerate(states):
            if len(q) != self.dimension:
                raise ValueError(f'state {i} has {len(q)} joints, expected {self.dimension}')
        if not len(states):
            return []
        t = self._torch
        mask = self._planner.graph_planner.check_samples_feasibility(
            t.tensor([list(q) for q in states], device='cuda', dtype=t.float32))
        # a single state comes back as a 0-d tensor
        return [bool(v) for v in np.atleast_1d(mask.detach().cpu().numpy()).tolist()]

    def check_motion(self, waypoints: Sequence[Sequence[float]], max_step: float = 0.05) -> bool:
        """True if every waypoint and every interpolated state between them (largest joint step
        `max_step` rad) is valid. A sampled check: gaps thinner than `max_step` can be missed."""
        if max_step <= 0:
            raise ValueError('max_step must be positive')
        pts = [np.asarray(q, dtype=np.float64) for q in waypoints]
        dense = []
        for a, b in zip(pts, pts[1:]):
            n = max(1, int(np.ceil(np.max(np.abs(b - a)) / max_step)))
            dense += [a + (b - a) * k / n for k in range(n)]
        dense.append(pts[-1]) if pts else None
        return all(self.check_states([list(q) for q in dense])) if dense else True

    # ---- internals --------------------------------------------------------------------------
    def _state(self, q: Sequence[float]):
        t = self._torch
        return self._JointState.from_position(
            t.tensor([list(q)], device='cuda', dtype=t.float32), joint_names=self.joint_names)

    def _check_state(self, q: Sequence[float], what: str) -> Optional[PlanResult]:
        if len(q) != self.dimension:
            return PlanResult(failure_reason_code='invalid_request', failure_stage='precheck',
                              message=f'{what} must have {self.dimension} joint values, got {len(q)}')
        return None

    def _wrap(self, result, last: Optional[Sequence[float]] = None) -> PlanResult:
        out = PlanResult(joint_names=self.joint_names)
        if result is None or not bool(result.success.any()):
            out.failure_reason_code = 'no_path'
            out.failure_stage = 'planning'
            out.message = 'cuRobo found no collision-free trajectory'
            if result is not None and getattr(result, 'total_time', None) is not None:
                out.planning_time_s = float(result.total_time)
            return out
        plan = result.get_interpolated_plan()
        rows = plan.position.reshape(-1, plan.position.shape[-1]).detach().cpu().numpy()
        # The plan also carries locked joints (e.g. gripper fingers); keep the active ones, in order.
        cols = [list(plan.joint_names).index(n) for n in self.joint_names] if plan.joint_names else None
        out.positions = (rows[:, cols] if cols else rows).tolist()
        out.dt = float(self._planner.trajopt_solver.config.interpolation_dt)
        out.planning_time_s = float(getattr(result, 'total_time', 0.0) or 0.0)
        out.success = True
        out.failure_stage = 'none'
        return out
