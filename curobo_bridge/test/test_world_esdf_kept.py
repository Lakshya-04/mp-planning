"""set_world keeps the ESDF field when the update carries no field. Runs without a GPU: torch and
cuRobo's scene types are replaced by small stand-ins that record what update_world receives."""
import sys
import types

import numpy as np

from curobo_bridge.planner import Box, CuroboPlanner, EsdfSpec


class _Recorder:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def _planner(monkeypatch):
    scene = types.ModuleType('curobo.scene')
    scene.Cuboid = _Recorder
    scene.VoxelGrid = _Recorder
    scene.Scene = _Recorder
    monkeypatch.setitem(sys.modules, 'curobo', types.ModuleType('curobo'))
    monkeypatch.setitem(sys.modules, 'curobo.scene', scene)

    class Torch:
        float32 = np.float32

        @staticmethod
        def as_tensor(x, device=None, dtype=None):
            return np.asarray(x, dtype=dtype)

    class Inner:
        worlds = []

        def update_world(self, world):
            self.worlds.append(world)

    p = object.__new__(CuroboPlanner)
    p._torch = Torch
    p._planner = Inner()
    p.max_boxes = 16
    p.esdf = EsdfSpec((0.4, 0.4, 0.4), 0.1, (0.0, 0.0, 0.0))
    p._world_set = False
    p._boxes = []
    p._field = None
    p._field_center = None
    return p


def _field_of(world):
    return world.voxel[0].feature_tensor


def test_box_only_world_update_keeps_the_esdf(monkeypatch):
    p = _planner(monkeypatch)
    field = np.full(p.esdf.shape(), 1.0, dtype=np.float32)
    field[1, 1, 1] = -0.2                                   # one obstacle voxel
    assert p.set_esdf(field) == 1
    box = Box('crate', (0.1, 0.1, 0.1), (0.0, 0.0, 0.0))
    assert p.set_world([box]) == 2                          # box added, field still counted
    kept = _field_of(p._planner.worlds[-1]).reshape(p.esdf.shape())
    assert kept[1, 1, 1] == np.float32(-0.2)
    assert [c.name for c in p._planner.worlds[-1].cuboid] == ['crate']


def test_clear_world_still_removes_the_esdf(monkeypatch):
    p = _planner(monkeypatch)
    field = np.full(p.esdf.shape(), 1.0, dtype=np.float32)
    field[1, 1, 1] = -0.2
    p.set_esdf(field)
    p.clear_world()
    assert float(_field_of(p._planner.worlds[-1]).min()) == 1.0
    p.set_world([])                                         # nothing to keep any more
    assert float(_field_of(p._planner.worlds[-1]).min()) == 1.0
