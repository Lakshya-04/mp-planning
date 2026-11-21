"""Endpoint precheck of plan_joint: pure Python, no CUDA, no cuRobo."""
from curobo_bridge.planner import CuroboPlanner, endpoint_failure


def test_endpoint_failure_names_the_invalid_end_and_ignores_unknown_checks():
    assert endpoint_failure(True, True) is None
    assert endpoint_failure(None, None) is None            # the check could not run: not a failure
    assert endpoint_failure(True, None) is None
    r = endpoint_failure(False, True)
    assert not r.success and r.failure_reason_code == 'start_in_collision' and r.failure_stage == 'precheck'
    assert endpoint_failure(True, False).failure_reason_code == 'goal_in_collision'
    both = endpoint_failure(False, False)
    assert both.failure_reason_code == 'start_in_collision' and 'start and goal' in both.message


def _planner_without_gpu(valid, precheck=True):
    """A CuroboPlanner shell (no CUDA, no cuRobo) whose state check returns `valid`; plan_cspace must not run."""
    p = object.__new__(CuroboPlanner)
    p._precheck_endpoints = precheck
    p.check_states = lambda states: valid if not isinstance(valid, Exception) else (_ for _ in ()).throw(valid)
    return p


def test_precheck_rejects_an_invalid_start_before_any_planning_attempt():
    p = _planner_without_gpu([False, True])
    out = p._precheck([0.0] * 7, [0.4] * 7)
    assert out.failure_reason_code == 'start_in_collision'


def test_precheck_passes_valid_ends_and_can_be_switched_off_or_fail_open():
    assert _planner_without_gpu([True, True])._precheck([0.0], [1.0]) is None
    assert _planner_without_gpu([False, False], precheck=False)._precheck([0.0], [1.0]) is None
    assert _planner_without_gpu(RuntimeError('no such api'))._precheck([0.0], [1.0]) is None
