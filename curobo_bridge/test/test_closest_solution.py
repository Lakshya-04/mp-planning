"""closest_solution: pure numpy, runs without torch / cuRobo."""
import numpy as np
import pytest

from curobo_bridge.planner import closest_solution, ik_weights_from_joint_types


def test_closest_solution_picks_nearest_successful_row():
    sols = np.array([[2.0, 0.0], [0.1, 0.1], [0.0, 0.0]])
    ok = np.array([True, True, False])      # the exact match failed, so it must not be chosen
    joints, dist, row = closest_solution(sols, ok, [0.0, 0.0])
    assert row == 1
    assert joints == pytest.approx([0.1, 0.1])
    assert dist == pytest.approx(np.hypot(0.1, 0.1))


def test_closest_solution_wraps_turns_but_not_a_lift():
    sols = np.array([[0.4, 0.5 + 2 * np.pi]])
    joints, _, _ = closest_solution(sols, np.array([True]), [0.4, 0.5])
    assert joints == pytest.approx([0.4, 0.5])
    joints, _, _ = closest_solution(np.array([[1.2, 0.0]]), np.array([True]), [0.0, 0.0])
    assert joints[0] == pytest.approx(1.2)      # within pi: left alone (prismatic-safe)


def test_closest_solution_none_when_nothing_succeeds():
    assert closest_solution(np.zeros((2, 3)), np.array([False, False]), [0, 0, 0]) is None


def test_closest_solution_does_not_shift_out_of_limits():
    sols = np.array([[2.5]])
    # seed at -3.5 would pull 2.5 to 2.5 - 2*pi = -3.78, outside [-2.9, 2.9]: keep the solver's value
    joints, _, _ = closest_solution(sols, np.array([True]), [-3.5], ([-2.9], [2.9]))
    assert joints == pytest.approx([2.5])


def test_weights_keep_lift_metres_from_counting_as_radians():
    # joints: [lift (m), shoulder (rad)]. A moves the lift 0.3 m, B moves the shoulder 0.35 rad.
    sols = np.array([[0.3, 0.1], [0.0, 0.35]])
    ok = np.ones(2, dtype=bool)
    seed = [0.0, 0.0]
    assert closest_solution(sols, ok, seed)[2] == 0           # unweighted: the slow lift is moved
    weights = ik_weights_from_joint_types(['lift', 'shoulder'], ['lift'])
    assert weights == pytest.approx([36.0, 1.0])
    joints, dist, row = closest_solution(sols, ok, seed, weights=weights)
    assert row == 1
    assert joints == pytest.approx([0.0, 0.35])
    assert dist == pytest.approx(0.35)
    with pytest.raises(ValueError):
        closest_solution(sols, ok, seed, weights=[1.0])
