"""Tests for the truth-to-estimate matching in port_error_node.

The constraint under test is uniqueness: two ports must never be scored
against the same estimate. Violating it makes the error look *better* exactly
when the estimator is worse, which is the kind of bug that never announces
itself. Run with ``./run.sh test``.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(
    0, str(Path(__file__).resolve().parent.parent / "ros_ws" / "src" / "port_error_node")
)

from port_error_node.assignment import (  # noqa: E402
    distance_matrix,
    match_errors,
    optimal_assignment,
)


def test_distance_matrix_shape_and_values():
    truths = [np.zeros(3), np.array([3.0, 4.0, 0.0])]
    estimates = [np.array([0.0, 0.0, 1.0])]
    d = distance_matrix(truths, estimates)
    assert d.shape == (2, 1)
    assert d[0, 0] == pytest.approx(1.0)
    assert d[1, 0] == pytest.approx(np.sqrt(9 + 16 + 1))


def test_distance_matrix_with_no_estimates():
    assert distance_matrix([np.zeros(3)], []).shape == (1, 0)


def test_assignment_is_one_to_one():
    costs = np.array([[1.0, 2.0], [1.1, 2.1]])
    pairs = optimal_assignment(costs)
    assert len(pairs) == 2
    assert len({r for r, _ in pairs}) == 2
    assert len({c for _, c in pairs}) == 2


def test_assignment_minimises_the_total_not_each_row():
    """Greedy would give row 0 column 0 and strand row 1 with the expensive one."""
    costs = np.array([[1.0, 2.0],
                      [1.0, 9.0]])
    # greedy by row: 1.0 + 9.0 = 10.0; optimal: 2.0 + 1.0 = 3.0
    pairs = optimal_assignment(costs)
    assert sum(costs[r, c] for r, c in pairs) == pytest.approx(3.0)
    assert dict(pairs) == {0: 1, 1: 0}


def test_two_ports_never_share_one_estimate():
    """The whole reason this module exists."""
    truths = [np.array([0.0, 0.0, 0.0]), np.array([0.02, 0.0, 0.0])]
    estimates = [np.array([0.01, 0.0, 0.0])]          # equidistant from both
    errors = match_errors(truths, estimates)
    assert sum(1 for e in errors if e is not None) == 1
    assert None in errors


def test_unmatched_port_reports_none_not_zero():
    errors = match_errors([np.zeros(3), np.ones(3)], [])
    assert errors == [None, None]


def test_each_port_gets_its_own_estimate_when_both_exist():
    truths = [np.array([0.0, 0.0, 0.0]), np.array([0.024, 0.0, 0.0])]
    estimates = [np.array([0.0235, 0.0, 0.0]), np.array([0.0005, 0.0, 0.0])]
    errors = match_errors(truths, estimates)
    assert all(e is not None for e in errors)
    assert errors[0] == pytest.approx(0.0005, abs=1e-9)
    assert errors[1] == pytest.approx(0.0005, abs=1e-9)


def test_swapped_estimate_order_gives_the_same_errors():
    """Estimate ids are arbitrary, so the result must not depend on their order."""
    truths = [np.array([0.0, 0.0, 0.0]), np.array([0.024, 0.0, 0.0])]
    a = [np.array([0.0005, 0.0, 0.0]), np.array([0.0235, 0.0, 0.0])]
    errors_forward = match_errors(truths, a)
    errors_reversed = match_errors(truths, list(reversed(a)))
    assert errors_forward == pytest.approx(errors_reversed)


def test_more_estimates_than_ports_uses_the_best_subset():
    truths = [np.array([0.0, 0.0, 0.0])]
    estimates = [np.array([5.0, 0.0, 0.0]), np.array([0.1, 0.0, 0.0])]
    assert match_errors(truths, estimates) == pytest.approx([0.1])


def test_assignment_handles_an_empty_matrix():
    assert optimal_assignment(np.zeros((0, 0))) == []
    assert optimal_assignment(np.zeros((2, 0))) == []


def test_assignment_rejects_a_non_matrix():
    with pytest.raises(ValueError, match="2-D"):
        optimal_assignment(np.array([1.0, 2.0]))


def test_assignment_refuses_an_oversized_problem():
    with pytest.raises(ValueError, match="refusing to brute-force"):
        optimal_assignment(np.zeros((9, 9)))


def test_assignment_matches_a_known_optimum_on_a_3x3():
    costs = np.array([[4.0, 1.0, 3.0],
                      [2.0, 0.0, 5.0],
                      [3.0, 2.0, 2.0]])
    pairs = optimal_assignment(costs)
    assert sum(costs[r, c] for r, c in pairs) == pytest.approx(5.0)
