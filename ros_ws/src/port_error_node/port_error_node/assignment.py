"""One-to-one matching of ground-truth ports to pose estimates.

The point of this module is the constraint: two ports must never be scored
against the *same* estimate. Taking each port's nearest estimate independently
does exactly that whenever one estimate sits between both ports, and the
resulting error looks flattering precisely when the estimator is doing badly --
a single good estimate would score twice and the missing one would never be
counted.

Pure numpy, no ROS, so the matching can be tested exhaustively.
"""

from __future__ import annotations

import itertools

import numpy as np

#: Brute force is exact and dependency-free, but factorial. Two ports is the
#: real case; this guards against someone pointing it at a hundred.
MAX_BRUTE_FORCE = 8


def distance_matrix(truths, estimates) -> np.ndarray:
    """Euclidean distance from every truth (row) to every estimate (column)."""
    truths = np.asarray(truths, dtype=float).reshape(len(truths), 3)
    if len(estimates) == 0:
        return np.zeros((len(truths), 0))
    estimates = np.asarray(estimates, dtype=float).reshape(len(estimates), 3)
    return np.linalg.norm(truths[:, None, :] - estimates[None, :, :], axis=2)


def optimal_assignment(costs):
    """Pair rows to columns one-to-one, minimising the total cost.

    Returns ``[(row, column), ...]`` of length ``min(rows, columns)``; every row
    and every column appears at most once. Minimising the *total* rather than
    letting each row grab its nearest is what makes the result independent of
    the order the rows happen to be in -- greedy matching gives a different
    answer depending on which port you process first.
    """
    costs = np.asarray(costs, dtype=float)
    if costs.ndim != 2:
        raise ValueError("costs must be a 2-D matrix")
    rows, columns = costs.shape
    if rows == 0 or columns == 0:
        return []
    if max(rows, columns) > MAX_BRUTE_FORCE:
        raise ValueError(
            f"refusing to brute-force a {rows}x{columns} assignment; "
            f"the limit is {MAX_BRUTE_FORCE}"
        )

    size = min(rows, columns)
    best_pairs, best_total = None, np.inf
    # Choose which rows take part (when there are more rows than columns), then
    # every injective mapping of those rows onto columns.
    for chosen_rows in itertools.combinations(range(rows), size):
        for chosen_columns in itertools.permutations(range(columns), size):
            total = sum(costs[r, c] for r, c in zip(chosen_rows, chosen_columns))
            if total < best_total:
                best_total = total
                best_pairs = list(zip(chosen_rows, chosen_columns))
    return best_pairs


def match_errors(truths, estimates):
    """Distance from each truth to its assigned estimate.

    Returns a list with one entry per truth, ``None`` where no estimate was
    available -- a port with nothing to pair against is reported as unmatched
    rather than silently borrowing the other port's estimate.
    """
    costs = distance_matrix(truths, estimates)
    errors = [None] * len(truths)
    for row, column in optimal_assignment(costs):
        errors[row] = float(costs[row, column])
    return errors
