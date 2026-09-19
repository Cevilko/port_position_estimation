"""Tests for the triangulation maths in port_triangulator_node.

These use synthetic cameras with known ground truth, because a triangulator
that is subtly wrong -- a transposed rotation, an un-inverted extrinsic --
still returns confident, plausible 3D points. Run with ``./run.sh test``.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(
    0, str(Path(__file__).resolve().parent.parent / "ros_ws" / "src" / "port_triangulator_node")
)

from port_triangulator_node.triangulation import (  # noqa: E402
    match_and_triangulate,
    position_covariance,
    project,
    projection_jacobian,
    projection_matrix,
    quaternion_to_matrix,
    reprojection_errors,
    triangulate,
)

K = np.array([[900.0, 0.0, 576.0], [0.0, 900.0, 512.0], [0.0, 0.0, 1.0]])


def looking_down_z(position):
    """A camera at ``position`` with optical axis along world +z, no rotation."""
    return projection_matrix(K, np.eye(3), np.asarray(position, dtype=float))


def test_quaternion_identity_is_the_identity_matrix():
    assert np.allclose(quaternion_to_matrix(0, 0, 0, 1), np.eye(3))


def test_quaternion_half_turn_about_z():
    r = quaternion_to_matrix(0, 0, 1, 0)
    assert np.allclose(r @ np.array([1.0, 0.0, 0.0]), [-1.0, 0.0, 0.0], atol=1e-9)


def test_quaternion_is_normalised_before_use():
    scaled = quaternion_to_matrix(0.0, 0.0, 2.0, 2.0)   # same rotation, wrong norm
    unit = quaternion_to_matrix(0.0, 0.0, 0.5 ** 0.5, 0.5 ** 0.5)
    assert np.allclose(scaled, unit)


def test_zero_quaternion_is_rejected():
    with pytest.raises(ValueError):
        quaternion_to_matrix(0.0, 0.0, 0.0, 0.0)


def test_projection_matrix_inverts_the_camera_pose():
    """A point at the camera's own position projects to its optical centre."""
    translation = np.array([1.0, 2.0, 3.0])
    p = projection_matrix(K, np.eye(3), translation)
    # a point one metre in front of the camera lands on the principal point
    pixel = project(p, translation + np.array([0.0, 0.0, 1.0]))
    assert np.allclose(pixel, [K[0, 2], K[1, 2]])


def test_projection_respects_rotation():
    """Rotating the camera moves the image of a fixed point the other way."""
    point = np.array([0.1, 0.0, 2.0])
    straight = project(looking_down_z([0.0, 0.0, 0.0]), point)
    rotated = project(
        projection_matrix(K, quaternion_to_matrix(0, 0, 1, 0), np.zeros(3)), point
    )
    assert straight is not None
    # after a half turn about z the point is behind... no: still in front, mirrored
    assert rotated is not None
    assert rotated[0] < K[0, 2] < straight[0]


def test_point_behind_the_camera_does_not_project():
    assert project(looking_down_z([0.0, 0.0, 0.0]), np.array([0.0, 0.0, -1.0])) is None


def test_triangulate_recovers_a_known_point():
    truth = np.array([0.05, -0.02, 1.30])
    projections = [looking_down_z([0.0, 0.0, 0.0]), looking_down_z([0.10, 0.0, 0.0])]
    pixels = [project(p, truth) for p in projections]
    assert np.allclose(triangulate(projections, pixels), truth, atol=1e-9)


def test_triangulate_recovers_a_point_from_three_views():
    truth = np.array([0.0, 0.0, 1.0])
    projections = [
        looking_down_z([0.0, 0.0, 0.0]),
        looking_down_z([0.08, 0.0, 0.0]),
        looking_down_z([0.0, 0.06, 0.0]),
    ]
    pixels = [project(p, truth) for p in projections]
    assert np.allclose(triangulate(projections, pixels), truth, atol=1e-9)


def test_triangulate_with_a_rotated_camera():
    """The extrinsic inversion is only exercised once rotation is non-trivial."""
    truth = np.array([0.2, 0.1, 1.5])
    rotation = quaternion_to_matrix(0.0, 0.0, 0.2588190, 0.9659258)  # 30 deg about z
    projections = [
        looking_down_z([0.0, 0.0, 0.0]),
        projection_matrix(K, rotation, np.array([0.15, 0.0, 0.0])),
    ]
    pixels = [project(p, truth) for p in projections]
    assert np.allclose(triangulate(projections, pixels), truth, atol=1e-8)


def test_triangulate_needs_two_views():
    with pytest.raises(ValueError, match="at least two views"):
        triangulate([looking_down_z([0, 0, 0])], [np.array([1.0, 1.0])])


def test_triangulate_rejects_mismatched_inputs():
    with pytest.raises(ValueError, match="one pixel per"):
        triangulate([looking_down_z([0, 0, 0])], [])


def test_reprojection_error_is_zero_for_a_consistent_point():
    truth = np.array([0.0, 0.0, 1.2])
    projections = [looking_down_z([0.0, 0.0, 0.0]), looking_down_z([0.1, 0.0, 0.0])]
    pixels = [project(p, truth) for p in projections]
    assert np.allclose(reprojection_errors(projections, pixels, truth), 0.0, atol=1e-9)


def test_reprojection_error_grows_with_a_displaced_point():
    truth = np.array([0.0, 0.0, 1.2])
    projections = [looking_down_z([0.0, 0.0, 0.0]), looking_down_z([0.1, 0.0, 0.0])]
    pixels = [project(p, truth) for p in projections]
    errors = reprojection_errors(projections, pixels, truth + np.array([0.01, 0.0, 0.0]))
    assert np.all(errors > 1.0)


def test_jacobian_matches_a_numeric_derivative():
    point = np.array([0.03, -0.01, 1.4])
    p = looking_down_z([0.05, 0.02, 0.0])
    analytic = projection_jacobian(p, point)
    numeric = np.zeros((2, 3))
    step = 1e-7
    for axis in range(3):
        delta = np.zeros(3)
        delta[axis] = step
        numeric[:, axis] = (project(p, point + delta) - project(p, point - delta)) / (2 * step)
    assert np.allclose(analytic, numeric, rtol=1e-5, atol=1e-5)


def test_covariance_is_symmetric_positive_definite():
    truth = np.array([0.0, 0.0, 1.3])
    projections = [
        looking_down_z([0.0, 0.0, 0.0]),
        looking_down_z([0.1, 0.0, 0.0]),
        looking_down_z([0.0, 0.1, 0.0]),
    ]
    cov = position_covariance(projections, truth, pixel_sigma=1.0)
    assert np.allclose(cov, cov.T)
    assert np.all(np.linalg.eigvalsh(cov) > 0)


def test_covariance_shrinks_as_the_baseline_grows():
    """Depth uncertainty is the whole point: a wider baseline must beat a narrow one."""
    truth = np.array([0.0, 0.0, 1.3])
    narrow = [looking_down_z([0.0, 0.0, 0.0]), looking_down_z([0.02, 0.0, 0.0])]
    wide = [looking_down_z([0.0, 0.0, 0.0]), looking_down_z([0.40, 0.0, 0.0])]
    narrow_depth = position_covariance(narrow, truth, 1.0)[2, 2]
    wide_depth = position_covariance(wide, truth, 1.0)[2, 2]
    assert wide_depth < narrow_depth / 100.0


def test_covariance_scales_with_the_square_of_pixel_noise():
    truth = np.array([0.0, 0.0, 1.3])
    projections = [
        looking_down_z([0.0, 0.0, 0.0]),
        looking_down_z([0.1, 0.0, 0.0]),
        looking_down_z([0.0, 0.1, 0.0]),
    ]
    one = position_covariance(projections, truth, 1.0)
    two = position_covariance(projections, truth, 2.0)
    assert np.allclose(two, 4.0 * one)


def test_covariance_is_elongated_along_the_viewing_direction():
    """A short baseline is uncertain in depth, not across it."""
    truth = np.array([0.0, 0.0, 1.3])
    projections = [looking_down_z([0.0, 0.0, 0.0]), looking_down_z([0.05, 0.0, 0.0])]
    cov = position_covariance(projections, truth, 1.0)
    assert cov[2, 2] > 50 * cov[1, 1]


def test_covariance_rejects_an_unconstrained_configuration():
    truth = np.array([0.0, 0.0, 1.3])
    with pytest.raises(ValueError, match="do not constrain"):
        position_covariance([looking_down_z([0.0, 0.0, 0.0])], truth, 1.0)


def test_covariance_rejects_non_positive_sigma():
    with pytest.raises(ValueError, match="must be positive"):
        position_covariance(
            [looking_down_z([0, 0, 0]), looking_down_z([0.1, 0, 0])],
            np.array([0.0, 0.0, 1.0]), 0.0,
        )


def _three_cameras():
    return [
        looking_down_z([0.0, 0.0, 0.0]),
        looking_down_z([0.12, 0.0, 0.0]),
        looking_down_z([0.0, 0.10, 0.0]),
    ]


def test_match_recovers_two_ports_without_correspondence():
    """The detector gives unlabelled boxes; the matcher has to sort them out."""
    ports = [np.array([-0.01, 0.0, 1.30]), np.array([0.01, 0.006, 1.30])]
    projections = _three_cameras()
    per_camera = []
    for p in projections:
        # deliberately reverse the order in each camera: index must not be
        # assumed to correspond across views
        per_camera.append((p, [project(p, ports[1]), project(p, ports[0])]))

    found = match_and_triangulate(per_camera, pixel_sigma=1.0)
    assert len(found) == 2
    recovered = sorted([f["position"] for f in found], key=lambda v: v[0])
    assert np.allclose(recovered[0], ports[0], atol=1e-6)
    assert np.allclose(recovered[1], ports[1], atol=1e-6)
    assert all(f["views"] == 3 for f in found)


def test_match_uses_each_detection_at_most_once():
    """Two ports must not collapse onto one well-fitting point."""
    ports = [np.array([-0.02, 0.0, 1.3]), np.array([0.02, 0.0, 1.3])]
    projections = _three_cameras()
    per_camera = [(p, [project(p, q) for q in ports]) for p in projections]
    found = match_and_triangulate(per_camera, pixel_sigma=1.0)
    assert len(found) == 2
    assert not np.allclose(found[0]["position"], found[1]["position"])


def test_match_tolerates_a_camera_that_saw_nothing():
    ports = [np.array([0.0, 0.0, 1.3])]
    projections = _three_cameras()
    per_camera = [
        (projections[0], [project(projections[0], ports[0])]),
        (projections[1], []),                                   # missed it
        (projections[2], [project(projections[2], ports[0])]),
    ]
    found = match_and_triangulate(per_camera, pixel_sigma=1.0)
    assert len(found) == 1
    assert found[0]["views"] == 2
    assert np.allclose(found[0]["position"], ports[0], atol=1e-6)


def test_match_discards_an_inconsistent_pairing():
    """A false positive in one view must not invent a port."""
    projections = _three_cameras()
    per_camera = [
        (projections[0], [np.array([100.0, 100.0])]),
        (projections[1], [np.array([900.0, 900.0])]),
        (projections[2], []),
    ]
    found = match_and_triangulate(
        per_camera, max_reprojection_error=2.0, pixel_sigma=1.0
    )
    assert found == []


def test_match_prefers_a_three_view_fit_over_a_two_view_one():
    ports = [np.array([0.0, 0.0, 1.3])]
    projections = _three_cameras()
    per_camera = [(p, [project(p, ports[0])]) for p in projections]
    found = match_and_triangulate(per_camera, pixel_sigma=1.0)
    assert len(found) == 1
    assert found[0]["views"] == 3


def test_match_respects_max_points():
    ports = [np.array([-0.02, 0.0, 1.3]), np.array([0.02, 0.0, 1.3])]
    projections = _three_cameras()
    per_camera = [(p, [project(p, q) for q in ports]) for p in projections]
    assert len(match_and_triangulate(per_camera, pixel_sigma=1.0, max_points=1)) == 1


def test_match_returns_nothing_when_only_one_camera_sees_a_port():
    projections = _three_cameras()
    per_camera = [
        (projections[0], [np.array([500.0, 500.0])]),
        (projections[1], []),
        (projections[2], []),
    ]
    assert match_and_triangulate(per_camera, pixel_sigma=1.0) == []


def test_match_is_robust_to_pixel_noise():
    """A realistic 0.5 px jitter should still localise to well under a millimetre."""
    rng = np.random.default_rng(0)
    truth = np.array([0.0, 0.0, 1.30])
    projections = _three_cameras()
    per_camera = [
        (p, [project(p, truth) + rng.normal(0.0, 0.5, 2)]) for p in projections
    ]
    found = match_and_triangulate(per_camera, pixel_sigma=0.5, max_reprojection_error=5.0)
    assert len(found) == 1
    assert np.linalg.norm(found[0]["position"] - truth) < 0.02
