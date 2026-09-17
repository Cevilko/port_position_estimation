"""Tests for the pure functions in ``scripts/project_port_bboxes.py``.

The projection maths is the part that can be checked without a bag or a
simulator: the quaternion-to-matrix conversion, the rigid inverse, the aperture
rectangle, the pinhole projection and the 2D box. Run with ``./run.sh test``.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

import project_port_bboxes as bbox  # noqa: E402


IDENTITY_QUAT = (0.0, 0.0, 0.0, 1.0)
ROT_Z_90 = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))

# fx = fy = 100, principal point at (320, 240)
K = np.array([[100.0, 0.0, 320.0], [0.0, 100.0, 240.0], [0.0, 0.0, 1.0]])


def test_quat_to_matrix_identity():
    assert bbox.quat_to_matrix(IDENTITY_QUAT) == pytest.approx(np.eye(3))


def test_quat_to_matrix_yaw_sends_x_to_y():
    rotated = bbox.quat_to_matrix(ROT_Z_90) @ np.array([1.0, 0.0, 0.0])
    assert rotated == pytest.approx([0.0, 1.0, 0.0], abs=1e-12)


def test_quat_to_matrix_is_orthonormal():
    matrix = bbox.quat_to_matrix((0.5, 0.5, 0.5, 0.5))
    assert matrix @ matrix.T == pytest.approx(np.eye(3), abs=1e-12)


def test_invert_transform_round_trips():
    original = bbox.transform_matrix([1.0, -2.0, 3.0], ROT_Z_90)
    assert original @ bbox.invert_transform(original) == pytest.approx(np.eye(4), abs=1e-12)


def test_invert_transform_matches_a_general_inverse():
    original = bbox.transform_matrix([0.3, 0.4, 0.5], (0.5, 0.5, 0.5, 0.5))
    assert bbox.invert_transform(original) == pytest.approx(np.linalg.inv(original))


def test_aperture_corners_span_the_requested_size():
    corners = bbox.aperture_corners(0.012, 0.008)
    assert corners[:, 0].max() - corners[:, 0].min() == pytest.approx(0.012)
    assert corners[:, 2].max() - corners[:, 2].min() == pytest.approx(0.008)


def test_aperture_corners_lie_in_the_local_xz_plane():
    """Local y is the outward normal, so the aperture must be flat in y."""

    assert bbox.aperture_corners(0.012, 0.008)[:, 1] == pytest.approx(0.0)


def test_aperture_corners_are_centred_by_default():
    corners = bbox.aperture_corners(0.012, 0.008)
    assert corners.mean(axis=0) == pytest.approx([0.0, 0.0, 0.0])


def test_aperture_corners_shift_by_the_offset():
    corners = bbox.aperture_corners(0.012, 0.008, offset_x=0.002, offset_z=-0.003)
    assert corners.mean(axis=0) == pytest.approx([0.002, 0.0, -0.003])


def test_project_points_puts_the_optical_axis_at_the_principal_point():
    pixels = bbox.project_points(np.array([[0.0, 0.0, 2.0]]), K)
    assert pixels[0] == pytest.approx([320.0, 240.0])


def test_project_points_scales_with_depth():
    near = bbox.project_points(np.array([[0.1, 0.0, 1.0]]), K)[0]
    far = bbox.project_points(np.array([[0.1, 0.0, 2.0]]), K)[0]
    assert near[0] - 320.0 == pytest.approx(2 * (far[0] - 320.0))


def test_project_points_drops_points_behind_the_camera():
    """A port behind the camera would otherwise project to a mirrored pixel."""

    points = np.array([[0.0, 0.0, -1.0], [0.0, 0.0, 1.0]])
    assert len(bbox.project_points(points, K)) == 1


def test_project_points_returns_empty_when_nothing_is_in_front():
    assert len(bbox.project_points(np.array([[0.0, 0.0, -1.0]]), K)) == 0


def test_bbox_from_points_is_the_axis_aligned_extent():
    pixels = np.array([[10.0, 20.0], [30.0, 5.0], [15.0, 25.0]])
    assert bbox.bbox_from_points(pixels) == (10.0, 5.0, 30.0, 25.0)


def test_bbox_from_points_returns_none_without_points():
    assert bbox.bbox_from_points(np.empty((0, 2))) is None


def test_bbox_from_points_clips_to_the_image():
    pixels = np.array([[-10.0, -10.0], [700.0, 700.0]])
    assert bbox.bbox_from_points(pixels, image_size=(640, 480)) == (0.0, 0.0, 639.0, 479.0)


def test_bbox_from_points_rejects_a_box_entirely_outside_the_image():
    pixels = np.array([[700.0, 700.0], [800.0, 800.0]])
    assert bbox.bbox_from_points(pixels, image_size=(640, 480)) is None


def test_port_bbox_centres_on_the_principal_point_when_port_faces_the_camera():
    """A port straight ahead, normal pointing back at the camera, is centred."""

    # Camera at the origin looking down optical +z; port 0.5 m ahead with its
    # local +y (the outward normal) pointing back at the camera.
    world_to_camera = np.eye(4)
    world_to_port = bbox.transform_matrix(
        [0.0, 0.0, 0.5], (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
    )
    box = bbox.port_bbox(world_to_camera, world_to_port, K, (640, 480), 0.012, 0.008)

    assert (box[0] + box[2]) / 2 == pytest.approx(320.0)
    assert (box[1] + box[3]) / 2 == pytest.approx(240.0)


def test_port_bbox_grows_as_the_camera_gets_closer():
    world_to_camera = np.eye(4)
    upright = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
    near = bbox.port_bbox(world_to_camera, bbox.transform_matrix([0, 0, 0.25], upright),
                          K, (640, 480), 0.012, 0.008)
    far = bbox.port_bbox(world_to_camera, bbox.transform_matrix([0, 0, 0.5], upright),
                         K, (640, 480), 0.012, 0.008)
    assert (near[2] - near[0]) == pytest.approx(2 * (far[2] - far[0]))


def test_port_bbox_is_none_when_the_port_is_behind_the_camera():
    world_to_camera = np.eye(4)
    upright = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
    behind = bbox.transform_matrix([0.0, 0.0, -0.5], upright)
    assert bbox.port_bbox(world_to_camera, behind, K, (640, 480), 0.012, 0.008) is None
