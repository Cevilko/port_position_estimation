"""Multi-view triangulation of the SFP ports, and the uncertainty of the result.

Pure numpy: no ROS imports, so every claim below is testable without a graph.

Conventions, because getting either wrong silently produces plausible garbage:

* A transform from TF is the pose of the camera **in** the world, so it maps
  camera coordinates to world ones: ``X_world = R @ X_cam + t``. Projection
  needs the inverse, hence the transpose in :func:`projection_matrix`.
* The frames are ROS **optical** frames -- z forward, x right, y down -- which
  is the convention ``camera_info``'s K already assumes, so K applies to
  camera-frame points with no axis permutation.
"""

from __future__ import annotations

import itertools

import numpy as np


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Rotation matrix from a ROS quaternion (x, y, z, w -- scalar last)."""
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0:
        raise ValueError("zero-length quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def projection_matrix(k: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """3x4 world->pixel matrix from intrinsics and a camera-to-world pose."""
    k = np.asarray(k, dtype=float).reshape(3, 3)
    rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
    translation = np.asarray(translation, dtype=float).reshape(3)
    # world -> camera is the inverse of the camera's pose in the world
    world_to_camera = np.empty((3, 4))
    world_to_camera[:, :3] = rotation.T
    world_to_camera[:, 3] = -rotation.T @ translation
    return k @ world_to_camera


def project(projection: np.ndarray, point: np.ndarray):
    """World point -> pixel. Returns None for a point at or behind the camera."""
    homogeneous = np.append(np.asarray(point, dtype=float).reshape(3), 1.0)
    image = np.asarray(projection, dtype=float) @ homogeneous
    if image[2] <= 1e-9:
        return None
    return image[:2] / image[2]


def triangulate(projections, points) -> np.ndarray:
    """Least-squares 3D point from N>=2 views (the DLT).

    Each view contributes two rows saying "the projection of X lies on the ray
    through this pixel"; the solution is the null space of the stack.
    """
    projections = [np.asarray(p, dtype=float).reshape(3, 4) for p in projections]
    points = [np.asarray(q, dtype=float).reshape(2) for q in points]
    if len(projections) != len(points):
        raise ValueError("need one pixel per projection matrix")
    if len(projections) < 2:
        raise ValueError("triangulation needs at least two views")

    rows = []
    for projection, (u, v) in zip(projections, points):
        rows.append(u * projection[2] - projection[0])
        rows.append(v * projection[2] - projection[1])
    _, _, vt = np.linalg.svd(np.asarray(rows))
    solution = vt[-1]
    if abs(solution[3]) < 1e-12:
        raise ValueError("degenerate configuration: point at infinity")
    return solution[:3] / solution[3]


def reprojection_errors(projections, points, world_point) -> np.ndarray:
    """Per-view pixel distance between the observation and the reprojection."""
    errors = []
    for projection, observed in zip(projections, points):
        predicted = project(projection, world_point)
        if predicted is None:
            errors.append(np.inf)
        else:
            errors.append(float(np.linalg.norm(predicted - np.asarray(observed, dtype=float))))
    return np.asarray(errors)


def projection_jacobian(projection: np.ndarray, world_point: np.ndarray) -> np.ndarray:
    """d(pixel)/d(world point), 2x3, for the covariance propagation."""
    projection = np.asarray(projection, dtype=float).reshape(3, 4)
    homogeneous = np.append(np.asarray(world_point, dtype=float).reshape(3), 1.0)
    numerator_u, numerator_v, denominator = projection @ homogeneous
    if abs(denominator) < 1e-12:
        raise ValueError("point is on the camera plane")
    # d/dX of (p_i . X~ / p_3 . X~), quotient rule, keeping only the spatial part
    du = (projection[0, :3] * denominator - numerator_u * projection[2, :3]) / denominator ** 2
    dv = (projection[1, :3] * denominator - numerator_v * projection[2, :3]) / denominator ** 2
    return np.vstack([du, dv])


def position_covariance(projections, world_point, pixel_sigma: float) -> np.ndarray:
    """3x3 covariance of a triangulated point, from pixel noise.

    First-order propagation of independent, isotropic pixel noise through the
    least-squares fit: the Fisher information is ``sum(J^T J) / sigma^2`` and
    the covariance is its inverse. Geometry is what dominates the result -- a
    point seen from a narrow baseline has a long, thin ellipsoid along the
    viewing direction, and this reproduces that.
    """
    if pixel_sigma <= 0:
        raise ValueError("pixel_sigma must be positive")
    information = np.zeros((3, 3))
    for projection in projections:
        jacobian = projection_jacobian(projection, world_point)
        information += jacobian.T @ jacobian
    information /= pixel_sigma ** 2
    # A rank-deficient information matrix means the views do not constrain the
    # point in some direction (one view, or perfectly collinear centres).
    if np.linalg.matrix_rank(information, tol=1e-9) < 3:
        raise ValueError("views do not constrain the point in all three axes")
    return np.linalg.inv(information)


def _combinations(detection_counts, min_views: int):
    """Every way of picking at most one detection per camera, >= min_views used."""
    options = [list(range(count)) + [None] for count in detection_counts]
    for choice in itertools.product(*options):
        if sum(1 for index in choice if index is not None) >= min_views:
            yield choice


def match_and_triangulate(
    per_camera,
    max_reprojection_error: float = 5.0,
    min_views: int = 2,
    pixel_sigma: float = 1.5,
    max_points: int | None = None,
):
    """Resolve which detections belong to the same port, and triangulate them.

    ``per_camera`` is a list of ``(projection_matrix, [pixel, ...])``, one entry
    per camera. The detector emits an unlabelled ``sfp_port`` per box, so which
    box in one view corresponds to which in another is not given and has to be
    recovered: every consistent assignment is triangulated, scored by RMS
    reprojection error, and the cheapest non-conflicting ones are kept. Each
    detection is consumed at most once, so two ports cannot collapse onto one.

    Returns a list of dicts sorted by position, lowest error first within the
    greedy pass, each with ``position``, ``covariance``, ``rms_error``,
    ``views`` and ``cameras``.
    """
    projections = [projection for projection, _ in per_camera]
    detections = [points for _, points in per_camera]

    candidates = []
    for choice in _combinations([len(points) for points in detections], min_views):
        used_projections, used_points, cameras = [], [], []
        for camera_index, detection_index in enumerate(choice):
            if detection_index is None:
                continue
            used_projections.append(projections[camera_index])
            used_points.append(detections[camera_index][detection_index])
            cameras.append(camera_index)
        try:
            position = triangulate(used_projections, used_points)
        except (ValueError, np.linalg.LinAlgError):
            continue
        errors = reprojection_errors(used_projections, used_points, position)
        if not np.all(np.isfinite(errors)):
            continue
        rms = float(np.sqrt(np.mean(errors ** 2)))
        if rms > max_reprojection_error:
            continue
        candidates.append({
            "choice": choice,
            "position": position,
            "rms_error": rms,
            "views": len(cameras),
            "cameras": cameras,
            "projections": used_projections,
        })

    # Prefer more views, then lower error: a 3-view fit at 1.2 px is a better
    # explanation than a 2-view fit at 0.9 px, which can always be made to fit.
    candidates.sort(key=lambda item: (-item["views"], item["rms_error"]))

    claimed = set()
    accepted = []
    for candidate in candidates:
        if max_points is not None and len(accepted) >= max_points:
            break
        keys = {
            (camera_index, detection_index)
            for camera_index, detection_index in enumerate(candidate["choice"])
            if detection_index is not None
        }
        if keys & claimed:
            continue
        try:
            covariance = position_covariance(
                candidate["projections"], candidate["position"], pixel_sigma
            )
        except (ValueError, np.linalg.LinAlgError):
            continue
        claimed |= keys
        accepted.append({
            "position": candidate["position"],
            "covariance": covariance,
            "rms_error": candidate["rms_error"],
            "views": candidate["views"],
            "cameras": candidate["cameras"],
        })

    # Positional, deterministic ordering. It is NOT semantic: if the fixture
    # yaws far enough the two ports swap places and so do their ids.
    accepted.sort(key=lambda item: tuple(np.round(item["position"], 6)))
    return accepted
