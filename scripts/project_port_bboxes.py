#!/usr/bin/python3
# port_position_estimation - Copyright (C) 2026 Cevilko <lvelicko03@gmail.com>
# SPDX-License-Identifier: AGPL-3.0-only
"""Project the SFP port entrances into an image as 2D bounding boxes.

The entrance prims are bare Xforms -- they carry a pose but no geometry -- so
the box comes from a rectangle of the port aperture's physical size placed in
the entrance frame, not from anything measurable in the USD. In that frame,
local +y is the outward normal (it points back at the camera) and local x is
the axis separating the two ports, so the aperture lies in the local x/z plane.

Everything else is the standard pinhole projection: the corners go from the
entrance frame to world using that frame's recorded transform, then into the
camera's optical frame by inverting the camera's transform, then through K.
The 2D box is the axis-aligned extent of the projected corners, which is tight
for a fronto-parallel port and a slight over-estimate as it turns away.

Inputs are the files ``extract_rosbag_samples.py`` writes, so a frame directory
is all it needs::

    ./run.sh bbox rosbag_samples/<bag>/frames/00000 --annotate out.jpg
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import yaml


#: Aperture of one SFP port opening, in metres (width along the entrance
#: frame's local x, height along local z). The entrance prims hold no geometry,
#: so this was measured off the rendered openings instead: across the frames
#: where a port is unoccluded it comes out at 12.20 +/- 0.35 by 7.14 +/- 0.28
#: mm, tighter than the 13.4 x 8.5 mm nominal SFP cage mouth.
DEFAULT_APERTURE_WIDTH = 0.0122
DEFAULT_APERTURE_HEIGHT = 0.00715

PORT_LABELS = ("sfp_port_0_entrance", "sfp_port_1_entrance")


def quat_to_matrix(q) -> np.ndarray:
    """Rotation matrix for a quaternion given as (x, y, z, w)."""
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def transform_matrix(translation, rotation) -> np.ndarray:
    """4x4 homogeneous transform from a translation and an (x, y, z, w) quat."""
    matrix = np.eye(4)
    matrix[:3, :3] = quat_to_matrix(rotation)
    matrix[:3, 3] = translation
    return matrix


def invert_transform(matrix: np.ndarray) -> np.ndarray:
    """Inverse of a rigid transform, without a general matrix inverse."""
    rotation = matrix[:3, :3]
    inverse = np.eye(4)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ matrix[:3, 3]
    return inverse


def aperture_corners(
    width: float, height: float, offset_x: float = 0.0, offset_z: float = 0.0
) -> np.ndarray:
    """The four aperture corners in the entrance frame (local x by local z).

    ``offset_x``/``offset_z`` shift the rectangle within that plane. They exist
    because image and TF in a recorded frame are not guaranteed to be the same
    instant, which moves both ports together by a few millimetres; they are not
    a property of the port and default to no shift.
    """
    half_w, half_h = width / 2.0, height / 2.0
    return np.array(
        [
            [offset_x - half_w, 0.0, offset_z - half_h],
            [offset_x + half_w, 0.0, offset_z - half_h],
            [offset_x + half_w, 0.0, offset_z + half_h],
            [offset_x - half_w, 0.0, offset_z + half_h],
        ]
    )


def project_points(points_camera: np.ndarray, intrinsics: np.ndarray) -> np.ndarray:
    """Project camera-frame points to pixels under the ROS optical convention.

    Points at or behind the image plane have no projection and are dropped, so
    the result can be shorter than the input (empty if the port is behind the
    camera).
    """
    in_front = points_camera[:, 2] > 1e-9
    visible = points_camera[in_front]
    if len(visible) == 0:
        return np.empty((0, 2))
    fx, fy = intrinsics[0, 0], intrinsics[1, 1]
    cx, cy = intrinsics[0, 2], intrinsics[1, 2]
    u = fx * visible[:, 0] / visible[:, 2] + cx
    v = fy * visible[:, 1] / visible[:, 2] + cy
    return np.column_stack([u, v])


def bbox_from_points(pixels: np.ndarray, image_size=None):
    """Axis-aligned (x_min, y_min, x_max, y_max) around ``pixels``.

    With ``image_size`` as (width, height) the box is clipped to the image and
    returned only if some of it is actually inside the frame.
    """
    if len(pixels) == 0:
        return None
    x_min, y_min = pixels.min(axis=0)
    x_max, y_max = pixels.max(axis=0)
    if image_size is not None:
        width, height = image_size
        if x_max < 0 or y_max < 0 or x_min > width - 1 or y_min > height - 1:
            return None
        x_min, x_max = max(0.0, x_min), min(float(width - 1), x_max)
        y_min, y_max = max(0.0, y_min), min(float(height - 1), y_max)
    return (float(x_min), float(y_min), float(x_max), float(y_max))


def load_yaml(path: Path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def load_intrinsics(camera_info_path: Path):
    """Return (K, (width, height)) from a CameraInfo yaml."""
    info = load_yaml(camera_info_path)
    return np.array(info["k"], dtype=float).reshape(3, 3), (
        int(info["width"]),
        int(info["height"]),
    )


def load_transform(transform_path: Path) -> np.ndarray:
    """Return the 4x4 transform from a TransformStamped yaml."""
    message = load_yaml(transform_path)["transform"]
    translation = message["translation"]
    rotation = message["rotation"]
    return transform_matrix(
        [translation["x"], translation["y"], translation["z"]],
        [rotation["x"], rotation["y"], rotation["z"], rotation["w"]],
    )


def port_bbox(world_to_camera, world_to_port, intrinsics, image_size, width, height,
              offset_x: float = 0.0, offset_z: float = 0.0):
    """Bounding box for one port, or None when it does not land in the image."""
    corners_local = aperture_corners(width, height, offset_x, offset_z)
    homogeneous = np.column_stack([corners_local, np.ones(len(corners_local))])
    corners_world = (world_to_port @ homogeneous.T).T
    corners_camera = (invert_transform(world_to_camera) @ corners_world.T).T[:, :3]
    return bbox_from_points(project_points(corners_camera, intrinsics), image_size)


def annotate(image_path: Path, boxes: dict, output_path: Path) -> None:
    import cv2

    image = cv2.imread(str(image_path))
    if image is None:
        raise RuntimeError(f"Could not read image: {image_path}")
    colours = {PORT_LABELS[0]: (0, 220, 0), PORT_LABELS[1]: (0, 160, 255)}
    for label, box in boxes.items():
        if box is None:
            continue
        x_min, y_min, x_max, y_max = (int(round(v)) for v in box)
        colour = colours.get(label, (255, 255, 255))
        cv2.rectangle(image, (x_min, y_min), (x_max, y_max), colour, 2)
        cv2.putText(
            image,
            label.replace("_entrance", ""),
            (x_min, max(0, y_min - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            colour,
            1,
            cv2.LINE_AA,
        )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_path), image):
        raise RuntimeError(f"Could not write image: {output_path}")


def bboxes_for_frame(frame_dir: Path, camera: str, width: float, height: float,
                     offset_x: float = 0.0, offset_z: float = 0.0):
    """Compute both port boxes for an extracted frame directory."""
    frame_dir = Path(frame_dir)
    camera_info_path = (
        frame_dir.parent.parent / "camera_info" / f"{camera}_camera_info.yaml"
    )
    intrinsics, image_size = load_intrinsics(camera_info_path)
    world_to_camera = load_transform(
        frame_dir / "tf_world_transforms" / f"world_to_{camera}.yaml"
    )

    boxes = {}
    for label in PORT_LABELS:
        world_to_port = load_transform(
            frame_dir / "tf_world_transforms" / f"world_to_{label}.yaml"
        )
        boxes[label] = port_bbox(
            world_to_camera, world_to_port, intrinsics, image_size, width, height,
            offset_x, offset_z,
        )
    return boxes, image_size


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frame", help="extracted frame directory, e.g. .../frames/00000")
    parser.add_argument("--camera", default="center_camera", help="which camera to project into")
    parser.add_argument("--width", type=float, default=DEFAULT_APERTURE_WIDTH,
                        help="port aperture width in metres (entrance frame local x)")
    parser.add_argument("--height", type=float, default=DEFAULT_APERTURE_HEIGHT,
                        help="port aperture height in metres (entrance frame local z)")
    parser.add_argument("--offset-x", type=float, default=0.0,
                        help="shift the aperture along the entrance frame's local x, in metres")
    parser.add_argument("--offset-z", type=float, default=0.0,
                        help="shift the aperture along the entrance frame's local z, in metres")
    parser.add_argument("--annotate", help="also write a copy of the image with the boxes drawn")
    parser.add_argument("--json", action="store_true", help="print JSON instead of a table")
    args = parser.parse_args()

    frame_dir = Path(args.frame)
    boxes, image_size = bboxes_for_frame(
        frame_dir, args.camera, args.width, args.height, args.offset_x, args.offset_z
    )

    if args.json:
        print(json.dumps({"frame": str(frame_dir), "camera": args.camera,
                          "image_size": image_size, "boxes": boxes}, indent=2))
    else:
        print(f"frame:  {frame_dir}")
        print(f"camera: {args.camera}  image {image_size[0]}x{image_size[1]}")
        for label, box in boxes.items():
            if box is None:
                print(f"  {label}: not visible")
            else:
                x_min, y_min, x_max, y_max = box
                print(
                    f"  {label}: x[{x_min:7.1f} {x_max:7.1f}] y[{y_min:7.1f} {y_max:7.1f}]"
                    f"  {x_max - x_min:5.1f}x{y_max - y_min:5.1f} px"
                )

    if args.annotate:
        image_path = frame_dir / "images" / f"{args.camera}_image.jpg"
        annotate(image_path, boxes, Path(args.annotate))
        print(f"annotated: {args.annotate}")


if __name__ == "__main__":
    main()
