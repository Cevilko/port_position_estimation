#!/usr/bin/python3
# port_position_estimation - Copyright (C) 2026 Cevilko <lvelicko03@gmail.com>
# SPDX-License-Identifier: AGPL-3.0-only
"""Extract every recorded frame's images, TFs, and CameraInfo from a ROS 2 bag.

``bag_recorder_node`` writes all of a trigger's topics under one identical bag
timestamp, so that timestamp groups the messages belonging to a single frame.
Each complete group becomes ``frames/<index>/`` holding the three camera images
and the world transforms resolved from that frame's own ``/tf``. CameraInfo is
written once per camera at the bag root, since intrinsics do not change.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from geometry_msgs.msg import TransformStamped
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.convert import message_to_yaml
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage


IMAGE_TOPICS = (
    "/center_camera/image",
    "/left_camera/image",
    "/right_camera/image",
)

CAMERA_INFO_TOPICS = (
    "/center_camera/camera_info",
    "/left_camera/camera_info",
    "/right_camera/camera_info",
)

TF_FRAME_ALIASES = {
    "center_camera": ("center_camera", "center_camera_optical"),
    "left_camera": ("left_camera", "left_camera_optical"),
    "right_camera": ("right_camera", "right_camera_optical"),
    "sfp_port_0_entrance": ("sfp_port_0_entrance",),
    "sfp_port_1_entrance": ("sfp_port_1_entrance",),
}

#: The topics that together make up one recorded frame. ``bag_recorder_node``
#: writes all of them with an identical bag timestamp per ``/record_rosbag``
#: trigger, so that timestamp is what groups them.
FRAME_TOPICS = IMAGE_TOPICS + ("/tf",)


#: Repo root, so emitted paths can be written relative to it.
REPO_ROOT = Path(__file__).resolve().parent.parent


def repo_relative(path: Path) -> str:
    """Render ``path`` relative to the repo root when it lives inside it.

    The manifest and the per-sample sidecars are committed, and a consumer may
    be on a different machine or a different checkout, so an absolute
    ``/home/<user>/...`` in them is wrong the moment the repo moves. Anything
    genuinely outside the repo keeps its absolute path -- that is information,
    not noise.
    """

    resolved = Path(path).resolve()
    try:
        return str(resolved.relative_to(REPO_ROOT))
    except ValueError:
        return str(resolved)


def topic_label(topic: str) -> str:
    return topic.strip("/").replace("/", "_")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def image_to_bgr(msg: Image) -> np.ndarray:
    encoding = msg.encoding.lower()
    channels_by_encoding = {
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
        "mono8": 1,
        "8uc1": 1,
    }
    if encoding not in channels_by_encoding:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    channels = channels_by_encoding[encoding]
    row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    useful = row[:, : msg.width * channels]

    if channels == 1:
        return useful.reshape(msg.height, msg.width)

    image = useful.reshape(msg.height, msg.width, channels)
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image


def quat_multiply(a: tuple[float, float, float, float], b: tuple[float, float, float, float]):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_rotate(q: tuple[float, float, float, float], v: tuple[float, float, float]):
    x, y, z, w = q
    vx, vy, vz = v
    # q * [v, 0] * conjugate(q), expanded to avoid extra dependencies.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def normalize_quat(q: tuple[float, float, float, float]):
    norm = math.sqrt(sum(component * component for component in q))
    if norm == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(component / norm for component in q)


def tf_parts(transform: TransformStamped):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return (
        (translation.x, translation.y, translation.z),
        (rotation.x, rotation.y, rotation.z, rotation.w),
    )


def resolve_world_transform(
    child_frame: str,
    transforms_by_child: dict[str, TransformStamped],
    world_frame: str = "world",
):
    def resolve(frame: str, stack: tuple[str, ...] = ()):
        if frame == world_frame:
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), None
        if frame in stack or frame not in transforms_by_child:
            return None

        transform = transforms_by_child[frame]
        parent = transform.header.frame_id.lstrip("/")
        parent_result = resolve(parent, stack + (frame,))
        if parent_result is None:
            return None

        parent_t, parent_q, _ = parent_result
        local_t, local_q = tf_parts(transform)
        rotated_t = quat_rotate(parent_q, local_t)
        world_t = tuple(parent_t[i] + rotated_t[i] for i in range(3))
        world_q = normalize_quat(quat_multiply(parent_q, local_q))
        return world_t, world_q, transform

    result = resolve(child_frame.lstrip("/"))
    if result is None:
        return None

    translation, rotation, source_transform = result
    output = TransformStamped()
    if source_transform is not None:
        output.header.stamp = source_transform.header.stamp
    output.header.frame_id = world_frame
    output.child_frame_id = child_frame
    output.transform.translation.x = translation[0]
    output.transform.translation.y = translation[1]
    output.transform.translation.z = translation[2]
    output.transform.rotation.x = rotation[0]
    output.transform.rotation.y = rotation[1]
    output.transform.rotation.z = rotation[2]
    output.transform.rotation.w = rotation[3]
    return output


def find_tf_sample(msg: TFMessage):
    transforms_by_child = {
        transform.child_frame_id.lstrip("/"): transform for transform in msg.transforms
    }
    matches: dict[str, str] = {}
    world_transforms: dict[str, TransformStamped] = {}

    for requested_label, aliases in TF_FRAME_ALIASES.items():
        actual_frame = next(
            (alias for alias in aliases if alias.lstrip("/") in transforms_by_child),
            None,
        )
        if actual_frame is None:
            return None

        world_transform = resolve_world_transform(
            actual_frame.lstrip("/"), transforms_by_child
        )
        if world_transform is None:
            return None
        matches[requested_label] = actual_frame.lstrip("/")
        world_transforms[requested_label] = world_transform

    return matches, world_transforms


def open_reader(bag_path: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def frame_dir_name(index: int) -> str:
    return f"{index:05d}"


def group_frames(records, required_topics=FRAME_TOPICS):
    """Yield ``(bag_timestamp_ns, {topic: data})`` once a group is complete.

    Every topic recorded for one trigger shares a bag timestamp, so that is the
    grouping key. Groups are emitted as soon as they fill and dropped from the
    buffer immediately, so a bag of thousands of frames costs roughly one frame
    of memory rather than all of them. Topics outside ``required_topics`` are
    ignored, and groups that never fill are simply never yielded.
    """
    pending: dict[int, dict[str, bytes]] = {}
    for topic, data, bag_timestamp_ns in records:
        if topic not in required_topics:
            continue
        group = pending.setdefault(bag_timestamp_ns, {})
        group[topic] = data
        if all(required in group for required in required_topics):
            yield bag_timestamp_ns, pending.pop(bag_timestamp_ns)


def _read_records(reader, on_camera_info):
    """Yield frame records, diverting CameraInfo messages to ``on_camera_info``."""
    while reader.has_next():
        topic, data, bag_timestamp_ns = reader.read_next()
        if topic in CAMERA_INFO_TOPICS:
            on_camera_info(topic, data)
            continue
        yield topic, data, bag_timestamp_ns


def _write_frame(bag_output: Path, index: int, bag_timestamp_ns: int, group, bag_path: Path):
    """Write one frame's images and world transforms.

    Returns the frame's output directory, or ``None`` if its ``/tf`` message
    did not contain every frame in TF_FRAME_ALIASES, in which case nothing is
    written for it.
    """
    tf_sample = find_tf_sample(deserialize_message(group["/tf"], TFMessage))
    if tf_sample is None:
        return None
    tf_matches, world_transforms = tf_sample

    frame_dir = bag_output / "frames" / frame_dir_name(index)
    frame_lines = [
        f"index: {index}",
        f"bag_timestamp_ns: {bag_timestamp_ns}",
        f"source_bag: {repo_relative(bag_path)}",
        "images:",
    ]

    for topic in IMAGE_TOPICS:
        msg = deserialize_message(group[topic], Image)
        label = topic_label(topic)
        image_path = frame_dir / "images" / f"{label}.jpg"
        image_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(image_path), image_to_bgr(msg)):
            raise RuntimeError(f"Failed to write image: {image_path}")
        metadata_path = image_path.with_suffix(".yaml")
        write_text(
            metadata_path,
            f"source_bag: {repo_relative(bag_path)}\n"
            f"source_topic: {topic}\n"
            f"bag_timestamp_ns: {bag_timestamp_ns}\n"
            f"header:\n"
            f"  stamp:\n"
            f"    sec: {msg.header.stamp.sec}\n"
            f"    nanosec: {msg.header.stamp.nanosec}\n"
            f"  frame_id: {msg.header.frame_id}\n"
            f"height: {msg.height}\n"
            f"width: {msg.width}\n"
            f"encoding: {msg.encoding}\n"
            f"is_bigendian: {msg.is_bigendian}\n"
            f"step: {msg.step}\n",
        )
        frame_lines.extend(
            [
                f"  - topic: {topic}",
                f"    file: {repo_relative(image_path)}",
                f"    metadata: {repo_relative(metadata_path)}",
            ]
        )

    frame_lines.extend(["tf_world_transforms:", "  source_topic: /tf", "  transforms:"])
    for requested_label, transform in world_transforms.items():
        path = frame_dir / "tf_world_transforms" / f"world_to_{requested_label}.yaml"
        write_text(path, message_to_yaml(transform))
        frame_lines.extend(
            [
                f"    - requested_frame: {requested_label}",
                f"      recorded_child_frame_id: {tf_matches[requested_label]}",
                f"      file: {repo_relative(path)}",
            ]
        )

    write_text(frame_dir / "frame.yaml", "\n".join(frame_lines) + "\n")
    return frame_dir


def extract_samples(bag_path: Path, output_root: Path) -> None:
    bag_path = bag_path.resolve()
    # Opened before the output directory is created, so a bag that cannot be
    # read does not leave an empty directory behind.
    reader = open_reader(bag_path)

    bag_output = output_root / bag_path.name
    bag_output.mkdir(parents=True, exist_ok=True)

    camera_info_outputs: dict[str, Path] = {}

    def capture_camera_info(topic: str, data: bytes) -> None:
        # Intrinsics are static, so the first message per camera stands for the
        # whole bag rather than being rewritten for every frame.
        if topic in camera_info_outputs:
            return
        path = bag_output / "camera_info" / f"{topic_label(topic)}.yaml"
        write_text(path, message_to_yaml(deserialize_message(data, CameraInfo)))
        camera_info_outputs[topic] = path

    frames: list[tuple[int, int, Path]] = []
    skipped = 0

    for bag_timestamp_ns, group in group_frames(_read_records(reader, capture_camera_info)):
        index = len(frames)
        frame_dir = _write_frame(bag_output, index, bag_timestamp_ns, group, bag_path)
        if frame_dir is None:
            skipped += 1
            continue
        frames.append((index, bag_timestamp_ns, frame_dir))

    if not frames:
        raise RuntimeError(
            f"Extraction produced no frames: no bag timestamp carried all of "
            f"{', '.join(FRAME_TOPICS)} with a resolvable TF tree "
            f"(skipped {skipped} group(s) whose TF could not be resolved)"
        )

    missing_camera_info = sorted(set(CAMERA_INFO_TOPICS) - set(camera_info_outputs))

    manifest_lines = [
        f"source_bag: {repo_relative(bag_path)}",
        f"output_directory: {repo_relative(bag_output)}",
        f"frame_count: {len(frames)}",
        f"skipped_groups: {skipped}",
        "camera_info:",
    ]
    for topic in CAMERA_INFO_TOPICS:
        if topic in camera_info_outputs:
            manifest_lines.extend(
                [
                    f"  - topic: {topic}",
                    f"    file: {repo_relative(camera_info_outputs[topic])}",
                ]
            )
    manifest_lines.append("frames:")
    for index, bag_timestamp_ns, frame_dir in frames:
        manifest_lines.extend(
            [
                f"  - index: {index}",
                f"    bag_timestamp_ns: {bag_timestamp_ns}",
                f"    directory: {repo_relative(frame_dir)}",
                f"    frame: {repo_relative(frame_dir / 'frame.yaml')}",
            ]
        )
    write_text(bag_output / "MANIFEST.yaml", "\n".join(manifest_lines) + "\n")

    print(f"extracted {len(frames)} frame(s) to {repo_relative(bag_output)}")
    if skipped:
        print(f"skipped {skipped} group(s) whose TF tree could not be resolved")
    if missing_camera_info:
        print(f"warning: no CameraInfo recorded for {', '.join(missing_camera_info)}")


def latest_bag(rosbags_root: Path) -> Path | None:
    """Return the most recently written bag directory under ``rosbags_root``.

    A bag directory is one holding a ``metadata.yaml``. Recorded bags carry a
    timestamped name, so there is no fixed path to fall back on.
    """
    candidates = [
        path
        for path in rosbags_root.iterdir()
        if path.is_dir() and (path / "metadata.yaml").is_file()
    ] if rosbags_root.is_dir() else []
    if not candidates:
        return None
    return max(candidates, key=lambda path: (path / "metadata.yaml").stat().st_mtime)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "bag",
        nargs="?",
        help="bag directory to extract; defaults to the newest one in rosbags/",
    )
    parser.add_argument("--output-root", default="rosbag_samples")
    args = parser.parse_args()

    if args.bag:
        bag = Path(args.bag)
    else:
        bag = latest_bag(Path("rosbags"))
        if bag is None:
            parser.error("no bag found in rosbags/; pass one explicitly")
        print(f"using newest bag: {bag}")

    extract_samples(bag, Path(args.output_root))


if __name__ == "__main__":
    main()
