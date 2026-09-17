#!/usr/bin/python3
"""Extract representative images, TFs, and CameraInfo from a ROS 2 bag."""

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


def extract_samples(bag_path: Path, output_root: Path) -> None:
    bag_path = bag_path.resolve()
    bag_output = output_root / bag_path.name
    bag_output.mkdir(parents=True, exist_ok=True)

    reader = open_reader(bag_path)
    image_outputs: dict[str, Path] = {}
    image_metadata: dict[str, str] = {}
    camera_info_output: Path | None = None
    camera_info_topic: str | None = None
    tf_outputs: dict[str, Path] = {}
    tf_matches: dict[str, str] = {}
    tf_bag_timestamp_ns: int | None = None

    while reader.has_next():
        topic, data, bag_timestamp_ns = reader.read_next()

        if topic in IMAGE_TOPICS and topic not in image_outputs:
            msg = deserialize_message(data, Image)
            image = image_to_bgr(msg)
            label = topic_label(topic)
            image_path = bag_output / "images" / label / "sample.jpg"
            image_path.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(image_path), image):
                raise RuntimeError(f"Failed to write image: {image_path}")
            image_outputs[topic] = image_path
            image_metadata[topic] = (
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
                f"step: {msg.step}\n"
            )
            write_text(image_path.with_suffix(".yaml"), image_metadata[topic])

        elif topic in CAMERA_INFO_TOPICS and camera_info_output is None:
            # The preferred topic order puts center_camera first; only accept
            # later topics if center_camera has not appeared by the end.
            if topic != CAMERA_INFO_TOPICS[0]:
                continue
            msg = deserialize_message(data, CameraInfo)
            camera_info_topic = topic
            camera_info_output = bag_output / "camera_info" / f"{topic_label(topic)}.yaml"
            write_text(camera_info_output, message_to_yaml(msg))

        elif topic == "/tf" and not tf_outputs:
            msg = deserialize_message(data, TFMessage)
            tf_sample = find_tf_sample(msg)
            if tf_sample is not None:
                tf_matches, world_transforms = tf_sample
                tf_bag_timestamp_ns = bag_timestamp_ns
                tf_dir = bag_output / "tf_world_transforms"
                for requested_label, transform in world_transforms.items():
                    path = tf_dir / f"world_to_{requested_label}.yaml"
                    write_text(path, message_to_yaml(transform))
                    tf_outputs[requested_label] = path

        if (
            len(image_outputs) == len(IMAGE_TOPICS)
            and camera_info_output is not None
            and len(tf_outputs) == len(TF_FRAME_ALIASES)
        ):
            break

    missing_images = sorted(set(IMAGE_TOPICS) - set(image_outputs))
    missing_tf = sorted(set(TF_FRAME_ALIASES) - set(tf_outputs))
    if missing_images or missing_tf or camera_info_output is None:
        raise RuntimeError(
            "Extraction incomplete: "
            f"missing_images={missing_images}, "
            f"missing_camera_info={camera_info_output is None}, "
            f"missing_tf={missing_tf}"
        )

    manifest_lines = [
        f"source_bag: {repo_relative(bag_path)}",
        f"output_directory: {repo_relative(bag_output)}",
        "images:",
    ]
    for topic in IMAGE_TOPICS:
        manifest_lines.extend(
            [
                f"  - topic: {topic}",
                f"    file: {repo_relative(image_outputs[topic])}",
                f"    metadata: {repo_relative(image_outputs[topic].with_suffix('.yaml'))}",
            ]
        )
    manifest_lines.extend(
        [
            "camera_info:",
            f"  topic: {camera_info_topic}",
            f"  file: {repo_relative(camera_info_output)}",
            "tf_world_transforms:",
            f"  source_topic: /tf",
            f"  bag_timestamp_ns: {tf_bag_timestamp_ns}",
            "  transforms:",
        ]
    )
    for requested_label in TF_FRAME_ALIASES:
        manifest_lines.extend(
            [
                f"    - requested_frame: {requested_label}",
                f"      recorded_child_frame_id: {tf_matches[requested_label]}",
                f"      file: {repo_relative(tf_outputs[requested_label])}",
            ]
        )
    write_text(bag_output / "MANIFEST.yaml", "\n".join(manifest_lines) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", nargs="?", default="rosbags/rosbag_0")
    parser.add_argument("--output-root", default="rosbag_samples")
    args = parser.parse_args()

    extract_samples(Path(args.bag), Path(args.output_root))


if __name__ == "__main__":
    main()
