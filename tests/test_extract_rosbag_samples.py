"""Tests for the pure functions in ``scripts/extract_rosbag_samples.py``.

These are the parts of the pipeline that can be checked without Isaac Sim, a
running ROS graph, or a 331 MB bag -- the quaternion maths, the TF chain
resolution, the image unpacking and the path emission. Everything else in the
pipeline needs the simulator, so this is the only layer where a change can be
verified cheaply. Keep it that way: if you add logic to the extractor, add it
as a pure function and test it here.

Run with::

    ./run.sh test

which is shorthand for sourcing ROS 2 and using the system interpreter::

    source /opt/ros/jazzy/setup.bash
    /usr/bin/python3 -m pytest tests/ -v

The interpreter matters. ``/usr/bin/python3`` (3.12) has cv2, pytest and ROS 2
Jazzy; the conda python that comes first on PATH has none of them.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

from geometry_msgs.msg import TransformStamped  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

import extract_rosbag_samples as extract  # noqa: E402


# Quaternions are (x, y, z, w), matching geometry_msgs.
IDENTITY = (0.0, 0.0, 0.0, 1.0)
ROT_Z_90 = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
ROT_Z_180 = (0.0, 0.0, 1.0, 0.0)


def make_transform(parent: str, child: str, translation, rotation) -> TransformStamped:
    transform = TransformStamped()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    (
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z,
    ) = translation
    (
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w,
    ) = rotation
    return transform


def transform_translation(transform: TransformStamped) -> tuple[float, float, float]:
    t = transform.transform.translation
    return (t.x, t.y, t.z)


def transform_rotation(transform: TransformStamped):
    r = transform.transform.rotation
    return (r.x, r.y, r.z, r.w)


# --------------------------------------------------------------------------
# quaternion maths
# --------------------------------------------------------------------------

def test_quat_multiply_identity_is_neutral():
    assert extract.quat_multiply(ROT_Z_90, IDENTITY) == pytest.approx(ROT_Z_90)
    assert extract.quat_multiply(IDENTITY, ROT_Z_90) == pytest.approx(ROT_Z_90)


def test_quat_multiply_composes_rotations():
    """Two 90 degree yaws compose into one 180 degree yaw."""

    assert extract.quat_multiply(ROT_Z_90, ROT_Z_90) == pytest.approx(ROT_Z_180, abs=1e-12)


def test_quat_multiply_is_not_commutative():
    a = (0.5, 0.5, 0.5, 0.5)
    b = ROT_Z_90
    assert extract.quat_multiply(a, b) != pytest.approx(extract.quat_multiply(b, a))


def test_quat_rotate_yaw_sends_x_to_y():
    assert extract.quat_rotate(ROT_Z_90, (1.0, 0.0, 0.0)) == pytest.approx(
        (0.0, 1.0, 0.0), abs=1e-12
    )


def test_quat_rotate_preserves_the_rotation_axis():
    assert extract.quat_rotate(ROT_Z_90, (0.0, 0.0, 3.0)) == pytest.approx(
        (0.0, 0.0, 3.0), abs=1e-12
    )


def test_quat_rotate_preserves_length():
    vector = (0.3, -1.4, 2.0)
    rotated = extract.quat_rotate((0.5, 0.5, 0.5, 0.5), vector)
    assert math.dist(rotated, (0, 0, 0)) == pytest.approx(math.dist(vector, (0, 0, 0)))


def test_normalize_quat_scales_to_unit_length():
    normalized = extract.normalize_quat((0.0, 0.0, 2.0, 2.0))
    assert sum(c * c for c in normalized) == pytest.approx(1.0)


def test_normalize_quat_falls_back_to_identity_on_zero():
    """A zero quaternion has no direction; identity is the only safe answer."""

    assert extract.normalize_quat((0.0, 0.0, 0.0, 0.0)) == IDENTITY


# --------------------------------------------------------------------------
# TF chain resolution -- the highest-risk hand-rolled logic in the extractor
# --------------------------------------------------------------------------

def test_resolve_world_transform_direct_child():
    transforms = {"cam": make_transform("world", "cam", (1.0, 2.0, 3.0), ROT_Z_90)}
    result = extract.resolve_world_transform("cam", transforms)

    assert transform_translation(result) == pytest.approx((1.0, 2.0, 3.0))
    assert transform_rotation(result) == pytest.approx(ROT_Z_90)
    assert result.header.frame_id == "world"
    assert result.child_frame_id == "cam"


def test_resolve_world_transform_composes_a_two_link_chain():
    """The child's offset must be rotated by the parent before being added.

    world -> base is a 90 degree yaw at x=1; base -> cam is +1 along base's own
    x, which after the yaw points along world +y. So cam sits at (1, 1, 0), not
    (2, 0, 0) -- getting this wrong is the classic TF composition bug.
    """

    transforms = {
        "base": make_transform("world", "base", (1.0, 0.0, 0.0), ROT_Z_90),
        "cam": make_transform("base", "cam", (1.0, 0.0, 0.0), IDENTITY),
    }
    result = extract.resolve_world_transform("cam", transforms)

    assert transform_translation(result) == pytest.approx((1.0, 1.0, 0.0), abs=1e-12)
    assert transform_rotation(result) == pytest.approx(ROT_Z_90, abs=1e-12)


def test_resolve_world_transform_composes_three_links():
    transforms = {
        "a": make_transform("world", "a", (0.0, 0.0, 1.0), IDENTITY),
        "b": make_transform("a", "b", (0.0, 0.0, 1.0), IDENTITY),
        "c": make_transform("b", "c", (0.0, 0.0, 1.0), IDENTITY),
    }
    result = extract.resolve_world_transform("c", transforms)
    assert transform_translation(result) == pytest.approx((0.0, 0.0, 3.0))


def test_resolve_world_transform_returns_none_for_orphan_chain():
    """A chain that never reaches the world frame is unusable, not an error."""

    transforms = {"cam": make_transform("floating", "cam", (1.0, 0.0, 0.0), IDENTITY)}
    assert extract.resolve_world_transform("cam", transforms) is None


def test_resolve_world_transform_returns_none_for_unknown_frame():
    assert extract.resolve_world_transform("nope", {}) is None


def test_resolve_world_transform_survives_a_cycle():
    """A malformed /tf with a loop must terminate rather than blow the stack."""

    transforms = {
        "a": make_transform("b", "a", (1.0, 0.0, 0.0), IDENTITY),
        "b": make_transform("a", "b", (1.0, 0.0, 0.0), IDENTITY),
    }
    assert extract.resolve_world_transform("a", transforms) is None


def test_resolve_world_transform_strips_leading_slashes():
    """Pre-Jazzy publishers emit ``/frame``; both spellings must resolve."""

    transforms = {"cam": make_transform("world", "cam", (1.0, 0.0, 0.0), IDENTITY)}
    assert extract.resolve_world_transform("/cam", transforms) is not None


def test_resolve_world_transform_normalizes_the_output_rotation():
    transforms = {
        "a": make_transform("world", "a", (0.0, 0.0, 0.0), (0.0, 0.0, 0.6, 0.6)),
        "b": make_transform("a", "b", (0.0, 0.0, 0.0), (0.0, 0.0, 0.6, 0.6)),
    }
    result = extract.resolve_world_transform("b", transforms)
    assert sum(c * c for c in transform_rotation(result)) == pytest.approx(1.0)


# --------------------------------------------------------------------------
# image unpacking
# --------------------------------------------------------------------------

def make_image(width, height, encoding, data, step=None) -> Image:
    msg = Image()
    msg.width = width
    msg.height = height
    msg.encoding = encoding
    msg.step = step if step is not None else width * {"rgb8": 3, "bgr8": 3, "mono8": 1}[encoding]
    msg.data = bytes(data)
    return msg


def test_image_to_bgr_swaps_red_and_blue():
    msg = make_image(1, 1, "rgb8", [10, 20, 30])
    assert extract.image_to_bgr(msg).reshape(-1).tolist() == [30, 20, 10]


def test_image_to_bgr_passes_bgr_through():
    msg = make_image(1, 1, "bgr8", [10, 20, 30])
    assert extract.image_to_bgr(msg).reshape(-1).tolist() == [10, 20, 30]


def test_image_to_bgr_honours_row_padding():
    """``step`` can exceed width*channels; the padding must not leak into pixels.

    This is the one case where a naive reshape silently produces a skewed
    image rather than raising, so it is worth pinning.
    """

    width, height = 2, 2
    padded_step = width * 3 + 4
    rows = []
    for row in range(height):
        rows += [row + 1, row + 1, row + 1, row + 1, row + 1, row + 1] + [99] * 4
    msg = make_image(width, height, "bgr8", rows, step=padded_step)

    result = extract.image_to_bgr(msg)
    assert result.shape == (height, width, 3)
    assert 99 not in result.reshape(-1).tolist()
    assert result[0].reshape(-1).tolist() == [1] * 6
    assert result[1].reshape(-1).tolist() == [2] * 6


def test_image_to_bgr_handles_mono8():
    msg = make_image(2, 2, "mono8", [1, 2, 3, 4])
    assert extract.image_to_bgr(msg).tolist() == [[1, 2], [3, 4]]


def test_image_to_bgr_rejects_unknown_encoding():
    msg = Image()
    msg.width = msg.height = 1
    msg.step = 2
    msg.encoding = "16UC1"
    msg.data = bytes([0, 0])
    with pytest.raises(ValueError, match="Unsupported image encoding"):
        extract.image_to_bgr(msg)


def test_image_to_bgr_is_case_insensitive_about_encoding():
    msg = make_image(1, 1, "rgb8", [1, 2, 3])
    msg.encoding = "RGB8"
    assert extract.image_to_bgr(msg).reshape(-1).tolist() == [3, 2, 1]


# --------------------------------------------------------------------------
# path emission -- these strings get committed, so they must stay portable
# --------------------------------------------------------------------------

def test_repo_relative_strips_the_repo_root():
    inside = extract.REPO_ROOT / "rosbag_samples" / "rosbag_0" / "MANIFEST.yaml"
    assert extract.repo_relative(inside) == "rosbag_samples/rosbag_0/MANIFEST.yaml"


def test_repo_relative_keeps_outside_paths_absolute():
    assert extract.repo_relative(Path("/etc/hostname")) == "/etc/hostname"


def test_repo_relative_never_emits_a_home_path_for_repo_content():
    """Regression: the manifest used to embed /home/<user>/... everywhere."""

    emitted = extract.repo_relative(extract.REPO_ROOT / "rosbags" / "rosbag_0")
    assert not emitted.startswith("/")
    assert "home" not in emitted


def test_topic_label_flattens_a_topic_into_a_filename():
    assert extract.topic_label("/center_camera/image") == "center_camera_image"
    assert extract.topic_label("/tf") == "tf"


def _make_bag(root: Path, name: str, mtime: float) -> Path:
    bag = root / name
    bag.mkdir(parents=True)
    metadata = bag / "metadata.yaml"
    metadata.write_text("version: 5\n")
    os.utime(metadata, (mtime, mtime))
    return bag


def test_latest_bag_picks_the_most_recently_written_bag(tmp_path):
    _make_bag(tmp_path, "rosbag_20260101_000000_000000", mtime=1000)
    newest = _make_bag(tmp_path, "rosbag_20260917_183618_346247", mtime=3000)
    _make_bag(tmp_path, "rosbag_20260501_120000_000000", mtime=2000)

    assert extract.latest_bag(tmp_path) == newest


def test_latest_bag_ignores_directories_without_metadata(tmp_path):
    real = _make_bag(tmp_path, "rosbag_real", mtime=1000)
    (tmp_path / "not_a_bag").mkdir()

    assert extract.latest_bag(tmp_path) == real


def test_latest_bag_returns_none_when_there_is_nothing_to_find(tmp_path):
    assert extract.latest_bag(tmp_path) is None
    assert extract.latest_bag(tmp_path / "missing") is None


def test_group_frames_groups_topics_sharing_a_bag_timestamp():
    records = [
        ("/center_camera/image", b"c1", 100),
        ("/left_camera/image", b"l1", 100),
        ("/right_camera/image", b"r1", 100),
        ("/tf", b"t1", 100),
        ("/center_camera/image", b"c2", 200),
        ("/left_camera/image", b"l2", 200),
        ("/right_camera/image", b"r2", 200),
        ("/tf", b"t2", 200),
    ]

    groups = list(extract.group_frames(records))

    assert [timestamp for timestamp, _ in groups] == [100, 200]
    assert groups[0][1]["/center_camera/image"] == b"c1"
    assert groups[1][1]["/tf"] == b"t2"


def test_group_frames_never_mixes_two_timestamps_into_one_frame():
    """Interleaved writes must not borrow a topic from a neighbouring frame."""

    records = [
        ("/center_camera/image", b"c1", 100),
        ("/center_camera/image", b"c2", 200),
        ("/left_camera/image", b"l2", 200),
        ("/right_camera/image", b"r2", 200),
        ("/tf", b"t2", 200),
    ]

    groups = list(extract.group_frames(records))

    assert len(groups) == 1
    timestamp, group = groups[0]
    assert timestamp == 200
    assert group["/center_camera/image"] == b"c2"


def test_group_frames_drops_incomplete_groups():
    records = [
        ("/center_camera/image", b"c1", 100),
        ("/left_camera/image", b"l1", 100),
        ("/tf", b"t1", 100),
    ]

    assert list(extract.group_frames(records)) == []


def test_group_frames_ignores_topics_outside_a_frame():
    records = [
        ("/center_camera/camera_info", b"info", 100),
        ("/center_camera/image", b"c1", 100),
        ("/left_camera/image", b"l1", 100),
        ("/right_camera/image", b"r1", 100),
        ("/tf", b"t1", 100),
    ]

    _, group = next(iter(extract.group_frames(records)))

    assert "/center_camera/camera_info" not in group
    assert sorted(group) == sorted(extract.FRAME_TOPICS)


def test_group_frames_releases_each_group_as_it_completes():
    """Memory must not grow with bag length: ~1950 frames cannot be buffered."""

    def records():
        for stamp in range(0, 500, 100):
            for topic in extract.FRAME_TOPICS:
                yield topic, b"x", stamp

    assert len(list(extract.group_frames(records()))) == 5


def test_frame_dir_name_sorts_lexicographically():
    names = [extract.frame_dir_name(i) for i in (0, 9, 10, 1000)]
    assert names == ["00000", "00009", "00010", "01000"]
    assert names == sorted(names)
