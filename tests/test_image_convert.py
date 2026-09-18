"""Tests for the cv_bridge replacement in yolo_detector_node.

This is the layer where a silent mistake is worst: a wrong channel order or a
mishandled row stride produces an image that looks plausible, trains nothing
and detects nothing, with no error anywhere. Run with ``./run.sh test``.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(
    0, str(Path(__file__).resolve().parent.parent / "ros_ws" / "src" / "yolo_detector_node")
)

from yolo_detector_node.image_convert import (  # noqa: E402
    bgr_to_image_fields,
    image_to_bgr,
)


def test_rgb8_channels_are_reversed_to_bgr():
    data = bytes([10, 20, 30])
    out = image_to_bgr(1, 1, 3, "rgb8", data)
    assert out[0, 0].tolist() == [30, 20, 10]


def test_bgr8_channels_are_left_alone():
    out = image_to_bgr(1, 1, 3, "bgr8", bytes([10, 20, 30]))
    assert out[0, 0].tolist() == [10, 20, 30]


def test_mono8_is_replicated_across_three_channels():
    out = image_to_bgr(1, 2, 2, "mono8", bytes([7, 9]))
    assert out.shape == (1, 2, 3)
    assert out[0, 0].tolist() == [7, 7, 7]
    assert out[0, 1].tolist() == [9, 9, 9]


def test_rgba8_drops_alpha_and_reverses():
    out = image_to_bgr(1, 1, 4, "rgba8", bytes([10, 20, 30, 255]))
    assert out[0, 0].tolist() == [30, 20, 10]


def test_padded_rows_are_not_sheared():
    """step > width*channels: the padding must be dropped per row, not folded in."""

    width, height = 2, 2
    step = 8  # 6 bytes of pixels + 2 bytes of padding
    rows = [bytes([1, 2, 3, 4, 5, 6, 0, 0]), bytes([7, 8, 9, 10, 11, 12, 0, 0])]
    out = image_to_bgr(height, width, step, "bgr8", b"".join(rows))
    assert out[0, 0].tolist() == [1, 2, 3]
    assert out[0, 1].tolist() == [4, 5, 6]
    assert out[1, 0].tolist() == [7, 8, 9]
    assert out[1, 1].tolist() == [10, 11, 12]


def test_shape_matches_the_declared_size():
    out = image_to_bgr(4, 5, 15, "rgb8", bytes(4 * 15))
    assert out.shape == (4, 5, 3)


def test_output_is_writable_and_contiguous():
    """predict() writes into the array it is handed; a frombuffer view is not."""

    out = image_to_bgr(2, 2, 6, "bgr8", bytes(12))
    assert out.flags["WRITEABLE"]
    assert out.flags["C_CONTIGUOUS"]
    out[0, 0, 0] = 5  # must not raise


def test_unsupported_encoding_is_rejected():
    with pytest.raises(ValueError, match="unsupported encoding"):
        image_to_bgr(1, 1, 2, "16UC1", bytes(2))


def test_truncated_buffer_is_rejected():
    with pytest.raises(ValueError, match="buffer holds"):
        image_to_bgr(4, 4, 12, "bgr8", bytes(12))


def test_step_smaller_than_a_row_is_rejected():
    with pytest.raises(ValueError, match="step .* too small"):
        image_to_bgr(1, 4, 3, "bgr8", bytes(12))


def test_round_trip_through_image_fields():
    original = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
    height, width, step, encoding, data = bgr_to_image_fields(original)
    assert (height, width, step, encoding) == (2, 3, 9, "bgr8")
    assert np.array_equal(image_to_bgr(height, width, step, encoding, data), original)


def test_bgr_to_image_fields_rejects_a_non_colour_array():
    with pytest.raises(ValueError, match="expected HxWx3"):
        bgr_to_image_fields(np.zeros((4, 4), dtype=np.uint8))
