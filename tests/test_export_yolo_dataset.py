"""Tests for the pure functions in ``scripts/export_yolo_dataset.py``.

The label arithmetic and the split rule are the parts worth pinning down: a
sign or normalisation error here produces a dataset that trains without
complaint and scores nonsense. Run with ``./run.sh test``.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

import export_yolo_dataset as export  # noqa: E402


def test_to_yolo_line_normalises_centre_and_size():
    line = export.to_yolo_line(0, (100.0, 200.0, 200.0, 300.0), (1000, 1000))
    class_index, cx, cy, w, h = line.split()

    assert class_index == "0"
    assert float(cx) == pytest.approx(0.15)
    assert float(cy) == pytest.approx(0.25)
    assert float(w) == pytest.approx(0.10)
    assert float(h) == pytest.approx(0.10)


def test_to_yolo_line_is_all_within_the_unit_square():
    line = export.to_yolo_line(0, (0.0, 0.0, 1151.0, 1023.0), (1152, 1024))
    assert all(0.0 <= float(v) <= 1.0 for v in line.split()[1:])


def test_to_yolo_line_keeps_the_class_index():
    assert export.to_yolo_line(1, (10.0, 10.0, 20.0, 20.0), (100, 100)).startswith("1 ")


def test_split_for_sends_every_nth_frame_to_val():
    splits = [export.split_for(i, val_every=5) for i in range(10)]
    assert splits == ["val", "train", "train", "train", "train"] * 2


def test_split_for_can_disable_the_val_split():
    assert all(export.split_for(i, val_every=0) == "train" for i in range(5))


def test_split_for_is_deterministic():
    """Re-exporting must not reshuffle frames between train and val."""

    assert [export.split_for(i, 5) for i in range(20)] == [
        export.split_for(i, 5) for i in range(20)
    ]


def test_box_is_usable_accepts_a_normal_box():
    assert export.box_is_usable((100.0, 100.0, 120.0, 118.0), (1152, 1024), 8.0, 1.0)


def test_box_is_usable_rejects_missing_boxes():
    assert not export.box_is_usable(None, (1152, 1024), 8.0, 1.0)


def test_box_is_usable_rejects_boxes_below_the_size_floor():
    assert not export.box_is_usable((100.0, 100.0, 104.0, 104.0), (1152, 1024), 8.0, 1.0)


def test_box_is_usable_rejects_a_box_clipped_by_the_image_edge():
    """A port half outside the frame has an unknown true extent."""

    assert not export.box_is_usable((0.0, 100.0, 20.0, 118.0), (1152, 1024), 8.0, 1.0)
    assert not export.box_is_usable((1100.0, 100.0, 1151.0, 118.0), (1152, 1024), 8.0, 1.0)


def test_box_is_usable_rejects_a_box_touching_the_bottom_edge():
    assert not export.box_is_usable((100.0, 1000.0, 120.0, 1023.0), (1152, 1024), 8.0, 1.0)
