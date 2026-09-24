#!/usr/bin/python3
# port_position_estimation - Copyright (C) 2026 Cevilko <lvelicko03@gmail.com>
# SPDX-License-Identifier: AGPL-3.0-only
"""Turn extracted frames into a YOLO detection dataset.

Reads the frames ``extract_rosbag_samples.py`` wrote, projects the SFP port
apertures into each camera with ``project_port_bboxes``, and writes the layout
Ultralytics expects::

    <out>/images/{train,val}/<bag>_<frame>_<camera>.jpg
    <out>/labels/{train,val}/<bag>_<frame>_<camera>.txt
    <out>/data.yaml

The split is by frame index, never by image. The three cameras of one frame see
the same instant from rigidly connected viewpoints, so splitting at random
would put near-duplicates on both sides and quietly inflate validation scores.

Both ports are one class by default. They are the same part and nothing in the
pixels distinguishes them; telling them apart is a question about position in
the scene, which the recorded transforms already answer exactly. ``--per-port``
keeps them separate if you want the detector to try anyway.
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import project_port_bboxes as ports


CAMERAS = ("center_camera", "left_camera", "right_camera")


def to_yolo_line(class_index: int, box, image_size) -> str:
    """One YOLO row: class, centre x/y, width, height, all normalised."""
    width, height = image_size
    x_min, y_min, x_max, y_max = box
    return (
        f"{class_index} "
        f"{((x_min + x_max) / 2) / width:.6f} "
        f"{((y_min + y_max) / 2) / height:.6f} "
        f"{(x_max - x_min) / width:.6f} "
        f"{(y_max - y_min) / height:.6f}"
    )


def split_for(index: int, val_every: int) -> str:
    """Assign a frame to train or val deterministically, by index."""
    if val_every <= 0:
        return "train"
    return "val" if index % val_every == 0 else "train"


def box_is_usable(box, image_size, min_pixels: float, edge_margin: float) -> bool:
    """Reject boxes too small to learn from, or clipped by the image edge.

    A box touching the border is a port that is partly outside the frame; its
    true extent is unknown, so training on the clipped extent teaches the wrong
    size.
    """
    if box is None:
        return False
    x_min, y_min, x_max, y_max = box
    if (x_max - x_min) < min_pixels or (y_max - y_min) < min_pixels:
        return False
    width, height = image_size
    return not (
        x_min <= edge_margin
        or y_min <= edge_margin
        or x_max >= width - 1 - edge_margin
        or y_max >= height - 1 - edge_margin
    )


def export(
    bag_dir: Path,
    output_root: Path,
    val_every: int = 5,
    per_port: bool = False,
    min_pixels: float = 8.0,
    edge_margin: float = 1.0,
    width: float = ports.DEFAULT_APERTURE_WIDTH,
    height: float = ports.DEFAULT_APERTURE_HEIGHT,
    link: bool = False,
    drop_inconsistent: bool = False,
):
    bag_dir = Path(bag_dir).resolve()
    output_root = Path(output_root)
    class_names = list(ports.PORT_LABELS) if per_port else ["sfp_port"]

    for split in ("train", "val"):
        (output_root / "images" / split).mkdir(parents=True, exist_ok=True)
        (output_root / "labels" / split).mkdir(parents=True, exist_ok=True)

    counts = {"train": 0, "val": 0}
    boxes_written = 0
    images_without_boxes = 0
    dropped = 0
    inconsistent_images = 0

    frame_dirs = sorted((bag_dir / "frames").iterdir())
    for index, frame_dir in enumerate(frame_dirs):
        split = split_for(index, val_every)
        for camera in CAMERAS:
            image_path = frame_dir / "images" / f"{camera}_image.jpg"
            if not image_path.is_file():
                continue
            try:
                frame_boxes, image_size = ports.bboxes_for_frame(
                    frame_dir, camera, width, height
                )
            except FileNotFoundError:
                continue

            rows = []
            dropped_here = 0
            for label, box in frame_boxes.items():
                if not box_is_usable(box, image_size, min_pixels, edge_margin):
                    if box is not None:
                        dropped += 1
                        dropped_here += 1
                    continue
                class_index = class_names.index(label) if per_port else 0
                rows.append(to_yolo_line(class_index, box, image_size))

            # A port that projects into the frame but is too small or clipped
            # cannot be labelled, yet it is still visible in the pixels. Writing
            # the image anyway teaches the detector that a visible port is
            # background, and it contradicts the other port in the same image.
            # Dropping the image loses one sample; keeping it corrupts two.
            if drop_inconsistent and dropped_here:
                inconsistent_images += 1
                continue

            stem = f"{bag_dir.name}_{frame_dir.name}_{camera}"
            destination = output_root / "images" / split / f"{stem}.jpg"
            if link:
                if destination.exists() or destination.is_symlink():
                    destination.unlink()
                destination.symlink_to(image_path)
            else:
                shutil.copy2(image_path, destination)
            # An image with no rows is a legitimate negative for YOLO: an empty
            # label file says "nothing here", which is not the same as no file.
            (output_root / "labels" / split / f"{stem}.txt").write_text(
                "\n".join(rows) + ("\n" if rows else ""), encoding="utf-8"
            )
            counts[split] += 1
            boxes_written += len(rows)
            if not rows:
                images_without_boxes += 1

    names = "\n".join(f"  {i}: {name}" for i, name in enumerate(class_names))
    (output_root / "data.yaml").write_text(
        f"# Generated by scripts/export_yolo_dataset.py from {bag_dir.name}\n"
        f"path: {output_root.resolve()}\n"
        f"train: images/train\n"
        f"val: images/val\n"
        f"nc: {len(class_names)}\n"
        f"names:\n{names}\n",
        encoding="utf-8",
    )

    return {
        "train_images": counts["train"],
        "val_images": counts["val"],
        "boxes": boxes_written,
        "images_without_boxes": images_without_boxes,
        "dropped_boxes": dropped,
        "inconsistent_images": inconsistent_images,
        "classes": class_names,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", nargs="?", help="extracted bag dir under rosbag_samples/")
    parser.add_argument("--output-root", default="yolo_dataset")
    parser.add_argument("--val-every", type=int, default=5,
                        help="put every Nth frame in val (0 disables the val split)")
    parser.add_argument("--per-port", action="store_true",
                        help="two classes instead of one shared sfp_port class")
    parser.add_argument("--min-pixels", type=float, default=8.0,
                        help="drop boxes smaller than this on either side")
    parser.add_argument("--drop-inconsistent", action="store_true",
                        help="skip images where a visible port could not be labelled "
                             "(too small or clipped), rather than labelling it as background")
    parser.add_argument("--link", action="store_true",
                        help="symlink images instead of copying them")
    args = parser.parse_args()

    if args.bag:
        bag_dir = Path(args.bag)
    else:
        roots = sorted(Path("rosbag_samples").glob("*/frames"))
        if not roots:
            parser.error("no extracted bags in rosbag_samples/; pass one explicitly")
        bag_dir = max((r.parent for r in roots), key=lambda p: p.stat().st_mtime)
        print(f"using newest extraction: {bag_dir}")

    summary = export(
        bag_dir, Path(args.output_root), args.val_every, args.per_port,
        args.min_pixels, link=args.link, drop_inconsistent=args.drop_inconsistent,
    )
    print(f"classes:            {summary['classes']}")
    print(f"train images:       {summary['train_images']}")
    print(f"val images:         {summary['val_images']}")
    print(f"boxes written:      {summary['boxes']}")
    print(f"images with no box: {summary['images_without_boxes']}")
    print(f"boxes dropped:      {summary['dropped_boxes']} (too small or clipped by the edge)")
    print(f"images skipped:     {summary['inconsistent_images']} (a visible port could not be labelled)")
    print(f"data.yaml:          {Path(args.output_root) / 'data.yaml'}")


if __name__ == "__main__":
    main()
