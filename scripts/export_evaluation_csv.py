#!/usr/bin/env python3
"""Export per-sample triangulation results as CSV, for plotting.

The running ROS nodes compute all of this and then reduce it to a log line;
nothing persists the per-sample values. This script reproduces them **offline**
over an extracted run, which matters: the published figures for this pipeline
were measured this way, over every frame, and the live path scores a different
subset (it drops sets when time-sync or a tf lookup fails). Exporting from a bag
replay instead would produce numbers that quietly disagree with the tables.

Two files are written:

``position-errors.csv``        sec,nanosec,error_m,n_views,distance_m,mahalanobis
``reprojection-residuals.csv`` sec,nanosec,rms_error_px,n_views

``error_m`` is the distance from a triangulated port to its assigned ground
truth, paired **one-to-one** so two ports can never score against the same
estimate. ``distance_m`` is from the centre camera's optical origin to the
triangulated point. ``mahalanobis`` uses the triangulator's own covariance, so
it depends on ``--pixel-sigma`` -- the default 1.5 is what the reported figure
of ~0.18 was measured at.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "ros_ws" / "src" / "port_triangulator_node"))
sys.path.insert(0, str(REPO / "ros_ws" / "src" / "port_error_node"))

from port_error_node.assignment import optimal_assignment, distance_matrix  # noqa: E402
from port_triangulator_node.triangulation import (  # noqa: E402
    match_and_triangulate,
    projection_matrix,
    quaternion_to_matrix,
)

CAMERAS = ("center_camera", "left_camera", "right_camera")
PORTS = ("sfp_port_0_entrance", "sfp_port_1_entrance")
REFERENCE_CAMERA = "center_camera"


def load_yaml(path: Path):
    return yaml.safe_load(path.read_text())


def translation_of(document) -> np.ndarray:
    t = document["transform"]["translation"]
    return np.array([t["x"], t["y"], t["z"]])


def pose_of(document):
    t = document["transform"]["translation"]
    q = document["transform"]["rotation"]
    return (
        quaternion_to_matrix(q["x"], q["y"], q["z"], q["w"]),
        np.array([t["x"], t["y"], t["z"]]),
    )


def mahalanobis(delta: np.ndarray, covariance: np.ndarray) -> float | None:
    """sqrt(d' S^-1 d), or None if the covariance is not invertible."""
    try:
        return float(np.sqrt(delta @ np.linalg.inv(covariance) @ delta))
    except np.linalg.LinAlgError:
        return None


def intrinsics_for(bag_dir: Path):
    return {
        camera: np.asarray(
            load_yaml(bag_dir / "camera_info" / f"{camera}_camera_info.yaml")["k"],
            dtype=float,
        ).reshape(3, 3)
        for camera in CAMERAS
    }


def export(bag_dir: Path, weights: Path, output_dir: Path, pixel_sigma: float,
           max_reprojection_error: float, conf: float, imgsz: int):
    from ultralytics import YOLO

    bag_dir = Path(bag_dir)
    intrinsics = intrinsics_for(bag_dir)
    model = YOLO(str(weights))
    frames = sorted((bag_dir / "frames").iterdir())

    position_rows, residual_rows = [], []
    frames_without_solution = 0

    for frame_dir in frames:
        tf_dir = frame_dir / "tf_world_transforms"
        try:
            truths = [translation_of(load_yaml(tf_dir / f"world_to_{p}.yaml")) for p in PORTS]
        except FileNotFoundError:
            continue

        stamp = None
        projections, images, used_cameras = [], [], []
        for camera in CAMERAS:
            image_path = frame_dir / "images" / f"{camera}_image.jpg"
            transform_path = tf_dir / f"world_to_{camera}.yaml"
            if not image_path.is_file() or not transform_path.is_file():
                continue
            rotation, translation = pose_of(load_yaml(transform_path))
            projections.append(projection_matrix(intrinsics[camera], rotation, translation))
            images.append(str(image_path))
            used_cameras.append(camera)
            if camera == REFERENCE_CAMERA:
                header = load_yaml(frame_dir / "images" / f"{camera}_image.yaml")["header"]
                stamp = (int(header["stamp"]["sec"]), int(header["stamp"]["nanosec"]))
                reference_origin = translation

        if stamp is None or len(projections) < 2:
            continue

        detections = []
        for result in model.predict(images, imgsz=imgsz, conf=conf, verbose=False):
            detections.append([
                np.array([(b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0])
                for b in result.boxes.xyxy.cpu().numpy()
            ])

        found = match_and_triangulate(
            list(zip(projections, detections)),
            max_reprojection_error=max_reprojection_error,
            pixel_sigma=pixel_sigma,
            max_points=len(PORTS),
        )
        if not found:
            frames_without_solution += 1
            continue

        for port in found:
            residual_rows.append({
                "sec": stamp[0], "nanosec": stamp[1],
                "rms_error_px": round(port["rms_error"], 6),
                "n_views": port["views"],
            })

        # One-to-one, exactly as port_error_node does it: a port left without a
        # partner is dropped, never scored against its neighbour's estimate.
        estimates = [port["position"] for port in found]
        costs = distance_matrix(truths, estimates)
        for truth_index, estimate_index in optimal_assignment(costs):
            port = found[estimate_index]
            delta = port["position"] - truths[truth_index]
            position_rows.append({
                "sec": stamp[0], "nanosec": stamp[1],
                "error_m": round(float(costs[truth_index, estimate_index]), 9),
                "n_views": port["views"],
                "distance_m": round(float(np.linalg.norm(port["position"] - reference_origin)), 6),
                "mahalanobis": (
                    round(m, 6) if (m := mahalanobis(delta, port["covariance"])) is not None else ""
                ),
            })

    output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(output_dir / "position-errors.csv",
              ["sec", "nanosec", "error_m", "n_views", "distance_m", "mahalanobis"],
              position_rows)
    write_csv(output_dir / "reprojection-residuals.csv",
              ["sec", "nanosec", "rms_error_px", "n_views"], residual_rows)
    return position_rows, residual_rows, len(frames), frames_without_solution


def write_csv(path: Path, fieldnames, rows) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="extracted bag directory under rosbag_samples/")
    parser.add_argument("--weights", default=str(REPO / "runs/sfp_yolo26s_p2/weights/best.pt"))
    parser.add_argument("--output-dir", default="figure-delivery")
    parser.add_argument("--pixel-sigma", type=float, default=1.5,
                        help="covariance scale; the reported ~0.18 Mahalanobis is at 1.5")
    parser.add_argument("--max-reprojection-error", type=float, default=5.0)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--imgsz", type=int, default=1152)
    args = parser.parse_args()

    positions, residuals, frames, unsolved = export(
        Path(args.bag), Path(args.weights), Path(args.output_dir),
        args.pixel_sigma, args.max_reprojection_error, args.conf, args.imgsz,
    )

    errors = np.array([r["error_m"] for r in positions])
    views = [r["n_views"] for r in positions]
    residual_values = np.array([r["rms_error_px"] for r in residuals])
    mahal = np.array([r["mahalanobis"] for r in positions if r["mahalanobis"] != ""])

    print(f"frames                 : {frames} ({unsolved} with no solution)")
    print(f"position-errors.csv    : {len(positions)} rows")
    print(f"reprojection-residuals : {len(residuals)} rows")
    print(f"error_m      median {np.median(errors)*1000:.2f} mm  "
          f"p90 {np.percentile(errors,90)*1000:.2f}  max {np.max(errors)*1000:.2f}")
    print(f"mahalanobis  median {np.median(mahal):.3f}")
    print(f"rms_error_px median {np.median(residual_values):.3f}")
    print(f"n_views      " + "  ".join(f"{v}-view {views.count(v)}" for v in sorted(set(views))))


if __name__ == "__main__":
    main()
