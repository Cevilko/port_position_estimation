# port_position_estimation - Copyright (C) 2026 Cevilko <lvelicko03@gmail.com>
# SPDX-License-Identifier: AGPL-3.0-only
"""Run the SFP port detector on live camera topics.

Subscribes to one or more ``sensor_msgs/Image`` topics, runs the trained YOLO
model on each frame, and publishes ``vision_msgs/Detection2DArray`` alongside
an optional annotated image.

This node MUST run under the venv interpreter that has torch -- see
``./run.sh detect``. ``ros2 run yolo_detector_node ...`` will not work: colcon
gives the console script a ``/usr/bin/python3`` shebang and that interpreter
has no torch. Both interpreters are Python 3.12, which is the only reason
rclpy and ultralytics can share one process at all.
"""

from __future__ import annotations

import os
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from .image_convert import image_to_bgr, bgr_to_image_fields


DEFAULT_CAMERAS = ["/center_camera", "/left_camera", "/right_camera"]

#: Where the trained detector lands, relative to the repository root.
DEFAULT_WEIGHTS_RELATIVE = Path("runs/sfp_yolo26s_p2/weights/best.pt")


def find_repository_root():
    """Walk up from this file looking for the repo, or None.

    With ``--symlink-install`` (what ``./run.sh build`` uses) the installed
    module is a symlink back into ``ros_ws/src/``, so ``__file__`` resolves into
    the source tree and the walk finds the checkout. A non-symlink install has
    no path back, and callers fall through to the other candidates.
    """
    for directory in Path(__file__).resolve().parents:
        if (directory / "run.sh").is_file() and (directory / "ros_ws").is_dir():
            return directory
    return None


def default_weights() -> Path:
    """The default for the ``model`` parameter, always an absolute path.

    This used to be the *relative* ``runs/...`` path, which only worked because
    ``./run.sh detect`` cd's to the repository first. Under ``ros2 run`` the
    node inherits the caller's working directory, so the same default resolved
    to nothing and the load failed with a bare FileNotFoundError.

    ``YOLO_WEIGHTS`` wins if set, so a different checkout or a shared model
    needs no argument. Otherwise the path is anchored to the repository this
    file lives in, which makes the default independent of where it is called
    from.
    """
    override = os.environ.get("YOLO_WEIGHTS")
    if override:
        return Path(override).expanduser().resolve()
    repository = find_repository_root()
    if repository is not None:
        return repository / DEFAULT_WEIGHTS_RELATIVE
    # No path back to the checkout (a non-symlink install). Fall back to the
    # working directory so the value is still absolute and still says plainly
    # what was looked for.
    return (Path.cwd() / DEFAULT_WEIGHTS_RELATIVE).resolve()


def resolve_weights(parameter_value: str) -> Path:
    """Make the ``model`` parameter absolute and check it exists.

    A relative value is resolved against the working directory, which is what a
    caller passing one would expect.
    """
    path = Path(parameter_value).expanduser()
    if not path.is_absolute():
        path = (Path.cwd() / path).resolve()
    if path.is_file():
        return path
    raise FileNotFoundError(
        f"no detector weights at {path}\n"
        "Train a model with './run.sh train', or point the node at one:\n"
        "  ros2 run yolo_detector_node yolo_detector_node --ros-args"
        " -p model:=/abs/path/to/best.pt\n"
        "  (or set YOLO_WEIGHTS=/abs/path/to/best.pt)"
    )


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_detector_node")

        # An absolute path, anchored to this checkout, so the node works from
        # any working directory -- `ros2 run` does not cd anywhere.
        self.declare_parameter("model", str(default_weights()))
        # 1152 is not a detail. The model was trained at the camera's native
        # width and the ports are ~17 px there; the ultralytics default of 640
        # would shrink them to ~9 px and the detector would look far worse than
        # it is. Changing this silently degrades results.
        self.declare_parameter("imgsz", 1152)
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.7)
        self.declare_parameter("device", "")
        self.declare_parameter("cameras", DEFAULT_CAMERAS)
        self.declare_parameter("image_suffix", "/image")
        self.declare_parameter("publish_annotated", True)
        # Reliability follows the DDS "offered >= requested" rule, where
        # RELIABLE outranks BEST_EFFORT. So a BEST_EFFORT subscriber matches
        # ANY publisher, while a RELIABLE one matches only a RELIABLE
        # publisher -- RELIABLE is the stricter request, not the safer one.
        # It is the default because the scene's cameras publish RELIABLE
        # (bag_recorder_node subscribes that way and receives frames) and that
        # pairing will not drop a frame. Against a best-effort publisher,
        # which is what most real camera drivers use for sensor data, this
        # subscriber matches nothing: ROS 2 logs an "incompatible QoS" warning
        # on both sides and no callback ever fires. That is what
        # best_effort:=true is for.
        self.declare_parameter("best_effort", False)

        self.imgsz = int(self.get_parameter("imgsz").value)
        self.conf = float(self.get_parameter("conf").value)
        self.iou = float(self.get_parameter("iou").value)
        model_path = resolve_weights(str(self.get_parameter("model").value))

        from ultralytics import YOLO  # imported late so --help style runs stay fast

        self.get_logger().info(f"loading {model_path}")
        self.model = YOLO(str(model_path))
        device = str(self.get_parameter("device").value)
        if device:
            self.model.to(device)

        self._warm_up()

        qos = QoSProfile(
            reliability=(
                QoSReliabilityPolicy.BEST_EFFORT
                if bool(self.get_parameter("best_effort").value)
                else QoSReliabilityPolicy.RELIABLE
            ),
            history=QoSHistoryPolicy.KEEP_LAST,
            # Depth 1: inference is slower than the camera, and a deeper queue
            # buys latency that grows without bound instead of dropping frames.
            depth=1,
        )

        suffix = str(self.get_parameter("image_suffix").value)
        publish_annotated = bool(self.get_parameter("publish_annotated").value)
        self.detection_publishers = {}
        self.annotated_publishers = {}
        self.subscriptions_by_camera = {}

        for camera in list(self.get_parameter("cameras").value):
            topic = f"{camera}{suffix}"
            self.detection_publishers[camera] = self.create_publisher(
                Detection2DArray, f"{camera}/detections", 10
            )
            if publish_annotated:
                self.annotated_publishers[camera] = self.create_publisher(
                    Image, f"{camera}/detections_image", 1
                )
            self.subscriptions_by_camera[camera] = self.create_subscription(
                Image, topic, self._make_callback(camera), qos
            )
            self.get_logger().info(f"{topic} -> {camera}/detections")

        self.frames = 0
        self.total_seconds = 0.0
        self.create_timer(10.0, self._report)

    def _warm_up(self) -> None:
        """First inference pays for CUDA init; do it before any callback."""
        import numpy as np

        blank = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
        started = time.monotonic()
        self.model.predict(blank, imgsz=self.imgsz, verbose=False)
        self.get_logger().info(f"warm-up inference: {time.monotonic() - started:.2f}s")

    def _make_callback(self, camera: str):
        def callback(message: Image) -> None:
            self._on_image(camera, message)
        return callback

    def _on_image(self, camera: str, message: Image) -> None:
        try:
            frame = image_to_bgr(
                message.height, message.width, message.step,
                message.encoding, message.data,
            )
        except ValueError as error:
            self.get_logger().warn(f"{camera}: {error}", throttle_duration_sec=5.0)
            return

        started = time.monotonic()
        result = self.model.predict(
            frame, imgsz=self.imgsz, conf=self.conf, iou=self.iou, verbose=False
        )[0]
        self.total_seconds += time.monotonic() - started
        self.frames += 1

        self.detection_publishers[camera].publish(
            self._to_detection_array(result, message)
        )

        publisher = self.annotated_publishers.get(camera)
        if publisher is not None:
            annotated = result.plot()
            height, width, step, encoding, data = bgr_to_image_fields(annotated)
            out = Image()
            out.header = message.header
            out.height, out.width = height, width
            out.encoding, out.step = encoding, step
            out.is_bigendian = 0
            out.data = data
            publisher.publish(out)

    def _to_detection_array(self, result, message: Image) -> Detection2DArray:
        array = Detection2DArray()
        # The image's own header, so a consumer can line detections up with the
        # frame they came from and with /tf at that instant.
        array.header = message.header

        boxes = result.boxes
        names = result.names
        for index in range(len(boxes)):
            x_min, y_min, x_max, y_max = boxes.xyxy[index].tolist()
            detection = Detection2D()
            detection.header = message.header

            box = BoundingBox2D()
            box.center.position.x = (x_min + x_max) / 2.0
            box.center.position.y = (y_min + y_max) / 2.0
            box.center.theta = 0.0
            box.size_x = x_max - x_min
            box.size_y = y_max - y_min
            detection.bbox = box

            hypothesis = ObjectHypothesisWithPose()
            class_index = int(boxes.cls[index].item())
            hypothesis.hypothesis.class_id = str(names.get(class_index, class_index))
            hypothesis.hypothesis.score = float(boxes.conf[index].item())
            detection.results.append(hypothesis)

            array.detections.append(detection)
        return array

    def _report(self) -> None:
        if not self.frames:
            self.get_logger().warn(
                "no images received yet -- check the topic names and that the "
                "publisher's QoS matches (try best_effort:=true)"
            )
            return
        average = self.total_seconds / self.frames * 1000.0
        self.get_logger().info(
            f"{self.frames} frames, {average:.1f} ms/frame inference"
        )


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT raises the first, SIGTERM the second. `./run.sh stop` sends
        # INT and escalates to TERM, so catching only one leaves a traceback
        # on an ordinary, requested shutdown.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
