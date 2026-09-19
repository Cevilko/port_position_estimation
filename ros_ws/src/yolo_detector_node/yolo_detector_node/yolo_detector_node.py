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

import time

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


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_detector_node")

        self.declare_parameter("model", "runs/sfp_yolo26s_p2/weights/best.pt")
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
        # The scene's cameras publish RELIABLE (bag_recorder_node subscribes
        # that way and receives frames). A BEST_EFFORT subscriber would match
        # nothing and sit silent, which is the classic "my node gets no
        # images" failure, so reliable is the default here too.
        self.declare_parameter("best_effort", False)

        self.imgsz = int(self.get_parameter("imgsz").value)
        self.conf = float(self.get_parameter("conf").value)
        self.iou = float(self.get_parameter("iou").value)
        model_path = str(self.get_parameter("model").value)

        from ultralytics import YOLO  # imported late so --help style runs stay fast

        self.get_logger().info(f"loading {model_path}")
        self.model = YOLO(model_path)
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
