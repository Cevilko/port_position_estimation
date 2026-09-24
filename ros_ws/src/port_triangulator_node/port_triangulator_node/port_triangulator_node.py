# port_position_estimation - Copyright (C) 2026 Cevilko <lvelicko03@gmail.com>
# SPDX-License-Identifier: AGPL-3.0-only
"""Triangulate the SFP ports from multi-camera YOLO detections.

Subscribes, per camera, to ``camera_info`` (intrinsics) and ``detections``
(``vision_msgs/Detection2DArray`` from ``yolo_detector_node``), plus ``/tf`` for
extrinsics. Publishes one ``geometry_msgs/PoseWithCovariance`` per port, in the
world frame.

The three detection streams are synchronised on their stamps, because the
cameras ride a moving arm: mixing a detection from one instant with an
extrinsic from another is the same error that once cost this project 9 px of
label accuracy. Transforms are looked up at the detection's own stamp for the
same reason.

Like the detector, this MUST run under the venv interpreter -- see
``./run.sh triangulate``.
"""

from __future__ import annotations

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray

from .triangulation import (
    latest_stamp,
    match_and_triangulate,
    projection_matrix,
    quaternion_to_matrix,
)

DEFAULT_CAMERAS = ["/center_camera", "/left_camera", "/right_camera"]


class PortTriangulatorNode(Node):
    def __init__(self):
        super().__init__("port_triangulator_node")

        self.declare_parameter("cameras", DEFAULT_CAMERAS)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("max_ports", 2)
        self.declare_parameter("min_views", 2)
        # Pixel noise of a detected box centre. The projected labels this model
        # was trained on sat ~0.7 px from the visible aperture, and the detector
        # adds its own error on top, so 1.5 px is a deliberately mild
        # over-estimate: it inflates the covariance rather than overstating
        # confidence.
        self.declare_parameter("pixel_sigma", 1.5)
        self.declare_parameter("max_reprojection_error", 5.0)
        self.declare_parameter("queue_size", 10)
        self.declare_parameter("sync_slop", 0.05)
        # Stamped by default: a bare PoseWithCovariance carries neither the
        # frame the position is in nor the instant it describes, which a
        # consumer needs in order to do anything with a moving scene.
        self.declare_parameter("publish_stamped", True)
        # A triangulated point says nothing about orientation, so the rotation
        # block is filled with this rather than a small number that would claim
        # an orientation we never estimated.
        self.declare_parameter("orientation_variance", 1e6)
        self.declare_parameter("report_period", 10.0)

        self.cameras = list(self.get_parameter("cameras").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.max_ports = int(self.get_parameter("max_ports").value)
        self.min_views = int(self.get_parameter("min_views").value)
        self.pixel_sigma = float(self.get_parameter("pixel_sigma").value)
        self.max_reprojection_error = float(self.get_parameter("max_reprojection_error").value)
        self.publish_stamped = bool(self.get_parameter("publish_stamped").value)
        self.orientation_variance = float(self.get_parameter("orientation_variance").value)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.intrinsics: dict[str, np.ndarray] = {}
        self.info_subscriptions = []
        camera_info_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        for camera in self.cameras:
            self.info_subscriptions.append(
                self.create_subscription(
                    CameraInfo, f"{camera}/camera_info",
                    self._make_info_callback(camera), camera_info_qos,
                )
            )

        message_type = PoseWithCovarianceStamped if self.publish_stamped else PoseWithCovariance
        self.pose_publishers = [
            self.create_publisher(message_type, f"~/port_{index}/pose", 10)
            for index in range(self.max_ports)
        ]

        queue_size = int(self.get_parameter("queue_size").value)
        self.detection_subscribers = [
            Subscriber(self, Detection2DArray, f"{camera}/detections")
            for camera in self.cameras
        ]
        self.synchronizer = ApproximateTimeSynchronizer(
            self.detection_subscribers, queue_size,
            float(self.get_parameter("sync_slop").value),
        )
        self.synchronizer.registerCallback(self._on_detections)

        self.synced = 0
        self.published = 0
        self.last_errors: list[float] = []
        self.create_timer(float(self.get_parameter("report_period").value), self._report)

        self.get_logger().info(
            f"triangulating {len(self.cameras)} cameras into '{self.world_frame}', "
            f"pixel_sigma={self.pixel_sigma}, min_views={self.min_views}"
        )

    def _make_info_callback(self, camera: str):
        def callback(message: CameraInfo) -> None:
            self.intrinsics[camera] = np.asarray(message.k, dtype=float).reshape(3, 3)
        return callback

    def _projection_for(self, camera: str, frame_id: str, stamp):
        """World->pixel matrix for one camera at one instant, or None."""
        intrinsics = self.intrinsics.get(camera)
        if intrinsics is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(self.world_frame, frame_id, stamp)
        except tf2_ros.TransformException as error:
            self.get_logger().warn(
                f"{camera}: no transform {self.world_frame} <- {frame_id}: {error}",
                throttle_duration_sec=5.0,
            )
            return None
        rotation = transform.transform.rotation
        translation = transform.transform.translation
        return projection_matrix(
            intrinsics,
            quaternion_to_matrix(rotation.x, rotation.y, rotation.z, rotation.w),
            np.array([translation.x, translation.y, translation.z]),
        )

    def _on_detections(self, *messages: Detection2DArray) -> None:
        self.synced += 1
        per_camera = []
        # Parallel to per_camera, so a port can be stamped from exactly the
        # cameras that contributed to it rather than from the whole set.
        stamps = []
        for camera, message in zip(self.cameras, messages):
            frame_id = message.header.frame_id
            if not frame_id:
                continue
            projection = self._projection_for(camera, frame_id, message.header.stamp)
            if projection is None:
                continue
            centres = [
                np.array([d.bbox.center.position.x, d.bbox.center.position.y])
                for d in message.detections
            ]
            per_camera.append((projection, centres))
            stamps.append((message.header.stamp.sec, message.header.stamp.nanosec))

        if len(per_camera) < self.min_views:
            return

        found = match_and_triangulate(
            per_camera,
            max_reprojection_error=self.max_reprojection_error,
            min_views=self.min_views,
            pixel_sigma=self.pixel_sigma,
            max_points=self.max_ports,
        )
        if not found:
            return

        for index, port in enumerate(found[: self.max_ports]):
            seconds, nanoseconds = latest_stamp(stamps[i] for i in port["cameras"])
            self.pose_publishers[index].publish(
                self._to_message(port, int(seconds), int(nanoseconds))
            )
        self.published += 1
        self.last_errors = [port["rms_error"] for port in found]

    def _to_message(self, port, seconds: int, nanoseconds: int):
        pose = PoseWithCovariance()
        position = port["position"]
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation.w = 1.0

        covariance = np.zeros((6, 6))
        covariance[:3, :3] = port["covariance"]
        # Orientation was never estimated; say so loudly instead of implying a
        # confident identity rotation.
        covariance[3, 3] = covariance[4, 4] = covariance[5, 5] = self.orientation_variance
        pose.covariance = covariance.reshape(-1).tolist()

        if not self.publish_stamped:
            return pose
        stamped = PoseWithCovarianceStamped()
        stamped.header.stamp.sec = seconds
        stamped.header.stamp.nanosec = nanoseconds
        # The position is a world-frame point, and saying so is the whole
        # reason for preferring the stamped message.
        stamped.header.frame_id = self.world_frame
        stamped.pose = pose
        return stamped

    def _report(self) -> None:
        if not self.synced:
            self.get_logger().warn(
                "no synchronised detections yet -- check that every camera in "
                "'cameras' publishes <camera>/detections and <camera>/camera_info, "
                "and that sync_slop is wide enough"
            )
            return
        errors = (
            f", last rms {', '.join(f'{e:.2f}' for e in self.last_errors)} px"
            if self.last_errors else ""
        )
        self.get_logger().info(
            f"{self.synced} synchronised sets, {self.published} triangulated{errors}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PortTriangulatorNode()
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
