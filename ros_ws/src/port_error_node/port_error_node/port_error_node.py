"""Score the triangulated port estimates against the transform tree.

Reads the true port poses from TF and the estimates from
``port_triangulator_node``, pairs them **one-to-one**, and publishes the
absolute distance for each port on ``~/port_<n>/error`` as a
``std_msgs/Float64`` in metres.

The pairing is the substance. Letting each port take its nearest estimate
independently allows both ports to score against the same one, which makes the
reported error look *better* the worse the estimator is doing: one good
estimate would be counted twice and the missing one never counted at all. This
node solves a one-to-one assignment instead, minimising the total distance, so
every estimate is spent once and a port with nothing left to pair against is
reported as unmatched rather than borrowing its neighbour's.

Runs under the venv interpreter like the other nodes -- see ``./run.sh error``.
"""

from __future__ import annotations

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64

from .assignment import match_errors

DEFAULT_TRUTH_FRAMES = ["sfp_port_0_entrance", "sfp_port_1_entrance"]
DEFAULT_ESTIMATE_TOPICS = [
    "/port_triangulator_node/port_0/pose",
    "/port_triangulator_node/port_1/pose",
]


class PortErrorNode(Node):
    def __init__(self):
        super().__init__("port_error_node")

        self.declare_parameter("truth_frames", DEFAULT_TRUTH_FRAMES)
        self.declare_parameter("estimate_topics", DEFAULT_ESTIMATE_TOPICS)
        self.declare_parameter("world_frame", "world")
        # Estimates for the two ports are stamped independently (each carries
        # the newest detection that fed it), so they rarely share a stamp
        # exactly. Anything within this window counts as the same instant.
        self.declare_parameter("sync_window", 0.05)
        self.declare_parameter("report_period", 10.0)

        self.truth_frames = list(self.get_parameter("truth_frames").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.sync_window = float(self.get_parameter("sync_window").value)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.error_publishers = [
            self.create_publisher(Float64, f"~/port_{index}/error", 10)
            for index in range(len(self.truth_frames))
        ]

        #: topic -> (stamp_seconds, position). Only the newest per topic is kept.
        self.latest: dict[str, tuple[float, np.ndarray]] = {}
        self.subscriptions_by_topic = []
        for topic in list(self.get_parameter("estimate_topics").value):
            self.subscriptions_by_topic.append(
                self.create_subscription(
                    PoseWithCovarianceStamped, topic,
                    self._make_callback(topic), 10,
                )
            )

        self.samples = 0
        self.unmatched = 0
        self.errors_seen: list[float] = []
        self.create_timer(float(self.get_parameter("report_period").value), self._report)
        self.get_logger().info(
            f"scoring {self.truth_frames} against {len(self.subscriptions_by_topic)} "
            f"estimate topics, one-to-one"
        )

    def _make_callback(self, topic: str):
        def callback(message: PoseWithCovarianceStamped) -> None:
            position = message.pose.pose.position
            stamp = Time.from_msg(message.header.stamp)
            self.latest[topic] = (
                stamp.nanoseconds * 1e-9,
                np.array([position.x, position.y, position.z]),
            )
            self._evaluate(message.header.stamp)
        return callback

    def _truth_positions(self, stamp):
        """Every port's true position at ``stamp``, or None if any is missing.

        All-or-nothing on purpose: scoring one port against a transform from a
        different instant is the error this project keeps paying for.
        """
        positions = []
        for frame in self.truth_frames:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.world_frame, frame, stamp
                )
            except tf2_ros.TransformException as error:
                self.get_logger().warn(
                    f"no transform {self.world_frame} <- {frame}: {error}",
                    throttle_duration_sec=5.0,
                )
                return None
            translation = transform.transform.translation
            positions.append(np.array([translation.x, translation.y, translation.z]))
        return positions

    def _evaluate(self, stamp) -> None:
        if not self.latest:
            return
        newest = max(seconds for seconds, _ in self.latest.values())
        # Only estimates from the same instant take part; a stale one left over
        # from an earlier cycle would otherwise be matched against fresh truth.
        estimates = [
            position for seconds, position in self.latest.values()
            if newest - seconds <= self.sync_window
        ]
        truths = self._truth_positions(stamp)
        if truths is None:
            return

        errors = match_errors(truths, estimates)
        for index, error in enumerate(errors):
            if error is None:
                self.unmatched += 1
                continue
            self.error_publishers[index].publish(Float64(data=error))
            self.errors_seen.append(error)
        self.samples += 1

    def _report(self) -> None:
        if not self.errors_seen:
            self.get_logger().warn(
                "no errors computed yet -- is port_triangulator_node publishing, "
                "and does /tf carry the truth frames?"
            )
            return
        errors = np.array(self.errors_seen)
        self.get_logger().info(
            f"{self.samples} sets scored, {len(errors)} port errors: "
            f"median {np.median(errors) * 1000:.2f} mm, "
            f"p90 {np.percentile(errors, 90) * 1000:.2f} mm, "
            f"max {errors.max() * 1000:.2f} mm, {self.unmatched} unmatched"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PortErrorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
