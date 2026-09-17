import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from std_srvs.srv import Trigger

import tf2_ros
import rosbag2_py

#: Frames resolved against ``world`` at the captured image's stamp. These are
#: what the extractor and the bounding-box projection consume.
TRACKED_FRAMES = (
    'center_camera_optical',
    'left_camera_optical',
    'right_camera_optical',
    'sfp_port_0_entrance',
    'sfp_port_1_entrance',
)
WORLD_FRAME = 'world' 

class BagRecorderNode(Node):
    def __init__(self):
        super().__init__('bag_recorder_node')
        self.record_service = self.create_service(Trigger, '/record_rosbag', self.record_callback)
        self.center_image_subscriber = self.create_subscription(Image, '/center_camera/image', self.center_image_callback, 1)
        self.center_camera_info_subscriber = self.create_subscription(CameraInfo, '/center_camera/camera_info', self.center_camera_info_callback, 1)
        self.left_image_subscriber = self.create_subscription(Image, '/left_camera/image', self.left_image_callback, 1)
        self.left_camera_info_subscriber = self.create_subscription(CameraInfo, '/left_camera/camera_info', self.left_camera_info_callback, 1)
        self.right_image_subscriber = self.create_subscription(Image, '/right_camera/image', self.right_image_callback, 1)
        self.right_camera_info_subscriber = self.create_subscription(CameraInfo, '/right_camera/camera_info', self.right_camera_info_callback, 1)
        # /tf is consumed through a tf2 buffer rather than latched raw: the
        # cameras ride the arm, so the newest tree describes a slightly
        # different pose than the newest image. The buffer lets each recording
        # resolve transforms at its own image's stamp instead.
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.center_image = None
        self.center_camera_info = None
        self.left_image = None
        self.left_camera_info = None
        self.right_image = None
        self.right_camera_info = None

        # rosbag2 refuses to reopen an existing bag directory, so each instance
        # gets its own timestamped uri instead of a fixed one.
        self.bag_uri = 'rosbag_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f')

        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(
            uri=self.bag_uri,
            storage_id='mcap')
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)
        self.get_logger().info('Recording to ' + os.path.abspath(self.bag_uri))

        topic_info = rosbag2_py.TopicMetadata(
            id=0,
            name='/center_camera/image',
            type='sensor_msgs/msg/Image',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

        topic_info = rosbag2_py.TopicMetadata(
            id=1,
            name='/center_camera/camera_info',
            type='sensor_msgs/msg/CameraInfo',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

        topic_info = rosbag2_py.TopicMetadata(
            id=2,
            name='/left_camera/image',
            type='sensor_msgs/msg/Image',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

        topic_info = rosbag2_py.TopicMetadata(
            id=3,
            name='/left_camera/camera_info',
            type='sensor_msgs/msg/CameraInfo',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

        topic_info = rosbag2_py.TopicMetadata(
            id=4,
            name='/right_camera/image',
            type='sensor_msgs/msg/Image',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

        topic_info = rosbag2_py.TopicMetadata(
            id=5,
            name='/right_camera/camera_info',
            type='sensor_msgs/msg/CameraInfo',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

        topic_info = rosbag2_py.TopicMetadata(
            id=6,
            name='/tf',
            type='tf2_msgs/msg/TFMessage',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

    def center_image_callback(self, msg):
        self.center_image = msg

    def center_camera_info_callback(self, msg):
        self.center_camera_info = msg

    def left_image_callback(self, msg):
        self.left_image = msg

    def left_camera_info_callback(self, msg):
        self.left_camera_info = msg

    def right_image_callback(self, msg):
        self.right_image = msg

    def right_camera_info_callback(self, msg):
        self.right_camera_info = msg

    def lookup_tf_at(self, stamp):
        """Resolve every tracked frame against world at ``stamp``.

        Returns a TFMessage, or None with a reason if any frame is unavailable
        at that time -- a partial tree would silently mislabel a sample.
        """
        transforms = []
        for frame in TRACKED_FRAMES:
            try:
                transforms.append(
                    self.tf_buffer.lookup_transform(WORLD_FRAME, frame, stamp)
                )
            except tf2_ros.TransformException as error:
                return None, f'{frame}: {error}'
        message = TFMessage()
        message.transforms = transforms
        return message, None

    def record_callback(self, request, response):
        images = {
            '/center_camera/image': self.center_image,
            '/center_camera/camera_info': self.center_camera_info,
            '/left_camera/image': self.left_image,
            '/left_camera/camera_info': self.left_camera_info,
            '/right_camera/image': self.right_image,
            '/right_camera/camera_info': self.right_camera_info,
        }

        missing = [topic for topic, message in images.items() if message is None]
        if missing:
            response.success = False
            response.message = 'Nothing recorded; missing messages on: ' + ', '.join(missing)
            self.get_logger().warn(response.message)
            return response

        # The captured image defines the instant; the transforms are resolved
        # to match it rather than being whatever /tf happened to publish last.
        capture_stamp = self.center_image.header.stamp
        tf_message, reason = self.lookup_tf_at(capture_stamp)
        if tf_message is None:
            response.success = False
            response.message = f'Nothing recorded; no transform at image stamp ({reason})'
            self.get_logger().warn(response.message)
            return response

        messages = dict(images)
        messages['/tf'] = tf_message

        timestamp = self.get_clock().now().nanoseconds
        for topic, message in messages.items():
            self.writer.write(topic, serialize_message(message), timestamp)

        response.success = True
        response.message = 'Recorded data to rosbag.'
        return response

def main(args=None):
    rclpy.init(args=args)
    bag_recorder_node = BagRecorderNode()
    try:
        rclpy.spin(bag_recorder_node)
    except:
        pass
    finally:
        bag_recorder_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()