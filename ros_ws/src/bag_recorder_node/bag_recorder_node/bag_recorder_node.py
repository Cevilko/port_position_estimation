import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from std_srvs.srv import Trigger

import rosbag2_py

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
        self.tf_subscriber = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        
        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(
            uri='maj_beg',
            storage_id='mcap')
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

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

    def tf_callback(self, msg):
        self.tf_message = msg

    def record_callback(self, request, response):
        timestamp = self.get_clock().now().nanoseconds

        self.writer.write(
            '/center_camera/image',
            serialize_message(self.center_image),
            timestamp)

        self.writer.write(
            '/center_camera/camera_info',
            serialize_message(self.center_camera_info),
            timestamp)

        self.writer.write(
            '/left_camera/image',
            serialize_message(self.left_image),
            timestamp)

        self.writer.write(
            '/left_camera/camera_info',
            serialize_message(self.left_camera_info),
            timestamp)

        self.writer.write(
            '/right_camera/image',
            serialize_message(self.right_image),
            timestamp)

        self.writer.write(
            '/right_camera/camera_info',
            serialize_message(self.right_camera_info),
            timestamp)

        self.writer.write(
            '/tf',
            serialize_message(self.tf_message),
            timestamp)

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