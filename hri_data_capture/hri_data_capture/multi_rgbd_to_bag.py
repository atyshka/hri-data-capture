import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions
from rosbag2_py._storage import TopicMetadata
from datetime import datetime
import argparse
from rclpy.serialization import serialize_message  # Add this import

class MultiRGBDToBagRecorder(Node):
    def __init__(self, cam_names, bag_base_name='rgbd_bag'):
        super().__init__('multi_rgbd_to_bag_recorder')
        dt_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_name = f"{bag_base_name}_{dt_str}"
        self.get_logger().info(f'Bag file will be saved as: {bag_name}')
        self.writer = SequentialWriter()
        storage_options = StorageOptions(uri=bag_name, storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        self.writer.open(storage_options, converter_options)
        self._subscriptions = []
        for cam_name in cam_names:
            rgb_topic = f'/{cam_name}/rgb/image_raw'
            depth_topic = f'/{cam_name}/depth/image_raw'
            self.writer.create_topic(TopicMetadata(
                id=0,
                name=rgb_topic,
                type='sensor_msgs/msg/Image',
                serialization_format='cdr'
            ))
            self.writer.create_topic(TopicMetadata(
                id=0,
                name=depth_topic,
                type='sensor_msgs/msg/Image',
                serialization_format='cdr'
            ))
            self._subscriptions.append(
                self.create_subscription(Image, rgb_topic, self.make_callback(rgb_topic), 10)
            )
            self._subscriptions.append(
                self.create_subscription(Image, depth_topic, self.make_callback(depth_topic), 10)
            )
        self.get_logger().info(f'Recording RGB and Depth for cameras: {cam_names}')

    def make_callback(self, topic_name):
        def callback(msg):
            # Serialize the message before writing
            serialized_msg = serialize_message(msg)
            self.writer.write(topic_name, serialized_msg, self.get_clock().now().nanoseconds)
        return callback

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument('--cameras', required=True, help='Comma-separated list of camera namespaces')
    parser.add_argument('--bag_base_name', default='rgbd_bag')
    parsed_args, unknown = parser.parse_known_args()
    cam_names = [s.strip() for s in parsed_args.cameras.split(',') if s.strip()]
    recorder = MultiRGBDToBagRecorder(cam_names, parsed_args.bag_base_name)
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info("Keyboard interrupt received, shutting down...")



if __name__ == '__main__':
    main()
