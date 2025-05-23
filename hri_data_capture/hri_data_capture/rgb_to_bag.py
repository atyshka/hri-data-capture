import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions
from rosbag2_py._storage import TopicMetadata
from datetime import datetime

class RGBToBagRecorder(Node):
    def __init__(self):
        super().__init__('rgb_to_bag_recorder')
        # Declare and get bag_base_name parameter
        self.declare_parameter('bag_base_name', 'rgb_bag')
        bag_base_name = self.get_parameter('bag_base_name').get_parameter_value().string_value
        # Append datetime suffix to bag name
        dt_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_name = f"{bag_base_name}_{dt_str}"
        self.get_logger().info(f'Bag file will be saved as: {bag_name}')
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.listener_callback,
            10
        )
        self.writer = SequentialWriter()
        storage_options = StorageOptions(uri=bag_name, storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        self.writer.open(storage_options, converter_options)
        self.writer.create_topic(
            TopicMetadata(
                name='/camera/rgb/image_raw',
                type='sensor_msgs/msg/Image',
                serialization_format='cdr'
            )
        )
        self.get_logger().info('RGB to Bag Recorder initialized.')

    def listener_callback(self, msg):
        self.writer.write('/camera/rgb/image_raw', msg, self.get_clock().now().nanoseconds)
        self.get_logger().info('Image written to bag file.')

def main(args=None):
    rclpy.init(args=args)
    recorder = RGBToBagRecorder()
    rclpy.spin(recorder)
    recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()