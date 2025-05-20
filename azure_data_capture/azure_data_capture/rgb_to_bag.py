import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions

class RGBToBagRecorder(Node):
    def __init__(self):
        super().__init__('rgb_to_bag_recorder')
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.listener_callback,
            10
        )
        self.writer = SequentialWriter()
        storage_options = StorageOptions(uri='rgb_bag', storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        self.writer.open(storage_options, converter_options)
        self.writer.create_topic({'name': '/camera/rgb/image_raw', 'type': 'sensor_msgs/msg/Image', 'serialization_format': 'cdr'})
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