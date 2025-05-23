import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import huecodec

class HueDepthEncoder(Node):
    def __init__(self):
        super().__init__('hue_depth_encoder')
        self.declare_parameter('input_topic', '/depth/image_raw')
        self.declare_parameter('output_topic', '/depth/hue_encoded')
        self.declare_parameter('min_depth', 0.0)
        self.declare_parameter('max_depth', 10.0)
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.min_depth = self.get_parameter('min_depth').get_parameter_value().double_value * 1000.0  # Convert to mm
        self.max_depth = self.get_parameter('max_depth').get_parameter_value().double_value * 1000.0
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.input_topic, self.callback, 10)
        self.pub = self.create_publisher(Image, self.output_topic, 10)
        self.get_logger().info(f'Subscribing to {self.input_topic}, publishing to {self.output_topic}')

    def callback(self, msg):
        try:
            # Check for depth image encoding
            if msg.encoding not in [
                '16UC1', '32FC1', 'mono16', 'type_16UC1', 'type_32FC1', 'TYPE_16UC1', 'TYPE_32FC1']:
                self.get_logger().warn(f"Received non-depth image encoding: {msg.encoding}. Skipping.")
                return
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # Clip depth values
            mask = (cv_img >= self.min_depth) & (cv_img <= self.max_depth)
            clipped = np.where(mask, cv_img, 0)
            # Use huecodec API: hue_img = huecodec.encode(depth_img, min_value, max_value)
            hue_img = huecodec.depth2rgb(clipped, zrange=(self.min_depth, self.max_depth))
            # Convert back to ROS Image message
            hue_msg = self.bridge.cv2_to_imgmsg(hue_img, encoding='rgb8')
            hue_msg.header = msg.header
            self.pub.publish(hue_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to encode depth image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = HueDepthEncoder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
