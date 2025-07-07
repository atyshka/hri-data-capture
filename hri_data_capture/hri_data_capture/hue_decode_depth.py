import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import huecodec

class HueDepthDecoder(Node):
    def __init__(self):
        super().__init__('hue_depth_decoder')
        self.declare_parameter('input_topic', '/depth/hue_encoded')
        self.declare_parameter('output_topic', '/depth/decoded')
        self.declare_parameter('min_depth', 0.0)
        self.declare_parameter('max_depth', 10.0)
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.min_depth = self.get_parameter('min_depth').get_parameter_value().double_value # Convert to mm
        self.max_depth = self.get_parameter('max_depth').get_parameter_value().double_value
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.input_topic, self.callback, 10)
        self.pub = self.create_publisher(Image, self.output_topic, 10)
        self.get_logger().info(f'Subscribing to {self.input_topic}, publishing to {self.output_topic}')

    def callback(self, msg):
        try:
            # Check for RGB image encoding
            if msg.encoding not in ['rgb8', 'bgr8']:
                self.get_logger().warn(f"Received non-RGB image encoding: {msg.encoding}. Skipping.")
                return
            
            # Convert ROS Image to OpenCV
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            # Use huecodec API to decode the RGB image back to depth
            depth_img = huecodec.rgb2depth(cv_img, zrange=(self.min_depth, self.max_depth))
            
            # Convert back to ROS Image message (32-bit float)
            depth_msg = self.bridge.cv2_to_imgmsg(depth_img, encoding='32FC1')
            depth_msg.header = msg.header
            self.pub.publish(depth_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to decode hue image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = HueDepthDecoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()