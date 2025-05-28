import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from ffmpeg_image_transport_msgs.msg import FFMPEGPacket
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions
from rosbag2_py._storage import TopicMetadata
from datetime import datetime
import argparse
from rclpy.serialization import serialize_message
import os
import time

class MultiRGBDToBagRecorder(Node):
    def __init__(self, cam_names, bag_base_name='rgbd_bag', compressed=False):
        super().__init__('multi_rgbd_to_bag_recorder')
        dt_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.bag_name = f"{bag_base_name}_{dt_str}"
        self.get_logger().info(f'Bag file will be saved as: {self.bag_name}')
        self.writer = SequentialWriter()
        storage_options = StorageOptions(uri=self.bag_name, storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        self.writer.open(storage_options, converter_options)
        self._subscriptions = []
        
        # Record start time
        self.start_time = time.time()
        
        for cam_name in cam_names:
            
            if compressed:
                rgb_topic = f'/{cam_name}/rgb/image_raw/compressed'
                depth_topic = f'/{cam_name}/hue_encoded_depth/ffmpeg'
            else:
                rgb_topic = f'/{cam_name}/rgb/image_raw'
                depth_topic = f'/{cam_name}/depth/image_raw'
        
            rgb_type = 'sensor_msgs/msg/CompressedImage' if compressed else 'sensor_msgs/msg/Image'
            self.writer.create_topic(TopicMetadata(
                id=0,
                name=rgb_topic,
                type=rgb_type,
                serialization_format='cdr'
            ))

            depth_type = 'ffmpeg_image_transport_msgs/msg/FFMPEGPacket' if compressed else 'sensor_msgs/msg/Image'
            self.writer.create_topic(TopicMetadata(
                id=0,
                name=depth_topic,
                type=depth_type,
                serialization_format='cdr'
            ))

            self._subscriptions.append(
                self.create_subscription(CompressedImage if compressed else Image, rgb_topic, self.make_callback(rgb_topic), 10)
            )
            self._subscriptions.append(
                self.create_subscription(FFMPEGPacket if compressed else Image, depth_topic, self.make_callback(depth_topic), 10)
            )
            # Subscribe to CameraInfo topics and log info
            rgb_info_topic = f'/{cam_name}/rgb/camera_info'
            depth_info_topic = f'/{cam_name}/depth/camera_info'

            self.writer.create_topic(TopicMetadata(
                id=0,
                name=rgb_info_topic,
                type='sensor_msgs/msg/CameraInfo',
                serialization_format='cdr'
            ))
            self.writer.create_topic(TopicMetadata(
                id=0,
                name=depth_info_topic,
                type='sensor_msgs/msg/CameraInfo',
                serialization_format='cdr'
            ))

            self._subscriptions.append(
                self.create_subscription(CameraInfo, rgb_info_topic, self.make_callback(rgb_info_topic), 10)
            )
            self._subscriptions.append(
                self.create_subscription(CameraInfo, depth_info_topic, self.make_callback(depth_info_topic), 10)
            )
        self.get_logger().info(f'Recording RGB and Depth for cameras: {cam_names}')
        # self.context.on_shutdown(self.print_stats)

    def make_callback(self, topic_name):
        def callback(msg):
            # Serialize the message before writing
            serialized_msg = serialize_message(msg)
            self.writer.write(topic_name, serialized_msg, self.get_clock().now().nanoseconds)
        return callback
    
    def print_stats(self):
        """Calculate and print elapsed time and bag size"""
        # Calculate elapsed time
        elapsed_time = time.time() - self.start_time
        hours, remainder = divmod(elapsed_time, 3600)
        minutes, seconds = divmod(remainder, 60)
        
        # Calculate bag size
        total_size = 0
        bag_path = self.bag_name
        
        if os.path.isdir(bag_path):
            # If bag is a directory (standard for ROS2 bags), sum all files
            for dirpath, dirnames, filenames in os.walk(bag_path):
                for filename in filenames:
                    file_path = os.path.join(dirpath, filename)
                    total_size += os.path.getsize(file_path)
        elif os.path.isfile(bag_path):
            # If bag is a single file
            total_size = os.path.getsize(bag_path)
            
        # Convert to human-readable format
        size_mb = total_size / (1024 * 1024)
        
        # Print stats
        print(f"Recording statistics:")
        print(f"- Elapsed time: {int(hours)}h {int(minutes)}m {seconds:.2f}s")
        print(f"- Bag file size: {size_mb:.2f} MB")
        print(f"- MB per minute: {size_mb * 60 / elapsed_time:.2f} MB/min" if elapsed_time > 0 else "MB per minute: N/A (no data recorded)")

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument('--cameras', required=True, help='Comma-separated list of camera namespaces')
    parser.add_argument('--bag_base_name', default='rgbd_bag')
    parser.add_argument('--compressed', action='store_true', help='Enable compressed recording')
    parsed_args, unknown = parser.parse_known_args()
    cam_names = [s.strip() for s in parsed_args.cameras.split(',') if s.strip()]
    recorder = MultiRGBDToBagRecorder(cam_names, parsed_args.bag_base_name, parsed_args.compressed)
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.print_stats()
        # recorder.get_logger().info("Keyboard interrupt received, shutting down...")
        #recorder.print_stats()  # Print elapsed time and bag size on shutdown

if __name__ == '__main__':
    main()
