from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from std_msgs.msg import Header
import rclpy
from rclpy.node import Node
import time

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.subscription = self.create_subscription(
            Int32,
            'sensor_distance',
            self.process_serial_data,
            10
        )
        self.ranges = [0.0] * 2048
        self.current_step = 0
        self.last_scan_time = self.get_clock().now()

    def process_serial_data(self, distance_mm):
        self.get_logger().info(f"Received distance: {distance_mm.data} mm")
        self.ranges[self.current_step] = distance_mm.data / 1000.0  # Convert mm to meters
        self.current_step += 1
        self.publish_scan()

        if self.current_step >= 2048:
           
            self.current_step = 0

        

    def publish_scan(self):
        scan = LaserScan()
        scan.header = Header()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'

        scan.angle_min = 0.0
        scan.angle_max = 2 * 3.14159
        scan.angle_increment = (scan.angle_max - scan.angle_min) / 2048

        rotation_time = 7.13
        scan.time_increment = rotation_time/2048
        scan.scan_time = rotation_time

        scan.range_min = 0.02
        scan.range_max = 8.0
        scan.ranges = self.ranges.copy()

        self.publisher.publish(scan)
        self.last_scan_time = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)
    lidar_publisher = LidarPublisher()
    try:
        rclpy.spin(lidar_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_publisher.destroy_node()
        rclpy.shutdown()
