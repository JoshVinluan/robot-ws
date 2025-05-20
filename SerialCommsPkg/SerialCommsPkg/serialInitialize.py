import rclpy
from rclpy.node import Node
from example_interfaces.msg import Bool
from std_msgs.msg import Int32
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import LaserScan
import serial
import threading
import math
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class SerialInitializeNode(Node):
    def __init__(self):
        super().__init__('serial_initialize')

        possible_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1']
        self.baudrate = 115200
        self.serial_connection = None

        for port in possible_ports:
            try:
                self.serial_connection = serial.Serial(port, self.baudrate, timeout=1)
                self.serial_port = port
                self.get_logger().info(f'Successfully connected to {self.serial_port} at {self.baudrate} baud.')
                break
            except serial.SerialException as e:
                self.get_logger().warn(f'Failed to connect to {port}: {e}')
        
        if self.serial_connection is None:
            self.get_logger().error('Failed to connect to any serial port.')
            return

        #### Subscriber to "toClean" topic ####
        self.toClean_subscriber = self.create_subscription(
            Bool,
            'toClean',
            self.to_clean_callback,
            10
        )
        self.get_logger().info('Subscriber to "toClean" topic initialized.')
        #### END OF: Subscriber to "toClean" topic ####

        #### Subscribe to "toVacuum" ####
        self.toVacuum_subscriber = self.create_subscription(
            Bool,
            'toVacuum',
            self.to_vacuum_callback,
            10
        )
        self.get_logger().info('Subscriber to "toVacuum" topic initialized.')
        #### END OF: Subscribe to "toVacuum" ####

        #### Subscribe to "toWiper" ####
        self.toWiper_subscriber = self.create_subscription(
            Bool,
            'toWiper',
            self.to_wiper_callback,
            10
        )
        self.get_logger().info('Subscriber to "toWiper" topic initialized.')
        #### END OF: Subscribe to "toWiper" ####

        self.cmd_vel_stamped_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info('Subscriber to "cmd_vel" topic initialized.')
        #### END OF: Subscriber to "cmd_vel_stamped" topic ####


    def to_vacuum_callback(self, msg: Bool):
        try:
            if self.serial_connection.is_open:
                if msg.data:
                    message = "Vacuum message received: True\n"
                else:
                    message = "Vacuum message received: False\n"
                self.serial_connection.write(message.encode())
                self.get_logger().info(f"Sent to ESP32: {message.strip()}")
            else:
                self.get_logger().error("Serial connection is not open.")
        except Exception as e:
            self.get_logger().error(f"Error writing to serial port: {e}")

    def to_wiper_callback(self, msg: Bool):
        try:
            if self.serial_connection.is_open:
                if msg.data:
                    message = "Wiper message received: True\n"
                else:
                    message = "Wiper message received: False\n"
                self.serial_connection.write(message.encode())
                self.get_logger().info(f"Sent to ESP32: {message.strip()}")
            else:
                self.get_logger().error("Serial connection is not open.")
        except Exception as e:
            self.get_logger().error(f"Error writing to serial port: {e}")

    def to_clean_callback(self, msg: Bool):
        # Send the message to ESP32 via serial
        try:
            if self.serial_connection.is_open:
                if msg.data:
                    message = "Clean message received: True\n"
                else:
                    message = "Clean message received: False\n"
                self.serial_connection.write(message.encode())
                self.get_logger().info(f"Sent to ESP32: {message.strip()}")
            else:
                self.get_logger().error("Serial connection is not open.")
        except Exception as e:
            self.get_logger().error(f"Error writing to serial port: {e}")
    
    def cmd_vel_callback(self, msg: Twist):
        # Send velocity commands to ESP32 via serial
        try:
            if self.serial_connection.is_open:
                message = f"vlin:{msg.linear.x},vang:{msg.angular.z}\n"
                self.serial_connection.write(message.encode())
                self.get_logger().info(f"Sent to ESP32: {message.strip()}")
            else:
                self.get_logger().error("Unable to send velocity comamnds. Serial connection is not open.")
        except Exception as e:
            self.get_logger().error(f"Error writing to serial port: {e}")
    
    
    

def main(args=None):
    rclpy.init(args=args)
    node = SerialInitializeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node.')
    finally:
        if hasattr(node, 'serial_connection') and node.serial_connection.is_open:
            node.serial_connection.close()
            node.get_logger().info('Serial connection closed.')
        rclpy.shutdown()

if __name__ == '__main__':
    main()
