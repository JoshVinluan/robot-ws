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

        #### Publishers for sensor readings ####
        self.tof_publisher = self.create_publisher(
            Int32,
            'sensor_distance',
            10
        )

        #### Start a thread to continuously read from the serial port ####
        self.read_thread = threading.Thread(target=self.tofScan)
        self.read_thread.daemon = True
        self.read_thread.start()
        #### END OF: Start a thread to continuously read from the serial port ####

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
    
    def tofScan(self):
        # Continuously read from the serial port
        while rclpy.ok():
            try:
                if self.serial_connection.is_open:
                    raw_data = self.serial_connection.readline()
                    try:
                        data = raw_data.decode('utf-8', errors='ignore').strip()
                        if data:
                            self.get_logger().info(f"Received from ESP32: {data}")

                            # Parse sensor readings and publish to ROS2 topic
                            if data.startswith("Distance:"):
                                try:
                                    # Extract the part after "Distance:"
                                    distance_str = data.split(":")[-1].strip()
                                    # Remove the " mm" suffix and convert to integer
                                    distance = int(distance_str.replace(" mm", "").strip())
                                    msg = Int32()
                                    msg.data = distance
                                    self.tof_publisher.publish(msg)
                                    self.get_logger().info(f"Published sensor distance: {msg.data}")
    
                                except ValueError:
                                    self.get_logger().error(f"Invalid distance value: {data}")
                    

                    except UnicodeDecodeError as e:
                        self.get_logger().error(f"Decoding error: {e}")
                    
                else:
                    self.get_logger().error("Serial connection is not open.")
                    break
            except serial.SerialException as e:
                self.get_logger().error(f"Error reading from serial port: {e}")
                break
    

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
