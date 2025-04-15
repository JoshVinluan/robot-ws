import rclpy
from rclpy.node import Node
from example_interfaces.msg import Bool
from std_msgs.msg import Int32
from geometry_msgs.msg import PointStamped, Twist
import serial
import threading

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
        self.point_publisher = self.create_publisher(
            PointStamped,
            'sensor_point',
            10
        )
        #### END OF: Publishers for sensor readings ####

        #### Start a thread to continuously read from the serial port ####
        self.read_thread = threading.Thread(target=self.tofScan)
        self.read_thread.daemon = True
        self.read_thread.start()
        #### END OF: Start a thread to continuously read from the serial port ####

    def to_clean_callback(self, msg: Bool):

        # Send the message to ESP32 via serial
        try:
            if self.serial_connection.is_open:
                message = f"Bool message received: {msg.data}\n"
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

                                    # Publish PointStamped message
                                    point_msg = PointStamped()
                                    point_msg.header.stamp = self.get_clock().now().to_msg()
                                    point_msg.header.frame_id = "laser_frame"
                                    point_msg.point.x = 0.0
                                    point_msg.point.y = 0.0
                                    point_msg.point.z = float(distance) / 1000.0  # Convert mm to meters
                                    self.point_publisher.publish(point_msg)                                
                                except ValueError:
                                    self.get_logger().error(f"Invalid distance value: {data}")
                    
                            '''
                            if data.startswith("Bool message received:"):
                                bool_value = data.split(":")[-1].strip().lower() == "true"
                                msg = Bool()
                                msg.data = bool_value
                                #self.fromESP32_publisher.publish(msg)
                                self.get_logger().info(f"Published Bool message: {msg.data}")'
                            '''
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
