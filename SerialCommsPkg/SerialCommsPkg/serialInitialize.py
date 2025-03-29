import rclpy
from rclpy.node import Node
from example_interfaces.msg import Bool
import serial

class SerialInitializeNode(Node):
    def __init__(self):
        super().__init__('serial_initialize')
        self.serial_port = '/dev/ttyUSB0' 
        self.baudrate = 115200

        # Initialize the serial connection
        try:
            self.serial_connection = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.get_logger().info(f'Successfully connected to {self.serial_port} at {self.baudrate} baud.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to {self.serial_port}: {e}')
            return
        
        self.toClean_subscriber = self.create_subscription(
            Bool,
            'toClean',
            self.to_clean_callback,
            10
        )
        self.get_logger().info('Subscriber to "toClean" topic initialized.')

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
