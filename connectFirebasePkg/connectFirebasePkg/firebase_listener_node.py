import rclpy
from rclpy.node import Node
from example_interfaces.msg import Bool
from geometry_msgs.msg import Twist
import firebase_admin
from firebase_admin import credentials, db
import atexit

class FirebaseListenerNode(Node):
    def __init__(self):
        super().__init__('firebase_listener_node')

        cred = credentials.Certificate('/home/josh/robot-ws/src/connectFirebasePkg/credentials.json')
        firebase_admin.initialize_app(cred, {
            'databaseURL': 'https://testing-a04d9-default-rtdb.firebaseio.com/'
        })
        
        
        # This is from firebase's database.
        self.clean_ref = db.reference('robot_commands/Clean')
        self.vacuum_ref = db.reference('features/vacuum')
        self.wiper_ref = db.reference('features/wiper')


        self.toClean_publisher = self.create_publisher(Bool, 'toClean', 10)
        self.toVacuum_publisher = self.create_publisher(Bool, 'toVacuum', 10)
        self.toWiper_publisher = self.create_publisher(Bool, 'toWiper', 10)

        # Check for updates
        self.clean_ref.listen(self.on_clean_changed)
        self.vacuum_ref.listen(self.on_vacuum_changed)
        self.wiper_ref.listen(self.on_wiper_changed)

        # Set initial connection status to true
        self.update_status()

        # Register the shutdown handler using atexit.
        atexit.register(self.handle_shutdown)

        # Update status every 60 seconds
        self.status_timer = self.create_timer(60.0, self.update_status)

    def on_clean_changed(self, event):


        try:
            # Get the current value from Firebase
            Clean = event.data
            
            # Check if the value is None
            # If it is None, set Clean to False
            if Clean is None:
                self.get_logger().info('Clean value is None. Defaulting to False.')
                Clean = False
            
            # Log the received value
            self.get_logger().info(f'To clean status updated: {Clean}')

            # Create a message to publish
            msg = Bool()
            msg.data = Clean

            # If value is True, publish the message
            if Clean is True:
                self.get_logger().info('Clean message is sent.')
                self.toClean_publisher.publish(msg)
            elif Clean is False:
                self.get_logger().info('Stopping cleaning functions.')
                self.toClean_publisher.publish(msg)   
                
        except Exception as e:
            self.get_logger().error(f"Error processing 'Clean' data: {e}")

    def on_vacuum_changed(self, event):
        try:
            vacuum_state = event.data
            if vacuum_state is None:
                self.get_logger().info('Vacuum value is None. Defaulting to False.')
                vacuum_state = False
            self.get_logger().info(f'To vacuum status updated: {vacuum_state}')
            msg = Bool()
            msg.data = vacuum_state
            
            if vacuum_state is True:
                self.get_logger().info('Vacuum message is sent.')
                self.toVacuum_publisher.publish(msg)
            elif vacuum_state is False:
                self.get_logger().info('Stopping vacuum functions.')
                self.toVacuum_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error processing 'Vacuum' data: {e}")

    def on_wiper_changed(self, event):
        try:
            wiper_state = event.data
            if wiper_state is None:
                self.get_logger().info('Wiper value is None. Defaulting to False.')
                wiper_state = False
            self.get_logger().info(f'To wiper status updated: {wiper_state}')
            msg = Bool()
            msg.data = wiper_state
            
            if wiper_state is True:
                self.get_logger().info('Wiper message is sent.')
                self.toWiper_publisher.publish(msg)
            elif wiper_state is False:
                self.get_logger().info('Stopping wiper functions.')
                self.toWiper_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error processing 'Wiper' data: {e}")


    def update_status(self):
        ref = db.reference('device_status/connected')
        ref.set(True)
    
    def handle_shutdown(self):
    # Disconnect from Firebase
        try:
            ref = db.reference('device_status/connected')
            ref.set(False)
            self.get_logger().info('Disconnected from Firebase.')
        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = FirebaseListenerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.handle_shutdown()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()