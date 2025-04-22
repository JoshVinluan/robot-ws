import rclpy
from rclpy.node import Node
from example_interfaces.msg import Bool
from geometry_msgs.msg import Twist
import firebase_admin
from firebase_admin import credentials, db

class FirebaseListenerNode(Node):
    def __init__(self):
        super().__init__('firebase_listener_node')

        cred = credentials.Certificate('/home/josh/robot-ws/src/connectFirebasePkg/credentials.json')
        firebase_admin.initialize_app(cred, {
            'databaseURL': 'https://testing-a04d9-default-rtdb.firebaseio.com/'
        })
        
        
        # This is from firebase's database.
        self.ref = db.reference('robot_commands/Clean')

        self.toClean_publisher = self.create_publisher(Bool, 'toClean', 10)

        # Check for updates
        self.ref.listen(self.on_clean_changed)

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
            # This message is from the example_interfaces package.
            # It is a boolean message.
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
            self.get_logger().error(f'Error processing Firebase data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = FirebaseListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()