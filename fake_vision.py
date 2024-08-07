# Import necessary libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from auto_aim_interfaces.msg import Target
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class TransformLookup(Node):
    def __init__(self):
        super().__init__('transform_lookup')
        self.publisher = self.create_publisher(Target, '/tracker/target', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.publish_transform)  # 20Hz

    def publish_transform(self):
        try:
            # Lookup the transform
            transform = self.tf_buffer.lookup_transform('yaw_link', 'enemy', rclpy.time.Time())
            
            # Publish the transform
            target = Target()
            target.header = transform.header
            target.tracking = True
            target.id='1'
            target.position.x = transform.transform.translation.x
            target.position.y = transform.transform.translation.y
            target.position.z = transform.transform.translation.z
            self.publisher.publish(target)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not transform: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TransformLookup()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()