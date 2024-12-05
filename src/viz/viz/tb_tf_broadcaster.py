import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
import tf2_ros

class IMUToTFNode(Node):
    def __init__(self):
        super().__init__('imu_to_tf_broadcaster')
        
        # Subscribe to the imu_data topic
        self.subscription = self.create_subscription(
            Imu,
            'imu_data',
            self.imu_callback,
            10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info("IMU to TF broadcaster started")

    def imu_callback(self, msg):
        # Create a TransformStamped message
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'  # Updated parent frame
        transform.child_frame_id = 'imu_link'   # Updated child frame

        # if the first loop, log the current orientation,

        # Set the orientation from the IMU message
        transform.transform.rotation.x = msg.orientation.x
        transform.transform.rotation.y = msg.orientation.y
        transform.transform.rotation.z = msg.orientation.z
        transform.transform.rotation.w = msg.orientation.w

        transform.transform.translation.x = 0.0635
        transform.transform.translation.y = 0.0635
        transform.transform.translation.z = 0.0635

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = IMUToTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
