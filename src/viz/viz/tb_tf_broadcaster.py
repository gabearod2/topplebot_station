import rclpy
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
import tf2_ros

class IMUToTFNode(Node):
    def __init__(self):
        super().__init__('imu_to_tf_broadcaster')
        self.first_quaternion_logged = False
        
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
        transform.header.frame_id = 'world'  # Updated parent frame
        transform.child_frame_id = 'base_link'   # Updated child frame

        # x y z
        self.initial_translation_vector = np.array([0.0635, 0.0635, 0.0635])

        # Getting the current quaternion
        self.current_quat = np.array([
            msg.orientation.y,
            msg.orientation.x,
            -msg.orientation.z,
            msg.orientation.w])

        # if the first loop, log the current orientation as the first 
        if not self.first_quaternion_logged:
            self.first_quaternion = self.current_quat
            self.first_quaternion_logged = True
        
        # Get the initial rotation
        self.initial_r = R.from_quat(self.first_quaternion)

        # Get the inverse of that initial rotation
        self.initial_inv_r = self.initial_r.inv()

        # Get rotation from euler of 90 deg

        # Getting the current rotation, neglecting the inital IMU orientation
        self.current_r = R.from_quat(self.current_quat)*self.initial_inv_r

        # Applying the rotation to the intial translation vector
        self.current_transform = self.current_r.apply(self.initial_translation_vector)

        # Getting the quaternion from the current orientation - scalar last as default
        self.current_orientation = R.as_quat(self.current_r)

        # Setting the actual transformation now
        transform.transform.rotation.x = self.current_orientation[0]
        transform.transform.rotation.y = self.current_orientation[1]
        transform.transform.rotation.z = self.current_orientation[2]
        transform.transform.rotation.w = self.current_orientation[3]

        transform.transform.translation.x = self.current_transform[0]
        transform.transform.translation.y = self.current_transform[1]
        transform.transform.translation.z = self.current_transform[2]

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
