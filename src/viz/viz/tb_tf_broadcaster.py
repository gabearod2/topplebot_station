import rclpy
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
import tf2_ros

'''
This node publishes the Odometry for the robot. 

Needs the following debugging:
TODO: Ensure the quaternion is handled correctly
TODO: Add logging at each step
TODO: Edit urdf so that all frames are visualized
TODO: Ensure the calibration axes work correctly
'''


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_broadcaster')
        self.first_run = True
        
        # Subscribe to the imu_data topic
        self.subscription = self.create_subscription(
            Imu,
            'imu_data',
            self.imu_callback,
            10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info("Odometry broadcaster started")

        # Initial node positions in the world frame
        self.length = 0.05 #[m]
        self.initial_node_positions = np.array([ 
            [0,             0,             0            ], # node 0
            [0,             self.length,   0            ], # node 1
            [0,             self.length,   self.length  ], # node 2
            [0,             0,             self.length  ], # node 3
            [self.length,   0,             0            ], # node 4
            [self.length,   self.length,   0            ], # node 5
            [self.length,   self.length,   self.length  ], # node 6
            [self.length,   0,             self.length  ], # node 7
            [self.length/2, self.length/2, self.length/2]  # base_link node 
        ]) # 9 Rows, 3 Columms

        # Initializing variables
        self.bl_g = np.zeros((1,3))
        self.q = np.zeros((1,4))
        self.bl_node_vectors = np.zeros((8, 3))# in the base_link frame
        self.bl_node_vectors_diffs = np.zeros((9, 3))
        self.bn_node_positions = np.zeros((8,3))
        self.n0_node_positions = np.zeros((1,3))
        self.topple_translation = np.zeros((1,3)) # world frame
        self.current_topple_translation = np.zeros((1,3))
        self.last_bn_index = 0 # initial balancing node index
        self.q_correction = np.array([
            -0.68031357,
            -0.20766731,
            -0.21457080,
             0.66933332
        ])

    def imu_callback(self, msg):

        ### Getting the current quaternion ###
        self.q = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])

        self.get_logger().info(f'Current Quaternion: {self.q}')

        self.q = self.rotate_quaternion(self.q, self.q_correction)
        
        self.get_logger().info(f'Corrected Quaternion: {self.q}')

        ### Handling the Initial Position ###

        # Initializing with node 0 as the balancing node, world and bn frames are aligned.
        if self.first_run: 
            self.bn_node_positions = self.initial_node_positions
            self.first_run = False

        ### Determining the Current Balancing Node ###
        
        # First getting the gravity vector in the base_link frame
        # Note: We are assuming it initializes w/ a q = [0,0,0,1], if not make it so through better calibration
        # TODO: VERIFY THIS IS THE CASE. 
        self.bl_g = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z])
        
        self.get_logger().info(f'Current Linear Acceleration Vector: {self.bl_g}') 
        
        # Have to correct this vector as well?
        self.bl_g = self.rotate_vec(self.q_correction, self.bl_g)

        self.get_logger().info(f'Updated Linear Acceleration Vector: {self.bl_g}') 
        
        #-np.linalg.norm([self.length/2, self.length/2])

        # Find the vectors from base_link node to each node of the cube in the base_link frame
        for i in range(8):
            self.bl_node_vectors[i] = self.bn_node_positions[i,:] - self.bn_node_positions[8,:]
            #self.get_logger().info(f'Node Vectors: {self.bl_node_vectors}') 
            self.bl_node_vectors_diffs = self.bl_node_vectors[i] - self.bl_g[0]

        
        # Finding the balancing node
        self.bn_index = np.argmin(self.bl_node_vectors_diffs)

        ### Determining node positions in the world frame from the Balancing Node ###

        # Determine if there is a translation and keep adding them up.
        if self.bn_index != self.last_bn_index:
            self.current_topple_translation = self.bn_node_positions[self.bn_index] - self.bn_node_positions[self.last_bn_index]
            print(self.current_topple_translation)
            #self.get_logger().info(f'Current Topple Translation: {self.current_topple_translation}')
            self.bn_node_positions = self.bn_node_positions - self.current_topple_translation
            #self.get_logger().info(f'Updated BN Node Positions: {self.bn_node_positions}') 
            self.topple_translation += self.current_topple_translation

        ### Update Node Positions based on Current Orientation ###
        for i in range(9):
            self.bn_node_positions[i] = self.rotate_vec(self.q, self.bn_node_positions[i])

        # Broadcasting the transforms
        self.translation = self.topple_translation.flatten().tolist()

        self.broadcast_transform('node_0', 'base_link',self.initial_node_positions[8,:],  self.q)
        self.broadcast_transform('balancing_node', 'node_0',self.bn_node_positions[0,:])
        self.broadcast_transform('world', 'balancing_node',self.translation)

    def broadcast_transform(self, parent_frame, child_frame, translation, rotation=None):
        """Broadcast a transform between two frames."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame

        # Setting the translation vector
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        
        # Use identity rotation if not specified
        if rotation is None:
            rotation = np.array([0.0, 0.0, 0.0, 1.0])

        rotation = [float(r) for r in rotation]
        
        # Setting the rotation (quaternion)
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]

        self.tf_broadcaster.sendTransform(transform)
    
    def rotate_vec(self, q, v):
        """
        Rotate a vector, v, using a unit quaternion q.
        
        Params:
        - q: numpy array of shape (4,), the unit quaternion (x, y, z, w)
        - v: numpy array of shape (3,), the 3D vector to rotate
        
        Returns:
        - v_rot: numpy array of shape (3,), the 3D rotated vector
        """
        # Normalize the quaternion to ensure it is a unit quaternion
        q = q / np.linalg.norm(q)

        # Extract components of the quaternion
        x, y, z, w = q

        # Compute intermediate values
        # This avoids creating additional temporary quaternions
        ww = w * w
        xx = x * x
        yy = y * y
        zz = z * z
        wx = w * x
        wy = w * y
        wz = w * z
        xy = x * y
        xz = x * z
        yz = y * z

        # Construct the rotation matrix derived from the quaternion
        rot_matrix = np.array([
            [ww + xx - yy - zz, 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), ww - xx + yy - zz, 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), ww - xx - yy + zz]
        ])

        # Rotate the vector
        v_rot = np.dot(rot_matrix, v)

        return v_rot

    def rotate_quaternion(self, q1, q2):
        """
        Rotate a quaternion q1 using another quaternion q2.
        
        Params:
        - q1: numpy array of shape (4,), the quaternion to be rotated (x, y, z, w)
        - q2: numpy array of shape (4,), the rotation quaternion (x, y, z, w)
        
        Returns:
        - q_rot: numpy array of shape (4,), the rotated quaternion (x, y, z, w)
        """
        # Normalize the quaternions to ensure they are unit quaternions
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)
        
        # Extract components of q1 and q2
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        
        # Compute the quaternion product q_rot = q2 * q1
        x_rot = w2 * x1 + x2 * w1 + y2 * z1 - z2 * y1
        y_rot = w2 * y1 - x2 * z1 + y2 * w1 + z2 * x1
        z_rot = w2 * z1 + x2 * y1 - y2 * x1 + z2 * w1
        w_rot = w2 * w1 - x2 * x1 - y2 * y1 - z2 * z1
        
        # Combine the components into the rotated quaternion
        q_rot = np.array([x_rot, y_rot, z_rot, w_rot])
        
        return q_rot


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
