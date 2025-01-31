import rclpy
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
import tf2_ros

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
        
        # Creating the TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info("Odometry broadcaster started")

        # Initial Node Positions in Both Frames
        self.length = 0.05 #[m]
        self.initial_w_node_positions = np.array([ 
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
        self.initial_bl_node_positions = np.array([
            [-self.length/2, -self.length/2, -self.length/2], # node 0
            [-self.length/2,  self.length/2, -self.length/2], # node 1
            [-self.length/2,  self.length/2,  self.length/2], # node 2
            [-self.length/2, -self.length/2,  self.length/2], # node 3
            [ self.length/2, -self.length/2, -self.length/2], # node 4
            [ self.length/2,  self.length/2, -self.length/2], # node 5
            [ self.length/2,  self.length/2,  self.length/2], # node 6
            [ self.length/2, -self.length/2,  self.length/2], # node 7
            [ 0            ,  0            ,  0            ]  # base_link node
        ])

        # GRAVITY VECTORS 
        self.bl_g_w = np.array([0,0,-1])
        self.bl_g_bl = np.zeros((1,3))

        # QUATERNIONS
        self.q = np.zeros((4))
        self.q_correction = np.zeros((4))
        
        # NODE VECTOR INFORMATION
        self.bl_node_vectors_w = np.zeros((8, 3))
        self.bl_node_vectors_diffs_w = np.zeros((8, 3))
        self.norms_w = np.zeros((8))
        self.bl_node_vectors_bl = np.zeros((8, 3))
        self.bl_node_vectors_diffs_bl = np.zeros((8, 3))
        self.norms_bl = np.zeros((8))

        # TRANSLATION INFORMATION
        self.bn_translation = np.zeros(3) 
        self.bl_translation = np.zeros(3)
        self.bn_displacement = np.zeros(3)
        
        # BALANCING NODE INFORMATION
        self.last_bn_index_w = 0
        self.current_bn_index_w = 0
        self.last_bn_index_bl = 0
        self.current_bn_index_bl = 0       

    def imu_callback(self, msg):
        
        ''' Acquiring AHRS Current Quaternion in Calibration Frame'''
        self.q = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w], dtype=np.float64)
        self.get_logger().info(f'AHRS Quaternion: {self.q}')

        ''' Handling the Initial Position and First Iteration '''
        if self.first_run: 
            self.q_correction = self.inverse_quaternion(self.q)
            self.first_run = False
              
        self.get_logger().info(f'Correction Quaternion: {self.q_correction}')
        self.q = self.rotate_quaternion(self.q, self.q_correction)
        self.get_logger().info(f'World Frame Quaternion: {self.q}')

        self.w_node_positions = np.copy(self.initial_w_node_positions)
        self.bl_node_positions = np.copy(self.initial_bl_node_positions)

        ''' Updating the Location of the Nodes based on the Current Orientation'''
        for i in range(len(self.w_node_positions)):
            self.w_node_positions[i, :] = self.active_rotate_vec(self.q, self.w_node_positions[i, :])
            self.bl_node_positions[i, :] = self.passive_rotate_vec(self.q, self.bl_node_positions[i, :])
        self.get_logger().info(f'All of the World Node Positions: {self.w_node_positions}') 
        self.get_logger().info(f'All of the Base_Link Node Positions: {self.bl_node_positions}') 

        ''' Finding the Projected Gravity Vector based on the Current Orientation '''
        self.bl_g_bl = self.passive_rotate_vec(self.q,self.bl_g_w)
        self.get_logger().info(f'The Projected Gravity Vector in the Base_link Frame: {self.bl_g_bl}') 
        self.get_logger().info(f'The Gravity Vector in the World Frame: {self.bl_g_w}') 
 
        ''' Determining the Balancing Node based on the Current Orientation in the World Frame'''
        self.bl_node_position_bl = np.copy(self.bl_node_positions[8,:])
        self.bl_node_position_w = np.copy(self.w_node_positions[8,:])
        self.get_logger().info(f'The Base_Link Node Position in the Base_Link Frame: {self.bl_node_position_bl}') 
        self.get_logger().info(f'The Base_Link Node Position in the World Frame: {self.bl_node_position_w}') 

        for i in range(8):
            self.bl_node_vectors_w[i,:] = self.w_node_positions[i,:] - self.bl_node_position_w
            self.bl_node_vectors_bl[i,:] = self.bl_node_positions[i,:] - self.bl_node_position_bl

            self.bl_node_vectors_w[i,:] = self.bl_node_vectors_w[i,:]/np.linalg.norm(self.bl_node_vectors_w[i,:])
            self.bl_node_vectors_bl[i,:] = self.bl_node_vectors_bl[i,:]/np.linalg.norm(self.bl_node_vectors_bl[i,:])

            self.bl_node_vectors_diffs_w[i,:] = self.bl_node_vectors_w[i,:] - self.bl_g_w
            self.bl_node_vectors_diffs_bl[i,:] = self.bl_node_vectors_bl[i,:] - self.bl_g_bl

            self.norms_w[i] = np.linalg.norm(self.bl_node_vectors_diffs_w[i,:])
            self.norms_bl[i] = np.linalg.norm(self.bl_node_vectors_diffs_bl[i,:])

        self.get_logger().info(f'Node Vectors in the Base_link Frame: {self.bl_node_vectors_bl}') 
        self.get_logger().info(f'Node Vector Differences from g in Base_link Frame: {self.bl_node_vectors_diffs_bl}')
        self.get_logger().info(f'Norms of Node Vector Differences from g in Base_link Frame: {self.norms_bl}')

        self.get_logger().info(f'Node Vectors in the World Frame: {self.bl_node_vectors_w}')
        self.get_logger().info(f'Node Vector Differences from g in World Frame: {self.bl_node_vectors_diffs_w}')
        self.get_logger().info(f'Norms of Node Vector Differences from g in World Frame: {self.norms_w}')

        self.current_bn_index_w = np.argmin(self.norms_w)
        self.current_bn_index_bl = np.argmin(self.norms_bl)

        self.get_logger().info(f'The Current Balancing Node in Base_link Frame:{self.current_bn_index_bl}')
        self.get_logger().info(f'The Current Balancing Node in World Frame:{self.current_bn_index_w}')
        
        ''' Determining the Translation TFs ''' 
        # Determine if the balancing node changed, and find the displacement
        if self.current_bn_index_w != self.last_bn_index_w:
            self.bn_displacement = self.w_node_positions[self.current_bn_index_w,:] - self.w_node_positions[self.last_bn_index_w,:]   
        else:
            self.bn_displacement = np.zeros(3)       

        self.get_logger().info(f'The Current Balancing Node displacement in the World Frame:{self.bn_displacement}')   
        
        # Update the translations (base_link and balancing_node
        self.bl_translation = self.w_node_positions[8,:] - self.w_node_positions[self.current_bn_index_w,:]
        self.bn_translation = self.bn_translation + self.bn_displacement

        ''' Broadcasting the full Transforms '''
        self.bn_translation = self.bn_translation.flatten().tolist()
        self.get_logger().info(f'The Current Balancing Node Translation:{self.bn_translation}')
        self.bl_translation = self.bl_translation.flatten().tolist()
        self.get_logger().info(f'The Current Base_Link Translation:{self.bl_translation}')
        self.bl_rotation = self.q.flatten().tolist()
        self.broadcast_transform('balancing_node', 'base_link',self.bn_translation, [0,0,0,1])
        self.broadcast_transform('world', 'balancing_node',self.bl_translation, self.bl_rotation)

        # TODO: Log the last 30 index numbers and if the last 15 are different, then 
        self.last_bn_index_w = self.current_bn_index_w

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
    
    def active_rotate_vec(self, q, v):
        """
        Rotate a vector, v, using a unit quaternion q.
        
        Params:
        - q: numpy array of shape (4,), the unit quaternion (x, y, z, w)
        - v: numpy array of shape (3,), the 3D vector to rotate
        
        Returns:
        - v_rot: numpy array of shape (3,), the 3D rotated vector
        """
        # Normalize the quaternion to ensure it is a unit quaternion
        norm_q = np.linalg.norm(q)
        if norm_q > 1e-6:  # Avoid division by zero
            q = q / norm_q

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
    
    def passive_rotate_vec(self, q, v):
        """
        Rotate a vector, v, using a unit quaternion q.
        
        Params:
        - q: numpy array of shape (4,), the unit quaternion (x, y, z, w)
        - v: numpy array of shape (3,), the 3D vector to rotate
        
        Returns:
        - v_rot: numpy array of shape (3,), the 3D rotated vector
        """
        # Normalize the quaternion to ensure it is a unit quaternion
        norm_q = np.linalg.norm(q)
        if norm_q > 1e-6:  # Avoid division by zero
            q = q / norm_q

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
        v_rot = np.dot(rot_matrix.T, v)

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
    
    def inverse_quaternion(self, q):
        """
        Generate the inverse quaternion from a given quaternion

        Params:
        - q: numpy array of shape (4,), the quaternion to be inverted (x, y, z, w)

        Returns:
        - q_inv: numpy array of shape (4,), the inverse quaternion (-x,-y,-z, w)
        """
        # Normalize the quaternion to ensure it is a unit quaternion
        q = q / np.linalg.norm(q)

        # Extract the components of q
        x, y, z, w = q

        # Creat and return the inverse quaternion.
        q_inv = np.array([-x, -y, -z, w])
        return q_inv


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
