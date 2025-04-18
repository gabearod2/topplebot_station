import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

length = 1

def rotate_vec(q, v):
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

def plot_nodes(positions, title):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2], c='r', marker='o')

    # Annotate points
    for i, pos in enumerate(positions):
        ax.text(pos[0], pos[1], pos[2], f'{i}', color='black')

    # Set axis labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(title)

    plt.show()

def rotate_quaternion(q1, q2):
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

def inverse_quaternion(q):
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

# The initial node positions
node_positions = np.array([ 
    [0,             0,             0            ], # node 0
    [0,             length,        0            ], # node 1
    [0,             length,        length       ], # node 2
    [0,             0,             length       ], # node 3
    [length,        0,             0            ], # node 4
    [length,        length,        0            ], # node 5
    [length,        length,        length       ], # node 6
    [length,        0,             length       ], # node 7
    [length/2,      length/2,      length/2     ]  # base_link node 
])

# The possible correction quaternions
q_y  = np.array([0,0,0.70710,0.70710])
q_x  = np.array([0,0,-0.70710,0.70710])
q_xy = np.array([0,0,1,0])

# A random orientation of the cube
q_random = np.array([-0.12940952255126037, 0.12940952255126037, -0.017037086855465847, 0.9829629131445342])
q_random = q_random / np.linalg.norm(q_random)  # Normalize quaternion

# Update all of the positions based on the orientation
for i in range(len(node_positions)):
    node_positions[i, :] = rotate_vec(q_random, node_positions[i, :])

# Plot the rotated node positions
plot_nodes(node_positions, "Rotated Node Positions")

print(node_positions)

balancing_node = np.copy(node_positions[5,:])

# Update the positions based on the new balancing node
for i in range(len(node_positions)):
    node_positions[i, :] = node_positions[i, :] - balancing_node 
print(node_positions)
# Plot the rotated node positions
plot_nodes(node_positions, "Translated Node Positions")

# Now updating the quaternion to correct for the rotation
q_updated = rotate_quaternion(q_random,q_xy)

# Update all of the positions based on the orientation
for i in range(len(node_positions)):
    node_positions[i, :] = rotate_vec(q_updated, node_positions[i, :])

print(node_positions)

plot_nodes(node_positions, "Rotated Node Positions")

# What is the world node position?


