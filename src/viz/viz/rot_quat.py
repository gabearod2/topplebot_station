import numpy as np

def normalize_quaternion(q):
    return q / np.linalg.norm(q)

def quaternion_inverse(q):
    x, y, z, w = q
    return np.array([-x, -y, -z, w])

def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    ])

# Given quaternion
q = np.array([0.68031359, 0.20766732, 0.21457081, 0.66933334])
q_desired = np.array([0, 0, 0, 1])

# Normalize the given quaternion
q = normalize_quaternion(q)

# Compute the inverse of the given quaternion
q_inv = quaternion_inverse(q)

# Compute the rotation quaternion
q_r = quaternion_multiply(q_desired, q_inv)
print("Rotation Quaternion:", q_r)
