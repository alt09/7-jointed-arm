import math
import numpy as np

from Movement import kinematics
def Yaw_pitch_roll_from_quaternion(quaternion):
    """
    Converts a quaternion to yaw, pitch, and roll angles.
    Args:
        quaternion (list): A list of 4 values representing the quaternion [x, y, z, w].
    Returns:
        tuple: A tuple containing the yaw, pitch, and roll angles in radians.
    """
    roll = math.atan2(2 * (quaternion[3] * quaternion[0] + quaternion[1] * quaternion[2]), 1 - 2 * (quaternion[0] ** 2 + quaternion[1] ** 2))
    pitch = math.asin(2 * (quaternion[3] * quaternion[1] - quaternion[2] * quaternion[0]))
    yaw = math.atan2(2 * (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1]), 1 - 2 * (quaternion[1] ** 2 + quaternion[2] ** 2))
   
    
    return yaw, pitch, roll

def quaternion_from_yaw_pitch_roll(yaw, pitch, roll):
    """
    Converts yaw, pitch, and roll angles to a quaternion.
    Args:
        yaw (float): The yaw angle in degrees.
        pitch (float): The pitch angle in degrees.
        roll (float): The roll angle in degrees.
    Returns:
        list: A list of 4 values representing the quaternion [x, y, z, w].
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [x, y, z, w]

def rotation_x(angle):
    """
    Returns the rotation matrix for a rotation around the x-axis by the given angle.
    Args:
        angle (float): The angle of rotation in radians.
    Returns:
        np.ndarray: A 3x3 rotation matrix.
    """
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s, c]])

def rotation_y(angle):
    """
    Returns the rotation matrix for a rotation around the y-axis by the given angle.
    Args:
        angle (float): The angle of rotation in radians.
    Returns:
        np.ndarray: A 3x3 rotation matrix.
    """
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]])

def rotation_z(angle):
    """
    Returns the rotation matrix for a rotation around the z-axis by the given angle.
    Args:
        angle (float): The angle of rotation in radians.
    Returns:
        np.ndarray: A 3x3 rotation matrix.
    """
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([[c, -s, 0],
                     [s, c, 0],
                     [0, 0, 1]])

def rpy_rotation(roll, pitch, yaw):
    """
    URDF-style RPY rotation.

    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    """
    R_z = rotation_z(yaw)
    R_y = rotation_y(pitch)
    R_x = rotation_x(roll)

    return R_z @ R_y @ R_x  # The order of multiplication matters

def translation(x,y,z):
    """
    Returns a 4x4 transformation matrix given a translation and rotation in RPY format.
    Args:
        x (float): The x-coordinate of the translation.
        y (float): The y-coordinate of the translation.
        z (float): The z-coordinate of the translation.
    Returns:
        np.ndarray: A 4x4 transformation matrix.
    """
    T = np.eye(4)

    T[:3, 3] = [x,y,z]


    return T

def rotation_about_axis(axis, angle):
    #Rodrigues' rotation formula
    axis = np.asarray(axis,dtype=float)
    axis = axis / np.linalg.norm(axis)

    x,y,z = axis

    c = math.cos(angle)
    s = math.sin(angle)
    v = 1 - c
    R = np.array([
        [
            c + x*x*v,
            x*y*v -z*s,
            x*z*v +y*s
        ],
        [
            y*x*v +z*s,
            c + y*y*v,
            y*z*v - x*s
        ],
        [
            z*x*v - y*s,
            z*y*v + x*s,
            c + z*z*v 
        ]
    ])

    T = np.eye(4)

    T[:3,:3] = R
    return T

def rotation_matrix(axis, angle):

    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)

    x, y, z = axis

    c = math.cos(angle)
    s = math.sin(angle)
    C = 1 - c

    return np.array([
        [c + x*x*C,     x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s,   c + y*y*C,   y*z*C - x*s],
        [z*x*C - y*s,   z*y*C + x*s, c + z*z*C]
    ])
def calculate_jacobian(q):
    """
    Calculates the Jacobian matrix for the robotic arm given a set of joint angles.
    Args:
        q (list): A list of joint angles for the robotic arm(in radians).
                q = [q1, q2, q3, q4, q5, q6, q7]
    Returns:
        np.ndarray: A 6x7 Jacobian matrix.
    """
    T, joint_positions, joint_axes_world, transforms = kinematics.forward_kinematics(q)
    end_effector_position = T[:3, 3]

    J = np.zeros((6, 7))

    for i in range(7):
        joint_axis = joint_axes_world[i]
        joint_position = joint_positions[i]

        # Linear velocity component
        J_linear = np.cross(joint_axis, end_effector_position - joint_position)

        # Angular velocity component
        J_angular = joint_axis

        J[:3, i] = J_linear
        J[3:, i] = J_angular

    return J
def damped_least_squares(J, error, damping=0.01):
    """
    Computes the change in joint angles using the Damped Least Squares method.
    Args:
        J (numpy.ndarray): The Jacobian matrix.
        error (numpy.ndarray): The error vector.
        damping (float): The damping factor.
    Returns:
        numpy.ndarray: The change in joint angles.
    """
    identity = np.eye(3)
    j_damped_inverse = (
        J.T @ np.linalg.inv(
            J @ J.T + damping**2 * identity
            )
    )

    delta_q = j_damped_inverse @ error
    return delta_q