import Utils.utils as utils
import numpy as np
import math
def inverse_kinematics(arm_id, target_position, target_orientation):
    """
    Computes the inverse kinematics for the robotic arm to reach a target position and orientation.
    Args:
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
        target_position (list): A list of 3 coordinates [x, y, z] representing the target position in 3D space.
        target_orientation (list): A list of 4 coordinates [x, y, z, w] representing the target orientation as a quaternion.
    Returns:
        list: A list of joint angles for the robotic arm to reach the target position and orientation.
    """

    target_joint_positions = []

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

joint_origins = [

    # joint 1
    [0.0, 0.0, 0.613788085],

    # joint 2
    [0.0, -0.345, 0.0],

    # joint 3
    [0.323, 0.0, 0.0],

    # joint 4
    [0.37, 0.0, 3.646211915],

    # joint 5
    [0.335, 0.0, -3.646211915],

    # joint 6
    [0.0, -0.345, 0.0],

    # joint 7
    [0.0, 0.0, -0.613788085]
]

joint_axes = [

    # joint 1
    [0, 0, 1],

    # joint 2
    [0, 1, 0],

    # joint 3
    [1, 0, 0],

    # joint 4
    [1, 0, 0],

    # joint 5
    [1, 0, 0],

    # joint 6
    [0, 1, 0],

    # joint 7
    [0, 0, 1]
]

def forward_kinematics(q):
    """
    Computes the forward kinematics for the robotic arm given a set of joint angles(in radians). 
    Args:
        q (list): A list of joint angles for the robotic arm(in radians).
                q = [q1, q2, q3, q4, q5, q6, q7]
    Returns:
        list: 
        list: A list of transformation of every joint frame.
    """
    q = np.asarray(q, dtype=float)

    if len(q) != 7:
        raise ValueError("Expected 7 joint angles, got {}".format(len(q)))

    T = np.eye(4)

    transforms= []
    for i in range(7):

        T = T @ translation(joint_origins[i][0], joint_origins[i][1], joint_origins[i][2])  # Translate to the joint origin
        T = T @ rotation_about_axis(joint_axes[i], q[i])  # Rotate about the joint axis by the joint angle

        transforms.append(T.copy())

        R = rotation_about_axis(joint_axes[i], q[i])



        return T, transforms

import numpy as np
import math


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


joint_origins = np.array([
    [0.0,  0.0,  0.613788085],
    [0.0, -0.345, 0.0],
    [0.323, 0.0, 0.0],
    [0.37, 0.0, 3.646211915],
    [0.335, 0.0, -3.646211915],
    [0.0, -0.345, 0.0],
    [0.0, 0.0, -0.613788085]
])


joint_axes = np.array([
    [0, 0, 1],
    [0, 1, 0],
    [1, 0, 0],
    [1, 0, 0],
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
])


def test_fk(q):

    T = np.eye(4)

    for i in range(7):


        T[:3, 3] += T[:3, :3] @ joint_origins[i]

        R = rotation_matrix(
            joint_axes[i],
            q[i]
        )

        T[:3, :3] = T[:3, :3] @ R


    print("\nEND EFFECTOR POSITION:")
    print(T[:3, 3])

    print("\nEND EFFECTOR ORIENTATION:")
    print(T[:3, :3])

    return T
