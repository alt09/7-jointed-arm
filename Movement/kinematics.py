import Utils.utils as utils
import numpy as np
import math

import constants
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
    joint_positions = []
    joint_axes_world = []
    transforms = []

    transforms= []

    for i in range(7):

        T[:3,3] += T[:3,:3] @ constants.Constants.Robot.joint_origins[i]
        joint_positions.append(T[:3,3].copy())
        axis_world = T[:3,:3] @ constants.Constants.Robot.joint_axes[i]
        joint_axes_world.append(axis_world.copy())
        R = utils.rotation_matrix(constants.Constants.Robot.joint_axes[i], q[i])
        T[:3,:3] = T[:3,:3] @ R
        transforms.append(T.copy())
        

    return T, joint_positions, joint_axes_world, transforms