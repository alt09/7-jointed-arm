import Utils.utils as utils
import numpy as np
import math

import constants

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

def inverse_kinematics(target_position,initial_q,max_iterations=1000, tolerance=1e-4,learning_rate=0.5, damping=0.05):

    q = np.asarray(initial_q, dtype=float).copy()
    target_position = np.asarray(target_position, dtype=float)

    q = np.clip(q, 
                constants.Constants.Robot.JOINT_MIN,
                constants.Constants.Robot.JOINT_MAX
                )
    for iteration in range(max_iterations):
        t, joint_positions, joint_axes_world, transforms = forward_kinematics(q)
        current_position = t[:3, 3]

        error = target_position - current_position

        error_magnitude = np.linalg.norm(error)
        if error_magnitude < tolerance:
            print(
                f"IK converged in {iteration} iterations "
                f"with error {error_magnitude}"
            )
            return q
        J = utils.calculate_jacobian(q)

        J_position = J[:3, :]

        delta_q = utils.damped_least_squares(J_position, error, damping)

        delta_q *= learning_rate

        max_joint_step = 0.1
        max_delta = np.max(np.abs(delta_q))
        if max_delta > max_joint_step:
            delta_q *= max_joint_step / max_delta

        q+= delta_q

        q = np.clip(q,
                    constants.Constants.Robot.JOINT_MIN,
                    constants.Constants.Robot.JOINT_MAX
                    )

    print(
        f"IK did not converge after {max_iterations} iterations. "
        f"Final error: {error_magnitude}"
    )

    return q