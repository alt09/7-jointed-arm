import Utils.utils as utils
import numpy as np

import constants

def forward_kinematics(q):
    """
    Computes the forward kinematics for the robotic arm given a set of joint angles(in radians). 
    Args:
        q (list): A list of joint angles for the robotic arm(in radians).
                q = [q1, q2, q3, q4, q5, q6, q7]
    Returns:
            - A list containing the following:
            - T (numpy.ndarray): The transformation matrix representing the end effector's pose.
            - joint_positions (list): A list of 7 joint positions in 3D space.
            - joint_axes_world (list): A list of 7 joint axes in world coordinates.
            - transforms (list): A list of 7 transformation matrices for each joint.
    """

    # Convert the input joint angles to a numpy array and ensure they are of type float
    q = np.asarray(q, dtype=float)
    # Check if the input joint angles have the correct length (7 for a 7-jointed arm)
    if len(q) != 7:

        raise ValueError("Expected 7 joint angles, got {}".format(len(q)))
    
    # Initialize the transformation matrix T as an identity matrix
    T = np.eye(4)
    joint_positions = []
    joint_axes_world = []
    transforms = []

    transforms= []

    # Iterate through each joint angle to compute the forward kinematics
    for i in range(7):

        # Compute the transformation matrix for the current joint using its origin and axis
        T[:3,3] += T[:3,:3] @ constants.Constants.Robot.JOINT_ORIGINS[i]
        joint_positions.append(T[:3,3].copy())

        # Compute the joint axis in world coordinates by transforming the local joint axis using the current transformation matrix
        axis_world = T[:3,:3] @ constants.Constants.Robot.JOINT_AXES[i]
        joint_axes_world.append(axis_world.copy())

        # Compute the rotation matrix for the current joint angle and update the transformation matrix T
        R = utils.rotation_matrix(constants.Constants.Robot.JOINT_AXES[i], q[i])

        # Update the transformation matrix T by applying the rotation R to the current orientation
        T[:3,:3] = T[:3,:3] @ R
        transforms.append(T.copy())
        

    return T, joint_positions, joint_axes_world, transforms

def inverse_kinematics(target_position,target_orientation,initial_q=None,max_iterations=1000, tolerance=1e-4,learning_rate=0.5, damping=0.05):
    """
    Computes the inverse kinematics for the robotic arm to reach a target position and orientation.
    Args:
        target_position (list): A list of 3 coordinates [x, y, z] representing the target position in 3D space.
        target_orientation (list): Rotation matrix representing the target orientation in 3D space, yaw pitch and roll in radians.
        initial_q (list, optional): A list of initial joint angles for the robotic arm(in radians). Defaults to None.
        max_iterations (int, optional): The maximum number of iterations for the IK solver. Defaults to 1000.
        tolerance (float, optional): The tolerance for convergence. Defaults to 1e-4.
        learning_rate (float, optional): The learning rate for the IK solver. Defaults to 0.5.
        damping (float, optional): The damping factor for the IK solver. Defaults to 0.05.
    Returns:
        list: A list of joint angles for the robotic arm(in radians) that achieve the target position and orientation.
    """

    # If the initial joint angles are not provided, start with a zero configuration
    if initial_q is None:

        q = np.zeros(7)

    # If the initial joint angles are provided, use them as the starting point
    else:

        q = np.asarray(initial_q, dtype=float)

    # Iterate to find the joint angles that achieve the target position and orientation    
    for i in range(max_iterations):

        T, joint_positions, joint_axes_world, transforms = forward_kinematics(q)

        error = utils.pose_error(T,target_position,target_orientation)

        # If the error is within the specified tolerance, return the current joint angles
        if np.linalg.norm(error) < tolerance:

            print(f"IK converged in {i} iterations.")

            return q
        
        # Compute the Jacobian matrix for the current joint angles
        J = utils.calculate_jacobian(q)

        delta_q = utils.damped_least_squares(J, error, damping)

        delta_q *= learning_rate

        max_joint_step = 0.1

        # Compute the maximum change in joint angles
        max_delta = np.max(np.abs(delta_q))

        # If the maximum change in joint angles exceeds the maximum allowed step, scale the changes down
        if max_delta > max_joint_step:

            delta_q *= max_joint_step / max_delta

        q+= delta_q

        # Clip the joint angles to their joint limits
        q = np.clip(q,
                    constants.Constants.Robot.JOINT_MIN,
                    constants.Constants.Robot.JOINT_MAX
                    )
        
    # If the maximum number of iterations is reached without convergence, print a message and return the current joint angles
    print(f"IK did not converge after {max_iterations} iterations. ")

    return q