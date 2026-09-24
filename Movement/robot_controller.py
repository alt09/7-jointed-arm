from Movement import kinematics
from Utils import pid, utils
from Vision import opencv
import pybullet as p
import math
import constants
import numpy as np

def set_joint_positions(joint_index, target_position, arm_id):
    """
    Sets the position of a specific joint in the robotic arm.
    Args:
        joint_index (int): The index of the joint to be set.
        target_position (float): The target position for the joint in radians.
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
    """

    p.setJointMotorControl2(
        arm_id,
        joint_index,
        p.POSITION_CONTROL,
        targetPosition=target_position, 
        force=100
    )

def go_to(arm_id, num_joints, target_joint_positions):
    """
    Moves the robotic arm to the specified joint positions.
    Args:
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
        num_joints (int): The number of joints in the robotic arm (7).
        target_joint_positions (list): A list of 7 target joint positions for the robotic arm.
    """

    for i in range(num_joints):

        set_joint_positions(i, target_joint_positions[i], arm_id)

def go_to_target(arm_id,target_position, target_orientation):
    """
    Moves the robotic arm to the specified target position.
    Args:
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
        target_position (list): A list of 3 target coordinates [x, y, z] for the end effector.
        target_orientation (list): A list of 4 target orientation values [x, y, z, w] for the end effector.
    """

    # Calculate the inverse kinematics to find the joint angles for the target position
    target_joint_positions = kinematics.inverse_kinematics(
        target_position = target_position,
        target_orientation = target_orientation,
        initial_q = get_current_joint_positions(arm_id)
    )
    # Move the arm to the target joint positions
    go_to_PD(arm_id, target_joint_positions)

def get_end_effector_state(arm_id):
    """
    Returns the current position of the end effector of the robotic arm.
    Args:
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
    Returns:
        list: A list of 2 values [End_effector_position, End_effector_orientation] representing the current position and orientation of the end effector.
    """

    end_effector_state = p.getLinkState(arm_id, constants.Constants.Robot.END_EFFECTOR_LINK_INDEX)
    quaternion = end_effector_state[5]  # Get the orientation of the end effector

    yaw, pitch, roll = utils.Yaw_pitch_roll_from_quaternion(quaternion)

    # Calculate the forward kinematics to find the position of the end effector
    end_effector_info = [end_effector_state[4],yaw,pitch,roll]  # Position is at index 4

    return end_effector_info

def auto_aim(target_3Dposition,viewMatrix1,viewMatrix2):
    """
    Returns the yaw, pitch, and roll angles needed to aim at a target 3D position from the average camera pose.
    Args:
        target_3Dposition (list): A list of 3 coordinates [x, y, z] representing the target position in 3D space.
        viewMatrix1 (list): The view matrix of the first camera.
        viewMatrix2 (list): The view matrix of the second camera.
    """

    avg_camera_pose = (opencv.camera_pose_from_view_matrix(viewMatrix1)[0] + opencv.camera_pose_from_view_matrix(viewMatrix2)[0]) / 2  # Get the average camera pose from the two view matrices
    d = target_3Dposition - avg_camera_pose  # Calculate the direction vector from the average camera position to the target position
    target_yaw = math.atan2(d[0], d[1])  # Calculate the yaw angle
    target_pitch = math.atan2(d[2], math.sqrt(d[0] ** 2 + d[1] ** 2))  # Calculate the pitch angle
    target_yaw = (target_yaw + math.pi) % (2 * math.pi) - math.pi
    target_pitch = (target_pitch + math.pi) % (2 * math.pi) - math.pi

    return target_yaw, target_pitch

# def cheats(arm_id,target_3Dposition,viewMatrix1,viewMatrix2, endeffector_info):
#     """
#     Aims the robotic arm at a target 3D position using the average camera pose.
#     Args:
#         arm_id (int): The ID of the robotic arm in the PyBullet simulation.
#         target_3Dposition (list): A list of 3 coordinates [x, y, z] representing the target position in 3D space.
#         viewMatrix1 (list): The view matrix of the first camera.
#         viewMatrix2 (list): The view matrix of the second camera.
#     """

#     if target_3Dposition is not None:

#         angle = auto_aim(target_3Dposition,viewMatrix1,viewMatrix2)  # Get the final angle from auto_aim
#         print("target_3Dposition",target_3Dposition)
#         end_effector_yaw = endeffector_info[1]  # Get the current position of the end effector
#         wrist_pitch = utils.Yaw_pitch_roll_from_quaternion(p.getLinkState(arm_id, 6)[5])[1]  # Get the current position of the wrist
#         set_joint_positions(7,-(end_effector_yaw+angle[0]), arm_id) #  Move the arm to the calculated joint positions
#         set_joint_positions(6,-(wrist_pitch+angle[1]), arm_id) 


def go_to_target_with_IK(viewMatrix1,viewMatrix2,projectionMatrix,rgba_img1,rgba_img2,arm_id,last_q_solution):
    """
    Moves the robotic arm to a target position detected by the cameras using inverse kinematics.
    Args:
        viewMatrix1 (list): The view matrix of the first camera.
        viewMatrix2 (list): The view matrix of the second camera.
        projectionMatrix (list): The projection matrix used for both cameras.
        rgba_img1 (numpy.ndarray): The RGBA image from the first camera.
        rgba_img2 (numpy.ndarray): The RGBA image from the second camera.
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
        last_q_solution (list): The last known joint angles of the robotic arm.
    Returns:
        list: The updated joint angles of the robotic arm.
    """

    if opencv.target_3d_pose(
        viewMatrix1,
        viewMatrix2,
        projectionMatrix,
        rgba_img1,
        rgba_img2,
        constants.Constants.Camera.DETECT_COLOR_MIN,
        constants.Constants.Camera.DETECT_COLOR_MAX
    ) is not None:

        caminfo = opencv.target_3d_pose(
            viewMatrix1,
            viewMatrix2,
            projectionMatrix,
            rgba_img1,
            rgba_img2,
            constants.Constants.Camera.DETECT_COLOR_MIN,
            constants.Constants.Camera.DETECT_COLOR_MAX
        )

        if caminfo[1] < 0.05: # 0.05 is the triangulation error threshold, if the error is less than this value, we consider the target to be detected

            pose = caminfo[0]
            print("Target pose",pose)

            yaw, pitch = auto_aim(pose,viewMatrix1,viewMatrix2)
            roll = np.radians(0)


            target_orientation = utils.rpy_rotation(roll,pitch,yaw)

            q_solution = kinematics.inverse_kinematics(
                target_position = pose,
                target_orientation = target_orientation,
                initial_q = get_current_joint_positions(arm_id)
            )
            last_q_solution = q_solution

            go_to_PD(arm_id, q_solution)
            print("last known position:",kinematics.forward_kinematics(q_solution)[0][:3, 3])

            return last_q_solution

        else:

            print("\nTriangulation error too high, not moving the arm.")
    else:

        if last_q_solution is not None:

            q_solution = last_q_solution
            print("No target detected, moving to last known position.")
            go_to_PD(arm_id, q_solution)
            print("last known position:",kinematics.forward_kinematics(q_solution)[0][:3, 3])

        else:

            print("No target detected and no last known position available.going to 0,0,0")
            go_to_target(arm_id, [0, 0, 0], [0, 0, 0, 1])

def stay(arm_id):
    """
    Keeps the robotic arm in its current position.
    Args:
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
    """

    for i in range(constants.Constants.Robot.END_EFFECTOR_LINK_INDEX+1):

        set_joint_velocities(i, 0, arm_id)

def go_to_PD(arm_id, last_q_solution):
    """
    Moves the robotic arm to a target position using a PD controller.
    Args:
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
        last_q_solution (list): The last known joint angles of the robotic arm.
    """
    current_joint_positions = get_current_joint_positions(arm_id)
    
    effective_inertias = calculate_effective_inertias(arm_id, current_joint_positions)

    if last_q_solution is not None:
        for joint_index, I_eff in enumerate(effective_inertias):
            set_joint_position_PD(joint_index, last_q_solution[joint_index], arm_id, I_eff)

def get_current_joint_positions(arm_id):
    """
    Returns the current joint positions of the robotic arm.
    Args:
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
    Returns:
        list: A list of current joint positions for the robotic arm.
    """

    return [p.getJointState(arm_id, i)[0] for i in range(p.getNumJoints(arm_id))]

def get_joint_info(arm_id):
    """
    Returns information about the joints of the robotic arm.
    Args:
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
    Returns:
        dict: A dictionary containing joint information, including joint names, indices, limits, and other properties.
    """

    joint_info = {}
    num_joints = p.getNumJoints(arm_id)

    for i in range(num_joints):

        info = p.getJointInfo(arm_id, i)
        joint_name = info[1].decode('utf-8')
        joint_type = info[2]

        if joint_type == p.JOINT_REVOLUTE:

            joint_info[joint_name] = {
                'index': i,
                'lower_limit': info[8],
                'upper_limit': info[9],
                'max_force': info[10],
                'max_velocity': info[11]
            }

    return joint_info

def set_joint_position_PD(joint_index, target_position, arm_id, I_eff):
    """
    Sets the position of a specific joint in the robotic arm using a PD controller.
    Args:
        joint_index (int): The index of the joint to be set.
        target_position (float): The target position for the joint in radians.
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
        I_eff (float): The effective inertia of the joint.
    """

    current_position, current_velocity,_ ,__ = p.getJointState(arm_id, joint_index)

    control_torque = pid.calculate_torque(I_eff, target_position, 0.0, current_position, current_velocity)

    p.setJointMotorControl2(
        arm_id,
        joint_index,
        p.TORQUE_CONTROL,
        force=control_torque
    )

def set_joint_velocities(joint_index, target_velocity, arm_id):
    """
    Sets the velocity of a specific joint in the robotic arm.
    Args:
        joint_index (int): The index of the joint to be set.
        target_velocity (float): The target velocity for the joint in radians/second.
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
    """

    p.setJointMotorControl2(
        arm_id,
        joint_index,
        p.VELOCITY_CONTROL,
        targetVelocity=target_velocity,
        force=1000
    )

def set_joint_torques(joint_index, target_torque, arm_id):
    """
    Sets the torque of a specific joint in the robotic arm.
    Args:
        joint_index (int): The index of the joint to be set.
        target_torque (float): The target torque for the joint.
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
    """
    
    p.setJointMotorControl2(
        arm_id,
        joint_index,
        p.TORQUE_CONTROL,
        force=target_torque
    )

def calculate_effective_inertias(arm_id, joint_positions):
    """
    Calculates the effective inertia of a specific joint in the robotic arm.
    Args:
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
        joint_positions (list): A list of current joint angles for the robotic arm.
        joint_index (int): The index of the joint for which to calculate the effective inertia.
    Returns:
        float: The effective inertia of the specified joint.
    """
    mass_matrix = np.array(
        p.calculateMassMatrix(arm_id, joint_positions)
    )
    return np.diag(mass_matrix)