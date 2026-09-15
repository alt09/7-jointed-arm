from Utils import utils
from Vision import opencv
import sim
import math
import numpy as np

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
def go_to(arm_id, num_joints, go_to):
    """
    Moves the robotic arm to the specified joint positions.
    Args:
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
        num_joints (int): The number of joints in the robotic arm (8).
        go_to (list): A list of 8 target joint positions for the robotic arm.
    """
    for i in range(num_joints):
        sim.set_joint_positions(i, go_to[i], arm_id)

def go_to_target(arm_id,target_position, target_orientation):
    """
    Moves the robotic arm to the specified target position.
    Args:
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
        target_position (list): A list of 3 target coordinates [x, y, z] for the end effector.
        target_orientation (list): A list of 4 target orientation values [x, y, z, w] for the end effector.
    """

    # Calculate the inverse kinematics to find the joint angles for the target position
    target_joint_positions = sim.p.calculateInverseKinematics(arm_id, 6, target_position, target_orientation)  # 7 is the index of the end effector link

    # print(f"Target joint positions: {target_joint_positions}")

    # Move the arm to the target joint positions
    for i in range(sim.p.getNumJoints(arm_id)):
        sim.set_joint_positions(i, target_joint_positions[i], arm_id)
def where_is_endeffector(arm_id):
    """
    Returns the current position of the end effector of the robotic arm.
    Args:
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
    Returns:
        list: A list of 2 values [End_effector_position, End_effector_orientation] representing the current position and orientation of the end effector.
    """
    quaternion = sim.p.getLinkState(arm_id, 6)[5]  # Get the orientation of the end effector

    yaw, pitch, roll = utils.Yaw_pitch_roll_from_quaternion(quaternion)
    # Calculate the forward kinematics to find the position of the end effector
    end_effector_state = sim.p.getLinkState(arm_id, 6)
    end_effector_position = [end_effector_state[4],yaw,pitch,roll]  # Position is at index 4

    return end_effector_position
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

    print("Yaw:", math.degrees(target_yaw))
    print("Pitch:", math.degrees(target_pitch))
    return target_yaw, target_pitch
def cheats(arm_id,target_3Dposition,viewMatrix1,viewMatrix2):
    """
    Aims the robotic arm at a target 3D position using the average camera pose.
    Args:
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
        target_3Dposition (list): A list of 3 coordinates [x, y, z] representing the target position in 3D space.
        viewMatrix1 (list): The view matrix of the first camera.
        viewMatrix2 (list): The view matrix of the second camera.
    """
    if target_3Dposition is not None:
        angle = auto_aim(target_3Dposition,viewMatrix1,viewMatrix2)  # Get the final angle from auto_aim
        print("target_3Dposition",target_3Dposition)
        end_effector_yaw = where_is_endeffector(arm_id)[1]  # Get the current position of the end effector
        wrist_pitch = utils.Yaw_pitch_roll_from_quaternion(sim.p.getLinkState(arm_id, 6)[5])[1]  # Get the current position of the wrist
        sim.set_joint_positions(7,-(end_effector_yaw+angle[0]), arm_id) # izquierda Move the arm to the calculated joint positions
        sim.set_joint_positions(6,-(wrist_pitch+angle[1]), arm_id) # abajo

def approach(arm_id,target_3Dposition,viewMatrix1,viewMatrix2):
    """
    Moves the robotic arm to a predefined approach position.
    Args:
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
        target_3Dposition (list): A list of 3 coordinates [x, y, z] representing the target position in 3D space.
        viewMatrix1 (list): The view matrix of the first camera.
        viewMatrix2 (list): The view matrix of the second camera.
    Returns:
        None
    """
    target_approach_position = target_3Dposition - np.array([1, 1, 0])  # Move 10 cm above the target position
    
    end_effector_yaw = where_is_endeffector(arm_id)[1]  # Get the current position of the end effector
    wrist_pitch = utils.Yaw_pitch_roll_from_quaternion(sim.p.getLinkState(arm_id, 6)[5])[1]  # Get the current position of the wrist
    

    diff_angle = auto_aim(target_approach_position,viewMatrix1,viewMatrix2)  # Get the final angle from auto_aim
    yaw =  -(end_effector_yaw+diff_angle[0])
    pitch = -(wrist_pitch+diff_angle[1])
    go_to_target(arm_id, target_3Dposition, utils.quaternion_from_yaw_pitch_roll(yaw, pitch, 0))  # Move to a predefined approach position
    print("target_3Dposition",target_3Dposition)

