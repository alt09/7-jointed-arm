from Movement import kinematics
from Utils import utils
from Vision import opencv
import sim
import math
import constants
import numpy as np

def go_to(arm_id, num_joints, go_to):
    """
    Moves the robotic arm to the specified joint positions.
    Args:
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
        num_joints (int): The number of joints in the robotic arm (7).
        go_to (list): A list of 7 target joint positions for the robotic arm.
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
    target_joint_positions = sim.p.calculateInverseKinematics(arm_id, constants.Constants.Robot.END_EFFECTOR_LINK_INDEX, target_position, target_orientation)  # 7 is the index of the end effector link

    # print(f"Target joint positions: {target_joint_positions}")

    # Move the arm to the target joint positions
    for i in range(constants.Constants.Robot.END_EFFECTOR_LINK_INDEX):
        sim.set_joint_positions(i, target_joint_positions[i], arm_id)
def where_is_endeffector(arm_id):
    """
    Returns the current position of the end effector of the robotic arm.
    Args:
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
    Returns:
        list: A list of 2 values [End_effector_position, End_effector_orientation] representing the current position and orientation of the end effector.
    """
    quaternion = sim.p.getLinkState(arm_id, constants.Constants.Robot.END_EFFECTOR_LINK_INDEX)[5]  # Get the orientation of the end effector

    yaw, pitch, roll = utils.Yaw_pitch_roll_from_quaternion(quaternion)
    # Calculate the forward kinematics to find the position of the end effector
    end_effector_state = sim.p.getLinkState(arm_id, constants.Constants.Robot.END_EFFECTOR_LINK_INDEX)
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
        sim.set_joint_positions(7,-(end_effector_yaw+angle[0]), arm_id) #  Move the arm to the calculated joint positions
        sim.set_joint_positions(6,-(wrist_pitch+angle[1]), arm_id) 


def go_to_target_with_IK(viewMatrix1,viewMatrix2,projectionMatrix,rgba_img1,rgba_img2,arm_id,last_q_solution):

    if opencv.target_3d_pose(viewMatrix1,viewMatrix2,projectionMatrix,rgba_img1,rgba_img2,constants.Constants.Camera.DETECT_COLOR_MIN, constants.Constants.Camera.DETECT_COLOR_MAX) is not None:
        caminfo = opencv.target_3d_pose(viewMatrix1,viewMatrix2,projectionMatrix,rgba_img1,rgba_img2,constants.Constants.Camera.DETECT_COLOR_MIN, constants.Constants.Camera.DETECT_COLOR_MAX)
        if caminfo[1] < 0.05:
            pose = caminfo[0]
            print("Target pose",pose)
            pose = caminfo[0]

            yaw, pitch = auto_aim(pose,viewMatrix1,viewMatrix2)
            roll = np.radians(0)


            target_orientation = utils.rpy_rotation(roll,pitch,yaw)
            q_solution = kinematics.inverse_kinematics(
                target_position = pose,
                target_orientation = target_orientation,
                initial_q = sim.get_joint_angle(arm_id)
                )
            last_q_solution = q_solution
            go_to(arm_id, len(q_solution), q_solution)
            print("robot position:",where_is_endeffector(arm_id))
            print("last known position:",kinematics.forward_kinematics(q_solution)[0][:3, 3])

            return last_q_solution

        else:
            print("\nTriangulation error too high, not moving the arm.")
    else:
        if last_q_solution is not None:
            q_solution = last_q_solution
            print("No target detected, moving to last known position.")
            go_to(arm_id, 7, q_solution)
            print("robot position:",where_is_endeffector(arm_id))
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
        sim.set_joint_velocities(i, 0, arm_id)