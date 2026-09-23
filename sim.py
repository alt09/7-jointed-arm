
import time
import numpy as np
import pybullet as p
import pybullet_data
import math
import constants
from Movement import dodge
from Movement import robot_controller
from Movement import kinematics
from Utils import pid
from Vision import opencv


print("Starting PyBullet simulation...")
def sim():
    """
    Runs the PyBullet simulation.
    """

    last_q_solution = None
    last_target_position = None
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, 0)
    

    #p.loadURDF("plane.urdf")  # load the plane
    arm_id = p.loadURDF("URDF/arm.urdf", basePosition=constants.Constants.Robot.ARM_BASE_POSITION, useFixedBase=True)
    r2d2_id = p.loadURDF("r2d2.urdf", basePosition=constants.Constants.Robot.R2D2_BASE_POSITION, useFixedBase=True)

    # Disable default motor control for all joints
    for joint_index in range(7):
        p.setJointMotorControl2(
            arm_id,
            joint_index,
            p.VELOCITY_CONTROL,
            targetVelocity=0,
            force=0
        )

    # Compute the projection matrix for both cameras
    projectionMatrix = p.computeProjectionMatrixFOV(
        fov = constants.Constants.Camera.FOV,
        aspect = constants.Constants.Camera.WIDTH/constants.Constants.Camera.HEIGHT,
        nearVal=0.1,
        farVal=100.0
    )

    while p.isConnected(client):
        
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

        # Camera 1 Position and Orientation 
        viewMatrix1 = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[
                robot_controller.where_is_endeffector(arm_id)[0][0],
                robot_controller.where_is_endeffector(arm_id)[0][1],
                robot_controller.where_is_endeffector(arm_id)[0][2]-0.2
                ],
            distance=0.1,
            yaw=(180/math.pi)*robot_controller.where_is_endeffector(arm_id)[1], # RAD to DEG
            pitch=(180/math.pi)*robot_controller.where_is_endeffector(arm_id)[2],
            roll=(180/math.pi)*robot_controller.where_is_endeffector(arm_id)[3],
            upAxisIndex=2
   		)
        
        # Camera 2 Position and Orientation
        viewMatrix2 = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[
                robot_controller.where_is_endeffector(arm_id)[0][0]+1,
                robot_controller.where_is_endeffector(arm_id)[0][1],
                robot_controller.where_is_endeffector(arm_id)[0][2]-0.2
                ],
            distance=0.1,
            yaw=(180/math.pi)*robot_controller.where_is_endeffector(arm_id)[1], # RAD to DEG
            pitch=(180/math.pi)*robot_controller.where_is_endeffector(arm_id)[2],
            roll=(180/math.pi)*robot_controller.where_is_endeffector(arm_id)[3],
            upAxisIndex=2
   		)



        # Get camera images from both cameras
        img_arr1 = p.getCameraImage(
            constants.Constants.Camera.WIDTH,
            constants.Constants.Camera.HEIGHT,
			viewMatrix=viewMatrix1,
            projectionMatrix=projectionMatrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )

        img_arr2 = p.getCameraImage(
            constants.Constants.Camera.WIDTH, constants.Constants.Camera.HEIGHT,
			viewMatrix=viewMatrix2,
            projectionMatrix=projectionMatrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )

        # Extract the RGBA image
        rgba_img1 = np.reshape(img_arr1[2], (constants.Constants.Camera.HEIGHT, constants.Constants.Camera.WIDTH, 4)).astype(np.uint8)
        rgba_img2 = np.reshape(img_arr2[2], (constants.Constants.Camera.HEIGHT, constants.Constants.Camera.WIDTH, 4)).astype(np.uint8)


        # Shows the center of mass of the target object in both camera images in two different windows 
        opencv.center_of_mass(rgba_img1, constants.Constants.Camera.DETECT_COLOR_MIN, constants.Constants.Camera.DETECT_COLOR_MAX,"Left")
        opencv.center_of_mass(rgba_img2, constants.Constants.Camera.DETECT_COLOR_MIN, constants.Constants.Camera.DETECT_COLOR_MAX,"Right")


    
        # Go near a target by 2 m
        last_q_solution, last_target_position,_,__ = dodge.close_to_target(
            viewMatrix1,
            viewMatrix2,
            projectionMatrix,
            rgba_img1,
            rgba_img2,
            arm_id,
            last_q_solution,
            last_target_position,

        )

        p.removeBody(r2d2_id)

        # this is a debug line to visualize the distance between the end effector and the last known target position
        line_id = p.addUserDebugLine(
            lineFromXYZ=robot_controller.where_is_endeffector(arm_id)[0],
            lineToXYZ=last_target_position,
            lineColorRGB=[1, 0, 0],
            lineWidth=1,
            lifeTime=0.2,
            physicsClientId=0
        )
        
        robot_controller.go_to_PD(arm_id, last_q_solution)

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

def get_joint_angle(arm_id):
    """
    Returns the current angles of all joints in the robotic arm.
    Args:
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
    Returns:
        list: A list of current joint angles for the robotic arm.
    """

    joint_positions = []
    num_joints = p.getNumJoints(arm_id)

    for i in range(num_joints):

        joint_state = p.getJointState(arm_id, i)
        joint_positions.append(joint_state[0])  # Append the position (first element of joint_state)

    return joint_positions

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