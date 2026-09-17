
import time
import numpy as np
import pybullet as p
import pybullet_data
import math
import Movement.kinematics as kinematics
from Utils import utils
from Utils.utils import orientation_error, pose_error, pose_error, rotation_about_axis, rotation_matrix
import Vision.opencv as opencv
import constants

import Movement.robot_controller as robot_controller

print("Starting PyBullet simulation...")
def sim():
    """
    Runs the PyBullet simulation.
    """
    q_solution = None
    last_q_solution = None
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, 0)
    

    #p.loadURDF("plane.urdf")  # load the plane
    arm_id = p.loadURDF("URDF/arm.urdf", basePosition=constants.Constants.Robot.arm_base_position, useFixedBase=True)
    r2d2_id = p.loadURDF("r2d2.urdf", basePosition=constants.Constants.Robot.r2d2_base_position, useFixedBase=True)
   
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
        # print((go_to.where_is_endeffector(arm_id)))
        projectionMatrix = p.computeProjectionMatrixFOV(
    	    fov = constants.Constants.Camera.fov,
            aspect = constants.Constants.Camera.width/constants.Constants.Camera.height,
            nearVal=0.1,
            farVal=100.0
    	)
        img_arr1 = p.getCameraImage(
            constants.Constants.Camera.width,
            constants.Constants.Camera.height,
			viewMatrix=viewMatrix1,
            projectionMatrix=projectionMatrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )
        img_arr2 = p.getCameraImage(
            constants.Constants.Camera.width, constants.Constants.Camera.height,
			viewMatrix=viewMatrix2,
            projectionMatrix=projectionMatrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )
        # Extract the RGBA image
        rgba_img1 = np.reshape(img_arr1[2], (constants.Constants.Camera.height, constants.Constants.Camera.width, 4)).astype(np.uint8)
        rgba_img2 = np.reshape(img_arr2[2], (constants.Constants.Camera.height, constants.Constants.Camera.width, 4)).astype(np.uint8)



        opencv.center_of_mass(rgba_img1, constants.Constants.Camera.detect_color_min, constants.Constants.Camera.detect_color_max,"Left")
        opencv.center_of_mass(rgba_img2, constants.Constants.Camera.detect_color_min, constants.Constants.Camera.detect_color_max,"Right")
        
        if opencv.target_3d_pose(viewMatrix1,viewMatrix2,projectionMatrix,rgba_img1,rgba_img2,constants.Constants.Camera.detect_color_min, constants.Constants.Camera.detect_color_max) is not None:
            caminfo = opencv.target_3d_pose(viewMatrix1,viewMatrix2,projectionMatrix,rgba_img1,rgba_img2,constants.Constants.Camera.detect_color_min, constants.Constants.Camera.detect_color_max)
            if caminfo[1] < 0.05:
                pose = caminfo[0]
                angle = np.radians(0)
                print("Target pose",pose)

                q_solution = kinematics.inverse_kinematics(
                    target_position = pose,
                    target_orientation = utils.rotation_matrix([0, 0, 1], angle),
                    initial_q = get_joint_angle(arm_id)
                    )
                last_q_solution = q_solution
                robot_controller.go_to(arm_id, len(q_solution), q_solution)
                print("robot position:",robot_controller.where_is_endeffector(arm_id))
                print("last known position:",kinematics.forward_kinematics(q_solution)[0][:3, 3])

            else:
                print("\nTriangulation error too high, not moving the arm.**********************************")
        else:
            if last_q_solution is not None:
                q_solution = last_q_solution
                print("No target detected, moving to last known position.")
                robot_controller.go_to(arm_id, 7, q_solution)
                print("robot position:",robot_controller.where_is_endeffector(arm_id))
                print("last known position:",kinematics.forward_kinematics(q_solution)[0][:3, 3])

            else:
                print("No target detected and no last known position available.going to 0,0,0")

                robot_controller.go_to_target(arm_id, [0, 0, 0], [0, 0, 0, 1])






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
        force=100
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

