
import time
import numpy as np
import pybullet as p
import pybullet_data
import math

import Vision.opencv as opencv

import Movement.go_to as go_to

print("Starting PyBullet simulation...")
target_last_pose = None
def sim():
    """
    Runs the PyBullet simulation.
    """
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, 0)
    

    #p.loadURDF("plane.urdf")  # load the plane
    arm_id = p.loadURDF("arm.urdf", basePosition=[0, 0, 1], useFixedBase=True)
    r2d2_id = p.loadURDF("r2d2.urdf", basePosition=[0, 4, 1], useFixedBase=True)

    width, height = 320, 240    
    while p.isConnected(client):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)
        # Camera 1 Position and Orientation 
        viewMatrix1 = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=[
            go_to.where_is(arm_id)[0][0],
            go_to.where_is(arm_id)[0][1],
            go_to.where_is(arm_id)[0][2]-0.2
        ],
        distance=0.1,
        yaw=(180/math.pi)*go_to.where_is(arm_id)[1], # RAD to DEG
        pitch=(180/math.pi)*go_to.where_is(arm_id)[2],
        roll=(180/math.pi)*go_to.where_is(arm_id)[3],
        upAxisIndex=2
   		)
        viewMatrix2 = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=[
            go_to.where_is(arm_id)[0][0]+1,
            go_to.where_is(arm_id)[0][1],
            go_to.where_is(arm_id)[0][2]-0.2
        ],
        distance=0.1,
        yaw=(180/math.pi)*go_to.where_is(arm_id)[1], # RAD to DEG
        pitch=(180/math.pi)*go_to.where_is(arm_id)[2],
        roll=(180/math.pi)*go_to.where_is(arm_id)[3],
        upAxisIndex=2
   		)
        # print((go_to.where_is(arm_id)))
        projectionMatrix = p.computeProjectionMatrixFOV(
    	    fov=60, aspect=width/height, nearVal=0.1, farVal=100.0
    	)
        img_arr1 = p.getCameraImage(
            width, height,
			viewMatrix=viewMatrix1,
            projectionMatrix=projectionMatrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )
        img_arr2 = p.getCameraImage(
            width, height,
			viewMatrix=viewMatrix2,
            projectionMatrix=projectionMatrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )
        # Extract the RGBA image
        rgba_img1 = np.reshape(img_arr1[2], (height, width, 4)).astype(np.uint8)
        rgba_img2 = np.reshape(img_arr2[2], (height, width, 4)).astype(np.uint8)


        opencv.center_of_mass(rgba_img1, [0, 0, 55], [0, 0, 255],"Left")
        opencv.center_of_mass(rgba_img2, [0, 0, 55], [0, 0, 255],"Right")
        
       # go_to.go_to_target(arm_id, [4, 0, 1])
        if opencv.target_3d_pose(viewMatrix1,viewMatrix2,projectionMatrix,rgba_img1,rgba_img2,[0, 0, 55], [0, 0, 255]) is not None:
            go_to.cheats(arm_id,opencv.target_3d_pose(viewMatrix1,viewMatrix2,projectionMatrix,rgba_img1,rgba_img2,[0, 0, 55], [0, 0, 255])[0],viewMatrix1,viewMatrix2)
            target_last_pose = opencv.target_3d_pose(viewMatrix1,viewMatrix2,projectionMatrix,rgba_img1,rgba_img2,[0, 0, 55], [0, 0, 255])[0]
        if target_last_pose is not None and opencv.target_3d_pose(viewMatrix1,viewMatrix2,projectionMatrix,rgba_img1,rgba_img2,[0, 0, 255], [0, 0, 100]) is None:
            go_to.cheats(arm_id,target_last_pose,viewMatrix1,viewMatrix2)
            print("target_last_pose",target_last_pose)
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

