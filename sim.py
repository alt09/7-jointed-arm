
import time
import numpy as np
import pybullet as p
import pybullet_data
import math
import constants
from Movement import dodge, robot_controller
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
        endeffector_info = robot_controller.get_end_effector_state(arm_id)

        viewMatrix1, viewMatrix2 = opencv.get_view_matrix(endeffector_info)
        img_arr1, img_arr2 = opencv.get_camera_images(viewMatrix1, viewMatrix2, projectionMatrix)
        rgba_img1 = opencv.extract_rgba_image(img_arr1)
        rgba_img2 = opencv.extract_rgba_image(img_arr2)

        opencv.show_center_of_mass(endeffector_info, [0, 100, 100], [10, 255, 255])
        # Go near a target by 2 m
        last_q_solution, last_target_position = dodge.approach(
            viewMatrix1,
            viewMatrix2,
            projectionMatrix,
            rgba_img1,
            rgba_img2,
            arm_id,
            last_q_solution,
            last_target_position,
            endeffector_info
        )

        # this is a debug line to visualize the distance between the end effector and the last known target position
        if last_target_position is not None:
            line_id = p.addUserDebugLine(
                lineFromXYZ = endeffector_info[0],
                lineToXYZ = last_target_position,
                lineColorRGB = [1, 0, 0],
                lineWidth = 1,
                lifeTime = 0.2,
                physicsClientId = 0
            )

        # remove r2d2
        # p.removeBody(r2d2_id)
