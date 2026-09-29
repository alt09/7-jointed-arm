
import time
import numpy as np
import pybullet as p
import pybullet_data
import math
import constants
from Movement import dodge, robot_controller, trajectory
from Vision import opencv
from Utils.logger import RobotLogger

def sim():
    """
    Runs the PyBullet simulation.
    """
    print("Starting PyBullet simulation...")

    last_q_solution = None
    last_target_position = None
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, 0)

    logger = RobotLogger()
    dt = 1.0 / 240.0  # Simulation time step
    sim_time = 0.0
    

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
    try:

        while p.isConnected(client):
            for i in range(7):

                state = robot_controller.get_joint_info(arm_id)

                actual_position = 1
                actual_velocity = 1

                desired_position = 0 

                # Replace with the actual torque
                # calculated by your controller.
                control_torque = 0

                logger.log_joint(
                    joint=i,
                    desired_position=desired_position,
                    actual_position=actual_position,
                    actual_velocity=actual_velocity,
                    control_torque=control_torque
                )
            
            p.stepSimulation()

            # Camera 1 Position and Orientation 
            endeffector_info = robot_controller.get_end_effector_state(arm_id)
            cam_info = opencv.get_info_from_camera_image(endeffector_info)


            # Go near a target by 2 m
            last_q_solution, last_target_position = dodge.approach(
                cam_info[0],
                cam_info[1],
                cam_info[2],
                cam_info[3],
                cam_info[4],
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

            t = trajectory.generate_trajectory(
                q_start=[
                    0.0,    # Joint 1
                    0.2,    # Joint 2
                    -0.3,   # Joint 3
                    0.0,    # Joint 4
                    0.5,    # Joint 5
                    -0.2,   # Joint 6
                    0.0     # Joint 7
                ],

                q_goal=[
                    1.0,    # Joint 1
                    -0.5,   # Joint 2
                    0.8,    # Joint 3
                    0.3,    # Joint 4
                    -0.4,   # Joint 5
                    0.6,    # Joint 6
                    -0.2    # Joint 7
                ],

                max_velocity=[
                    0.5,    # Joint 1: rad/s
                    0.4,    # Joint 2
                    0.6,    # Joint 3
                    0.5,    # Joint 4
                    0.4,    # Joint 5
                    0.6,    # Joint 6
                    0.5     # Joint 7
                ],

                max_acceleration=[
                    1.0,    # Joint 1: rad/s²
                    0.8,    # Joint 2
                    1.2,    # Joint 3
                    1.0,    # Joint 4
                    0.8,    # Joint 5
                    1.2,    # Joint 6
                    1.0     # Joint 7
                ],

                dt=0.02
            )
            for sample in t:
                print(
                    f"Time: {sample['time']}, Position: {sample['positions'][0]}, Velocity: {sample['velocities'][0]}, Acceleration: {sample['accelerations'][0]}"
                )
            break  # Remove this break to run the simulation continuously
    
    finally:    
        logger.close()