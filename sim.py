import numpy as np
import pybullet as p
import pybullet_data
import constants
from Movement import kinematics, dodge, robot_controller, trajectory
from Vision import opencv
from Utils import logger, utils


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

    log = logger.RobotLogger()
    dt = 1.0 / 240.0  # Simulation time step
    
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

            # Camera 1 Position and Orientation 
            endeffector_info = robot_controller.get_end_effector_state(arm_id)
            cam_info = opencv.get_info_from_camera_image(endeffector_info)


            # Go near a target by 2 m
            last_q_solution, last_target_position,_,__ = dodge.close_to_target(
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

            # get the yaw and pitch angles to aim the camera at the target
            yaw, pitch = robot_controller.auto_aim(last_target_position, cam_info[0], cam_info[1])
            roll = np.radians(0)
            target_orientation = utils.rpy_rotation(roll, pitch, yaw)

            # compute the joint angles to reach the target position with the desired orientation
            q_goal = kinematics.inverse_kinematics(last_target_position, target_orientation, last_q_solution)

            # get the current joint positions and velocities
            q_current = []
            qdot_current = []

            for joint in range(7):

                joint_state = p.getJointState(arm_id, joint)
                qdot_current.append(joint_state[1])  # joint velocity
                q_current.append(joint_state[0])  # joint position

            # generate a trajectory from the current joint positions to the goal joint positions
            traj = trajectory.generate_trajectory(
                q_start=np.array(q_current),
                q_goal=np.array(q_goal),
                max_velocity=np.array([1.0] * 7),  # max velocity for each joint
                max_acceleration=np.array([1.0] * 7),  # max acceleration for each joint
                dt=dt
            )

            # remove the previous R2D2 model if it exists
            if r2d2_id is not None:

                p.removeBody(r2d2_id)
                r2d2_id = None
            
            # follow the generated trajectory
            trajectory.follow_trajectory(arm_id, traj) 

            # this is a debug line to visualize the distance between the end effector and the last known target position
            if last_target_position is not None:

                line_id = p.addUserDebugLine(
                    lineFromXYZ = endeffector_info[0],
                    lineToXYZ = last_target_position,
                    lineColorRGB = [1, 0, 0],
                    lineWidth = 1,
                    # lifeTime = 0.1,  # line will be visible for 0.1 seconds                
                    physicsClientId = 0
                )

            for i in range(7):

                actual_position = q_current[i]
                actual_velocity = qdot_current[i]

                desired_position = q_goal[i]

                log.log_joint(
                    joint=i,
                    desired_position=desired_position,
                    actual_position=actual_position,
                    actual_velocity=actual_velocity,
                    additional_info=robot_controller.get_end_effector_state(arm_id)[0],  # Log the end effector position as additional info
                    target_position=last_target_position.tolist(),
                    end_time=10.0  # Stop logging after 10 seconds 
                )
    
    finally:  

        # Close the logger and plot the joint data
        log.close()
        print("Simulation ended. Generating plots...")
        logger.plot_joint_data(log.file.name)