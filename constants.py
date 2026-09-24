import numpy as np


class Constants:
    class Camera:

        WIDTH = 320 # width of the camera image
        HEIGHT = 240 # height of the camera image
        FOV = 60 # Field of View in degrees
        DETECT_COLOR_MAX = [0, 0, 255]  # Bright color for detection
        DETECT_COLOR_MIN = [0, 0, 55]   # Dark color for detection

    class Robot:

        ARM_BASE_POSITION = [0, 0, 0] # Base positions for the robot
        R2D2_BASE_POSITION = [0, 4, 1] # Base positions for the R2D2 robot
        END_EFFECTOR_LINK_INDEX = 6 # Index of the end effector link in the robot's URDF model

        JOINT_ORIGINS = np.array([ 
            [0.0,  0.0,  0.613788085],
            [0.0, -0.345, 0.0],
            [0.323, 0.0, 0.0],
            [0.37, 0.0, 3.646211915],
            [0.335, 0.0, -3.646211915],
            [0.0, -0.345, 0.0],
            [0.0, 0.0, -0.613788085]
        ]) # Joint origins for the robot's joints in 3D space

        JOINT_AXES = np.array([
            [0, 0, 1],
            [0, 1, 0],
            [1, 0, 0],
            [1, 0, 0],
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ]) # Joint axes for the robot's joints in 3D space

        JOINT_MIN = np.array([
            -3.124,
            -3.124,
            -3.124,
            -3.124,
            -3.124,
            -3.124,
            -3.124
        ]) # Minimum joint angles

        JOINT_MAX = np.array([
            3.124,
            3.124,
            3.124,
            3.124,
            3.124,
            3.124,
            3.124
        ]) # Maximum joint angles
        
        TARGET_SEPARATION = 0 # Separation in meters between the target and the end effector when the end effector is at the target position

        DESIRED_SETTLING_TIME = 0.4 # Desired settling time in seconds for the end effector to reach the target position

        DAMPING_RATIO = 1 # Damping ratio for the end effector's motion to the target position

        NATURAL_FREQUENCY = 4.0 / (DAMPING_RATIO * DESIRED_SETTLING_TIME) # Natural frequency for the end effector's motion to the target position
