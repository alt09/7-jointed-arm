import numpy as np


class Constants:
    class Camera:
        WIDTH = 320
        HEIGHT = 240
        FOV = 60
        DETECT_COLOR_MAX = [0, 0, 255]  # Green color for detection
        DETECT_COLOR_MIN = [0, 0, 55]   # Dark color for detection
    class Robot:
        ARM_BASE_POSITION = [0, 0, 0]
        R2D2_BASE_POSITION = [0, 4, 1]
        END_EFFECTOR_LINK_INDEX = 6

        JOINT_ORIGINS = np.array([
        [0.0,  0.0,  0.613788085],
        [0.0, -0.345, 0.0],
        [0.323, 0.0, 0.0],
        [0.37, 0.0, 3.646211915],
        [0.335, 0.0, -3.646211915],
        [0.0, -0.345, 0.0],
        [0.0, 0.0, -0.613788085]
        ])

        JOINT_AXES = np.array([
        [0, 0, 1],
        [0, 1, 0],
        [1, 0, 0],
        [1, 0, 0],
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
        ])

        JOINT_MIN = np.array([
            -3.124,
            -3.124,
            -3.124,
            -3.124,
            -3.124,
            -3.124,
            -3.124
        ])

        JOINT_MAX = np.array([
            3.124,
            3.124,
            3.124,
            3.124,
            3.124,
            3.124,
            3.124
        ])
