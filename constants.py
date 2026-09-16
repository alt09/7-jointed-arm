import numpy as np


class Constants:
    class Camera:
        width = 320
        height = 240
        fov = 60
        detect_color_max = [0, 255, 0]  # Green color for detection
        detect_color_min = [0, 0, 55]   # Dark color for detection
    class Robot:
        arm_urdf_path = "URDF/arm.urdf"
        r2d2_urdf_path = "r2d2.urdf"
        arm_base_position = [0, 0, 1]
        r2d2_base_position = [0, 4, 1]
        end_effector_link_index = 6

        joint_origins = np.array([
        [0.0,  0.0,  0.613788085],
        [0.0, -0.345, 0.0],
        [0.323, 0.0, 0.0],
        [0.37, 0.0, 3.646211915],
        [0.335, 0.0, -3.646211915],
        [0.0, -0.345, 0.0],
        [0.0, 0.0, -0.613788085]
        ])

        joint_axes = np.array([
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
