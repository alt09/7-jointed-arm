from Utils import utils
from Movement import robot_controller, kinematics
from Vision import opencv
import constants
import numpy as np


def close_to_target(viewMatrix1, viewMatrix2, projectionMatrix, rgba_img1, rgba_img2, arm_id, last_q_solution, last_target_position, endeffector_info, separation=2):
    """
    Determines if the end effector is close to the target.
    Args:
        viewMatrix1 (list): The view matrix of the first camera.
        viewMatrix2 (list): The view matrix of the second camera.
        projectionMatrix (list): The projection matrix used for both cameras.
        rgba_img1 (numpy.ndarray): The RGBA image from the first camera.
        rgba_img2 (numpy.ndarray): The RGBA image from the second camera.
        arm_id (int): The ID of the robot arm.
        last_q_solution (list): The last known joint angles of the robot arm.
        last_target_position (list): The last known position of the target.
        separation (float): The distance within which the end effector is considered close to the target.
    Returns:
        tuple: A tuple containing:
            the updated joint angles,
            target position,
            and boolean values indicating if the end effector is close to the target and if it is seeing the target.
    """

    is_close = False
    is_seeing_target = False

    caminfo = opencv.target_3d_pose(
        viewMatrix1,
        viewMatrix2,
        projectionMatrix,
        rgba_img1,
        rgba_img2,
        constants.Constants.Camera.DETECT_COLOR_MIN,
        constants.Constants.Camera.DETECT_COLOR_MAX
    )

    if caminfo is not None:

        if caminfo[1] < 0.05: # 0.05 is the triangulation error threshold, if the error is less than this value, we consider the target to be detected

            pose = caminfo[0]

            print("Target pose:", pose)
            print("Triangulation error:", caminfo[1])

            current_position = np.array(endeffector_info[0])
            position_error = np.linalg.norm(current_position - np.array(pose))

            print(f"Position error: {position_error:.4f} m")

            if position_error < separation:

                # is close and can see the target
                print("End effector is close to target.")
                is_close = True
                is_seeing_target = True

                return last_q_solution, pose, is_close, is_seeing_target

            else:

                # is not close to target, but can see it
                print("End effector is not close to target.")

                yaw, pitch = robot_controller.auto_aim(pose, viewMatrix1, viewMatrix2)
                roll = np.radians(0)
                target_orientation = utils.rpy_rotation(roll, pitch, yaw)
                q_solution = kinematics.inverse_kinematics(pose,target_orientation,robot_controller.get_current_joint_positions(arm_id))
                last_q_solution = q_solution
                last_target_position = pose

                is_close = False
                is_seeing_target = True

                return last_q_solution, last_target_position, is_close, is_seeing_target

        else:

            # is not close to target, and cannot see it
            print("Triangulation error too high.")

            return last_q_solution, last_target_position, is_close, is_seeing_target
    else:

        if last_q_solution is not None and last_target_position is not None:

            current_position = np.array(endeffector_info[0])

            position_error = np.linalg.norm(current_position - np.array(last_target_position))

            print(f"No target detected. "
                  f"Distance from last target: {position_error:.4f} m")

            if position_error < separation:

                # is close to last known target, but cannot see it
                print("End effector is close to last known target.")
                is_close = True
                is_seeing_target = False

            else:

                # is not close to last known target, and cannot see it
                is_close = False
                is_seeing_target = False
                print("Moving toward last known target.")
        else:

            print("No target and no last known target.")
            is_close = False
            is_seeing_target = False

        return last_q_solution, last_target_position, is_close, is_seeing_target

def approach(viewMatrix1, viewMatrix2, projectionMatrix, rgba_img1, rgba_img2, arm_id, last_q_solution, last_target_position, endeffector_info):

    """
    Approaches the target based on the camera views and the current state of the robot arm.
    Args:
        viewMatrix1 (list): The view matrix of the first camera.
        viewMatrix2 (list): The view matrix of the second camera.
        projectionMatrix (list): The projection matrix used for both cameras.
        rgba_img1 (numpy.ndarray): The RGBA image from the first camera.
        rgba_img2 (numpy.ndarray): The RGBA image from the second camera.
        arm_id (int): The ID of the robot arm.
        last_q_solution (list): The last known joint angles of the robot arm.
        last_target_position (list): The last known position of the target.
    Returns:
        tuple: A tuple containing the updated joint angles and target position.
    """
    n, a, is_close, is_seeing_target = close_to_target(
        viewMatrix1,
        viewMatrix2,
        projectionMatrix,
        rgba_img1,
        rgba_img2,
        arm_id,
        last_q_solution,
        last_target_position,
        endeffector_info,
        separation=constants.Constants.Robot.TARGET_SEPARATION
    )
    
    if is_close:
        print("End effector is close to target.")
        robot_controller.stay(arm_id)

        return n, a
    
    else:

        if is_seeing_target:

            print("end effector can see the target, moving toward it.")
            robot_controller.go_to_PD(arm_id, n)
            return n, a
        else:

            print("End effector cannot see the target, moving toward last known position.")
            robot_controller.go_to_PD(arm_id, n)
            
            return n, a
