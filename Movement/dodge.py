from Utils import utils
import Movement.robot_controller
import Movement.kinematics as kinematics
from Utils import utils
import Vision.opencv as opencv
import constants
import numpy as np


import sim

def close_to_target(viewMatrix1, viewMatrix2, projectionMatrix, rgba_img1, rgba_img2, arm_id, last_q_solution, last_target_position, separation=2):
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
        if caminfo[1] < 0.05:
            pose = caminfo[0]

            print("Target pose:", pose)
            print("Triangulation error:", caminfo[1])

            current_position = np.array(Movement.robot_controller.where_is_endeffector(arm_id)[0])
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

                yaw, pitch = Movement.robot_controller.auto_aim(pose, viewMatrix1, viewMatrix2)
                roll = np.radians(0)
                target_orientation = utils.rpy_rotation(roll, pitch, yaw)
                q_solution = kinematics.inverse_kinematics(pose,target_orientation,sim.get_joint_angle(arm_id))
                last_q_solution = q_solution
                last_target_position = pose

                is_close = False
                is_seeing_target = True

                return last_q_solution, last_target_position, is_close, is_seeing_target

        else:

            print("Triangulation error too high.")
            # is not close to target, and cannot see it
            return last_q_solution, last_target_position, is_close, is_seeing_target
    else:

        if last_q_solution is not None and last_target_position is not None:

            current_position = np.array(Movement.robot_controller.where_is_endeffector(arm_id)[0])

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

def aproach(viewMatrix1, viewMatrix2, projectionMatrix, rgba_img1, rgba_img2, arm_id, last_q_solution, last_target_position, separation=2):
        n, a, is_close, is_seeing_target = close_to_target(
            viewMatrix1,
            viewMatrix2,
            projectionMatrix,
            rgba_img1,
            rgba_img2,
            arm_id,
            last_q_solution,
            last_target_position,
            separation=constants.Constants.Robot.TARGET_SEPARATION
        )
        if is_close:
            print("End effector is close to target.")
            Movement.robot_controller.stay(arm_id)
            return n, a
        else:
            if is_seeing_target:


                print("end effector can see the target, moving toward it.")
                Movement.robot_controller.go_to(arm_id, 7, n)
                return n, a
            else:
                print("End effector cannot see the target, moving toward last known position.")
                Movement.robot_controller.go_to(arm_id, 7, n)
                return n, a
