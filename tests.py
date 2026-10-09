import numpy as np
import pybullet as p
import pybullet_data
import constants
from Movement import kinematics, dodge, robot_controller, trajectory
from Vision import opencv
from Utils import logger, utils

def test_kinematics(number_of_tests=10):
    """
    Test the kinematics functions.
    Args:
        number_of_tests (int): Number of random tests to perform.
    """
    log = logger.RobotLogger()
    success_count = 0
    max_iterations_reachead = 0
    
    for i in range(number_of_tests):
    
    # Generate a random 3D position inside a sphere of radius 7 meters
        direction = np.random.normal(size=3)
        direction /= np.linalg.norm(direction)

        radius = 7 * np.random.random() ** (1 / 3)

        random_3d_pose = (direction * radius).tolist()

        print("q_random:", random_3d_pose)

        yaw = np.random.uniform(-np.pi, np.pi)
        pitch = np.random.uniform(-np.pi, np.pi)
        roll = np.random.uniform(-np.pi, np.pi)

        random_orientation = utils.rpy_rotation(roll, pitch, yaw)


        q_solution = kinematics.inverse_kinematics(random_3d_pose, random_orientation,max_iterations=1001)
        if q_solution[1] == 1001 :
            print(f"Test {i + 1}: IK did not converge after {q_solution[1]} iterations.")
            print(f"Expected: {random_3d_pose}, Got: {kinematics.forward_kinematics(q_solution[0])[0][:3, 3].tolist()}")
            max_iterations_reachead += 1
            continue
        position_solution = kinematics.forward_kinematics(q_solution[0])[0][:3, 3] # Extract the position from the transformation matrix

        print("q_solution:", q_solution[0])
        print("end_effector_position_solution:", position_solution)

        pose_error = utils.pose_error(kinematics.forward_kinematics(q_solution[0])[0], random_3d_pose, random_orientation)
        orientation_error = np.linalg.norm(pose_error.tolist()[3:6])  # Calculate the Euclidean norm of the orientation error
        pose_error = np.linalg.norm(pose_error.tolist()[0:3])  # Calculate the Euclidean norm of the position error
        print("pose_error:", pose_error)
        print("orientation_error:", orientation_error)

        log.log_joint(
            joint=i, # instead of logging the joint index, we can log the test number
            desired_position=pose_error,  # Log the pose error as the desired position
            actual_position=orientation_error,  # Log the orientation error as the actual position
            actual_velocity=[yaw, pitch, roll],  # Log the random orientation as the actual velocity
            additional_info=position_solution.tolist(),  
            target_position=random_3d_pose,
        )

        if pose_error < 1e-3 and orientation_error < 1e-3:

            print(f"Test {i + 1}: Success")
            success_count += 1

        else:

            print(f"Test {i + 1}: Failure")
            print(f"Expected: {random_3d_pose}, Got: {position_solution}")

        print("--------------------------------------------------")
        print(f"Total Successes: {success_count}/{number_of_tests}")
        print(f"Total Failed: {number_of_tests - success_count}/{number_of_tests}")
        print(f"Total Max Iterations Reached: {max_iterations_reachead}/{number_of_tests}")
    log.close()
    print("Simulation ended. Generating plots...")
    logger.test_logger(log.file.name, succesful_tests=success_count)
