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
    logger.test_logger(log.file.name, succesful_tests=success_count, kind='kinematics')

def test_vision(number_of_tests=8):
    """
    Test the vision functions.
    Args:
        number_of_tests (int): Number of random tests to perform.
    """
    last_q_solution = None
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, 0)

    log = logger.RobotLogger()
    #p.loadURDF("plane.urdf")  # load the plane
    robot = p.loadURDF("cube.urdf", basePosition=(0, 0, 0), useFixedBase=True)
    can_see = []
    iterations=0

    size = int(np.cbrt(number_of_tests)) # size of the grid to place the spheres in a square pattern
    print(f"Placing {number_of_tests} spheres in a {size:.2f}x{size:.2f} grid.")

        
    for i in range(size):
        for j in range(size):
            for k in range(size):
                r2d2_id = p.loadURDF("sphere2.urdf", basePosition=(j-size/2, i+7, k-size/2), useFixedBase=True)

                print(f"Placing sphere at ({j-size/2}, {i+7}, {k-size/2}), test {iterations + 1}/{number_of_tests}")
                p.stepSimulation()

                
                

                viewMatrix1 = p.computeViewMatrixFromYawPitchRoll(
                    cameraTargetPosition = [0.5,0.6,0
                        ],
                    distance = 0.1,
                    yaw = 0, 
                    pitch = 0,
                    roll = 0,
                    upAxisIndex = 2
                )

                # Camera 2 Position and Orientation
                viewMatrix2 = p.computeViewMatrixFromYawPitchRoll(
                    cameraTargetPosition=[
                        -0.5,0.6,0
                        ],
                    distance = 0.1,
                    yaw = 0, 
                    pitch = 0,
                    roll = 0,
                    upAxisIndex = 2
                )
                cam_info =opencv.get_info_from_camera_image(endeffector_info=None, viewMatrix1=viewMatrix1, viewMatrix2=viewMatrix2)
                line_id = p.addUserDebugLine(
                            lineFromXYZ = opencv.camera_pose_from_view_matrix(viewMatrix1)[0].tolist(),
                            lineToXYZ = [1, 0, 0],
                            lineColorRGB = [1, 0, 0],
                            lineWidth = 1,
                            # lifeTime = 0.1,  # line will be visible for 0.1 seconds                
                            physicsClientId = 0
                        )
                line2_id = p.addUserDebugLine(
                            lineFromXYZ = opencv.camera_pose_from_view_matrix(viewMatrix2)[0].tolist(),
                            lineToXYZ = [1, 0, 0],
                            lineColorRGB = [0, 1, 0],
                            lineWidth = 1,
                            # lifeTime = 0.1,  # line will be visible for 0.1 seconds                
                            physicsClientId = 0
                        )
                            # Go near a target by 2 m
                
                last_target_position = None
                last_q_solution, last_target_position,_,is_seeing = dodge.close_to_target(
                    cam_info[0],
                    cam_info[1],
                    cam_info[2],
                    cam_info[3],
                    cam_info[4],
                    None,
                    last_q_solution,
                    last_target_position,
                    [[0.5,0.5,0],0]
                )

                # print("last_q_solution:", last_q_solution)
                p.removeBody(r2d2_id)
                print("last_target_position:", last_target_position.tolist() if last_target_position is not None else None)
                can_see.append(True)

                error = np.linalg.norm(np.array(last_target_position.tolist()) - np.array([j-size/2, i+7, k-size/2])) if last_target_position is not None else None
                distance = np.linalg.norm(np.array([j-size/2, i+7, k-size/2]) - np.array([0.5,0.5,0]))
                log.log_joint(
                    joint=iterations, # instead of logging the joint index, we can log the test number
                    desired_position = (last_target_position.tolist() if last_target_position is not None else [0,0,0]),  # Log the Vision position as the desired position
                    actual_position=error,  # Log the error as the actual position
                    actual_velocity=distance,  # Log the distance as the actual velocity
                    additional_info=can_see if can_see else [False],  
                    target_position=[j-size/2, i+7, k-size/2],
                )
                iterations += 1
                if last_target_position is None:
                    can_see[-1] = False
    print(f"Total Successes: {sum(can_see)}/{number_of_tests}, {size}")

    log.close()
    logger.test_logger(log.file.name, succesful_tests=sum(can_see), kind='vision')
