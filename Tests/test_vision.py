import numpy as np
import pybullet as p
import pybullet_data
from Movement import dodge
from Vision import opencv
from Utils import logger

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
                log.log_tests(
                    any1=iterations, #  we can log the test number
                    any2 = (last_target_position.tolist() if last_target_position is not None else [0,0,0]),  # Log the Vision position 
                    any3=error,  # Log the error 
                    any4=distance,  # Log the distance 
                    any5=can_see if can_see else [False],  
                    any6=[j-size/2, i+7, k-size/2],
                )
                iterations += 1
                if last_target_position is None:
                    can_see[-1] = False
    print(f"Total Successes: {sum(can_see)}/{number_of_tests}, {size}")

    log.close()
    logger.test_logger(log.file.name, succesful_tests=sum(can_see), kind='vision')
