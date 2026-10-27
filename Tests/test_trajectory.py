
import numpy as np
from Movement import trajectory
from Utils import logger
import constants

def test_trajectory(number_of_tests=100):
    dt = 1.0 / 240.0
    log = logger.RobotLogger()

    for i in range(number_of_tests):
        # Define the start and goal joint positions for the trajectory
        q_start = np.random.uniform(low=-np.pi, high=np.pi, size=7)  # Random start positions for 7 joints
        q_goal = np.random.uniform(low=-np.pi, high=np.pi, size=7)  # Random goal positions for 7 joints

        # generate a trajectory from the current joint positions to the goal joint positions
        traj = trajectory.generate_trajectory(
            q_start=np.array(q_start),
            q_goal=np.array(q_goal),
            max_velocity=np.array(constants.Constants.Robot.JOINT_MAX_VELOCITY),  # max velocity for each joint
            max_acceleration=np.array(constants.Constants.Robot.JOINT_MAX_ACCELERATION),  # max acceleration for each joint
            dt=dt
        )

        # print(traj) 
        log.log_tests(
            any1=i,
            any2=q_start,
            any3=q_goal,
            any4=([point["time"] for point in traj]), # position error
            any5=([point["velocities"].tolist() for point in traj]), # actual velocity 
            any6=([point["accelerations"].tolist() for point in traj]) # additional info
        )
        print(f"Test {i+1}/{number_of_tests} completed.")
    log.close()
    logger.test_logger(log.file.name, kind='trajectory', succesful_tests=number_of_tests)

                