import pybullet as p
import numpy as np
from time import time
from  Movement import robot_controller

def generate_trajectory(q_start, q_goal, max_velocity, max_acceleration, dt=0.02):
    """
    Generates a trajectory from the starting joint positions to the goal joint positions.
    Args:
        q_start (np.ndarray): Starting joint positions.
        q_goal (np.ndarray): Goal joint positions.
        max_velocity (np.ndarray): Maximum velocity for each joint.
        max_acceleration (np.ndarray): Maximum acceleration for each joint.
        dt (float): Time step for the trajectory.
    Returns:
        list: A list of dictionaries containing time, positions, velocities, and accelerations for each time step.
    """
    # Convert inputs to numpy arrays and ensure they are of type float64
    q_start = np.array(q_start, dtype = np.float64)
    q_goal = np.array(q_goal, dtype = np.float64)

    vmax = np.array(max_velocity, dtype = np.float64)
    amax = np.array(max_acceleration, dtype = np.float64)

    # Validate inputs
    if dt <= 0 or np.any(vmax <= 0) or np.any(amax <= 0):

        raise ValueError("dt, velocity, and acceleration must be positive")

    delta = q_goal - q_start
    distance = np.abs(delta)

    # Check if the distance is effectively zero for all joints
    if np.all(distance < 1e-10):

        return [{
            "time": 0.0,
            "positions": q_goal.copy(),
            "velocities": np.zeros_like(q_goal),
            "accelerations": np.zeros_like(q_goal)
        }]
    
    # Determine which joints are moving (i.e., have a non-zero distance to travel)
    moving = distance > 1e-10

    V = np.min(vmax[moving] / distance[moving])
    A = np.min(amax[moving] / distance[moving])

    # Calculate the time to reach peak velocity and the time to cruise at peak velocity
    if V**2 / A >= 1.0:

        peak_velocity = np.sqrt(A)
        t_accel = peak_velocity / A
        t_cruise = 0.0

    # If the peak velocity is less than the maximum velocity, calculate the time to accelerate and cruise
    else:
        
        peak_velocity = V
        t_accel = V / A
        t_cruise = (1.0 - V**2 / A) / V

    # Calculate the total duration of the trajectory
    duration = 2 * t_accel + t_cruise

    times = np.arange(0.0, duration, dt)
    times = np.append(times, duration)

    trajectory = []

    # Generate the trajectory by calculating positions, velocities, and accelerations at each time step
    for t in times:

        # Calculate the normalized position (s), velocity (sd), and acceleration (sdd) based on the current time
        if t < t_accel:

            s = 0.5 * A * t**2
            sd = A * t
            sdd = A

        # If the current time is during the cruise phase, calculate the normalized position, velocity, and acceleration accordingly
        elif t < t_accel + t_cruise:

            s = 0.5 * A * t_accel**2 + peak_velocity * (t - t_accel)
            sd = peak_velocity
            sdd = 0.0

        # If the current time is during the deceleration phase, calculate the normalized position, velocity, and acceleration accordingly
        elif t < duration:

            remaining = duration - t
            s = 1.0 - 0.5 * A * remaining**2
            sd = A * remaining
            sdd = -A

        # If the current time exceeds the total duration, set the normalized position, velocity, and acceleration to their final values
        else:

            s = 1.0
            sd = 0.0
            sdd = 0.0
        # Append the calculated values to the trajectory list
        trajectory.append({
            "time": float(t),
            "positions": q_start + s * delta,
            "velocities": sd * delta,
            "accelerations": sdd * delta
        })

    return trajectory

def follow_trajectory(arm_id, trajectory):
    """
    Follows a given trajectory by commanding the robot's joints to move to the desired positions and velocities at each time step.
    Args:
        arm_id (int): The ID of the robot arm in the PyBullet simulation.
        trajectory (list): A list of dictionaries containing time, positions, velocities, and accelerations for each time step.
    """
    for sample in trajectory:

        # Extract the desired joint positions and velocities from the trajectory sample
        q_desired = np.array(sample["positions"])
        qdot_desired = np.array(sample["velocities"])
        q_current = [] 
        qdot_current = [] 

        # Get the current joint positions and velocities from the simulation
        for joint in range(7):

            state = p.getJointState(arm_id, joint)
            qdot_current.append(state[1])  # joint velocity
            q_current.append(state[0])  # joint position
        
        # Convert the current joint positions and velocities to numpy arrays
        q_current = np.array(q_current)
        qdot_current = np.array(qdot_current)

        # Command the robot to move to the desired joint positions and velocities using a PD controller
        robot_controller.go_to_PD(arm_id, q_desired, qdot_desired)

        # Step the simulation to update the robot's state
        p.stepSimulation()
