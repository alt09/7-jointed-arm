import constants
import numpy as np


def calculate_PD_gain(I_eff): 
    """
    Calculates the proportional and derivative gains for a PID controller.
    Args:
        I_eff (float): The effective inertia of the joint.
    Returns:
        tuple: A tuple containing the proportional gain (kp) and the derivative gain (kD).
    """
    kp = I_eff * constants.Constants.Robot.NATURAL_FREQUENCY **2
    kD = 2 * constants.Constants.Robot.DAMPING_RATIO * I_eff * constants.Constants.Robot.NATURAL_FREQUENCY

    return kp, kD

def calculate_torque(I_eff, target_position, target_velocity, current_position, current_velocity):
    """
    Calculates the torque to be applied to a joint using a PD controller.
    Args:
        I_eff (float): The effective inertia of the joint.
        target_position (float): The target position of the joint.
        target_velocity (float): The target velocity of the joint.
        current_position (float): The current position of the joint.
        current_velocity (float): The current velocity of the joint.
    Returns:
        float: The torque to be applied to the joint.
    """
    Kp, Kd = calculate_PD_gain(I_eff)

    tau = Kp * (target_position - current_position) + Kd * (target_velocity - current_velocity)

    tau = np.clip(tau, -100, 100)  # Limit the torque to a reasonable range

    return tau

