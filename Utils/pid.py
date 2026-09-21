import sim

def calculate_PD_gain(I_eff ,natural_frequency, damping_ratio): 
    """
    Calculates the proportional and derivative gains for a PID controller.
    Args:
        I_eff (float): The effective inertia of the joint.
        natural_frequency (float): The natural frequency of the system.
        damping_ratio (float): The damping ratio of the system.
    Returns:
        tuple: A tuple containing the proportional gain (kp) and the derivative gain (kD).
    """
    kp = I_eff * natural_frequency**2
    kD = 2 * damping_ratio * I_eff * natural_frequency

    return kp, kD

def PID(arm_id, joint_index, joint_positions, natural_frequency, damping_ratio, target_position, target_velocity, position, velocity):
    
    kP, kD = calculate_PD_gain(arm_id, joint_index, joint_positions, natural_frequency, damping_ratio)

    tau = kP * (target_position - position) \
        + kD * (target_velocity - velocity)

    return tau

