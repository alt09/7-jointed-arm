import math
def Yaw_pitch_roll_from_quaternion(quaternion):
    """
    Converts a quaternion to yaw, pitch, and roll angles.
    Args:
        quaternion (list): A list of 4 values representing the quaternion [x, y, z, w].
    Returns:
        tuple: A tuple containing the yaw, pitch, and roll angles.
    """
    x, y, z, w = quaternion

    # Calculate yaw (z-axis rotation)
    sin_yaw = 2 * (w * z + x * y)
    cos_yaw = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)

    # Calculate pitch (y-axis rotation)
    sin_pitch = 2 * (w * y - z * x)
    if abs(sin_pitch) >= 1:
        # Handle the gimbal lock case
        pitch = math.copysign(math.pi / 2, sin_pitch)
    else:
        pitch = math.asin(sin_pitch)

    # Calculate roll (x-axis rotation)
    sin_roll = 2 * (w * x + y * z)
    cos_roll = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)

    yaw = math.degrees(yaw)
    pitch = math.degrees(pitch)
    roll = math.degrees(roll)
    
    return yaw, pitch, roll
def quaternion_from_yaw_pitch_roll(yaw, pitch, roll):
    """
    Converts yaw, pitch, and roll angles to a quaternion.
    Args:
        yaw (float): The yaw angle in degrees.
        pitch (float): The pitch angle in degrees.
        roll (float): The roll angle in degrees.
    Returns:
        list: A list of 4 values representing the quaternion [x, y, z, w].
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [x, y, z, w]
