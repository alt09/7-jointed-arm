import math
def Yaw_pitch_roll_from_quaternion(quaternion):
    """
    Converts a quaternion to yaw, pitch, and roll angles.
    Args:
        quaternion (list): A list of 4 values representing the quaternion [x, y, z, w].
    Returns:
        tuple: A tuple containing the yaw, pitch, and roll angles in radians.
    """
    roll = math.atan2(2 * (quaternion[3] * quaternion[0] + quaternion[1] * quaternion[2]), 1 - 2 * (quaternion[0] ** 2 + quaternion[1] ** 2))
    pitch = math.asin(2 * (quaternion[3] * quaternion[1] - quaternion[2] * quaternion[0]))
    yaw = math.atan2(2 * (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1]), 1 - 2 * (quaternion[1] ** 2 + quaternion[2] ** 2))
   
    
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
