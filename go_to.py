import sim
import math
def go_to(arm_id, num_joints, go_to):
    """
    Moves the robotic arm to the specified joint positions.
    Args:
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
        num_joints (int): The number of joints in the robotic arm (8).
        go_to (list): A list of 8 target joint positions for the robotic arm.
    """
    for i in range(num_joints):
        sim.set_joint_positions(i, go_to[i], arm_id)

def go_to_target(arm_id,target_position):
    """
    Moves the robotic arm to the specified target position.
    Args:
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
        target_position (list): A list of 3 target coordinates [x, y, z] for the end effector.
    """
    # Get the current joint positions
    current_joint_positions = sim.get_joint_angle(arm_id)

    print(f"Current joint positions: {current_joint_positions}")

    # Calculate the inverse kinematics to find the joint angles for the target position
    target_joint_positions = sim.p.calculateInverseKinematics(arm_id, 7, target_position)

    print(f"Target joint positions: {target_joint_positions}")

    # Move the arm to the target joint positions
    for i in range(sim.p.getNumJoints(arm_id)):
        sim.set_joint_positions(i, target_joint_positions[i], arm_id)
def where_is(arm_id,):
    """
    Returns the current position of the end effector of the robotic arm.
    Args:
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
    Returns:
        list: A list of 3 coordinates [x, y, z] representing the current position of the end effector.
    """
    quaternion = sim.p.getLinkState(arm_id, 7)[5]  # Get the orientation of the end effector

    roll = math.atan2(2 * (quaternion[3] * quaternion[0] + quaternion[1] * quaternion[2]), 1 - 2 * (quaternion[0] ** 2 + quaternion[1] ** 2))
    pitch = math.asin(2 * (quaternion[3] * quaternion[1] - quaternion[2] * quaternion[0]))
    yaw = math.atan2(2 * (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1]), 1 - 2 * (quaternion[1] ** 2 + quaternion[2] ** 2))
    # Calculate the forward kinematics to find the position of the end effector
    end_effector_state = sim.p.getLinkState(arm_id, 7)
    end_effector_position = [end_effector_state[4],yaw,pitch,roll]  # Position is at index 4
    print(f"End effector position: {end_effector_position}")

    return end_effector_position