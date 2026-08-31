import sim
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
    current_joint_positions = sim.get_joint_positions(arm_id)

    print(f"Current joint positions: {current_joint_positions}")

    # Calculate the inverse kinematics to find the joint angles for the target position
    target_joint_positions = sim.p.calculateInverseKinematics(arm_id, 7, target_position)

    print(f"Target joint positions: {target_joint_positions}")

    # Move the arm to the target joint positions
    for i in range(sim.p.getNumJoints(arm_id)):
        sim.set_joint_positions(i, target_joint_positions[i], arm_id)