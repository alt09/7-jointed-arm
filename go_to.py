import sim
def go_to(arm_id, num_joints, go_to):
    """
    Computes the forward kinematics of the robotic arm and prints the joint positions.

    Args:
        arm_id (int): The ID of the robotic arm in the PyBullet simulation.
        num_joints (int): The number of joints in the robotic arm (8).
        go_to (list): A list of 8 target joint positions for the robotic arm.
    """
    joint_positions = sim.get_joint_positions(arm_id)

    for i in range(num_joints):
        sim.set_joint_positions(i, go_to[i], arm_id)