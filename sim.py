import os
import time

import pybullet as p
import pybullet_data
import go_to


def main():
	client = p.connect(p.GUI)
	p.setAdditionalSearchPath(pybullet_data.getDataPath())
	p.setGravity(0, 0, 0)

	#p.loadURDF("plane.urdf") # load the plane
	urdf_path = os.path.join(os.path.dirname(__file__), "arm.urdf")# load the arm
	p.setAdditionalSearchPath(os.path.dirname(urdf_path))
	arm_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0], useFixedBase=True)

	print(f"Loaded arm with id: {arm_id}")
	

	while p.isConnected(client):
		p.stepSimulation()
		time.sleep(1.0 / 240.0)
def get_joint_info(arm_id):
	joint_info = {}
	num_joints = p.getNumJoints(arm_id)

	for i in range(num_joints):
		info = p.getJointInfo(arm_id, i)
		joint_name = info[1].decode('utf-8')
		joint_type = info[2]
		print(info)

		if joint_type == p.JOINT_REVOLUTE:
			joint_info[joint_name] = {
				'index': i,
				'lower_limit': info[8],
				'upper_limit': info[9],
				'max_force': info[10],
				'max_velocity': info[11]
			}
	return joint_info
def get_joint_positions(arm_id):
	joint_positions = []
	num_joints = p.getNumJoints(arm_id)

	for i in range(num_joints):
		joint_state = p.getJointState(arm_id, i)
		joint_positions.append(joint_state[0])  # Append the position (first element of joint_state)
	return joint_positions

def set_joint_positions( joint_index, target_position,arm_id):
	p.setJointMotorControl2(
		arm_id,
		joint_index,
		p.POSITION_CONTROL,
		targetPosition=target_position,
		force=100
	)
if __name__ == "__main__":
	main()
