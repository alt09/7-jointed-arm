import os
import time

import pybullet as p
import pybullet_data


def main():
	client = p.connect(p.GUI)
	p.setAdditionalSearchPath(pybullet_data.getDataPath())
	p.setGravity(0, 0, 0)

	p.loadURDF("plane.urdf")

	urdf_path = os.path.join(os.path.dirname(__file__), "test.urdf")
	p.setAdditionalSearchPath(os.path.dirname(urdf_path))
	arm_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0], useFixedBase=True)

	print(f"Loaded arm with id: {arm_id}")

	try:
		while p.isConnected(client):
			p.stepSimulation()
			time.sleep(1.0 / 240.0)
	except KeyboardInterrupt:
		pass
	finally:
		if p.isConnected():
			p.disconnect()


if __name__ == "__main__":
	main()
