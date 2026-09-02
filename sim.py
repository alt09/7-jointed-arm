
import time

import cv2
import numpy as np
import pybullet as p
import pybullet_data
import math

import go_to

print("Starting PyBullet simulation...")
def main():
    print("Starting simulation...")
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, 0)
    

    #p.loadURDF("plane.urdf")  # load the plane
    arm_id = p.loadURDF("arm.urdf", basePosition=[0, 0, 0], useFixedBase=True)
    r2d2_id = p.loadURDF("r2d2.urdf", basePosition=[5, 5, 1], useFixedBase=True)
 
    print(f"Loaded arm with id: {arm_id}")
    width, height = 320, 240
    go_to.go_to_target(arm_id, [4, 4, 1])
    while p.isConnected(client):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)
        # Camera Position and Orientation
        viewMatrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=[go_to.where_is(arm_id)[0][0]+0.2, go_to.where_is(arm_id)[0][1]+0.2, go_to.where_is(arm_id)[0][2]+0.2],
        distance=0.1,
        yaw=(180/math.pi)*go_to.where_is(arm_id)[1],
        pitch=(180/math.pi)*go_to.where_is(arm_id)[2],
        roll=(180/math.pi)*go_to.where_is(arm_id)[3],
        upAxisIndex=2
   		)
        print((go_to.where_is(arm_id)))
        projectionMatrix = p.computeProjectionMatrixFOV(
    	    fov=60, aspect=width/height, nearVal=0.1, farVal=100.0
    	)
        img_arr = p.getCameraImage(
            width, height,
			viewMatrix=viewMatrix,
            projectionMatrix=projectionMatrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )
        
        # Extract RGB array (Index 2 holds the pixel data)
        rgba_img = np.reshape(img_arr[2], (height, width, 4)).astype(np.uint8)

        bgr_img = cv2.cvtColor(rgba_img, cv2.COLOR_RGBA2BGR)

        hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        M = cv2.moments(mask)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            #print(f"Detected red object at ({cX}, {cY})")

            cv2.circle(bgr_img, (cX, cY), 5, (0, 255, 0), -1)

        cv2.imshow("Camera Feed", bgr_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


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


def get_joint_angle(arm_id):
    joint_positions = []
    num_joints = p.getNumJoints(arm_id)

    for i in range(num_joints):
        joint_state = p.getJointState(arm_id, i)
        joint_positions.append(joint_state[0])  # Append the position (first element of joint_state)
    return joint_positions


def set_joint_positions(joint_index, target_position, arm_id):
    p.setJointMotorControl2(
        arm_id,
        joint_index,
        p.POSITION_CONTROL,
        targetPosition=target_position,
        force=100
    )


if __name__ == "__main__":
    main()
