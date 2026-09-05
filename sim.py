
import time
import numpy as np
import pybullet as p
import pybullet_data
import math
import Vision.opencv as opencv

import Movement.go_to as go_to

print("Starting PyBullet simulation...")

def sim():
    print("Starting simulation...")
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, 0)
    

    #p.loadURDF("plane.urdf")  # load the plane
    arm_id = p.loadURDF("arm.urdf", basePosition=[0, 0, 0], useFixedBase=True)
    r2d2_id = p.loadURDF("r2d2.urdf", basePosition=[0, 4, 0], useFixedBase=True)
 
    #print(f"Loaded arm with id: {arm_id}")
    width, height = 320, 240
  #  go_to.go_to_target(arm_id, [0, 4, 1])
    while p.isConnected(client):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)
        # Camera 1 Position and Orientation 
        viewMatrix1 = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=[go_to.where_is(arm_id)[0][0], go_to.where_is(arm_id)[0][1], go_to.where_is(arm_id)[0][2]-0.2],
        distance=0.1,
        yaw=(180/math.pi)*go_to.where_is(arm_id)[1], # RAD to DEG
        pitch=(180/math.pi)*go_to.where_is(arm_id)[2],
        roll=(180/math.pi)*go_to.where_is(arm_id)[3],
        upAxisIndex=2
   		)
        viewMatrix2 = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=[go_to.where_is(arm_id)[0][0]+1, go_to.where_is(arm_id)[0][1], go_to.where_is(arm_id)[0][2]-0.2],
        distance=0.1,
        yaw=(180/math.pi)*go_to.where_is(arm_id)[1], # RAD to DEG
        pitch=(180/math.pi)*go_to.where_is(arm_id)[2],
        roll=(180/math.pi)*go_to.where_is(arm_id)[3],
        upAxisIndex=2
   		)
        # print((go_to.where_is(arm_id)))
        projectionMatrix = p.computeProjectionMatrixFOV(
    	    fov=60, aspect=width/height, nearVal=0.1, farVal=100.0
    	)
        img_arr1 = p.getCameraImage(
            width, height,
			viewMatrix=viewMatrix1,
            projectionMatrix=projectionMatrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )
        img_arr2 = p.getCameraImage(
            width, height,
			viewMatrix=viewMatrix2,
            projectionMatrix=projectionMatrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )
        # Extract the RGBA image
        rgba_img1 = np.reshape(img_arr1[2], (height, width, 4)).astype(np.uint8)
        rgba_img2 = np.reshape(img_arr2[2], (height, width, 4)).astype(np.uint8)

        opencv.center_of_mass(rgba_img1, [0, 0, 55], [0, 0, 100],"Left")
        opencv.center_of_mass(rgba_img2, [0, 0, 55], [0, 0, 100],"Right")
        print("Depth:", opencv.find_depth(rgba_img1,rgba_img2, [0, 0, 55], [0, 0, 100]))
        #print("r2d2",p.getLinkState(r2d2_id, 1)[4])
        #print("arm",p.getLinkState(arm_id, 7)[4])
        d = math.hypot(p.getLinkState(r2d2_id, 1)[4][0]-p.getLinkState(arm_id, 0)[4][0],p.getLinkState(r2d2_id, 1)[4][1]-p.getLinkState(arm_id, 0)[4][1],p.getLinkState(r2d2_id, 1)[4][2]-p.getLinkState(arm_id, 0)[4][2]-0.2)
        print("distance", d)

        if opencv.find_depth(rgba_img1,rgba_img2, [0, 0, 55], [0, 0, 100]) != None:
            if d - opencv.find_depth(rgba_img1,rgba_img2, [0, 0, 55], [0, 0, 100]) < 0.1 and d - opencv.find_depth(rgba_img1,rgba_img2, [0, 0, 55], [0, 0, 100]) > -0.1:
                print("is close")
            else:
                print("is far by", d - opencv.find_depth(rgba_img1,rgba_img2, [0, 0, 55], [0, 0, 100]))
def get_joint_info(arm_id):
    joint_info = {}
    num_joints = p.getNumJoints(arm_id)

    for i in range(num_joints):
        info = p.getJointInfo(arm_id, i)
        joint_name = info[1].decode('utf-8')
        joint_type = info[2]
        #print(info)

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

