import cv2
import sim 
import numpy as np

def moments(rgba_img,lower_color,upper_color):

    bgr_img = cv2.cvtColor(rgba_img, cv2.COLOR_RGBA2BGR)
    hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower_color), np.array(upper_color))    

    M = cv2.moments(mask)
    return M
def center_of_mass(rgba_img, lower_color,upper_color, LoR):
    bgr_img = cv2.cvtColor(rgba_img, cv2.COLOR_RGBA2BGR)
    hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower_color), np.array(upper_color))

    M = cv2.moments(mask)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        #print(f"Detected red object at ({cX}, {cY})")

        cv2.circle(bgr_img, (cX, cY), 5, (0, 255, 0), -1)
    if LoR == "Left":
        cv2.imshow("Camera Feed 1", bgr_img)
    if LoR == "Right":
        cv2.imshow("Camera Feed 2", bgr_img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return None

def test_new_cameras(rgba_img):
    bgr_img = cv2.cvtColor(rgba_img, cv2.COLOR_RGBA2BGR)
    cv2.imshow("Camera Feed 2", bgr_img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return None

def target_3d_pose(R,x,y,z,rgba_img1,rgba_img2,lower_color,upper_color): #find the 3D position of the target object using stereo vision
    K=np.array([[277.128,0,160],[0,207.846,120],[0,0,1]])
    M1=moments(rgba_img1, lower_color, upper_color)
    M2=moments(rgba_img2, lower_color, upper_color)
    if M1["m00"] != 0 and M2["m00"] != 0:
        u1 = M1["m10"] / M1["m00"]
        v1 = M1["m01"] / M1["m00"]
        u2 = M2["m10"] / M2["m00"]
        v2 = M2["m01"] / M2["m00"]
        pixel1=np.array([[u1],[v1],[1]])
        pixel2=np.array([[u2],[v2],[1]])

        ray_camera1 = np.linalg.inv(K) @ pixel1
        ray_camera2 = np.linalg.inv(K) @ pixel2

        ray_camera1 = ray_camera1 / np.linalg.norm(ray_camera1)
        ray_camera2 = ray_camera2 / np.linalg.norm(ray_camera2)

        ray_world1 = R @ ray_camera1
        ray_world2 = R @ ray_camera2

        ray_world1 = ray_world1.flatten()
        ray_world2 = ray_world2.flatten()

        ray_world1 = ray_world1 / np.linalg.norm(ray_world1)
        ray_world2 = ray_world2 / np.linalg.norm(ray_world2)

        C1 = np.array([x,y,z-0.2],dtype=float)
        C2 = np.array([x+1,y,z-0.2],dtype=float)

        D1 = ray_world1
        D2 = ray_world2

        w0 = C1 - C2

        a = np.dot(D1, D1)
        b = np.dot(D1, D2)
        c = np.dot(D2, D2)  
        d = np.dot(D1, w0)
        e = np.dot(D2, w0)

        denominator = a * c - b * b
        if abs(denominator) < 1e-8:
            return None
        t = (b * e - c * d) / denominator
        s = (a * e - b * d) / denominator


        point1 = C1 + t * D1
        point2 = C2 + s * D2
        target_position = (point1 + point2) / 2
        triangulation_error = np.linalg.norm(point1 - point2)
        print("C1:", C1)
        print("C2:", C2)

        print("ray camera 1:", ray_camera1.flatten())
        print("ray camera 2:", ray_camera2.flatten())

        print("ray world 1:", ray_world1)
        print("ray world 2:", ray_world2)
        print("arm:", x, y, z)
        return target_position, triangulation_error

def rotation_matrix(yaw, pitch, roll):

    cy = np.cos(yaw)
    sy = np.sin(yaw)

    cp = np.cos(pitch)
    sp = np.sin(pitch)

    cr = np.cos(roll)
    sr = np.sin(roll)

    Rz = np.array([
        [cy, -sy, 0],
        [sy,  cy, 0],
        [0,    0, 1]
    ])

    Ry = np.array([
        [cp, 0, sp],
        [0,  1, 0],
        [-sp, 0, cp]
    ])

    Rx = np.array([
        [1, 0,  0],
        [0, cr, -sr],
        [0, sr,  cr]
    ])

    R = Rz @ Ry @ Rx

    return R
 