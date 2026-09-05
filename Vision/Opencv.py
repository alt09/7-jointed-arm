import cv2
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

def target_3d_pose(view_matrix_1,view_matrix_2,projectionMatrix,rgba_img1,rgba_img2,lower_color,upper_color): #find the 3D position of the target object using stereo vision

    M1=moments(rgba_img1, lower_color, upper_color)
    M2=moments(rgba_img2, lower_color, upper_color)

    if M1["m00"] != 0 and M2["m00"] != 0:


        u1 = M1["m10"] / M1["m00"]
        v1 = M1["m01"] / M1["m00"]
        
        C1 = camera_pose_from_view_matrix(view_matrix_1)[0]
        C2 = camera_pose_from_view_matrix(view_matrix_2)[0]

        u2 = M2["m10"] / M2["m00"]
        v2 = M2["m01"] / M2["m00"]

        D1 = pixel_to_world_ray(u1, v1, view_matrix_1, projectionMatrix, 320, 240)
        D2 = pixel_to_world_ray(u2, v2, view_matrix_2, projectionMatrix, 320, 240)

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

        return target_position, triangulation_error


def camera_pose_from_view_matrix(viewMatrix):

    view = np.array(viewMatrix, dtype=float).reshape((4, 4), order='F')

    camera_to_world = np.linalg.inv(view)

    C = camera_to_world[:3, 3]
    R = camera_to_world[:3, :3]

    return C, R
def pixel_to_world_ray(u,v,view_matrix,projection_matrix,width,height):
    x_ndc = (2.0 * u) / width - 1.0
    y_ndc = 1.0 - (2.0 * v) / height

    clip_space_point = np.array([x_ndc, y_ndc, -1.0, 1.0])

    P = np.array(projection_matrix, dtype=float).reshape((4, 4), order='F')
    V = np.array(view_matrix, dtype=float).reshape((4, 4), order='F')

    inv_projection = np.linalg.inv(P)

    camera_point = inv_projection @ clip_space_point
    camera_point /= camera_point[3]

    inv_view = np.linalg.inv(V)
    world_point = inv_view @ np.array([camera_point[0], camera_point[1], camera_point[2], 1.0])
    world_point /= world_point[3]

    camera_position = inv_view[:3, 3]

    ray = world_point[:3] - camera_position
    ray /= np.linalg.norm(ray)

    return ray