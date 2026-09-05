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

def test_second_camera(rgba_img):
    bgr_img = cv2.cvtColor(rgba_img, cv2.COLOR_RGBA2BGR)
    cv2.imshow("Camera Feed 2", bgr_img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return None
