import cv2
import sim 
import numpy as np

def moments(rgba_img,lower_color,upper_color):

    bgr_img = cv2.cvtColor(rgba_img, cv2.COLOR_RGBA2BGR)
    hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
    lower_color = np.array(lower_color) # HSV color
    upper_color = np.array(upper_color) # HSV color
    mask = cv2.inRange(hsv, lower_color, upper_color)    

    M = cv2.moments(mask)
    return M
def center_of_mass(rgba_img, lower_color,upper_color):
    bgr_img = cv2.cvtColor(rgba_img, cv2.COLOR_RGBA2BGR)
    M = moments(bgr_img, lower_color, upper_color)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        cv2.circle(bgr_img, (cX, cY), 5, (0, 255, 0), -1)

    cv2.imshow("Camera Feed", bgr_img)