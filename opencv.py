import cv2
import mediapipe as mp
from mediapipe.tasks import python
import numpy as np
import pyautogui

#initialize mediapipe config objects
base_options = python.BaseOptions(model_asset_path='hand_landmarker.task')
options = mp.tasks.vision.HandLandmarkerOptions(base_options=base_options, num_hands=2)
detector = mp.tasks.vision.HandLandmarker.create_from_options(options)

#intialize mediapipe drawing objects
mp_drawing = mp.tasks.vision.drawing_utils
mp_hands = mp.tasks.vision.HandLandmarksConnections
mp_drawing_styles = mp.tasks.vision.drawing_styles

#Draw landmarks on hand image using mediapipe drawing utils
def draw_landmarks(img, detection_results):
    img_copy = np.copy(img)

   

    for hand_landmarks in detection_results.hand_landmarks:
        mp_drawing.draw_landmarks(
            img_copy,
            hand_landmarks,
            mp_hands.HAND_CONNECTIONS,
            mp_drawing_styles.get_default_hand_landmarks_style(),
            mp_drawing_styles.get_default_hand_connections_style()
        )
        
        # Using PyAutoGUI and MediaPipe Landmarks
        index_tip_Left = hand_landmarks[8]
        index_mcp_Left = hand_landmarks[5]
        palm_Left = hand_landmarks[0]
        pyautogui.keyDown("ctrlleft")
        if index_tip_Left.y < index_mcp_Left.y and round(index_tip_Left.x, index_mcp_Left.x): 
            pyautogui.scroll(-1000)
        if index_tip_Left.y > index_mcp_Left.y and round(index_tip_Left.x, index_mcp_Left.x):
            pyautogui.scroll(1000)

        pyautogui.mouseUp(button='left')
        if (index_tip_Left.y- palm_Left.y) < 0.08 and (index_tip_Left.y- palm_Left.y) > -0.3:  
            pyautogui.mouseDown(button='middle')
            pyautogui.dragRel(xOffset=( palm_Left.x -.5)* 500,yOffset=( palm_Left.y -0.5 )* 500,mouseDownUp=False, duration=.5)
            pyautogui.mouseUp(button='middle')

        pyautogui.keyUp("ctrlleft")
    return img_copy

def round(a,b):
    if (a-b)<0.08 and (a-b)>-0.03:
        return True
    else:
        return False

#Detect hand landmarks using mediapipe model
def detect_hands(image):
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

    detection_result = detector.detect(img)
    drawn_image = draw_landmarks(img.numpy_view(), detection_result)

    return cv2.cvtColor(drawn_image, cv2.COLOR_RGB2BGR)

#code to read video from webcam using OpenCV
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()

    if ret:
        img = detect_hands(frame)
        cv2.imshow("img", img)

        key = cv2.waitKey(1)
        if key == 27:
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()