import cv2


# '0' is usually the built-in laptop camera. Use 1, 2, etc., for external USB webcams.
cap = cv2.VideoCapture(0)  # Change the index if you have multiple cameras

# Optional check to ensure the webcam opened correctly
if not cap.isOpened():
    print("Error: Could not open the webcam.")
    exit()

while True:
    # 'ret' is a boolean (True/False), 'frame' is the actual image array
    ret, frame = cap.read()

    # If the frame was not grabbed successfully, break the loop
    if not ret:
        print("Error: Failed to grab a frame.")
        break

    # 3. Display the resulting frame in a window named 'Webcam Feed'
    cv2.imshow('Webcam Feed', frame)

    # cv2.waitKey(1) waits 1ms for a key press; & 0xFF extracts the character code
    if cv2.waitKey(1) == ord('q'):
        break

# When everything is done, release the capture object and destroy windows
cap.release()
cv2.destroyAllWindows()