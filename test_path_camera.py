from djitellopy import Tello
import cv2
import numpy as np
import time

DRONE_IP = "192.168.1.63"

tello = Tello(host=DRONE_IP)
stream_started = False

try:
    tello.connect()
    print("Drone connected. Battery:", tello.get_battery(), "%")

    tello.set_video_direction(Tello.CAMERA_DOWNWARD)
    tello.streamon()
    stream_started = True

    reader = tello.get_frame_read()
    time.sleep(5)

    frame = reader.frame
    if frame is None:
        raise RuntimeError("No camera frame received")

    # Keep only usable downward-camera region
    frame = frame[:240, :320].copy()

    cv2.imwrite("new_path_original.jpg", frame)

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    # Detect dark path
    mask = cv2.adaptiveThreshold(
        gray, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        31, 10
    )

    # Reduce small noise
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_OPEN, kernel
    )

    cv2.imwrite("new_path_mask.jpg", mask)

    print("Original image saved: new_path_original.jpg")
    print("Detected mask saved: new_path_mask.jpg")
    print("Resolution:", frame.shape)

finally:
    if stream_started:
        tello.streamoff()
    tello.end()
