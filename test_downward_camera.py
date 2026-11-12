from djitellopy import Tello
import cv2
import time

t = Tello(host="192.168.1.63")

try:
    t.connect()
    t.set_video_direction(Tello.CAMERA_DOWNWARD)
    t.streamon()

    reader = t.get_frame_read()
    time.sleep(5)

    for i in range(5):
        frame = reader.frame
        if frame is not None:
            filename = f"downward_frame_{i+1}.jpg"
            cv2.imwrite(filename, frame.copy())
            print("Saved:", filename, frame.shape)
        time.sleep(2)

finally:
    t.streamoff()
    t.end()
