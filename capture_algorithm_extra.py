from djitellopy import Tello
import cv2, time
from pathlib import Path

out = Path("captures/algorithm_extra")
out.mkdir(parents=True, exist_ok=True)

t = Tello(host="192.168.1.63")

try:
    t.connect()
    t.set_video_direction(Tello.CAMERA_DOWNWARD)
    t.streamon()
    r = t.get_frame_read()
    time.sleep(5)

    for i in range(1, 21):
        frame = r.frame.copy()
        filename = out / f"path_{i:02d}.jpg"
        cv2.imwrite(str(filename), frame)
        print(f"Saved {filename}", flush=True)
        time.sleep(1)

finally:
    t.end()

