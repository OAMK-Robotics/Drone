from djitellopy import Tello
from pathlib import Path
import cv2
import time

output = Path("captures/straight_path")
output.mkdir(parents=True, exist_ok=True)

for attempt in range(1, 4):
    print(f"\nVideo connection attempt {attempt}/3", flush=True)
    tello = Tello(host="192.168.1.63")
    streaming = False

    try:
        tello.connect()
        tello.set_video_direction(Tello.CAMERA_DOWNWARD)
        tello.streamon()
        streaming = True

        reader = tello.get_frame_read()
        time.sleep(8)

        for i in range(1, 6):
            frame = reader.frame.copy()
            brightness = float(frame.mean())

            if brightness <= 5:
                print(f"Image {i}: black frame, skipped")
            else:
                filename = output / f"straight_{i:02d}.jpg"
                saved = cv2.imwrite(str(filename), frame)
                print(
                    f"Image {i}: saved={saved}, "
                    f"brightness={brightness:.1f}, "
                    f"shape={frame.shape}"
                )

            time.sleep(2)

        print("Capture session completed.")
        break

    except Exception as e:
        print(f"Attempt {attempt} failed: {e}", flush=True)
        if attempt == 3:
            print("STOP: No usable video session established.")

    finally:
        if streaming:
            try:
                tello.streamoff()
            except Exception as e:
                print(f"Stream cleanup warning: {e}")
        tello.end()

    time.sleep(3)
