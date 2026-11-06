from djitellopy import Tello
from threading import Thread, Event
import time
import cv2


tello = Tello()
tello.connect()

battery = tello.get_battery()
print("Battery:", battery, "%")

if battery < 40:
    raise RuntimeError(
        f"Battery too low for testing: {battery}%"
    )

# Enable pad detection BEFORE takeoff
tello.enable_mission_pads()

# 0 = downward only
# 1 = forward only
# 2 = both downward and forward
tello.set_mission_pad_detection_direction(0)

tello.streamon()
time.sleep(2)

frame_read = tello.get_frame_read()
time.sleep(2)

keep_recording = True
recording_failed = Event()
recording_ready = Event()


def record():
    frame = frame_read.frame

    if frame is None:
        print("Error: Camera frame not available")
        recording_failed.set()
        return

    height, width = frame.shape[:2]

    video = cv2.VideoWriter(
        "video.avi",
        cv2.VideoWriter_fourcc(*"XVID"),
        30,
        (width, height),
    )
    if not video.isOpened():
        print("Error: VideoWriter failed to open")
        recording_failed.set()
        return

    recording_ready.set()

    try:
        while keep_recording:
            if frame_read.frame is not None:
                video.write(frame_read.frame)
            time.sleep(1 / 30)

    except Exception as e:
        print("Recording error:", e)
        recording_failed.set()

    finally:
        video.release()


recorder = Thread(target=record)
recorder.start()


# These variables MUST be defined before the while loop
target_found = False
last_printed_pad = None

search_direction = "right"
search_distance = 50  # cm
max_search_steps = 10  # safety limit
search_step = 0


try:

    if not recording_ready.wait(timeout=5):
        raise RuntimeError(
            "Recording initialization timed out"
        )

    if recording_failed.is_set():
        raise RuntimeError("Recording failed; takeoff cancelled")

    tello.takeoff()
    # tello.move_up(100)
    time.sleep(2)

    test_start = time.monotonic()
    test_duration = 10  # seconds

    while time.monotonic() - test_start < test_duration:
        pad = tello.get_mission_pad_id()

        if pad in (1, 3):
            pad_x = tello.get_mission_pad_distance_x()
            pad_y = tello.get_mission_pad_distance_y()
            pad_z = tello.get_mission_pad_distance_z()

            print(
                f"Pad: {pad} | "
                f"X: {pad_x} cm | "
                f"Y: {pad_y} cm | "
                f"Z: {pad_z} cm"
            )
        else:
            print(f"No target pad detected (ID: {pad})")

        if pad != last_printed_pad:
            print("Mission pad ID:", pad)
            last_printed_pad = pad

        if pad == 1:
            print("Table 1 found")


        elif pad == 3:
            print("Table 2 found")


        #elif pad == -2:
            #print("Mission-pad detection is not enabled")
            #break

        else:

            time.sleep(0.5)

    if target_found:
        print("Landing on found table")
    else:
        print("No table found; landing")
    #tello.move_back(200)
    tello.land()

except Exception as e:
    print("Error:", e)

    try:
        #tello.move_back(200)
        tello.land()
    except Exception:
        pass

finally:
    keep_recording = False
    recorder.join()

    try:
        tello.disable_mission_pads()
    except Exception:
        pass

    tello.streamoff()
    tello.end()