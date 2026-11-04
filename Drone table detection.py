from djitellopy import Tello
from threading import Thread
import time
import cv2


tello = Tello()
tello.connect()

print("Battery:", tello.get_battery(), "%")

# Enable pad detection BEFORE takeoff
tello.enable_mission_pads()

# 0 = downward only
# 1 = forward only
# 2 = both downward and forward
tello.set_mission_pad_detection_direction(2)

tello.streamon()
time.sleep(2)

frame_read = tello.get_frame_read()
time.sleep(2)

keep_recording = True


def record():
    height, width, _ = frame_read.frame.shape
    video = cv2.VideoWriter(
        "video.avi",
        cv2.VideoWriter_fourcc(*"XVID"),
        30,
        (width, height),
    )

    while keep_recording:
        if frame_read.frame is not None:
            video.write(frame_read.frame)
        time.sleep(1 / 30)

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
    tello.takeoff()
    # tello.move_up(100)
    time.sleep(2)

    while not target_found and search_step < max_search_steps:
        pad = tello.get_mission_pad_id()

        if pad != last_printed_pad:
            print("Mission pad ID:", pad)
            last_printed_pad = pad

        if pad == 1:
            print("Table 1 found")
            target_found = True

        elif pad == 3:
            print("Table 2 found")
            target_found = True

        #elif pad == -2:
            #print("Mission-pad detection is not enabled")
            #break

        else:
            #print(f"No pad found; moving {search_direction} {search_distance} cm")

            if search_direction == "right":
                tello.move_right(50)
                search_direction = "left"
                tello.move_left(50)
            else:
                tello.move_left(50)
                search_direction = "right"
                tello.move_right(50)

            search_step += 1
            time.sleep(0.5)

    if target_found:
        print("Landing on found table")
    else:
        print("No table found; landing")
    tello.move_back(200)
    tello.land()

except Exception as e:
    print("Error:", e)

    try:
        tello.move_back(200)
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