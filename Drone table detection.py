from djitellopy import Tello
from threading import Thread, Event
import time
import cv2
import paho.mqtt.client as mqtt

MQTT_BROKER = "192.168.1.96"
MQTT_PORT = 1883
MQTT_TOPIC = "restaurant/tello/events/customer-detected"



def publish_table_event(pad):
    message = f"Table {1 if pad == 1 else 2} found"
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

    try:
        client.connect(MQTT_BROKER, MQTT_PORT, keepalive=30)
        client.loop_start()

        result = client.publish(MQTT_TOPIC, message, qos=1)
        result.wait_for_publish(timeout=3)
        print("MQTT published:", message)

    except Exception as e:
        print("MQTT publishing failed:", e)

    finally:
        client.disconnect()
        client.loop_stop()

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
net_displacement = 0  # cm; right is positive


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

    # Search at START, RIGHT 50 cm, LEFT 50 cm
    scan_positions = [0, 50, -50]

    for position in scan_positions:
        movement = position - net_displacement

        if movement > 0:
            tello.move_right(movement)
            net_displacement += movement
        elif movement < 0:
            tello.move_left(abs(movement))
            net_displacement += movement

        time.sleep(1)

        pad = tello.get_mission_pad_id()
        print("Position:", net_displacement, "cm | Pad:", pad)

        if pad == 1:
            print("Table 1 found")
            publish_table_event(pad)
            target_found = True
            break
        elif pad == 3:
            print("Table 2 found")
            publish_table_event(pad)
            target_found = True
            break

    # Return to START before landing
    if net_displacement > 0:
        tello.move_left(net_displacement)
    elif net_displacement < 0:
        tello.move_right(abs(net_displacement))

    net_displacement = 0
    print("Return movement completed")
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

