import time
import cv2
import statistics
from djitellopy import Tello
from hover.junction_marker_detector import detect_junction_marker

t = Tello(host="192.168.1.63")
connected = False
streaming = False
takeoff_attempted = False

def marker_position(reader, label):
    points = []
    for i in range(6):
        frame = reader.frame
        if frame is not None and frame.shape == (720, 320, 3):
            image = cv2.rotate(
                cv2.cvtColor(frame[:240, :320].copy(), cv2.COLOR_RGB2BGR),
                cv2.ROTATE_180
            )
            marker = detect_junction_marker(image)
            name = f"captures/right_axis_{label}_{i+1:02d}.png"
            cv2.imwrite(name, image)
            print(f"Sample {i+1}: {marker['center'] if marker else 'NOT DETECTED'}", flush=True)
            if marker:
                points.append(marker["center"])
        time.sleep(0.15)

    if len(points) < 4:
        return None

    return (
        round(statistics.median(p[0] for p in points), 1),
        round(statistics.median(p[1] for p in points), 1)
    )

try:
    t.connect()
    connected = True
    battery = t.get_battery()
    print("Battery:", battery, "%")
    if battery < 40:
        raise RuntimeError("Battery too low")

    t.set_video_direction(Tello.CAMERA_DOWNWARD)
    t.streamon()
    streaming = True
    reader = t.get_frame_read()
    time.sleep(3)

    if input("Type FLY for short calibration: ").strip() != "FLY":
        raise RuntimeError("Cancelled")

    takeoff_attempted = True
    t.takeoff()
    time.sleep(2)

    before = marker_position(reader, 'before')
    print("BEFORE:", before, flush=True)

    if before is None:
        raise RuntimeError("Marker lost - no movement")

    print("Small RIGHT pulse", flush=True)
    for _ in range(4):
        t.send_rc_control(10, 0, 0, 0)
        time.sleep(0.15)

    t.send_rc_control(0, 0, 0, 0)
    time.sleep(0.4)

    after = marker_position(reader, 'after')
    print("AFTER:", after, flush=True)

    if after:
        print("PIXEL SHIFT:", (
            round(after[0] - before[0], 1),
            round(after[1] - before[1], 1)
        ))

except (Exception, KeyboardInterrupt) as e:
    print("STOP:", e)

finally:
    if takeoff_attempted:
        try:
            t.send_rc_control(0, 0, 0, 0)
        except Exception as e:
            print("WARNING: Neutral RC command failed:", e)
        try:
            t.land()
            print("Landing acknowledged; verify touchdown")
        except Exception as e:
            print("CRITICAL landing error:", e)
        time.sleep(2)

    if streaming:
        try:
            t.streamoff()
        except Exception as e:
            print("Stream cleanup:", e)

    if connected:
        try:
            t.end()
        except Exception as e:
            print("Connection cleanup:", e)
