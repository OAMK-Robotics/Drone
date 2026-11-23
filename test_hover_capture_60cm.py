"""DJI Tello: guarded stationary hover image capture near 60 cm indicated height."""
from datetime import datetime
from pathlib import Path
import time

import cv2
from djitellopy import Tello

TELLO_IP = "192.168.1.63"  # University ASUS network; verify for each session
MIN_BATTERY = 40
PHOTO_COUNT = 8
TARGET_HEIGHT_CM = 60  # Approximate telemetry target, NOT guaranteed tabletop clearance
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240  # Upper region of the observed 320x720 corrupted feed

folder = Path("captures") / f"hover60_{datetime.now():%Y%m%d_%H%M%S}"
folder.mkdir(parents=True, exist_ok=True)

tello = Tello(host=TELLO_IP)
connected = False
streaming = False
takeoff_attempted = False


def save_frame(frame, name):
    if frame is None or frame.ndim != 3 or frame.shape[0] < IMAGE_HEIGHT or frame.shape[1] < IMAGE_WIDTH:
        raise RuntimeError("Missing or unexpectedly small camera frame")
    # DJITelloPy supplies RGB frames; OpenCV imwrite expects BGR.
    raw = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    crop = raw[:IMAGE_HEIGHT, :IMAGE_WIDTH].copy()
    if not cv2.imwrite(str(folder / f"{name}_raw.png"), raw):
        raise RuntimeError(f"Could not save raw image for {name}")
    if not cv2.imwrite(str(folder / f"{name}_crop.png"), crop):
        raise RuntimeError(f"Could not save cropped image for {name}")
    if not cv2.imwrite(str(folder / f"{name}_crop_rot180.png"), cv2.rotate(crop, cv2.ROTATE_180)):
        raise RuntimeError(f"Could not save rotated crop for {name}")
    return float(crop.mean()), float(crop.std()), frame.shape


try:
    print("1. Connecting to Tello; NO FLIGHT yet...", flush=True)
    tello.connect()
    connected = True
    battery = tello.get_battery()
    print(f"CONNECTED | Battery: {battery}%")
    if battery < MIN_BATTERY:
        raise RuntimeError(f"Battery below {MIN_BATTERY}%. Flight cancelled.")

    tello.set_video_direction(Tello.CAMERA_DOWNWARD)
    tello.streamon()
    streaming = True
    reader = tello.get_frame_read()
    print("2. Checking video for up to 10 seconds...", flush=True)
    preview = None
    for _ in range(20):
        time.sleep(0.5)
        candidate = reader.frame
        if candidate is None:
            continue
        if candidate.ndim != 3 or candidate.shape[0] < IMAGE_HEIGHT or candidate.shape[1] < IMAGE_WIDTH:
            continue
        top = candidate[:IMAGE_HEIGHT, :IMAGE_WIDTH]
        if top.max() > 8 and top.std() > 10:
            preview = candidate.copy()
            break
    if preview is None:
        raise RuntimeError("No valid video frame. Flight cancelled.")
    mean, variation, shape = save_frame(preview, "preflight")
    print(f"Preflight camera: shape={shape}, crop mean={mean:.1f}, variation={variation:.1f}")
    print(f"Preview: {folder / 'preflight_crop.png'}")
    print("Open the preview and check that it shows the real path, not green/black corruption.")
    print("Use a clear, stable launch/landing area. Nobody may hold the drone during takeoff.")
    approval = input("Type FLY60 for a stationary hover + one guarded descent toward 60 cm (anything else cancels): ").strip()
    if approval != "FLY60":
        print("Cancelled; no takeoff.")
    else:
        print("3. TAKEOFF — no horizontal commands. Optional single descent when telemetry permits.", flush=True)
        takeoff_attempted = True  # Set BEFORE takeoff to cover ambiguous command failures
        tello.takeoff()
        time.sleep(2)
        initial_height = int(tello.get_height())
        print(f"Initial reported hover height: {initial_height} cm")
        if 80 <= initial_height <= 100:
            descent = initial_height - TARGET_HEIGHT_CM
            print(f"Requesting one {descent} cm downward move (SDK minimum 20 cm).", flush=True)
            tello.move_down(descent)
            time.sleep(2)
        else:
            print("No descent: initial reported height is outside calibrated 80–100 cm window.")
        measured_height = int(tello.get_height())
        print(f"Capture height reported: {measured_height} cm (not guaranteed clearance above table)")
        if measured_height < 40:
            raise RuntimeError("Reported height below 40 cm; capture cancelled for landing.")
        telemetry_lines = [f"initial_height_cm={initial_height}", f"capture_height_cm={measured_height}"]
        for index in range(1, PHOTO_COUNT + 1):
            frame = reader.frame.copy()
            mean, variation, shape = save_frame(frame, f"hover_{index:02d}")
            height = int(tello.get_height())
            try:
                tof = tello.get_distance_tof()
            except Exception:
                tof = "unavailable"
            telemetry_lines.append(f"hover_{index:02d}: height_cm={height}, tof_cm={tof}, variation={variation:.1f}")
            print(f"Photo {index}/{PHOTO_COUNT} | height={height} cm | TOF={tof} | variation={variation:.1f}", flush=True)
            time.sleep(0.7)
        (folder / "capture_telemetry.txt").write_text("\n".join(telemetry_lines) + "\n", encoding="utf-8")
        print("Capture complete. Landing now.", flush=True)
except KeyboardInterrupt:
    print("\nINTERRUPTED — attempting controlled landing.", flush=True)
except Exception as exc:
    print(f"ERROR: {exc} — attempting landing if takeoff was attempted.", flush=True)
finally:
    if takeoff_attempted:
        try:
            tello.land()
            print("Landing command acknowledged.", flush=True)
        except Exception as exc:
            print(f"WARNING: Landing command failed: {exc}", flush=True)
        # Keep SDK open while descent is underway; height is only an approximate indicator.
        touchdown_reported = False
        for _ in range(20):
            time.sleep(0.5)
            try:
                height = tello.get_height()
                if height <= 15:
                    touchdown_reported = True
                    break
            except Exception:
                break
        if not touchdown_reported:
            print("Touchdown not confirmed by telemetry. Visually verify landing BEFORE closing SDK.", flush=True)
            try:
                input("After the drone is visibly on the surface, press Enter: ")
            except EOFError:
                pass
        time.sleep(1)
    if streaming:
        try:
            tello.streamoff()
        except Exception as exc:
            print(f"Stream shutdown warning: {exc}")
    if connected:
        try:
            tello.end()
        except Exception as exc:
            print(f"SDK cleanup warning: {exc}")
    print(f"Saved images folder: {folder}", flush=True)
