"""Tello EDU takeoff recording, one guarded descent near 40 cm, 8 images, land.

IMPORTANT: Tello SDK takeoff is automatic, NOT limited to 40 cm. The drone
usually rises higher, then this script MAY descend once toward 40 cm.
Use only in a clear, unobstructed floor area (not a narrow tabletop).
"""

from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path
import threading
import time

import cv2
import numpy as np
from djitellopy import Tello

DRONE_IP = "192.168.1.63"
MIN_BATTERY_PERCENT = 40
TARGET_HEIGHT_CM = 40
HEIGHT_TOLERANCE_CM = 10  # reported height, NOT independently calibrated clearance
CAMERA_INTERVAL_S = 0.125  # 8 images/sec, including during ascent
TARGET_IMAGES = 8
CROP_H, CROP_W = 240, 320


def camera_image(reader) -> np.ndarray:
    rgb = reader.frame.copy()
    if rgb is None or rgb.ndim != 3 or rgb.shape[0] < CROP_H or rgb.shape[1] < CROP_W:
        raise RuntimeError(f"Invalid camera frame shape: {getattr(rgb, 'shape', None)}")
    return cv2.rotate(
        cv2.cvtColor(rgb[:CROP_H, :CROP_W], cv2.COLOR_RGB2BGR),
        cv2.ROTATE_180,
    )


def save_image(path: Path, image: np.ndarray) -> None:
    if not cv2.imwrite(str(path), image):
        raise OSError(f"Failed to save {path}")


def main():
    output = Path("captures") / ("takeoff40_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    output.mkdir(parents=True, exist_ok=False)
    drone = Tello(host=DRONE_IP)
    connected = False
    stream_started = False
    reader = None
    takeoff_attempted = False
    landing_acknowledged = False
    start = time.monotonic()
    stop_video = threading.Event()
    video_thread = None
    phase = ["PRETAKEOFF"]
    frames = []
    events = []
    recorder_errors = []

    def event(label):
        t = round(time.monotonic() - start, 3)
        events.append((label, t))
        phase[0] = label
        print(f"{label}: {t:.2f}s")

    def record_from_before_takeoff():
        n = 0
        next_time = time.monotonic()
        while not stop_video.is_set():
            now = time.monotonic()
            if now < next_time:
                stop_video.wait(min(0.05, next_time - now))
                continue
            next_time = max(next_time + CAMERA_INTERVAL_S, now)
            try:
                name = f"flight_{n:04d}.png"
                save_image(output / name, camera_image(reader))
                frames.append((name, round(now - start, 3), phase[0]))
                n += 1
            except Exception as exc:
                recorder_errors.append(str(exc))
                stop_video.set()

    try:
        print("Connecting — NO FLIGHT yet...")
        drone.connect()
        connected = True
        battery = drone.get_battery()
        print("CONNECTED | Battery:", battery, "%")
        if battery < MIN_BATTERY_PERCENT:
            raise RuntimeError("Battery below 40%; flight cancelled")

        drone.set_video_direction(Tello.CAMERA_DOWNWARD)
        drone.streamon()
        stream_started = True
        reader = drone.get_frame_read()

        preview = None
        for _ in range(40):
            time.sleep(0.25)
            try:
                candidate = camera_image(reader)
                if float(candidate.mean()) > 15 and float(candidate.std()) > 10:
                    preview = candidate
                    break
            except (AttributeError, RuntimeError):
                pass
        if preview is None:
            raise RuntimeError("No valid downward-camera image; cancelling flight")
        preview_path = output / "preflight.png"
        save_image(preview_path, preview)
        print("Preview:", preview_path)
        print("WARNING: takeoff automatically rises above 40 cm; it cannot be capped at 40 cm.")
        print("Use only a large CLEAR, FLAT FLOOR area. Not the narrow table or near people.")
        answer = input("Check the preview and surroundings. Type FLY40 to proceed: ").strip()
        if answer != "FLY40":
            print("Cancelled. No flight.")
            return

        start = time.monotonic()
        video_thread = threading.Thread(target=record_from_before_takeoff, daemon=True)
        video_thread.start()
        time.sleep(0.25)  # recording actually starts BEFORE sending takeoff
        if recorder_errors or not frames:
            raise RuntimeError("Recording failed before takeoff")

        event("TAKEOFF_SENT")
        takeoff_attempted = True
        drone.takeoff()
        event("TAKEOFF_OK")
        initial_height = int(drone.get_height())
        print("Reported height after automatic takeoff:", initial_height, "cm")
        if not 30 <= initial_height <= 90:
            raise RuntimeError("Unexpected height; no descent or target capture; landing")

        if initial_height > TARGET_HEIGHT_CM + HEIGHT_TOLERANCE_CM:
            # SDK minimum movement is 20 cm; permit only one bounded downward command.
            descent = max(20, round((initial_height - TARGET_HEIGHT_CM) / 10) * 10)
            if descent > 50:
                raise RuntimeError("Descent would exceed safety limit of 50 cm; landing")
            event(f"DOWN_{descent}_SENT")
            drone.move_down(descent)
            event(f"DOWN_{descent}_OK")

        # Telemetry may update slightly after move_down() returns. Do not hover
        # indefinitely while waiting: maximum 1 second for a plausible reading.
        confirmed_height = None
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            h = int(drone.get_height())
            if abs(h - TARGET_HEIGHT_CM) <= HEIGHT_TOLERANCE_CM:
                confirmed_height = h
                break
            time.sleep(0.1)
        if confirmed_height is None:
            raise RuntimeError("Reported height not within 40 +/- 10 cm; landing without target images")

        event("TARGET_CAPTURE_START")
        print(f"Reported height {confirmed_height} cm. Saving 8 images immediately...")
        next_frame = time.monotonic()
        for i in range(1, TARGET_IMAGES + 1):
            sleep = next_frame - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            if recorder_errors:
                raise RuntimeError("Background recorder failed: " + recorder_errors[0])
            now = time.monotonic()
            name = f"at40_{i:02d}.png"
            save_image(output / name, camera_image(reader))
            frames.append((name, round(now - start, 3), "TARGET_CAPTURE"))
            print(f"Frame {i}/8 | height={drone.get_height()} cm")
            next_frame += CAMERA_INTERVAL_S
        event("TARGET_CAPTURE_COMPLETE")
        # No extra hover: land immediately after frame 8.
        event("LAND_SENT")
        drone.land()
        landing_acknowledged = True
        event("LAND_OK")
        print("Landing acknowledged; visually confirm touchdown.")
    except (Exception, KeyboardInterrupt) as exc:
        print("STOPPED:", exc)
    finally:
        if takeoff_attempted and not landing_acknowledged:
            print("Safety landing requested...")
            try:
                drone.land()
                landing_acknowledged = True
                event("SAFETY_LAND_OK")
            except Exception as exc:
                print("WARNING: landing command failed; drone may still be airborne:", exc)
        stop_video.set()
        if video_thread is not None:
            video_thread.join(timeout=3)
        if stream_started:
            try:
                drone.streamoff()
            except Exception as exc:
                print("Stream cleanup warning:", exc)
        if connected:
            try:
                drone.end()
            except Exception:
                pass
        with (output / "frames.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(["filename", "seconds_from_takeoff_start", "phase"])
            writer.writerows(frames)
        with (output / "events.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(["event", "seconds_from_takeoff_start"])
            writer.writerows(events)
        print(f"Saved {len(frames)} frames (including ascent) in {output}")
        if recorder_errors:
            print("Recording error:", recorder_errors[0])
        if takeoff_attempted and not landing_acknowledged:
            print("DANGER: LANDING WAS NOT ACKNOWLEDGED")


if __name__ == "__main__":
    main()
