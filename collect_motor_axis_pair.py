"""Collect a Tello motor-axis control comparison with drift baseline.

Offline check (NO TELLO):
    python -B collect_motor_axis_pair.py --self-test

Live, guarded experiment (REAL TAKEOFF AND RC MOVEMENT):
    python -B collect_motor_axis_pair.py --fly --axis right
    python -B collect_motor_axis_pair.py --fly --axis forward

Run from C:\\python\\Drone\\Drone, with clear unobstructed space.
Requires visual supervision. This is NOT a stabilization controller.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
import statistics
import time

import cv2
import numpy as np

HOST = "192.168.1.63"
MIN_BATTERY = 40
SAMPLES = 6
INTERVAL_S = 0.15
RC_SPEED = 10
RC_SEND_COUNT = 4
MAX_ELAPSED_S = 25.0
CROP_H, CROP_W = 240, 320


def to_oriented_image(rgb):
    if rgb is None or not isinstance(rgb, np.ndarray) or rgb.shape != (720, 320, 3):
        raise RuntimeError(f"Unexpected video shape: {getattr(rgb, 'shape', None)}")
    return cv2.rotate(cv2.cvtColor(rgb[:CROP_H, :CROP_W].copy(),
                                     cv2.COLOR_RGB2BGR), cv2.ROTATE_180)


def quality(markers):
    """Gate RC pulses on enough marker detections, centrality, and steadiness."""
    pts = [p for p in markers if p is not None]
    if len(pts) < 4:
        return False, f"Only {len(pts)}/{len(markers)} valid Marker 23 detections"
    x = statistics.median(p[0] for p in pts)
    y = statistics.median(p[1] for p in pts)
    if abs(x - 160) > 70 or abs(y - 120) > 70:
        return False, f"Marker too far from frame centre: ({x:.1f},{y:.1f})"
    spread = max(((px-x)**2 + (py-y)**2)**0.5 for px,py in pts)
    if spread > 18:
        return False, f"Marker moving too much during sample: spread {spread:.1f}px"
    return True, f"{len(pts)}/{len(markers)} valid; median ({x:.1f},{y:.1f}); spread {spread:.1f}px"


def rc_vector(axis, sign):
    if axis == "right":
        return (sign * RC_SPEED, 0, 0, 0)
    if axis == "forward":
        return (0, sign * RC_SPEED, 0, 0)
    raise ValueError("Invalid axis")


def pulse(drone, axis, sign, log):
    """One bounded commanded pulse; always attempt a neutral RC at the end."""
    vector = rc_vector(axis, sign)
    log("PULSE_START", vector)
    try:
        for _ in range(RC_SEND_COUNT):
            drone.send_rc_control(*vector)
            time.sleep(INTERVAL_S)
    finally:
        try:
            drone.send_rc_control(0, 0, 0, 0)
        except Exception as exc:
            print("WARNING: neutral RC failed:", exc, flush=True)
        log("PULSE_STOP", (0, 0, 0, 0))


def run_live(axis):
    # Import Tello ONLY after explicit --fly. Safe for --self-test.
    from djitellopy import Tello
    from hover.junction_marker_detector import detect_junction_marker

    root = Path("captures") / f"paired_axis_{axis}_{datetime.now():%Y%m%d_%H%M%S}"
    root.mkdir(parents=True, exist_ok=False)
    drone = Tello(host=HOST)
    connected = streaming = takeoff_attempted = False
    started = time.monotonic()
    csv_file = (root / "measurements.csv").open("w", newline="", encoding="utf-8")
    writer = csv.writer(csv_file)
    writer.writerow(["seconds", "phase", "sample", "marker_x_px", "marker_y_px", "image_file", "event_detail"])

    def log(phase, detail="", sample="", marker=None, image=""):
        elapsed = round(time.monotonic() - started, 3)
        x, y = marker if marker is not None else ("", "")
        writer.writerow([elapsed, phase, sample, x, y, image, str(detail)])
        csv_file.flush()

    def guard_time():
        if takeoff_attempted and time.monotonic() - airborne_since > MAX_ELAPSED_S:
            raise RuntimeError("Airborne duration budget exceeded: landing")

    def capture(reader, label):
        markers = []
        for i in range(1, SAMPLES + 1):
            guard_time()
            image = to_oriented_image(reader.frame)
            name = f"{label}_{i:02d}.png"
            if not cv2.imwrite(str(root / name), image):
                raise RuntimeError(f"Image write failure: {name}")
            info = detect_junction_marker(image)
            center = tuple(info["center"]) if info is not None else None
            markers.append(center)
            log(label, sample=i, marker=center, image=name)
            print(f"{label} {i}/{SAMPLES}: {center}", flush=True)
            time.sleep(INTERVAL_S)
        good, summary = quality(markers)
        log(label + "_SUMMARY", summary)
        print("GATE", label, "PASS" if good else "FAIL", summary, flush=True)
        return good

    try:
        print("REAL FLIGHT TEST: short RC +/- pulses only; NOT autonomous correction.", flush=True)
        drone.connect()
        connected = True
        battery = drone.get_battery()
        print("BATTERY:", battery, "%", flush=True)
        log("BATTERY", battery)
        if battery < MIN_BATTERY:
            raise RuntimeError("Battery below safety threshold; cancelled")
        drone.set_video_direction(Tello.CAMERA_DOWNWARD)
        drone.streamon()
        streaming = True
        reader = drone.get_frame_read()
        preview = None
        for _ in range(20):
            time.sleep(0.25)
            try:
                view = to_oriented_image(reader.frame)
            except RuntimeError:
                continue
            if view.std() > 10 and view.mean() > 8:
                preview = view
                break
        if preview is None:
            raise RuntimeError("No usable camera preview; cancelled")
        if not cv2.imwrite(str(root / "preflight.png"), preview):
            raise RuntimeError("Unable to save preflight preview")
        marker_preview = detect_junction_marker(preview)
        print("PREFLIGHT IMAGE:", root / "preflight.png", flush=True)
        print("PREFLIGHT MARKER:", None if marker_preview is None else marker_preview["center"], flush=True)
        print("ONLY fly in a large clear area with visual supervision. Never above a small table.", flush=True)
        approval = input(f"Review preview/area. Type FLY-{axis.upper()} to permit takeoff and +/- {axis} RC pulses: ").strip()
        if approval != f"FLY-{axis.upper()}":
            print("Cancelled; no takeoff", flush=True)
            return

        takeoff_attempted = True  # Set before ambiguous takeoff failure
        airborne_since = time.monotonic()
        log("TAKEOFF_START")
        drone.takeoff()
        log("TAKEOFF_ACK")
        time.sleep(2)
        print("REPORTED HEIGHT (not calibrated):", drone.get_height(), "cm", flush=True)

        # Two successive no-RC samples estimate time-dependent natural drift.
        if not capture(reader, "no_rc_1"):
            raise RuntimeError("No-RC baseline not reliable: landing without pulses")
        if not capture(reader, "no_rc_2"):
            raise RuntimeError("No-RC baseline not reliable: landing without pulses")

        guard_time()
        pulse(drone, axis, +1, log)
        if not capture(reader, "after_positive"):
            raise RuntimeError("Marker unreliable after positive pulse: no reverse pulse; landing")

        guard_time()
        pulse(drone, axis, -1, log)
        capture(reader, "after_negative")
        print("PAIRED EXPERIMENT COMPLETE — inspect CSV and PNGs offline.", flush=True)

    except KeyboardInterrupt:
        print("INTERRUPTED — initiating landing", flush=True)
        log("INTERRUPTED")
    except Exception as exc:
        print("STOP:", exc, flush=True)
        log("STOP", str(exc))
    finally:
        if takeoff_attempted:
            try:
                drone.send_rc_control(0, 0, 0, 0)
            except Exception as exc:
                print("WARNING: final neutral RC failed:", exc, flush=True)
            try:
                drone.land()
                print("LANDING COMMAND ACKNOWLEDGED — visually confirm touchdown", flush=True)
                log("LANDING_ACK")
            except Exception as exc:
                print("CRITICAL: landing command failed:", exc, flush=True)
                log("LANDING_ERROR", str(exc))
            time.sleep(2)
        if streaming:
            try:
                drone.streamoff()
            except Exception as exc:
                print("Stream cleanup error:", exc)
        if connected:
            try:
                drone.end()
            except Exception as exc:
                print("Connection cleanup error:", exc)
        csv_file.close()
        print("DATASET:", root, flush=True)
        print("Actual RC response and physical landing safety remain to be evaluated.", flush=True)


def self_test():
    from unittest.mock import patch
    assert rc_vector("right", +1) == (10, 0, 0, 0)
    assert rc_vector("right", -1) == (-10, 0, 0, 0)
    assert rc_vector("forward", +1) == (0, 10, 0, 0)
    assert rc_vector("forward", -1) == (0, -10, 0, 0)
    assert quality([(150, 115)] * 6)[0]
    assert not quality([None] * 6)[0]
    assert not quality([(300, 115)] * 6)[0]
    assert not quality([(150, 115), (200, 120)] * 3)[0]
    assert to_oriented_image(np.zeros((720, 320, 3), np.uint8)).shape == (240, 320, 3)
    class FakeDrone:
        def __init__(self, fail=False):
            self.calls = []
            self.fail = fail
        def send_rc_control(self, *command):
            self.calls.append(command)
            if self.fail and command[0] != 0:
                raise RuntimeError("simulated RC failure")
    for axis in ("right", "forward"):
        for sign in (-1, +1):
            fake = FakeDrone()
            with patch("time.sleep", return_value=None):
                pulse(fake, axis, sign, lambda *args: None)
            assert fake.calls == [rc_vector(axis, sign)] * RC_SEND_COUNT + [(0, 0, 0, 0)]
    fake = FakeDrone(fail=True)
    try:
        with patch("time.sleep", return_value=None):
            pulse(fake, "right", +1, lambda *args: None)
    except RuntimeError:
        pass
    else:
        raise AssertionError("Simulated failure did not propagate")
    assert fake.calls[-1] == (0, 0, 0, 0)
    print("PASS: RC axis signs, tracking gates, camera crop, pulse and neutral-failure cleanup")
    print("SELF-TEST ONLY. NO DJITELLOPY IMPORT, NO WIFI, NO FLIGHT OR MOTOR COMMANDS.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--self-test", action="store_true")
    group.add_argument("--fly", action="store_true")
    parser.add_argument("--axis", choices=("right", "forward"))
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.axis:
        parser.error("--fly requires --axis right or --axis forward")
    run_live(args.axis)


if __name__ == "__main__":
    main()
