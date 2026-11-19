"""Read-only camera adapter for the EXISTING Marker 23 hover controller.

Home: python -B junction_hover_observer.py --self-test
Home: python -B junction_hover_observer.py --replay captures/hover_20261119_114754

No DJITelloPy import, Tello connection, takeoff, RC control, or motor mapping.
For future integration with an existing camera loop:
    observer = MotorDisabledObserver()
    decision = observer.observe_tello_rgb(reader.frame, phase="HOVER")
    # decision['image_request_px'] is UNEXECUTED, never a motor command.

The controller might suggest image-space corrections, but this observer
NEVER performs any correction and NEVER calls accept_feedback(). Do NOT
interpret successive passive hover images as correction before/after pairs.
"""

import argparse
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

from junction_hover_controller import JunctionHoverController
from junction_marker_detector import detect_junction_marker

TARGET = (160.0, 120.0)
CROP_SHAPE = (240, 320, 3)
RAW_SHAPE = (720, 320, 3)


class MotorDisabledObserver:
    """Process camera observations and log recommendations without actuation."""

    def __init__(self):
        self.controller = JunctionHoverController(target_center=TARGET)
        self.frame_number = 0

    def _halt(self, reason):
        self.controller.latched = True
        return {
            "frame": self.frame_number,
            "center_px": None,
            "status": reason,
            "image_request_px": (0.0, 0.0),
            "request_executed": False,
            "latched": True,
        }

    def observe_oriented_bgr(self, image, phase="HOVER"):
        """Consume a 320x240 BGR frame already rotated to controller axes."""
        self.frame_number += 1
        if self.controller.latched:
            return self._halt("SAFETY_LATCHED_NEUTRAL")
        if (not isinstance(image, np.ndarray)
                or image.shape != CROP_SHAPE
                or image.dtype != np.uint8):
            return self._halt("INVALID_FRAME_LATCHED_NEUTRAL")
        try:
            detected = detect_junction_marker(image)
            # Observation-only: non-hover phases must never issue requests.
            # In particular, controller RECOVERY is NOT an actuation permission.
            safe_phase = "HOVER" if phase == "HOVER" else "OBSERVE"
            status, dx, dy = self.controller.update(detected, phase=safe_phase)
            if status != "IMAGE_REQUEST_ONLY":
                dx = dy = 0.0
            center = tuple(float(v) for v in detected["center"]) if detected else None
            if self.controller.latched and (dx != 0.0 or dy != 0.0):
                return self._halt("INVALID_LATCH_REQUEST_NEUTRAL")
            return {
                "frame": self.frame_number,
                "center_px": center,
                "status": status,
                "image_request_px": (float(dx), float(dy)),
                "request_executed": False,
                "latched": bool(self.controller.latched),
            }
        except (cv2.error, KeyError, ValueError, TypeError, IndexError, OverflowError, ZeroDivisionError):
            return self._halt("INVALID_OBSERVATION_LATCHED_NEUTRAL")

    def observe_saved_crop(self, image, phase="HOVER"):
        """Saved hover_*_crop.png: BGR, unrotated; rotate exactly once."""
        if (not isinstance(image, np.ndarray)
                or image.shape != CROP_SHAPE
                or image.dtype != np.uint8):
            return self.observe_oriented_bgr(None, phase=phase)
        return self.observe_oriented_bgr(cv2.rotate(image, cv2.ROTATE_180), phase)

    def observe_tello_rgb(self, frame, phase="HOVER"):
        """Raw reader.frame: 720x320 RGB, usable upper 240 rows."""
        if (not isinstance(frame, np.ndarray)
                or frame.shape != RAW_SHAPE
                or frame.dtype != np.uint8):
            return self.observe_oriented_bgr(None, phase=phase)
        image = cv2.rotate(
            cv2.cvtColor(frame[:240, :320].copy(), cv2.COLOR_RGB2BGR),
            cv2.ROTATE_180,
        )
        return self.observe_oriented_bgr(image, phase=phase)


def replay(folder):
    folder = Path(folder)
    files = sorted(folder.glob("hover_*_crop.png"))
    if not files:
        raise SystemExit(f"STOP: no hover_*_crop.png frames in {folder}")
    observer = MotorDisabledObserver()
    counts = Counter()
    detected = 0
    for path in files:
        image = cv2.imread(str(path))
        result = observer.observe_saved_crop(image)
        counts[result["status"]] += 1
        detected += result["center_px"] is not None
        print(f"{path.name}: center={result['center_px']} status={result['status']} "
              f"UNEXECUTED_image_request_px={result['image_request_px']}")
        assert not result["request_executed"]
        if result["latched"]:
            # Terminate at the first latch; later frames cannot re-arm it.
            print("STOP: safety latch reached; remaining frames not processed")
            break
    print(f"SUMMARY processed={sum(counts.values())}/{len(files)} "
          f"detected={detected} statuses={dict(counts)} "
          f"latched={observer.controller.latched}")
    print("OBSERVATION ONLY. NO DRONE CONNECTION, MOTOR COMMANDS, OR FEEDBACK CLAIMS.")


def self_test():
    # Synthetic ArUco 23 visual target in the controller's 320x240 frame.
    marker = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 23, 64
    )
    oriented = np.full(CROP_SHAPE, 255, dtype=np.uint8)
    oriented[88:152, 128:192] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)

    o = MotorDisabledObserver()
    r = o.observe_oriented_bgr(oriented)
    assert r["center_px"] is not None, r
    assert abs(r["center_px"][0] - 160) < 2, r
    assert abs(r["center_px"][1] - 120) < 2, r
    assert r["status"] == "ACQUIRING_NEUTRAL", r
    assert not r["request_executed"]
    print("PASS: synthetic Marker 23 visual detection")

    o = MotorDisabledObserver()
    saved = cv2.rotate(oriented, cv2.ROTATE_180)
    r = o.observe_saved_crop(saved)
    assert r["center_px"] is not None and r["status"] == "ACQUIRING_NEUTRAL", r
    assert abs(r["center_px"][0] - 160) < 2, r
    print("PASS: saved-crop image rotation")

    o = MotorDisabledObserver()
    raw = np.zeros(RAW_SHAPE, dtype=np.uint8)
    raw[:240] = cv2.cvtColor(saved, cv2.COLOR_BGR2RGB)
    r = o.observe_tello_rgb(raw)
    assert r["center_px"] is not None and r["status"] == "ACQUIRING_NEUTRAL", r
    print("PASS: raw RGB crop, conversion, and rotation")

    o = MotorDisabledObserver()
    for i in range(3):
        r = o.observe_oriented_bgr(oriented)
    assert r["status"] == "REFERENCE_SET_NEUTRAL", r
    r = o.observe_oriented_bgr(oriented)
    assert r["status"] == "HOLD_NEUTRAL" and r["image_request_px"] == (0, 0), r
    print("PASS: reference acquisition and centered hold")

    o = MotorDisabledObserver()
    # A marker displaced but within the 35-pixel controller safety threshold.
    shifted = np.full(CROP_SHAPE, 255, dtype=np.uint8)
    shifted[98:162, 148:212] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    for _ in range(3):
        o.observe_oriented_bgr(oriented)
    r = o.observe_oriented_bgr(shifted)
    assert r["status"] == "IMAGE_REQUEST_ONLY", r
    assert r["image_request_px"] != (0, 0) and not r["request_executed"], r
    print("PASS: nonzero image request remains UNEXECUTED")

    o = MotorDisabledObserver()
    r = o.observe_oriented_bgr(shifted, phase="RECOVERY")
    assert r["image_request_px"] == (0, 0) and not r["request_executed"], r
    assert o.controller.reference is None, r
    print("PASS: recovery and non-hover states cannot request actuation")

    o = MotorDisabledObserver()
    r = o.observe_tello_rgb(np.zeros((240, 320, 3), dtype=np.uint8))
    assert r["status"] == "INVALID_FRAME_LATCHED_NEUTRAL" and r["latched"], r
    r = o.observe_oriented_bgr(oriented)
    assert r["status"] == "SAFETY_LATCHED_NEUTRAL", r
    print("PASS: invalid frame fails closed and cannot re-arm")
    print("PASS: 7 motor-disabled observer tests")
    print("NO DJITELLOPY, NO WIFI CONNECTION, NO TAKEOFF, NO MOTOR COMMANDS.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--self-test", action="store_true")
    mode.add_argument("--replay", metavar="FOLDER", type=Path)
    args = parser.parse_args()
    if args.self_test:
        self_test()
    else:
        replay(args.replay)


if __name__ == "__main__":
    main()
