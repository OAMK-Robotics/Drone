"""Offline junction hovering. Never sends drone commands."""
import argparse
import csv
import math
from collections import Counter
from pathlib import Path

import cv2
from junction_marker_detector import detect_junction_marker


class JunctionHoverController:
    def __init__(self, target_center=None):
        self.reference = None
        self.target_center = target_center
        self.good = []
        self.previous = None
        self.latched = False
        self.reacquiring = False
        self.reacquire_count = 0
        self.recovery_previous = None
        self.recovery_good = 0

    def _recovery(self, marker):
        # Offline IMAGE-SPACE requests only. No motor commands.
        if self.target_center is None:
            return "RECOVERY_NO_TARGET_NEUTRAL", 0.0, 0.0

        if marker is None:
            self.recovery_previous = None
            self.recovery_good = 0
            return "RECOVERY_MARKER_LOST_NEUTRAL", 0.0, 0.0

        try:
            center = tuple(float(v) for v in marker["center"])
            corners = marker["corners"]
            side = math.dist(corners[0], corners[1])
        except (KeyError, IndexError, TypeError, ValueError):
            center, side = (float("nan"), float("nan")), 0.0

        if (len(center) != 2
                or not all(map(math.isfinite, (*center, side)))
                or side < 10
                or not (8 <= center[0] <= 312 and 8 <= center[1] <= 232)):
            self.recovery_previous = None
            self.recovery_good = 0
            return "RECOVERY_UNTRUSTED_NEUTRAL", 0.0, 0.0

        if self.recovery_previous is not None:
            old_center, old_side = self.recovery_previous
            if (math.dist(center, old_center) > 25
                    or abs(side / old_side - 1) > 0.25):
                self.recovery_previous = (center, side)
                self.recovery_good = 1
                return "RECOVERY_JUMP_RESET_NEUTRAL", 0.0, 0.0

        self.recovery_previous = (center, side)
        self.recovery_good += 1

        if self.recovery_good < 3:
            return "RECOVERY_ACQUIRING_NEUTRAL", 0.0, 0.0

        error = (
            center[0] - self.target_center[0],
            center[1] - self.target_center[1]
        )

        if math.hypot(*error) > 100:
            return "RECOVERY_TOO_FAR_NEUTRAL", 0.0, 0.0

        request = tuple(
            0.0 if abs(e) <= 6
            else round(max(-4.0, min(4.0, -0.5 * e)), 2)
            for e in error
        )

        status = (
            "RECOVERY_CENTERED_NEUTRAL"
            if request == (0.0, 0.0)
            else "RECOVERY_IMAGE_REQUEST_ONLY"
        )
        return status, *request

    def verify_correction(self, before, after):
        """Evaluate 2D marker-center feedback in pixels; NO motor control."""
        if self.latched:
            return "SAFETY_LATCHED_NEUTRAL"

        try:
            target = tuple(float(v) for v in self.target_center)
            a = tuple(float(v) for v in before["center"])
            b = tuple(float(v) for v in after["center"])
            if any(len(p) != 2 for p in (target, a, b)):
                raise ValueError("Expected 2D points")
            if not all(math.isfinite(v) for p in (target, a, b) for v in p):
                raise ValueError("Nonfinite coordinate")
            if not all(8 <= x <= 312 and 8 <= y <= 232 for x, y in (a, b)):
                raise ValueError("Marker outside image bounds")
        except (TypeError, ValueError, KeyError, IndexError, OverflowError):
            self.latched = True
            return "CORRECTION_UNVERIFIED_LATCHED"

        # Offline-only provisional tolerances; not flight-calibrated.
        deadband = 6.0
        minimum_improvement = 3.0
        maximum_axis_deterioration = 3.0
        before_error = tuple(abs(a[i] - target[i]) for i in (0, 1))
        after_error = tuple(abs(b[i] - target[i]) for i in (0, 1))

        if all(e <= deadband for e in after_error):
            if all(e <= deadband for e in before_error):
                return "CORRECTION_ALREADY_CENTERED_NEUTRAL"
            return "CORRECTION_CENTERED_NEUTRAL"

        if any(after_error[i] - before_error[i] > maximum_axis_deterioration
               for i in (0, 1)):
            self.latched = True
            return "CORRECTION_NOT_IMPROVED_LATCHED"

        improvement = math.hypot(*before_error) - math.hypot(*after_error)
        if improvement >= minimum_improvement:
            return "CORRECTION_IMPROVED"
        self.latched = True
        return "CORRECTION_NOT_IMPROVED_LATCHED"

    def update(self, marker, phase="HOVER"):
        if phase == "RECOVERY":
            if self.latched:
                return "SAFETY_LATCHED_NEUTRAL", 0.0, 0.0
            return self._recovery(marker)

        self.recovery_previous = None
        self.recovery_good = 0
        # Outside hover stage, observe only.
        # Never establish a reference or request corrections.
        if phase != "HOVER":
            if marker is None:
                return "OBSERVE_NO_MARKER_NEUTRAL", 0.0, 0.0
            return "OBSERVE_ONLY_NEUTRAL", 0.0, 0.0
        if self.latched:
            return "JUMP_LATCHED_NEUTRAL", 0.0, 0.0

        if marker is None:
            self.previous = None
            self.good.clear()
            self.reacquiring = self.reference is not None
            self.reacquire_count = 0
            return "MARKER_LOST_NEUTRAL", 0.0, 0.0

        center = tuple(float(v) for v in marker["center"])
        corners = marker["corners"]
        side = math.dist(corners[0], corners[1])

        if side < 10 or not all(
            math.isfinite(v) for v in (*center, side)
        ):
            self.previous = None
            self.good.clear()
            self.reacquiring = self.reference is not None
            self.reacquire_count = 0
            return "UNTRUSTED_MARKER_NEUTRAL", 0.0, 0.0

        if self.previous is not None:
            movement = math.dist(center, self.previous[0])
            scale_change = abs(side / self.previous[1] - 1)

            if movement > 25 or scale_change > 0.25:
                if self.reference is None:
                    self.previous = (center, side)
                    self.good.clear()
                    return "ACQUISITION_RESET_NEUTRAL", 0.0, 0.0
                self.latched = True
                return "JUMP_LATCHED_NEUTRAL", 0.0, 0.0

        self.previous = (center, side)

        if self.reacquiring:
            if math.dist(center, self.reference) > 35:
                self.latched = True
                return "REACQUIRE_TOO_FAR_LATCHED_NEUTRAL", 0.0, 0.0

            self.reacquire_count += 1
            if self.reacquire_count < 3:
                return "REACQUIRING_NEUTRAL", 0.0, 0.0

            self.reacquiring = False
            self.reacquire_count = 0
            return "TRACKING_REACQUIRED_NEUTRAL", 0.0, 0.0

        if self.reference is None:
            # 320x240 rotated downward-camera image.
            # Acquire a hover reference only near image centre.
            if abs(center[0] - 160) > 60 or abs(center[1] - 120) > 45:
                self.good.clear()
                return "NOT_CENTERED_NEUTRAL", 0.0, 0.0

            if self.good and math.dist(center, self.good[0]) > 5.0:
                self.good[:] = [center]
                return "ACQUISITION_MOVING_NEUTRAL", 0.0, 0.0
            self.good.append(center)

            if len(self.good) < 3:
                return "ACQUIRING_NEUTRAL", 0.0, 0.0

            self.reference = (
                self.target_center if self.target_center is not None
                else tuple(
                    sum(p[i] for p in self.good) / 3
                    for i in (0, 1)
                )
            )
            self.good.clear()
            return "REFERENCE_SET_NEUTRAL", 0.0, 0.0

        error = (
            center[0] - self.reference[0],
            center[1] - self.reference[1]
        )

        if math.hypot(*error) > 35:
            self.latched = True
            return "LARGE_ERROR_LATCHED_NEUTRAL", 0.0, 0.0

        requests = tuple(
            0.0 if abs(e) <= 6
            else round(max(-4.0, min(4.0, -0.5 * e)), 2)
            for e in error
        )

        status = (
            "HOLD_NEUTRAL"
            if requests == (0.0, 0.0)
            else "IMAGE_REQUEST_ONLY"
        )

        return status, *requests


def replay(folder, recovery_diagnostic=False):
    folder = Path(folder)
    with (folder / "frames.csv").open(newline="") as f:
        rows = list(csv.DictReader(f))

    controller = JunctionHoverController(target_center=(160.0, 120.0))
    counts = Counter()
    detections = 0

    for row in rows:

        path = folder / row["filename"]
        frame = cv2.imread(str(path))

        if frame is None:
            raise RuntimeError(f"Unreadable frame: {path}")

        marker = detect_junction_marker(frame)
        detections += marker is not None

        if recovery_diagnostic and row["phase"] in ("TARGET_CAPTURE", "TARGET_CAPTURE_START"):
            phase = "RECOVERY"
        else:
            phase = "HOVER" if row["phase"] == "TARGET_CAPTURE_START" else row["phase"]
        status, rx, ry = controller.update(marker, phase=phase)
        counts[status] += 1

        print(
            f'{row["seconds_from_takeoff_start"]}s '
            f'{status} image_request=({rx},{ry})'
        )

    print("Frames processed:", sum(counts.values()))
    print("Marker 23 detections:", detections)
    print("Decisions:", dict(counts))
    print("Reference:", controller.reference)
    print("Safety locked:", controller.latched)
    print("OFFLINE ONLY. NO DRONE CONNECTION.")


def self_test():
    def marker(x, y):
        return {
            "center": (x, y),
            "corners": [
                [0, 0], [16, 0], [16, 16], [0, 16]
            ]
        }

    c = JunctionHoverController()

    assert c.update(None)[0] == "MARKER_LOST_NEUTRAL"
    assert c.update(marker(100, 100))[0] == "ACQUIRING_NEUTRAL"
    assert c.update(marker(100, 100))[0] == "ACQUIRING_NEUTRAL"
    assert c.update(marker(100, 100))[0] == "REFERENCE_SET_NEUTRAL"
    assert c.update(marker(100, 100))[0] == "HOLD_NEUTRAL"

    assert c.update(marker(92, 100)) == (
        "IMAGE_REQUEST_ONLY", 4.0, 0.0
    )

    assert c.update(None)[0] == "MARKER_LOST_NEUTRAL"

    for i in range(3):
        status = c.update(marker(90, 100))[0]

    assert status == "TRACKING_REACQUIRED_NEUTRAL"
    assert c.reference == (100.0, 100.0)
    assert c.update(marker(150, 100))[0] == "JUMP_LATCHED_NEUTRAL"

    # Ascent must never create a hover reference or motor request.
    observing = JunctionHoverController()
    for x in (110, 160, 120, 155):
        assert observing.update(
            marker(x, 120), phase="ASCENT"
        ) == ("OBSERVE_ONLY_NEUTRAL", 0.0, 0.0)

    assert observing.update(None, phase="DESCENT")[0] == (
        "OBSERVE_NO_MARKER_NEUTRAL"
    )
    assert observing.reference is None
    assert not observing.latched

    # Hover acquisition becomes possible only when explicitly enabled.
    statuses = [
        observing.update(marker(160, 120), phase="HOVER")[0]
        for _ in range(3)
    ]
    assert statuses[-1] == "REFERENCE_SET_NEUTRAL"

    moving = JunctionHoverController(target_center=(160., 120.))
    assert moving.update(marker(120, 120))[0] == "ACQUIRING_NEUTRAL"
    assert moving.update(marker(140, 120))[0] == "ACQUISITION_MOVING_NEUTRAL"
    assert moving.update(marker(160, 120))[0] == "ACQUISITION_MOVING_NEUTRAL"
    assert moving.reference is None

    # Permanent offline regression tests for 2D feedback.
    cases = [
        ("X improves", (200, 120), (190, 120), "CORRECTION_IMPROVED", False),
        ("Y improves", (160, 160), (160, 145), "CORRECTION_IMPROVED", False),
        ("Both improve", (200, 160), (190, 145), "CORRECTION_IMPROVED", False),
        ("Y deteriorates", (200, 120), (190, 180), "CORRECTION_NOT_IMPROVED_LATCHED", True),
        ("X deteriorates", (160, 170), (170, 140), "CORRECTION_NOT_IMPROVED_LATCHED", True),
        ("Insufficient improvement", (200, 120), (198, 120), "CORRECTION_NOT_IMPROVED_LATCHED", True),
        ("Already centered", (160, 120), (160, 120), "CORRECTION_ALREADY_CENTERED_NEUTRAL", False),
        ("Enter deadband", (167, 120), (165, 120), "CORRECTION_CENTERED_NEUTRAL", False),
        ("Centered then drifts", (160, 120), (170, 120), "CORRECTION_NOT_IMPROVED_LATCHED", True),
        ("Marker lost", (200, 120), None, "CORRECTION_UNVERIFIED_LATCHED", True),
        ("Invalid coordinate", (200, 120), (float("nan"), 120), "CORRECTION_UNVERIFIED_LATCHED", True),
        ("No target", (200, 120), (190, 120), "CORRECTION_UNVERIFIED_LATCHED", True),
    ]
    for label, before, after, expected, should_latch in cases:
        c2d = JunctionHoverController(
            target_center=None if label == "No target" else (160.0, 120.0)
        )
        first = {"center": before}
        second = None if after is None else {"center": after}
        actual = c2d.verify_correction(first, second)
        assert actual == expected, (label, actual, expected)
        assert c2d.latched == should_latch, label
        print("PASS: 2D", label)
    assert c2d.verify_correction({"center": (200, 120)}, {"center": (190, 120)}) == "SAFETY_LATCHED_NEUTRAL"
    print("PASS: 12 offline 2D feedback regression cases + latched-state check")

    print("PASS: hover-controller safety and correction tests")
    print("NO DRONE CONNECTION. NO FLIGHT.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", nargs="?")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        self_test()
    elif args.folder:
        replay(args.folder)
    else:
        parser.error("Provide --self-test or captures folder")
