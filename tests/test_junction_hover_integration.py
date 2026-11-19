"""Offline integration checks for the existing Marker 23 hover controller.

Run from the Drone repository root:
    python -B test_junction_hover_integration.py

Uses a SYNTHETIC image-space plant for feedback checks and, when present,
replays SAVED hover PNGs for observational reporting. No drone, Wi-Fi,
DJITelloPy, MQTT, or motor commands. Neither test establishes motor mapping.
"""

from collections import Counter
from pathlib import Path
import argparse
import math

import cv2

from junction_hover_controller import JunctionHoverController
from junction_marker_detector import detect_junction_marker


TARGET = (160.0, 120.0)


def marker(x, y):
    """Synthetic ArUco observation: constant marker scale and finite center."""
    return {
        "id": 23,
        "center": (float(x), float(y)),
        "corners": [(0.0, 0.0), (16.0, 0.0), (16.0, 16.0), (0.0, 16.0)],
    }


def acquired_controller():
    controller = JunctionHoverController(target_center=TARGET)
    for expected in ("ACQUIRING_NEUTRAL", "ACQUIRING_NEUTRAL", "REFERENCE_SET_NEUTRAL"):
        decision = controller.update(marker(*TARGET), phase="HOVER")
        assert decision == (expected, 0.0, 0.0), (expected, decision)
    assert controller.reference == TARGET
    return controller


def check_nominal_feedback():
    """Synthetic plant obeys IMAGE-SPACE requests exactly, never actual RC."""
    controller = acquired_controller()
    observation = marker(176, 136)
    corrections = 0
    for _ in range(12):
        status, req_x, req_y = controller.update(observation, phase="HOVER")
        if status == "HOLD_NEUTRAL":
            assert req_x == req_y == 0
            assert not controller.latched
            assert all(abs(observation["center"][i] - TARGET[i]) <= 6 for i in (0, 1))
            assert corrections > 0
            print(f"PASS: nominal synthetic 2D loop converged in {corrections} correction steps")
            return
        assert status == "IMAGE_REQUEST_ONLY", (status, req_x, req_y)
        assert abs(req_x) <= 4 and abs(req_y) <= 4
        assert req_x != 0 and req_y != 0, "Initial synthetic test must exercise both axes"
        # IDEALIZED SIMULATOR ONLY: no assumed physical drone response.
        next_observation = marker(observation["center"][0] + req_x,
                                  observation["center"][1] + req_y)
        verified = controller.verify_correction(observation, next_observation)
        assert verified in ("CORRECTION_IMPROVED", "CORRECTION_CENTERED_NEUTRAL"), verified
        assert not controller.latched
        observation = next_observation
        corrections += 1
    raise AssertionError("Ideal synthetic loop did not converge within 12 steps")


def check_failure_feedback(gain, expected_label):
    """Wrong-way or no-response simulated outcome must latch before next request."""
    controller = acquired_controller()
    before = marker(176, 136)
    status, rx, ry = controller.update(before, phase="HOVER")
    assert status == "IMAGE_REQUEST_ONLY" and rx and ry
    after = marker(before["center"][0] + gain * rx,
                   before["center"][1] + gain * ry)
    assert controller.verify_correction(before, after) == "CORRECTION_NOT_IMPROVED_LATCHED"
    assert controller.latched
    assert controller.update(after, phase="HOVER") == ("JUMP_LATCHED_NEUTRAL", 0.0, 0.0)
    print(f"PASS: {expected_label} latches and blocks further image requests")


def check_marker_loss_feedback():
    controller = acquired_controller()
    before = marker(176, 136)
    assert controller.update(before, phase="HOVER")[0] == "IMAGE_REQUEST_ONLY"
    assert controller.verify_correction(before, None) == "CORRECTION_UNVERIFIED_LATCHED"
    assert controller.latched
    assert controller.update(None, phase="HOVER") == ("JUMP_LATCHED_NEUTRAL", 0.0, 0.0)
    print("PASS: missing feedback latches and blocks further image requests")


def check_neutral_states():
    controller = acquired_controller()
    assert controller.update(marker(164, 124), phase="HOVER") == ("HOLD_NEUTRAL", 0.0, 0.0)
    observer = JunctionHoverController(target_center=TARGET)
    assert observer.update(marker(176, 136), phase="ASCENT") == ("OBSERVE_ONLY_NEUTRAL", 0.0, 0.0)
    assert observer.reference is None
    print("PASS: centered hold and non-hover observation are neutral")


def replay_real_hover_captures(pattern):
    """Read-only replay; cannot verify corrections because these are hover photos."""
    folders = [p for p in sorted(Path().glob(pattern)) if p.is_dir()]
    if not folders:
        print(f"SKIP: no local hover folders matching {pattern!r}")
        return

    print("RECORDED HOVER REPLAY (observation only; no motor feedback):")
    for folder in folders:
        files = sorted(folder.glob("hover_*_crop.png"))
        if not files:
            continue
        controller = JunctionHoverController(target_center=TARGET)
        counts = Counter()
        centers = []
        missing = 0
        for path in files:
            frame = cv2.imread(str(path))
            if frame is None or frame.shape[:2] != (240, 320):
                raise RuntimeError(f"Missing/invalid saved cropped image: {path}")
            # SAVED hover crops are unrotated, unlike the live controller feed.
            oriented = cv2.rotate(frame, cv2.ROTATE_180)
            observed = detect_junction_marker(oriented)
            if observed is None:
                missing += 1
            else:
                centers.append(observed["center"])
            status, req_x, req_y = controller.update(observed, phase="HOVER")
            counts[status] += 1
            if controller.latched:
                assert req_x == req_y == 0, "Latched replay must not request correction"
        delta = (tuple(round(centers[-1][i] - centers[0][i], 2)
                       for i in (0, 1)) if len(centers) > 1 else None)
        print(f"  {folder.name}: detected={len(files)-missing}/{len(files)} "
              f"delta_px={delta} reference={controller.reference} "
              f"latched={controller.latched} statuses={dict(counts)}")
    print("RECORDED REPLAY COMPLETE. This is observation-only, not closed-loop flight evidence.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--captures-glob", default="captures/hover_20261119_*",
                        help="Saved hover folders; default November 19 recordings")
    args = parser.parse_args()

    check_nominal_feedback()
    check_failure_feedback(-1, "reverse-direction synthetic response")
    check_failure_feedback(0, "no-response synthetic response")
    check_marker_loss_feedback()
    check_neutral_states()
    print("PASS: 5 offline integration scenarios")
    replay_real_hover_captures(args.captures_glob)
    print("NO TELLO CONNECTION. NO MOTOR COMMANDS. NO FLIGHT VALIDATION.")


if __name__ == "__main__":
    main()
