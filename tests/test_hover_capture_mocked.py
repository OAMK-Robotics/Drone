"""Mocked integration tests for the existing test_hover_capture.py.

Run from the Drone repo root:
    python -B test_hover_capture_mocked.py

Patches the djitellopy import BEFORE executing the capture script. All tests
run in temporary directories with synthetic camera frames. There is NO network,
DJI connection, takeoff, RC command, or physical landing.
"""
from __future__ import annotations

import ast
import contextlib
import io
import os
from pathlib import Path
import runpy
import sys
import tempfile
import types
from unittest.mock import patch

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "test_hover_capture.py"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def make_camera_frame():
    """Make the confirmed 720x320 RGB feed, with oriented Marker 23 centered."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    tag = cv2.aruco.generateImageMarker(dictionary, 23, 64)
    oriented = np.full((240, 320, 3), 255, np.uint8)
    oriented[88:152, 128:192] = cv2.cvtColor(tag, cv2.COLOR_GRAY2BGR)
    unrotated_bgr = cv2.rotate(oriented, cv2.ROTATE_180)
    rgb = np.zeros((720, 320, 3), np.uint8)
    rgb[:240] = cv2.cvtColor(unrotated_bgr, cv2.COLOR_BGR2RGB)
    return rgb


FRAME = make_camera_frame()


class FakeReader:
    def __init__(self, drone):
        self.drone = drone
        self.hover_reads = 0

    @property
    def frame(self):
        if self.drone.airborne:
            self.hover_reads += 1
            if (self.drone.invalid_after is not None
                    and self.hover_reads >= self.drone.invalid_after):
                # The capture function accepts >=240 rows, but the shared
                # observer requires the verified exact 720x320 format.
                return np.concatenate((FRAME, FRAME[:1]), axis=0)
        return FRAME


class FakeTello:
    CAMERA_DOWNWARD = 1
    instances = []
    battery_value = 70
    fail_takeoff = False
    invalid_after = None

    def __init__(self, host=None):
        self.host = host
        self.calls = []
        self.airborne = False
        self.invalid_after = type(self).invalid_after
        self.reader = FakeReader(self)
        type(self).instances.append(self)

    def _log(self, action):
        self.calls.append(action)

    def connect(self):
        self._log("connect")

    def get_battery(self):
        self._log("get_battery")
        return type(self).battery_value

    def set_video_direction(self, value):
        self._log("set_video_direction")

    def streamon(self):
        self._log("streamon")

    def get_frame_read(self):
        self._log("get_frame_read")
        return self.reader

    def takeoff(self):
        self._log("takeoff")
        if type(self).fail_takeoff:
            raise RuntimeError("SIMULATED takeoff failure")
        self.airborne = True

    def get_height(self):
        self._log("get_height")
        return 60 if self.airborne else 0

    def land(self):
        self._log("land")
        self.airborne = False

    def streamoff(self):
        self._log("streamoff")

    def end(self):
        self._log("end")

    # Deliberately fail if a horizontal command is introduced in the script.
    def send_rc_control(self, *args, **kwargs):
        raise AssertionError("FORBIDDEN: motor RC command in observer-only capture")

    def __getattr__(self, name):
        if (name.startswith("move_") or name.startswith("rotate_")
                or name.startswith("go_") or name.startswith("curve_")):
            raise AssertionError(f"FORBIDDEN: {name} in observer-only capture")
        raise AttributeError(name)


def check_source():
    if not SCRIPT.is_file():
        raise SystemExit(f"STOP: capture source missing: {SCRIPT}")
    source = SCRIPT.read_text(encoding="utf-8-sig")
    tree = ast.parse(source, filename=str(SCRIPT))
    required = ("MotorDisabledObserver", "observe_tello_rgb", "request_executed")
    for name in required:
        assert name in source, f"Integration missing: {name}"
    forbidden = {"send_rc_control", "move_right", "move_left", "move_forward",
                 "move_back", "go_xyz_speed", "curve_xyz_speed", "emergency"}
    present = {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
    assert not (present & forbidden), f"Unexpected motor API: {present & forbidden}"
    print("PASS: capture source has observer integration and no horizontal motor API")


def run_case(name, *, approval="FLY", battery=70, fail_takeoff=False,
             invalid_after=None):
    FakeTello.instances.clear()
    FakeTello.battery_value = battery
    FakeTello.fail_takeoff = fail_takeoff
    FakeTello.invalid_after = invalid_after
    fake_djitellopy = types.ModuleType("djitellopy")
    fake_djitellopy.Tello = FakeTello
    buffer = io.StringIO()
    previous_cwd = Path.cwd()
    with tempfile.TemporaryDirectory(prefix="tello_mock_") as temporary:
        try:
            os.chdir(temporary)
            with (patch.dict(sys.modules, {"djitellopy": fake_djitellopy}),
                  patch("time.sleep", return_value=None),
                  patch("builtins.input", side_effect=lambda prompt="": approval),
                  contextlib.redirect_stdout(buffer)):
                runpy.run_path(str(SCRIPT), run_name="__main__")
            output = buffer.getvalue()
            assert len(FakeTello.instances) == 1, f"{name}: expected one fake drone"
            drone = FakeTello.instances[0]
            saved_crops = list(Path(temporary).rglob("hover_*_crop.png"))
            assert "connect" in drone.calls and "end" in drone.calls, drone.calls
            assert not drone.airborne, f"{name}: fake drone left airborne"
            if name == "cancel":
                assert "takeoff" not in drone.calls and "land" not in drone.calls
                assert not saved_crops
            elif name == "low_battery":
                assert "takeoff" not in drone.calls and "land" not in drone.calls
                assert "streamon" not in drone.calls
                assert "Battery below" in output
            elif name == "normal":
                assert drone.calls.count("takeoff") == drone.calls.count("land") == 1
                assert len(saved_crops) == 15, len(saved_crops)
                assert "HOLD_NEUTRAL" in output
                assert "UNEXECUTED_IMAGE_REQUEST" in output
            elif name == "observer_latch":
                assert drone.calls.count("takeoff") == drone.calls.count("land") == 1
                assert 0 < len(saved_crops) < 15
                assert "observer safety latch" in output
                assert "INVALID_FRAME_LATCHED_NEUTRAL" in output
            elif name == "takeoff_failure":
                assert drone.calls.count("takeoff") == drone.calls.count("land") == 1
                assert not saved_crops
                assert "SIMULATED takeoff failure" in output
            else:
                raise AssertionError(f"Unknown case {name}")
            if "streamon" in drone.calls:
                assert "streamoff" in drone.calls, drone.calls
            print(f"PASS: {name}: mock flight lifecycle and cleanup")
        finally:
            os.chdir(previous_cwd)


def main():
    check_source()
    run_case("cancel", approval="NO")
    run_case("low_battery", battery=30)
    run_case("normal")
    run_case("observer_latch", invalid_after=5)
    run_case("takeoff_failure", fail_takeoff=True)
    print("PASS: 5 mocked lifecycle scenarios + source safety check")
    print("MOCK ONLY. NO WIFI. NO REAL TELLO. NO FLIGHT OR MOTOR COMMANDS.")
    print("Physical flight safety and landing behavior remain unverified.")


if __name__ == "__main__":
    main()
