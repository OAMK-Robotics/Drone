"""Compatibility launcher for manual camera-axis calibration.

Actual implementation: calibration/calibrate_camera_axes_marker23.py.
Normal invocation CONNECTS to Tello for camera observation; no motor commands.
For offline checks use --self-test.
"""
from pathlib import Path
from runpy import run_path

if __name__ == "__main__":
    run_path(str(Path(__file__).resolve().parent / "calibration" / "calibrate_camera_axes_marker23.py"), run_name="__main__")
