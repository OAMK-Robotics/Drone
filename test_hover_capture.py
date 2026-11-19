"""Backwards-compatible entry point for the guarded camera-only Tello hover.

CAUTION: Executing this script can command a REAL Tello takeoff after FLY.
Original implementation: flight/test_hover_capture.py.
"""
if __name__ == "__main__":
    from runpy import run_module
    run_module("flight.test_hover_capture", run_name="__main__")
