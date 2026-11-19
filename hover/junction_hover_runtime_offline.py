"""Motor-disabled Marker 23 hover runtime using the EXISTING hover controller.

Run from the Drone repository root:
    python -B junction_hover_runtime_offline.py --self-test
    python -B junction_hover_runtime_offline.py --demo

No DJITelloPy, drone connection, RC commands or physical motor mapping.
The demo uses a deliberately IDEAL, SYNTHETIC image-space response; its
success is NOT evidence that a physical drone will move the same way.
"""

import argparse
import math

from .junction_hover_controller import JunctionHoverController

TARGET = (160.0, 120.0)


def synthetic_marker(x, y):
    """A synthetic Marker 23 observation for test/demo only."""
    return {
        "id": 23,
        "center": (float(x), float(y)),
        "corners": [[0.0, 0.0], [16.0, 0.0], [16.0, 16.0], [0.0, 16.0]],
    }


class OfflineHoverRuntime:
    """Gate one image-space request on verification of its observed feedback.

    observe() returns (status, image_dx, image_dy). The returned numbers are
    NOT DJI RC arguments and MUST NOT be forwarded to drone motor commands.
    accept_feedback() is called only after a distinct, externally observed
    result of an explicitly modeled image-space correction.

    Saved hover images were taken WITHOUT such corrections; DO NOT call
    accept_feedback() on successive saved hover photos as if they were paired
    before/after actuation measurements.
    """

    def __init__(self, controller=None, max_requests=6):
        if not isinstance(max_requests, int) or isinstance(max_requests, bool) or max_requests < 1:
            raise ValueError("max_requests must be a positive integer")
        self.controller = controller if controller is not None else JunctionHoverController(target_center=TARGET)
        self.max_requests = max_requests
        self.requests_made = 0
        self.pending_before = None

    @staticmethod
    def _marker23(marker):
        return isinstance(marker, dict) and marker.get("id") == 23

    def stop(self):
        """Latch the offline runtime. No physical landing/control is implied."""
        self.pending_before = None
        self.controller.latched = True
        return "RUNTIME_STOP_LATCHED_NEUTRAL", 0.0, 0.0

    def observe(self, marker, phase="HOVER"):
        if self.controller.latched:
            return "SAFETY_LATCHED_NEUTRAL", 0.0, 0.0
        # This runtime intentionally permits requests only during hover.
        # RECOVERY may produce image-only requests in the legacy controller;
        # we do NOT expose them as actuation candidates here.
        if phase != "HOVER":
            if self.pending_before is not None:
                return self.stop()
            return "NON_HOVER_OBSERVE_NEUTRAL", 0.0, 0.0
        if self.pending_before is not None:
            # Must supply explicit correction feedback before requesting again.
            # Unexpected marker loss while waiting is fail-closed.
            if marker is None or not self._marker23(marker):
                return self.stop()
            return "FEEDBACK_PENDING_NEUTRAL", 0.0, 0.0
        observation = marker if self._marker23(marker) else None
        status, dx, dy = self.controller.update(observation, phase="HOVER")
        if status != "IMAGE_REQUEST_ONLY":
            # Never export nonzero requests in any other state.
            return status, 0.0, 0.0
        if self.requests_made >= self.max_requests:
            self.controller.latched = True
            return "REQUEST_BUDGET_LATCHED_NEUTRAL", 0.0, 0.0
        if (not all(math.isfinite(v) for v in (dx, dy))
                or not all(abs(v) <= 4.0 for v in (dx, dy))
                or (dx == 0 and dy == 0)):
            return self.stop()
        # Snapshot only the center needed for verification; prevent caller
        # mutation of an observation from altering our pending baseline.
        self.pending_before = {"center": tuple(float(v) for v in observation["center"])}
        self.requests_made += 1
        return status, dx, dy

    def accept_feedback(self, marker_after):
        """Verify a modeled request result; never sends an actual movement."""
        if self.controller.latched:
            self.pending_before = None
            return "SAFETY_LATCHED_NEUTRAL"
        if self.pending_before is None:
            self.controller.latched = True
            return "UNEXPECTED_FEEDBACK_LATCHED_NEUTRAL"
        before = self.pending_before
        self.pending_before = None
        after = marker_after if self._marker23(marker_after) else None
        return self.controller.verify_correction(before, after)


def acquired_runtime(max_requests=6):
    runtime = OfflineHoverRuntime(max_requests=max_requests)
    for expected in ("ACQUIRING_NEUTRAL", "ACQUIRING_NEUTRAL", "REFERENCE_SET_NEUTRAL"):
        result = runtime.observe(synthetic_marker(*TARGET))
        assert result == (expected, 0.0, 0.0), result
    assert runtime.controller.reference == TARGET
    return runtime


def simulate(gain=1.0, max_requests=6, verbose=False):
    """IDEALIZED synthetic image response: after = before + gain * request."""
    r = acquired_runtime(max_requests)
    marker = synthetic_marker(176, 136)
    for iteration in range(12):
        state, dx, dy = r.observe(marker)
        if verbose:
            print(f"step={iteration} center={marker['center']} state={state} request_px=({dx},{dy})")
        if state == "HOLD_NEUTRAL":
            assert not r.controller.latched and r.pending_before is None
            return "HOLD_NEUTRAL", r.requests_made
        if state != "IMAGE_REQUEST_ONLY":
            return state, r.requests_made
        after = synthetic_marker(marker["center"][0] + gain * dx,
                                 marker["center"][1] + gain * dy)
        verdict = r.accept_feedback(after)
        if verbose:
            print(f"  synthetic_result={after['center']} feedback={verdict}")
        if r.controller.latched:
            return verdict, r.requests_made
        marker = after
    raise AssertionError("Synthetic scenario exceeded the step limit")


def self_test():
    outcome, count = simulate(1.0)
    assert outcome == "HOLD_NEUTRAL" and count == 3, (outcome, count)
    print("PASS: synthetic 2D response reaches neutral hold in 3 requests")

    for gain, label in [(-1.0, "wrong-way"), (0.0, "no-response")]:
        verdict, count = simulate(gain)
        assert verdict == "CORRECTION_NOT_IMPROVED_LATCHED" and count == 1
        print(f"PASS: synthetic {label} response latches")

    r = acquired_runtime()
    assert r.observe(synthetic_marker(176, 136))[0] == "IMAGE_REQUEST_ONLY"
    assert r.observe(synthetic_marker(176, 136)) == ("FEEDBACK_PENDING_NEUTRAL", 0.0, 0.0)
    assert r.requests_made == 1
    assert r.accept_feedback(synthetic_marker(172, 132)) == "CORRECTION_IMPROVED"
    assert r.pending_before is None
    print("PASS: no second request before explicit verified feedback")

    r = acquired_runtime()
    assert r.observe(synthetic_marker(176, 136))[0] == "IMAGE_REQUEST_ONLY"
    assert r.accept_feedback(None) == "CORRECTION_UNVERIFIED_LATCHED"
    assert r.observe(synthetic_marker(172, 132)) == ("SAFETY_LATCHED_NEUTRAL", 0.0, 0.0)
    print("PASS: marker loss during feedback blocks further requests")

    r = acquired_runtime(max_requests=1)
    _, dx, dy = r.observe(synthetic_marker(176, 136))
    assert r.accept_feedback(synthetic_marker(176+dx, 136+dy)) == "CORRECTION_IMPROVED"
    assert r.observe(synthetic_marker(176+dx, 136+dy)) == ("REQUEST_BUDGET_LATCHED_NEUTRAL", 0.0, 0.0)
    print("PASS: correction-request budget enforced")

    r = OfflineHoverRuntime()
    assert r.observe(synthetic_marker(176, 136), phase="RECOVERY") == ("NON_HOVER_OBSERVE_NEUTRAL", 0.0, 0.0)
    assert r.controller.reference is None
    print("PASS: no correction requests outside HOVER phase")

    r = acquired_runtime()
    assert r.observe(synthetic_marker(162, 123)) == ("HOLD_NEUTRAL", 0.0, 0.0)
    assert r.requests_made == 0
    print("PASS: centered marker does not request a correction")

    r = acquired_runtime()
    assert r.observe(synthetic_marker(176, 136))[0] == "IMAGE_REQUEST_ONLY"
    assert r.observe(None) == ("RUNTIME_STOP_LATCHED_NEUTRAL", 0.0, 0.0)
    print("PASS: unexpected loss while awaiting feedback fails closed")
    print("PASS: 9 offline runtime checks; no drone communication or motor commands")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--self-test", action="store_true")
    mode.add_argument("--demo", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
    else:
        result, count = simulate(verbose=True)
        print(f"SYNTHETIC OUTCOME={result}, requests={count}")
    print("OFFLINE ONLY. IMAGE-SPACE REQUESTS ARE NOT DJI RC COMMANDS.")


if __name__ == "__main__":
    main()
