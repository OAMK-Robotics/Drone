import ast
import unittest
from pathlib import Path
from unittest.mock import MagicMock, call


class TestFlightSafety(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Do not import the flight script: importing it connects to Tello.
        source = Path("vision_navigation.py").read_text(
            encoding="utf-8-sig"
        )
        tree = ast.parse(source)

        matches = [
            node for node in tree.body
            if isinstance(node, ast.Try)
            and any(
                isinstance(x, ast.Call)
                and isinstance(x.func, ast.Attribute)
                and x.func.attr == "takeoff"
                for x in ast.walk(node)
            )
        ]

        assert len(matches) == 1, "Flight structure changed"

        module = ast.fix_missing_locations(
            ast.Module(body=[matches[0]], type_ignores=[])
        )
        cls.program = compile(module, "<offline-flight>", "exec")

    def scenario(self, mode, expected_land):
        drone = MagicMock()
        drone.get_mission_pad_id.return_value = -1

        if mode == "interrupt":
            drone.get_mission_pad_id.side_effect = KeyboardInterrupt()

        elif mode == "navigation_error":
            drone.get_mission_pad_id.side_effect = RuntimeError(
                "Simulated navigation error"
            )

        elif mode == "takeoff_interrupt":
            drone.takeoff.side_effect = KeyboardInterrupt()

        elif mode == "movement_error":
            drone.move_right.side_effect = RuntimeError(
                "Simulated movement failure"
            )

        elif mode == "landing_failure":
            drone.get_mission_pad_id.side_effect = KeyboardInterrupt()
            drone.land.side_effect = RuntimeError(
                "Landing unavailable"
            )

        ready = MagicMock()
        ready.wait.return_value = mode != "preflight_failure"

        failed = MagicMock()
        failed.is_set.return_value = False

        recorder = MagicMock()

        context = {
            "tello": drone,
            "recording_ready": ready,
            "recording_failed": failed,
            "recorder": recorder,
            "takeoff_attempted": False,
            "landing_command_returned": False,
            "keep_recording": True,
            "net_displacement": 0,
            "target_found": False,
            "time": MagicMock(),
            "publish_table_event": MagicMock(),
        }

        exec(self.program, context)

        self.assertEqual(drone.land.call_count, expected_land)
        recorder.join.assert_called_once_with(timeout=5)
        drone.end.assert_called_once()
        drone.streamoff.assert_called_once()
        drone.disable_mission_pads.assert_called_once()

        # If a landing command was issued, it must precede cleanup.
        if expected_land:
            calls = drone.mock_calls
            self.assertLess(
                calls.index(call.land()),
                calls.index(call.streamoff())
            )
            self.assertLess(
                calls.index(call.land()),
                calls.index(call.end())
            )

        if mode == "preflight_failure":
            drone.takeoff.assert_not_called()
            self.assertFalse(context["takeoff_attempted"])

        if mode == "landing_failure":
            self.assertFalse(context["landing_command_returned"])

    def test_normal_mission(self):
        self.scenario("normal", 1)

    def test_ctrl_c_during_flight(self):
        self.scenario("interrupt", 1)

    def test_navigation_error(self):
        self.scenario("navigation_error", 1)

    def test_ctrl_c_during_takeoff(self):
        self.scenario("takeoff_interrupt", 1)

    def test_preflight_failure(self):
        self.scenario("preflight_failure", 0)

    def test_landing_failure_cleanup(self):
        self.scenario("landing_failure", 1)

    def test_movement_failure(self):
        self.scenario("movement_error", 1)


if __name__ == "__main__":
    unittest.main()