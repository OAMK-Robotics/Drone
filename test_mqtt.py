import ast
import unittest
from pathlib import Path
from unittest.mock import MagicMock

SOURCE = Path("Drone table detection.py")

class TestMQTT(unittest.TestCase):

    def test_table_messages(self):
        tree = ast.parse(
            SOURCE.read_text(encoding="utf-8-sig")
        )

        function = next(
            node for node in tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "publish_table_event"
        )

        module = ast.Module(body=[function], type_ignores=[])

        fake_mqtt = MagicMock()

        namespace = {
            "mqtt": fake_mqtt,
            "MQTT_BROKER": "192.168.1.96",
            "MQTT_PORT": 1883,
            "MQTT_TOPIC": "restaurant/tello/events/customer-detected"
        }

        exec(compile(module, "<test>", "exec"), namespace)

        for pad, message in [
            (1, "Table 1 found"),
            (3, "Table 2 found")
        ]:
            with self.subTest(pad=pad):
                fake_mqtt.reset_mock()

                namespace["publish_table_event"](pad)

                client = fake_mqtt.Client.return_value

                client.publish.assert_called_once_with(
                    namespace["MQTT_TOPIC"],
                    message,
                    qos=1
                )

                client.connect.assert_called_once_with(
                    namespace["MQTT_BROKER"],
                    namespace["MQTT_PORT"],
                    keepalive=30
                )

                print("PASS: Pad", pad, "->", message)

if __name__ == "__main__":
    unittest.main(verbosity=2)
