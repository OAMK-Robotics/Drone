import json
import time
from datetime import datetime, timezone

import paho.mqtt.client as mqtt
from easygopigo3 import EasyGoPiGo3

# ---------- MQTT configuration ----------
import os
from pathlib import Path
from dotenv import load_dotenv
from paho.mqtt import client as mqtt_module

load_dotenv(Path(__file__).resolve().parent / "mqtt_config.env")

MQTT_BROKER = os.environ["BROKER"]
MQTT_PORT = int(os.environ["PORT"])
MQTT_USERNAME = os.environ["USERNAME"]
MQTT_PASSWORD = os.environ["PASSWD"]

MQTT_COMMAND_TOPIC = os.environ["TOPIC"]
MQTT_CLIENT_ID = os.environ["BUTTON_CLIENT_ID"]

BUTTON_PORT = "AD1"

# ---------- MQTT ----------
mqtt_client = mqtt_module.Client(
    mqtt_module.CallbackAPIVersion.VERSION2,
    client_id=MQTT_CLIENT_ID,
    protocol=mqtt_module.MQTTv311,
)

mqtt_client.username_pw_set(
    username=MQTT_USERNAME,
    password=MQTT_PASSWORD,
)

mqtt_connected = False


def on_connect(client, userdata, flags, reason_code, properties):
    global mqtt_connected

    if not reason_code.is_failure:
        mqtt_connected = True
        print("Connected to MQTT broker")
    else:
        print("MQTT connection failed:", reason_code)


def on_disconnect(client, userdata, disconnect_flags, reason_code, properties):
    global mqtt_connected
    mqtt_connected = False
    print("Disconnected from MQTT broker")


mqtt_client.on_connect = on_connect
mqtt_client.on_disconnect = on_disconnect

mqtt_client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
mqtt_client.loop_start()


def publish_start_mission():
    payload = {
        "command": "start_mission",
        "timestamp": datetime.now(timezone.utc)
        .isoformat()
        .replace("+00:00", "Z"),
        "source": "raspi-gopigo-button",
    }

    result = mqtt_client.publish(
        MQTT_COMMAND_TOPIC,
        json.dumps(payload),
        qos=1,
    )

    if result.rc == mqtt.MQTT_ERR_SUCCESS:
        print("Start mission command published")
    else:
        print("MQTT publish failed:", result.rc)


# ---------- GoPiGo button ----------
try:
    gpg = EasyGoPiGo3()
    button = gpg.init_button_sensor(BUTTON_PORT)

    print(f"Waiting for button press on GoPiGo3 port {BUTTON_PORT}...")

    was_pressed = False
    last_press_time = 0

    while True:
        is_pressed = button.is_button_pressed()
        now = time.monotonic()

        # Detect a new press, with basic debounce
        if is_pressed and not was_pressed and now - last_press_time > 0.5:
            last_press_time = now

            if mqtt_connected:
                publish_start_mission()
            else:
                print("MQTT not connected; cannot send command")

        was_pressed = is_pressed
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Stopping GoPiGo3 start button controller")

finally:
    mqtt_client.loop_stop()
    mqtt_client.disconnect()