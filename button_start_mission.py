import json
import time
from datetime import datetime, timezone

import paho.mqtt.client as mqtt
from easygopigo3 import EasyGoPiGo3

# ---------- MQTT configuration ----------
MQTT_BROKER = "192.168.1.96"
MQTT_PORT = 1883

MQTT_COMMAND_TOPIC = "restaurant/tello/commands/start-mission"
MQTT_CLIENT_ID = "raspi-start-button-" + str(int(time.time()))

BUTTON_PORT = "AD1"

# ---------- MQTT ----------
mqtt_client = mqtt.Client(
    mqtt.CallbackAPIVersion.VERSION2,
    client_id=MQTT_CLIENT_ID,
    protocol=mqtt.MQTTv311,
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