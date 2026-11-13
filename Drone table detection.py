from djitellopy import Tello
import time
import random
import json
import threading
from datetime import datetime, timezone

from paho.mqtt import client as mqtt_module


# ---------- MQTT configuration ----------
MQTT_BROKER = "192.168.1.96"
MQTT_PORT = 1883
 
MQTT_EVENT_TOPIC = "restaurant/tello/events/customer-detected"
MQTT_COMMAND_TOPIC = "restaurant/tello/commands/start-mission"

MQTT_CLIENT_ID = "tello-" + str(random.randint(0, 1000))

message_id = 1
mqtt_connected = False
mission_requested = False
mission_running = False


mqtt_client = mqtt_module.Client(
    mqtt_module.CallbackAPIVersion.VERSION2,
    client_id=MQTT_CLIENT_ID,
    protocol=mqtt_module.MQTTv311,
)


def on_mqtt_connect(client, userdata, flags, reason_code, properties):
    global mqtt_connected

    if not reason_code.is_failure:
        mqtt_connected = True
        print("Connected to MQTT broker")

        client.subscribe(MQTT_COMMAND_TOPIC, qos=1)
        print(f"Subscribed to {MQTT_COMMAND_TOPIC}")
    else:
        print("MQTT connection failed: " + str(reason_code))


def on_mqtt_message(client, userdata, msg):
    global mission_requested

    try:
        payload = json.loads(msg.payload.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError):
        print("Invalid MQTT command payload received")
        return

    command = payload.get("command")

    if command == "start_mission":
        if mission_running:
            print("Mission already running; ignoring start command")
            return

        print("Start command received")
        mission_requested = True
    else:
        print(f"Ignored MQTT command: {command}")


mqtt_client.on_connect = on_mqtt_connect
mqtt_client.on_message = on_mqtt_message

mqtt_client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
mqtt_client.loop_start()


def publish(event_type, table_id=None, extra_data=None):
    """Publish an event using the required JSON schema."""
    global message_id

    data = {}

    if table_id is not None:
        data["table_id"] = table_id

    if extra_data:
        data.update(extra_data)

    payload = {
        "message_id": message_id,
        "event_type": event_type,
        "timestamp": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "source": "tello",
        "data": data
    }

    message = json.dumps(payload)

    result = mqtt_client.publish(MQTT_EVENT_TOPIC, message, qos=1)

    if result.rc != mqtt_module.MQTT_ERR_SUCCESS:
        print("MQTT publish failed: " + str(result.rc))

    print(message)

    message_id += 1


# ---------- Tello mission ----------
def run_tello_mission():
    global mission_running

    mission_running = True

    TELLO_IP = "192.168.1.63"

    tello = None

    try:
        tello = Tello(host=TELLO_IP)
        tello.connect()

        publish(
            "drone-connected",
            extra_data={"battery": tello.get_battery()}
        )

        tello.enable_mission_pads()
        tello.set_mission_pad_detection_direction(2)

        target_found = False
        last_printed_pad = None
        pad = None

        search_direction = "right"
        search_distance = 50
        max_search_steps = 10
        search_step = 0

        tello.takeoff()
        time.sleep(2)

        while not target_found and search_step < max_search_steps:
            pad = tello.get_mission_pad_id()

            if pad != last_printed_pad:
                publish("mission-pad-detected", table_id=pad)
                last_printed_pad = pad

            if pad == 1 or pad == 3:
                publish("customer-detected", table_id=pad)
                target_found = True

            else:
                if search_direction == "right":
                    tello.move_right(search_distance)
                    search_direction = "left"
                    tello.move_left(search_distance)
                else:
                    tello.move_left(search_distance)
                    search_direction = "right"
                    tello.move_right(search_distance)

                search_step += 1
                time.sleep(0.5)

        if target_found:
            publish("landing-at-table", table_id=pad)
        else:
            publish("no-table-found")

        tello.move_back(150)
        tello.land()

    except Exception as e:
        publish("error", extra_data={"message": str(e)})

        if tello is not None:
            try:
                tello.move_back(200)
                tello.land()
            except Exception:
                pass

    finally:
        if tello is not None:
            try:
                tello.disable_mission_pads()
            except Exception:
                pass

            try:
                tello.end()
            except Exception:
                pass

        mission_running = False


# ---------- Main loop ----------
try:
    print(f"Waiting for MQTT command on {MQTT_COMMAND_TOPIC}...")

    while True:
        if mission_requested and not mission_running:
            mission_requested = False
            run_tello_mission()

        time.sleep(0.5)

except KeyboardInterrupt:
    print("Stopping Tello MQTT controller")

finally:
    mqtt_client.loop_stop()
    mqtt_client.disconnect()