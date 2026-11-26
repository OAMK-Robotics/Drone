from djitellopy import Tello
import time
import random
import json
import threading
import traceback
from datetime import datetime, timezone

from paho.mqtt import client as mqtt_module


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
MQTT_EVENT_TOPIC = os.environ["EVENT_TOPIC"]
MQTT_CLIENT_ID = os.environ["DETECTION_CLIENT_ID"]

message_id = 1
mqtt_connected = False
mission_requested = False
mission_running = False

mission_lock = threading.Lock()


def log(message):
    print(message, flush=True)


mqtt_client = mqtt_module.Client(
    mqtt_module.CallbackAPIVersion.VERSION2,
    client_id=MQTT_CLIENT_ID,
    protocol=mqtt_module.MQTTv311,
)

mqtt_client.username_pw_set(
    username=MQTT_USERNAME,
    password=MQTT_PASSWORD,
)


def on_mqtt_connect(client, userdata, flags, reason_code, properties):
    global mqtt_connected

    if not reason_code.is_failure:
        mqtt_connected = True
        log("Connected to MQTT broker")

        result, subscription_id = client.subscribe(
            MQTT_COMMAND_TOPIC,
            qos=1,
        )

        if result == mqtt_module.MQTT_ERR_SUCCESS:
            log(f"Subscription requested: {MQTT_COMMAND_TOPIC}")
        else:
            log(f"MQTT subscription failed: {result}")
    else:
        mqtt_connected = False
        log(f"MQTT connection failed: {reason_code}")


def on_mqtt_disconnect(
    client,
    userdata,
    disconnect_flags,
    reason_code,
    properties,
):
    global mqtt_connected

    mqtt_connected = False
    log(f"Disconnected from MQTT broker: {reason_code}")


def on_mqtt_message(client, userdata, msg):
    global mission_requested

    try:
        payload = json.loads(msg.payload.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError):
        log("Invalid MQTT command payload received")
        return

    if not isinstance(payload, dict):
        log("Invalid MQTT command: expected a JSON object")
        return

    command = payload.get("command")

    if command == "start_mission":
        with mission_lock:
            if mission_running:
                log("Mission already running; ignoring start command")
                return

            if mission_requested:
                log("Mission already requested; ignoring duplicate command")
                return

            mission_requested = True

        log("Start command received")
    else:
        log(f"Ignored MQTT command: {command}")


mqtt_client.on_connect = on_mqtt_connect
mqtt_client.on_disconnect = on_mqtt_disconnect
mqtt_client.on_message = on_mqtt_message


def publish(event_type, table_id=None, extra_data=None):
    """Publish an event without allowing MQTT errors to interrupt flight."""
    global message_id

    # Ignore events with invalid table IDs.
    # Events without a table ID are still allowed.
    if table_id is not None:
        if not isinstance(table_id, int) or not 1 <= table_id <= 8:
            return

    try:
        data = {}

        if table_id is not None:
            # Publish mission pad 3 as table 2.
            data["table_id"] = 2 if table_id == 3 else table_id

        if extra_data:
            data.update(extra_data)

        # Keep your existing MQTT payload format.
        payload = {
            "data": data
        }

        message = json.dumps(payload)
        message_id += 1

        log(message)

        if not mqtt_connected:
            log("Warning: MQTT is not connected; event delivery is uncertain")

        result = mqtt_client.publish(
            MQTT_EVENT_TOPIC,
            message,
            qos=1,
        )

        if result.rc != mqtt_module.MQTT_ERR_SUCCESS:
            log(f"MQTT publish failed: {result.rc}")

    except Exception:
        log(f"Failed to publish event: {event_type}")
        traceback.print_exc()


# ---------- Tello mission ----------
def run_tello_mission():
    global mission_running

    with mission_lock:
        mission_running = True

    TELLO_IP = "192.168.1.63"

    tello = None
    stage = "initializing"
    takeoff_attempted = False
    landing_confirmed = False

    try:
        stage = "creating Tello connection"
        log(f"[CHECK] Connecting to Tello at {TELLO_IP}")

        tello = Tello(host=TELLO_IP)
        tello.connect()
        tello.set_speed(10)  # cm/s

        log("[CHECK] SDK connection and initial telemetry received")

        stage = "enabling mission pads"
        log("[CHECK] Enabling mission pads")
        tello.enable_mission_pads()

        stage = "setting mission-pad detection direction"
        tello.set_mission_pad_detection_direction(2)
        log("[CHECK] Mission-pad detection configured")

        target_found = False
        last_printed_pad = None
        pad = None

        max_search_steps = 5
        search_step = 0

        stage = "taking off"
        log("[CHECK] Sending takeoff command")

        # Set before the command because a timeout does not prove
        # that the drone stayed on the ground.
        takeoff_attempted = True
        tello.takeoff()

        log("[CHECK] Takeoff command completed")
        time.sleep(2)
        
        stage = "moving up 70 cm"
        log("[CHECK] Moving up 70 cm")
        tello.move_up(70)

        while not target_found and search_step < max_search_steps:
            stage = f"reading mission pad at search step {search_step}"
            pad = tello.get_mission_pad_id()

            # Only log valid mission-pad IDs.
            valid_pad = isinstance(pad, int) and 1 <= pad <= 8

            if valid_pad:
                log(
                    f"[CHECK] Search step {search_step}/{max_search_steps}, "
                    f"pad={pad}"
                )

                # Do not publish a table during pad detection.
                if pad != last_printed_pad:
                    last_printed_pad = pad
            else:
                # Do not print or publish invalid values.
                last_printed_pad = None

            if pad == 1 or pad == 3:
                # The only table publication in the mission.
                publish("customer-detected", table_id=pad)
                target_found = True

            else:
                stage = "moving forward 300 cm"
                log("[CHECK] Moving forward 300 cm")
                tello.move_forward(300)
                
                tello.rotate_clockwise(180)
                            
                time.sleep(1)
            search_step += 1
            time.sleep(0.5)

        if target_found:
            # Log locally instead of publishing the table again.
            table_id = 2 if pad == 3 else pad
            log(
                f"[CHECK] Table {table_id} found; "
                "returning and landing"
            )
        else:
            publish("no-table-found")

        stage = "landing"
        log("[CHECK] Sending landing command")
        tello.land()
        landing_confirmed = True

        log("[CHECK] Landing command completed")

    except BaseException as error:
        # Includes Ctrl+C so an interrupted flight still gets
        # a best-effort landing attempt.
        log(f"[ERROR] Mission failed during: {stage}")
        log(f"[ERROR] {type(error).__name__}: {error}")
        traceback.print_exc()

        # Attempt landing before any further MQTT activity.
        if (
            tello is not None
            and takeoff_attempted
            and not landing_confirmed
        ):
            try:
                log("[RECOVERY] Attempting landing without extra movement")
                tello.land()
                landing_confirmed = True
                log("[RECOVERY] Landing command completed")
            except Exception:
                log("[RECOVERY] Landing command failed")
                traceback.print_exc()
        else:
            log("[RECOVERY] No takeoff attempted; skipping landing command")

        publish(
            "error",
            extra_data={
                "message": str(error),
                "error_type": type(error).__name__,
                "stage": stage,
            },
        )

        if not isinstance(error, Exception):
            raise

    finally:
        try:
            if tello is not None:
                try:
                    log("[CLEANUP] Disabling mission pads")
                    tello.disable_mission_pads()
                except Exception as error:
                    log(f"[CLEANUP] Could not disable mission pads: {error}")

                try:
                    log("[CLEANUP] Closing Tello connection")
                    tello.end()
                except Exception:
                    log("[CLEANUP] Could not close Tello connection cleanly")
                    traceback.print_exc()
        finally:
            with mission_lock:
                mission_running = False

            log("[CHECK] Mission finished")


# ---------- Main loop ----------
try:
    log(f"Connecting to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    mqtt_client.loop_start()

    log(f"Waiting for MQTT command on {MQTT_COMMAND_TOPIC}...")

    while True:
        with mission_lock:
            start_now = mission_requested and not mission_running

            if start_now:
                mission_requested = False
                mission_running = True

        if start_now:
            run_tello_mission()

        time.sleep(0.5)

except KeyboardInterrupt:
    log("Stopping Tello MQTT controller")

finally:
    mqtt_client.disconnect()
    mqtt_client.loop_stop()