from djitellopy import Tello
import time
import random


from paho.mqtt import client as mqtt_module


# ---------- MQTT configuration ----------
MQTT_BROKER = "192.168.1.96"       # Your MQTT broker IP
MQTT_PORT = 1883
MQTT_TOPIC = "tello/table_detection"
MQTT_CLIENT_ID = "tello-" + str(random.randint(0, 1000))

mqtt_connected = False

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
    else:
        print("MQTT connection failed: " + str(reason_code))


mqtt_client.on_connect = on_mqtt_connect
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
mqtt_client.loop_start()


def publish(message):
    """Publish a message to MQTT and also print it locally."""
    result = mqtt_client.publish(MQTT_TOPIC, message)

    if result.rc != mqtt_module.MQTT_ERR_SUCCESS:
        print("MQTT publish failed: " + str(result.rc))

    print(message)

# ---------- Tello setup ----------
TELLO_IP = "192.168.1.63"  # REPLACE with the Tello's IP from your ASUS router


tello = Tello(host=TELLO_IP)
tello.connect()



#publish("Battery: " + str(tello.get_battery()) + "%")



# Enable mission-pad detection before takeoff
tello.enable_mission_pads()


# 0 = downward camera only
# 1 = forward camera only
# 2 = both downward and forward cameras
tello.set_mission_pad_detection_direction(2)



# These variables MUST be defined before the while loop
target_found = False
last_printed_pad = None



search_direction = "right"
search_distance = 50  # cm
max_search_steps = 10  # safety limit
search_step = 0
#tello.move_up(100)


try:
    tello.takeoff()
    time.sleep(2)



    while not target_found and search_step < max_search_steps:
        pad = tello.get_mission_pad_id()



        if pad != last_printed_pad:
            publish("Mission pad ID: " + str(pad))
            last_printed_pad = pad



        if pad == 1:
            publish("Table 1 found")
            target_found = True



        elif pad == 3:
            publish("Table 2 found")
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
        publish("Landing on found table")
    else:
        publish("No table found; landing")



    tello.move_back(200)
    tello.land()



except Exception as e:
    publish("Error: " + str(e))



    try:
        tello.move_back(200)
        tello.land()
    except Exception:
        pass



finally:
    try:
        tello.disable_mission_pads()
    except Exception:
        pass



    tello.end()



    mqtt_client.loop_stop()
    mqtt_client.disconnect()