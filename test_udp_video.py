import socket
from djitellopy import Tello

t = Tello(host="192.168.1.63")
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    s.bind(("0.0.0.0", 11111))
    s.settimeout(12)

    t.connect()
    t.set_video_direction(Tello.CAMERA_DOWNWARD)
    t.streamon()

    print("Waiting for video packets...", flush=True)

    try:
        data, address = s.recvfrom(4096)
        print("RECEIVED:", len(data), "bytes from", address)
    except socket.timeout:
        print("TIMEOUT: No video packets received")

finally:
    s.close()
    try:
        t.streamoff()
    except Exception as e:
        print("Stream cleanup warning:", e)
    t.end()
