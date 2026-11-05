"""
Configure a DJI Tello EDU to connect to an external Wi-Fi router.


Before running:
1. Turn on the Tello EDU.
2. Connect your computer to the drone's Wi-Fi network (usually TELLO-XXXXXX).
3. Use a 2.4 GHz router network; the Tello does not support 5 GHz.
"""


import socket
import sys
import time


TELLO_IP = "192.168.10.1"
TELLO_PORT = 8889
TIMEOUT = 5  # seconds


def send_command(sock: socket.socket, command: str) -> str:
    """Send one SDK command and return the drone's response."""
    sock.sendto(command.encode("utf-8"), (TELLO_IP, TELLO_PORT))


    try:
        response, address = sock.recvfrom(1024)
        return response.decode("utf-8", errors="replace").strip()
    except socket.timeout:
        return "TIMEOUT: no response from Tello"


def is_success(response: str) -> bool:
    """Tello may return either 'ok' or 'OK,drone will reboot in 3s'."""
    return response.lower().startswith("ok")


def connect_tello_to_router(router_ssid: str, router_password: str) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", TELLO_PORT))
    sock.settimeout(TIMEOUT)


    try:
        print("Entering SDK mode...")
        response = send_command(sock, "command")
        print(f"Tello response: {response}")


        if not is_success(response):
            print("Could not enter SDK mode. Check that your computer is connected to the Tello Wi-Fi.")
            sys.exit(1)


        # Optional: verify the current SDK version/firmware.
        print("Checking SDK version...")
        sdk_response = send_command(sock, "sdk?")
        print(f"Tello response: {sdk_response}")


        print(f"Telling Tello to join '{router_ssid}'...")
        response = send_command(sock, f"ap {router_ssid} {router_password}")
        print(f"Tello response: {response}")


        if is_success(response):
            print("\nSuccess: the Tello EDU is rebooting and should join the router shortly.")
            print("Wait about 20-30 seconds, then connect your computer to the same router.")
            print("Find the drone's new IP address in your router's DHCP/client list.")
            print("Use that new IP in future control scripts instead of 192.168.10.1.")
        else:
            print("\nFailed to configure station mode.")
            print(f"Raw Tello response: {response}")
            print("Check the SSID/password, ensure the router is 2.4 GHz, and try again.")


    finally:
        sock.close()



if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage:")
        print('  python tello_connect_router.py "YourRouterName" "YourRouterPassword"')
        sys.exit(1)


    ssid = sys.argv[1]
    password = sys.argv[2]


    connect_tello_to_router(ssid, password)