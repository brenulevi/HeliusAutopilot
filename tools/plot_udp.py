import serial
import socket
import json

SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 115200

UDP_IP = "127.0.0.1"
UDP_PORT = 9870

ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    line = ser.readline().decode("utf-8", errors="ignore").strip()

    if not line:
        continue

    try:
        values = line.split()

        if len(values) != 6:
            continue

        packet = {
            "accel": {
                "x": float(values[0]),
                "y": float(values[1]),
                "z": float(values[2]),
            },
            "mag": {
                "x": float(values[3]),
                "y": float(values[4]),
                "z": float(values[5]),
            }
        }

        data = json.dumps(packet).encode("utf-8")

        sock.sendto(data, (UDP_IP, UDP_PORT))

    except ValueError:
        pass