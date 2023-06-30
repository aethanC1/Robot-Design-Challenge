import serial
import rclpy
from rclpy.node import Node
import time

nano = serial.Serial('/dev/ttyUSB3', 9600, timeout = 1)
time.sleep(1)
nano.readline()
for i in range(8):
    nano.write(b'humidity')
    time.sleep(0.12)
    byte_string = nano.readline()
    reply = str(byte_string).strip("b'\\rn")

    print(reply)

nano.close()


