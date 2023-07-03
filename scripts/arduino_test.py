import serial
import time

nano = serial.Serial('/dev/ttyUSB0', 9600, timeout = 1)
time.sleep(1)
if nano.in_waiting:
    nano.readline()
for i in range(8):
    nano.write(b'ground')
    timeout = 0
    while not nano.in_waiting and timeout < 25:
        timeout += 1
        time.sleep(0.01)
    byte_string = nano.readline().decode('utf-8')
    reply = byte_string.strip("\\rn")

    print(reply)

nano.close()