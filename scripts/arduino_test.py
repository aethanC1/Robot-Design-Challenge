import serial
import time

nano = serial.Serial('COM3', 9600, timeout = 1)
time.sleep(1)
if nano.in_waiting:
    nano.readline()
for i in range(8):
    nano.write(b'ground')
    timeout = 0
    while not nano.in_waiting and timeout < 25:
        timeout += 1
        time.sleep(0.01)
    reply = nano.readline().decode('utf-8').rstrip()

    if reply == 'wood':
        print(reply)

nano.close()