#!/usr/bin/env python3
import serial

if __name__ == '__main__':
    cmd = "Hello from rpi!"

    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.flush()

    ser.write(cmd.encode('utf-8'))
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()

        print(line)
