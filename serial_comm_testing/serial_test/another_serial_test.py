#!/usr/bin/env python3
import serial
import time

if __name__ == '__main__':
    cmd = "l\n"
    ser = serial.Serial('COM3', 9600, timeout=1)

    ser.flush()
    waiting = True

    # line = ser.readline().decode('utf-8').rstrip()
    # print(line)

    while True:
        # ser.write(cmd.encode('utf-8'))
        # if ser.in_waiting > 0:
        #     waiting = False
        line = ser.readline().decode('utf-8').rstrip()

        print(line)

