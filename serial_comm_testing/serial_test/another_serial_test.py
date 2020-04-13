#!/usr/bin/env python3
import serial

if __name__ == '__main__':
    cmd = "Hello from rpi!\n"

    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.flush()

    
    while True:
        ser.write(cmd.encode('utf-8'))
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print(line)
