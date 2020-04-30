import serial
import time

arduino_address = '/dev/ttyACM0'
p_cmd = 140.289
r_cmd = 1
cmd_phrase = str(r_cmd) + "," + str(p_cmd) + "\n"

if __name__ == '__main__':
    ser_bus = serial.Serial(arduino_address, 9600, timeout=1)
    ser_bus.flush()
    while True:
        ser_bus.write(cmd_phrase.encode('utf-8'))
        if ser_bus.in_waiting > 0:
            arduino_response = ser_bus.readline().decode('utf-8').rstrip()
            print(arduino_response)
            print(time.time())
        time.sleep(0.01)