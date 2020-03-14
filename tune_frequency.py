import serial 
import numpy as np 

read_data = []
pwm = []

serialArduino = serial.Serial('/dev/ttyACM0',2000000)

for i in range(50,260,5):
    pwm.append(i)

print(pwm)