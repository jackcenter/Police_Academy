import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)


TRIG_1 = 18
ECHO_1 = 19


print("Distance Measurement in Progress")

GPIO.setup(TRIG_1, GPIO.OUT)
GPIO.setup(ECHO_1, GPIO.IN)

GPIO.output(TRIG_1, False)
print("Waiting for Sensor to Settle")
time.sleep(2)

GPIO.output(TRIG_1, True)
time.sleep(0.00001)
GPIO.output(TRIG_1, False)

while GPIO.input(ECHO_1) == 0:
    pulse_start = time.time()

while GPIO.input(ECHO_1) == 1:
    pulse_end = time.time()

pulse_duration = pulse_end - pulse_start
distance = pulse_duration * 17150   # centimeters
distance = round(distance, 2)

print("Distance: {} cm".format(distance))

GPIO.cleanup()
