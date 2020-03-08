import RPi.GPIO as GPIO
import time


def main():
    trig_pins, echo_pins, units = setup()
    distances = get_sonar_readings(trig_pins, echo_pins, units)
    print("Measurements: {0}, {1}, {2}".format(distances[0], distances[1], distances[2]))
    GPIO.cleanup()
    
    
def setup():
    GPIO.setmode(GPIO.BCM)

    units = 'in'    # 'cm'
    TRIG_1 = 18
    ECHO_1 = 19
    TRIG_2 = 20
    ECHO_2 = 21
    TRIG_3 = 22
    ECHO_3 = 23
    
    TRIG_PINS = [TRIG_1, TRIG_2, TRIG_3]
    ECHO_PINS = [ECHO_1, ECHO_2, ECHO_3]
    
    print("Distance Measurement in Progress")
    
    for trig, echo in zip(TRIG_PINS, ECHO_PINS):
        GPIO.setup(trig, GPIO.OUT)
        GPIO.setup(echo, GPIO.IN)
        GPIO.output(trig, False)
    
    print("Waiting for Sensor to Settle")
    time.sleep(2)
    
    return TRIG_PINS, ECHO_PINS, units


def get_sonar_readings(trig_pins, echo_pins, units):
    distances = list()
    
    for trigger, echo in zip(trig_pins, echo_pins):
        send_pulse(trigger)
        pulse_duration = get_pulse_duration(echo)
        distances.append(convert_duration_to_distance(pulse_duration, units))
        time.sleep(0.03)
    
    return distances
    
    
def send_pulse(trig: int, duration=0.00001):
    GPIO.output(trig, True)
    time.sleep(duration)
    GPIO.output(trig, False)
    

def get_pulse_duration(echo: int):
    while GPIO.input(echo) == 0:
        pulse_start = time.time()

    while GPIO.input(echo) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    return pulse_duration


def convert_duration_to_distance(duration: float, units='in'):
    unit_dict = {'cm': 17150, 'in': 6750}
    unit_conversion = unit_dict.get(units)
    distance = duration * unit_conversion
    return distance
    

if __name__ == '__main__':
    main()