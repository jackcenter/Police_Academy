import RPi.GPIO as GPIO
import time


def main():
    print_header()
    run_test()
    return 0


def print_header():
    print("Ultrasonic sensor module")


def run_test():
    print(get_ultrasonic_reading(18, 19, 'in'))


class Ultrasonic:
    def __init__(self, location, trig_pin, echo_pin, units='in'):
        self.position = location
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.units = units

        self.distance = 0.0
        self.k0 = 0.0
        self.k1 = 0.0

    def update(self):
        self.k0 = self.k1
        self.k1 = time.time()

        self.distance = get_ultrasonic_reading(self.trig_pin, self.echo_pin, self.units)

    def get_value(self):
        return self.distance

    def get_distance(self):
        return self.distance


def get_ultrasonic_reading(trigger, echo, units):
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(trigger, GPIO.OUT)
    GPIO.setup(echo, GPIO.IN)
    GPIO.output(trigger, False)

    send_pulse(trigger)
    pulse_duration = get_pulse_duration(echo)
    distance = convert_duration_to_distance(pulse_duration, units)

    GPIO.cleanup()

    return distance


def send_pulse(trig: int, duration=0.00001):
    GPIO.output(trig, True)
    time.sleep(duration)
    GPIO.output(trig, False)


def get_pulse_duration(echo: int):
    time_start = time.time()

    while GPIO.input(echo) == 0:
        pulse_start = time.time()

        if pulse_start - time_start > 0.1:
            return None

    while GPIO.input(echo) == 1:
        pulse_end = time.time()

        if pulse_end - time_start > 0.1:
            return None

    pulse_duration = pulse_end - pulse_start
    return pulse_duration


def convert_duration_to_distance(duration: float, units='in'):
    unit_dict = {'cm': 17150, 'in': 6750}
    unit_conversion = unit_dict.get(units)
    distance = duration * unit_conversion
    return distance

if __name__ == '__main_':
    main()