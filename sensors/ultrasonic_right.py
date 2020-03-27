import RPi.GPIO as GPIO
from sensors import sonar_measurements as sonar


def main():
    trig_pin, echo_pin, units = setup()
    distance = sonar.get_sonar_readings(trig_pin, echo_pin, units)
    return distance


def setup():
    units = 'in'  # 'cm'
    trig_pin = 18
    echo_pin = 19

    GPIO.setup(trig_pin, GPIO.OUT)
    GPIO.setup(echo_pin, GPIO.IN)
    GPIO.output(trig_pin, False)

    return trig_pin, echo_pin, units


if __name__ == '__main__':
    main()
