from sensors import sonar_measurements as sonar


def main():
    trig_pins, echo_pins, units = sonar.setup()     # TODO: this should be permanent

    # get measurements ================================================
    distances = sonar.get_sonar_readings(trig_pins, echo_pins, units)
    encoders = [0, 0]       # TODO: need to be able to get encoder readings



    y = []




if __name__ == '__main__':
    main()
