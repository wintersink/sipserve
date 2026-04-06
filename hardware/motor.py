import pigpio

class Motor:

    def __init__(self, pi, pwm_pin, dir_pin):
        self.pi = pi
        self.pwm = pwm_pin
        self.dir = dir_pin

        pi.set_mode(self.pwm, pigpio.OUTPUT)
        pi.set_mode(self.dir, pigpio.OUTPUT)

        pi.set_PWM_frequency(self.pwm, 1000)

    def set_speed(self, speed):
        """
        speed range: -1.0 to 1.0
        """

        if speed >= 0:
            self.pi.write(self.dir, 1)
        else:
            self.pi.write(self.dir, 0)

        duty = int(abs(speed) * 255)
        self.pi.set_PWM_dutycycle(self.pwm, duty)

    def stop(self):
        self.pi.set_PWM_dutycycle(self.pwm, 0)