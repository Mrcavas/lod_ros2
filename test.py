import pigpio
import time


class EscDriver:
    def __init__(self, pins):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Could not connect to pigpio daemon! Run 'sudo pigpiod'")
        self.pins = pins
        self.num_motors = len(pins)
        for pin in self.pins:
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.set_servo_pulsewidth(pin, 0)

    def arm(self):
        print("Arming ESCs...")
        self.set_all_pwm(1500)
        time.sleep(2.0)
        print("ESCs Armed.")

    def set_pwm(self, index, pwm_value):
        if 0 <= index < self.num_motors:
            self.pi.set_servo_pulsewidth(self.pins[index], pwm_value)

    def set_all_pwm(self, pwm_value):
        for i in range(self.num_motors):
            self.set_pwm(i, pwm_value)

    def stop(self):
        self.set_all_pwm(0)
        self.pi.stop()

    def __del__(self):
        self.stop()


driver = EscDriver([5, 6, 13, 19, 26])

driver.arm()

while True:
    i, pwm = map(int, input().split(" "))
    if i == -1:
        driver.set_all_pwm(pwm)
    else:
        driver.set_pwm(i, pwm)
