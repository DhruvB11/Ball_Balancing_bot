import RPi.GPIO as GPIO
import time
import math

class StepperMotor:
    def __init__(self, step_pin, dir_pin, steps_per_rev=200):
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.steps_per_rev = steps_per_rev
        self.current_angle = 0.0
        self.angle_per_step = 360.0 / steps_per_rev

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)

    def rotate_to(self, target_angle, delay=0.001):
        # Calculate shortest rotation
        delta_angle = target_angle - self.current_angle
        delta_angle = (delta_angle + 180) % 360 - 180  # Normalize [-180, 180)

        steps = int(abs(delta_angle) / self.angle_per_step)
        direction = delta_angle > 0

        GPIO.output(self.dir_pin, direction)

        for _ in range(steps):
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(delay)

        self.current_angle = (self.current_angle + delta_angle) % 360

    def cleanup(self):
        GPIO.cleanup([self.step_pin, self.dir_pin])
