import time
import math
from class_BBRobot import BBrobot
from class_Camera import PiCamera3Tracker
from class_PID import PID

# Initialize camera
camera = PiCamera3Tracker()
camera.set_up()

# Initialize robot
motor_pins = [[17, 18], [22, 23], [24, 25]]  # Example GPIO pin pairs for step/direction
robot = BBrobot(motor_pins)
robot.set_up()

# Initialize PID controllers for X and Y axes
pid_x = PID(Kp=0.25, Ki=0.01, Kd=0.005, output_limits=(-10, 10))
pid_y = PID(Kp=0.25, Ki=0.01, Kd=0.005, output_limits=(-10, 10))

try:
    robot.Initialize_posture()
    time.sleep(1)

    while True:
        # Get ball position
        ball_x, ball_y = camera.get_position()

        # Get PID outputs
        control_x = pid_x.compute(ball_x)
        control_y = pid_y.compute(ball_y)

        # Convert to theta and phi for posture control
        theta = math.degrees(math.atan2(control_y, control_x))
        phi = min(math.sqrt(control_x**2 + control_y**2), 20)

        # Command robot
        robot.control_t_posture([theta, phi, 0.0632], t=0.1)

except KeyboardInterrupt:
    print("Shutting down...")

finally:
    robot.clean_up()
    camera.clean_up()
