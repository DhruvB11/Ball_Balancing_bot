import cv2
import numpy as np
from picamera2 import Picamera2
import time

class Camera:
    def __init__(self):
        self.cam = Picamera2()
        config = self.cam.create_video_configuration(main={"format": 'RGB888', "size": (640, 480)})
        self.cam.configure(config)
        self.cam.start()
        time.sleep(1)  # Allow the camera to warm up

    def get_ball_position(self):
        # Capture a frame from the camera
        frame = self.cam.capture_array()

        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        # Define the color range for the ball (e.g., red)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])

        # Create masks for red color (handle wrap-around in hue)
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour (assumed to be the ball)
            largest = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(largest)

            if radius > 5:  # Filter out small noise
                # Normalize x and y from -1 to 1
                x_norm = 2 * (x / 640) - 1
                y_norm = 2 * (y / 480) - 1
                return [x_norm, y_norm]
        
        return [0, 0]  # Default if no ball found

    def close(self):
        self.cam.stop()
