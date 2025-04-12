import os
import sys
import argparse
import time
import cv2
import numpy as np
from ultralytics import YOLO
from picamera2 import Picamera2

# ================== ARGUMENT PARSING ==================
parser = argparse.ArgumentParser()
parser.add_argument('--model', required=True, help='Path to YOLO model (e.g., best.pt)')
parser.add_argument('--thresh', default=0.5, type=float, help='Confidence threshold (e.g., 0.5)')
parser.add_argument('--resolution', required=True, help='Resolution in WxH format (e.g., 640x480)')
args = parser.parse_args()

# ================== SETUP ==================
model_path = args.model
min_thresh = args.thresh
resW, resH = map(int, args.resolution.lower().split('x'))

# Validate model path
if not os.path.exists(model_path):
    print('ERROR: Model file not found.')
    sys.exit(1)

# Load model
model = YOLO(model_path)
labels = model.names

# Only detect this class
TARGET_CLASS_ID = 32
TARGET_CLASS_NAME = labels[TARGET_CLASS_ID]

# Setup camera
picam = Picamera2()
picam.configure(picam.create_video_configuration(main={"format": 'RGB888', "size": (resW, resH)}))
picam.start()

# Drawing setup box
color = (0, 255, 0)

# FPS buffer
frame_rate_buffer = []
fps_avg_len = 100
avg_frame_rate = 0

print("Running detection... Press 'q' to quit.")

# ================== INFERENCE LOOP ==================
while True:
    t_start = time.perf_counter()

    frame = picam.capture_array()

    results = model(frame, verbose=False)
    detections = results[0].boxes

    object_count = 0

    for det in detections:
        cls = int(det.cls.item())
        conf = det.conf.item()
        if cls != TARGET_CLASS_ID or conf < min_thresh:
            continue

        xyxy = det.xyxy.cpu().numpy().squeeze().astype(int)
        xmin, ymin, xmax, ymax = xyxy

        # Draw box and label
        cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
        label = f'{TARGET_CLASS_NAME}: {int(conf * 100)}%'
        cv2.putText(frame, label, (xmin, ymin - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        object_count += 1

    # FPS
    t_stop = time.perf_counter()
    fps = 1 / (t_stop - t_start)
    frame_rate_buffer.append(fps)
    if len(frame_rate_buffer) > fps_avg_len:
        frame_rate_buffer.pop(0)
    avg_frame_rate = np.mean(frame_rate_buffer)

    # Display
    cv2.putText(frame, f'FPS: {avg_frame_rate:.2f}', (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    cv2.putText(frame, f'Count: {object_count}', (10, 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    cv2.imshow('YOLO Detection', frame)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

# ================== CLEANUP ==================
picam.stop()
cv2.destroyAllWindows()
print(f'Final average FPS: {avg_frame_rate:.2f}')
