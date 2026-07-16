import time

import cv2
from picamera2 import Picamera2
from ultralytics import YOLO

# Initialize the Picamera2
picam2 = Picamera2()
picam2.preview_configuration.main.size = (1280, 720)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Load the trained YOLO26-OBB NCNN export (same directory as this script)
model = YOLO("model")

prev_time = time.perf_counter()

while True:
    # Capture frame-by-frame
    frame = picam2.capture_array()

    # Run NCNN inference on the frame
    results = model(frame)

    now = time.perf_counter()
    fps = 1.0 / (now - prev_time)
    prev_time = now
    print(f"{fps:.1f} FPS")

    # Display the raw frame
    cv2.imshow("Camera", frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) == ord("q"):
        break

# Release resources and close windows
cv2.destroyAllWindows()
