import time
from pathlib import Path

from ball_distance_calibration import (
    DEFAULT_DISTANCE_CALIBRATION_FILE,
    apply_camera_bearing_offset,
    calculate_ball_bearing_deg,
    get_distance_calibration_resolution,
    load_distance_calibration,
    predict_distance_from_calibration,
)
from hailo_ball import HailoBallDetector
from picamera2 import Picamera2

MODEL_DIR = Path(__file__).resolve().parent / "open-soccer-detect-n_hailo_model"
BALL_CONFIDENCE = 0.25

resolution = get_distance_calibration_resolution(DEFAULT_DISTANCE_CALIBRATION_FILE) or (
    1280,
    720,
)
distance_calibration = load_distance_calibration(
    resolution,
    DEFAULT_DISTANCE_CALIBRATION_FILE,
)

picam2 = Picamera2()
picam2.preview_configuration.main.size = resolution
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

detector = HailoBallDetector(MODEL_DIR, conf=BALL_CONFIDENCE)
prev_time = time.perf_counter()

try:
    while True:
        frame = picam2.capture_array()
        frame_height, frame_width = frame.shape[:2]
        detection = detector.best_ball(frame)

        now = time.perf_counter()
        fps = 1.0 / (now - prev_time)
        prev_time = now

        if detection is None:
            print(f"{fps:.1f} FPS  ball: not found")
            continue

        centre_x, centre_y = detection["centre"]
        angle_deg = apply_camera_bearing_offset(
            calculate_ball_bearing_deg(centre_x, centre_y, frame_width, frame_height)
        )
        distance_mm = predict_distance_from_calibration(
            distance_calibration,
            detection["radial_pixels"],
        )

        if distance_mm is None:
            print(f"{fps:.1f} FPS  ball: angle={angle_deg:.1f} deg  distance=n/a")
        else:
            print(
                f"{fps:.1f} FPS  ball: angle={angle_deg:.1f} deg  "
                f"distance={distance_mm:.0f} mm"
            )
finally:
    detector.close()
    picam2.stop()
