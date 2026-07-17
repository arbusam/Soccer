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
# Print every N frames so logging does not cap FPS.
PRINT_EVERY = 10

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
picam2.preview_configuration.controls = {"FrameRate": 60}
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

detector = HailoBallDetector(MODEL_DIR, conf=BALL_CONFIDENCE)
prev_time = time.perf_counter()
frame_i = 0
sum_capture_ms = 0.0
sum_infer_ms = 0.0

try:
    while True:
        t0 = time.perf_counter()
        frame = picam2.capture_array()
        t1 = time.perf_counter()
        frame_height, frame_width = frame.shape[:2]
        detection = detector.best_ball(frame)
        t2 = time.perf_counter()

        capture_ms = (t1 - t0) * 1000.0
        infer_ms = (t2 - t1) * 1000.0
        sum_capture_ms += capture_ms
        sum_infer_ms += infer_ms
        frame_i += 1

        now = time.perf_counter()
        fps = 1.0 / (now - prev_time)
        prev_time = now

        if frame_i % PRINT_EVERY != 0:
            continue

        avg_capture = sum_capture_ms / PRINT_EVERY
        avg_infer = sum_infer_ms / PRINT_EVERY
        sum_capture_ms = 0.0
        sum_infer_ms = 0.0

        if detection is None:
            print(
                f"{fps:.1f} FPS  capture={avg_capture:.1f}ms  "
                f"infer+post={avg_infer:.1f}ms  ball: not found"
            )
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
            dist_txt = "n/a"
        else:
            dist_txt = f"{distance_mm:.0f} mm"
        print(
            f"{fps:.1f} FPS  capture={avg_capture:.1f}ms  "
            f"infer+post={avg_infer:.1f}ms  "
            f"ball: angle={angle_deg:.1f} deg  distance={dist_txt}"
        )
finally:
    detector.close()
    picam2.stop()
