import math
import time
from pathlib import Path

import numpy as np
from ball_distance_calibration import (
    DEFAULT_DISTANCE_CALIBRATION_FILE,
    apply_camera_bearing_offset,
    calculate_ball_bearing_deg,
    get_distance_calibration_resolution,
    load_distance_calibration,
    predict_distance_from_calibration,
)
from picamera2 import Picamera2
from ultralytics import YOLO

MODEL_PATH = Path(__file__).resolve().parent / "open-soccer-obb-s_ncnn_model"
BALL_CLASS_NAME = "ball"
BALL_CONFIDENCE = 0.25


def _to_numpy(value):
    if value is None:
        return None
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        return value.numpy()
    return np.asarray(value)


def _is_ball_class(class_id, model_names):
    class_id = int(class_id)
    class_name = None
    if isinstance(model_names, dict):
        class_name = model_names.get(class_id)
    elif isinstance(model_names, (list, tuple)) and 0 <= class_id < len(model_names):
        class_name = model_names[class_id]
    if class_name is not None:
        return str(class_name).strip().lower() == BALL_CLASS_NAME
    return class_id == 0


def _best_ball_detection(result, frame_width, frame_height, model_names):
    candidates = []
    obb = getattr(result, "obb", None)
    if obb is None or getattr(obb, "cls", None) is None:
        return None

    classes = _to_numpy(obb.cls)
    confidences = _to_numpy(getattr(obb, "conf", None))
    polygons = _to_numpy(getattr(obb, "xyxyxyxy", None))
    boxes = _to_numpy(getattr(obb, "xyxy", None))

    for index, class_id in enumerate(classes):
        if not _is_ball_class(class_id, model_names):
            continue

        if polygons is not None:
            polygon = np.asarray(polygons[index], dtype=np.float32).reshape(-1, 2)
            min_x, min_y = polygon.min(axis=0)
            max_x, max_y = polygon.max(axis=0)
        elif boxes is not None:
            min_x, min_y, max_x, max_y = map(float, boxes[index])
        else:
            continue

        width = max_x - min_x
        height = max_y - min_y
        if width <= 0 or height <= 0:
            continue

        centre_x = min_x + (width / 2.0)
        centre_y = min_y + (height / 2.0)
        confidence = float(confidences[index]) if confidences is not None else 0.0
        candidates.append(
            {
                "centre": (centre_x, centre_y),
                "radial_pixels": math.hypot(
                    centre_x - (frame_width / 2.0),
                    centre_y - (frame_height / 2.0),
                ),
                "confidence": confidence,
            }
        )

    if not candidates:
        return None
    return max(candidates, key=lambda detection: detection["confidence"])


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

model = YOLO(str(MODEL_PATH), task="obb")
prev_time = time.perf_counter()

try:
    while True:
        frame = picam2.capture_array()
        frame_height, frame_width = frame.shape[:2]
        results = model.predict(frame, conf=BALL_CONFIDENCE, verbose=False)
        detection = (
            _best_ball_detection(results[0], frame_width, frame_height, model.names)
            if results
            else None
        )

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
    picam2.stop()
