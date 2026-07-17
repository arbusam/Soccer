import argparse
import time
from pathlib import Path

import cv2

from ball_distance_calibration import (
    DEFAULT_DISTANCE_CALIBRATION_FILE,
    apply_camera_bearing_offset,
    calculate_ball_bearing_deg,
    get_distance_calibration_resolution,
    load_distance_calibration,
    predict_distance_from_calibration,
)
from hailo_ball import HailoBallDetector

MODEL_DIR = Path(__file__).resolve().parent / "open-soccer-detect-n_hailo_model"
BALL_CONFIDENCE = 0.25
PRINT_EVERY = 10


def _print_detection(detection, frame_width, frame_height, distance_calibration, prefix=""):
    if detection is None:
        print(f"{prefix}ball: not found")
        return

    centre_x, centre_y = detection["centre"]
    angle_deg = apply_camera_bearing_offset(
        calculate_ball_bearing_deg(centre_x, centre_y, frame_width, frame_height)
    )
    distance_mm = predict_distance_from_calibration(
        distance_calibration,
        detection["radial_pixels"],
    )
    dist_txt = "n/a" if distance_mm is None else f"{distance_mm:.0f} mm"
    print(
        f"{prefix}ball: angle={angle_deg:.1f} deg  distance={dist_txt}  "
        f"conf={detection['confidence']:.3f}  bbox={detection['bbox']}"
    )


def run_image(image_path: Path, conf: float) -> None:
    bgr = cv2.imread(str(image_path))
    if bgr is None:
        raise SystemExit(f"Could not read image: {image_path}")
    frame = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    frame_height, frame_width = frame.shape[:2]

    resolution = (frame_width, frame_height)
    distance_calibration = load_distance_calibration(
        get_distance_calibration_resolution(DEFAULT_DISTANCE_CALIBRATION_FILE) or resolution,
        DEFAULT_DISTANCE_CALIBRATION_FILE,
    )

    with HailoBallDetector(MODEL_DIR, conf=conf) as detector:
        t0 = time.perf_counter()
        detection = detector.best_ball(frame)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0

    _print_detection(
        detection,
        frame_width,
        frame_height,
        distance_calibration,
        prefix=f"{elapsed_ms:.1f}ms  ",
    )

    # Draw and write a preview next to the input.
    if detection is not None:
        x, y, w, h = detection["bbox"]
        cv2.rectangle(bgr, (x, y), (x + w, y + h), (0, 165, 255), 2)
        cx, cy = detection["centre"]
        cv2.circle(bgr, (int(cx), int(cy)), 4, (0, 255, 0), -1)
    out_path = image_path.with_name(f"{image_path.stem}_hailo{image_path.suffix}")
    cv2.imwrite(str(out_path), bgr)
    print(f"wrote {out_path}")


def run_camera(conf: float) -> None:
    from picamera2 import Picamera2

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

    detector = HailoBallDetector(MODEL_DIR, conf=conf)
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
            prefix = (
                f"{fps:.1f} FPS  capture={avg_capture:.1f}ms  "
                f"infer+post={avg_infer:.1f}ms  "
            )
            _print_detection(
                detection,
                frame_width,
                frame_height,
                distance_calibration,
                prefix=prefix,
            )
    finally:
        detector.close()
        picam2.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description="Run Hailo ball detector on camera or an image.")
    parser.add_argument(
        "--image",
        type=Path,
        default=None,
        help="Run once on this image file (RGB via OpenCV) instead of the camera",
    )
    parser.add_argument("--conf", type=float, default=BALL_CONFIDENCE)
    args = parser.parse_args()

    if args.image is not None:
        run_image(args.image.resolve(), conf=args.conf)
    else:
        run_camera(conf=args.conf)


if __name__ == "__main__":
    main()
