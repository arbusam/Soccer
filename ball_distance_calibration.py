import argparse
import logging
import json
import math
import os
import socketserver
import threading
from http import server

import cv2
import numpy as np


DEFAULT_DISTANCE_CALIBRATION_FILE = "ball_distance_calibration.json"
ORANGE_LOWER_BOUND = np.array([0, 200, 140])
ORANGE_UPPER_BOUND = np.array([15, 255, 255])
MIN_CONTOUR_AREA = 200
MAX_BOUNDING_BOX_AREA = 50000


class StreamingOutput:
    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()

    def write(self, frame_bytes):
        with self.condition:
            self.frame = frame_bytes
            self.condition.notify_all()


class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            body = (
                "<html><head><title>Ball Distance Calibration</title></head>"
                "<body><img src='/stream.mjpg' /></body></html>"
            ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", len(body))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path.startswith("/stream.mjpg"):
            self.send_response(200)
            self.send_header("Age", 0)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            try:
                while True:
                    with self.server.output.condition:
                        self.server.output.condition.wait()
                        frame = self.server.output.frame
                    self.wfile.write(b"--FRAME\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except Exception as exc:
                logging.warning("Removed streaming client %s: %s", self.client_address, exc)
            return

        self.send_error(404)

    def log_message(self, format, *args):
        return


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, output, *args, **kwargs):
        self.output = output
        super().__init__(*args, **kwargs)


def _draw_status_overlay(frame, detection, samples):
    display_frame = frame.copy()
    frame_centre_x = int(display_frame.shape[1] / 2)
    frame_centre_y = int(display_frame.shape[0] / 2)
    cv2.line(
        display_frame,
        (frame_centre_x - 20, frame_centre_y),
        (frame_centre_x + 20, frame_centre_y),
        (0, 0, 255),
        2,
    )
    cv2.line(
        display_frame,
        (frame_centre_x, frame_centre_y - 20),
        (frame_centre_x, frame_centre_y + 20),
        (0, 0, 255),
        2,
    )

    status_lines = [
        f"samples: {len(samples)}",
        "Enter mm in terminal, # marks captured",
        "Commands: Enter refresh, r remove, f fit, q quit",
    ]

    if detection is not None:
        x, y, width, height = detection["bbox"]
        centre_x, centre_y = detection["centre"]
        cv2.drawContours(display_frame, [detection["contour"]], -1, (0, 165, 255), 2)
        cv2.rectangle(display_frame, (x, y), (x + width, y + height), (0, 255, 0), 2)
        cv2.circle(
            display_frame,
            (int(round(centre_x)), int(round(centre_y))),
            6,
            (255, 255, 255),
            -1,
        )
        cv2.line(
            display_frame,
            (frame_centre_x, frame_centre_y),
            (int(round(centre_x)), int(round(centre_y))),
            (255, 255, 255),
            2,
        )
        status_lines.append(f"radial pixels: {detection['radial_pixels']:.2f}")
        status_lines.append(
            f"bearing: {apply_camera_bearing_offset(detection['bearing_deg']):.2f} deg"
        )
    else:
        status_lines.append("ball not detected")

    for line_index, line in enumerate(status_lines):
        cv2.putText(
            display_frame,
            line,
            (20, 40 + (line_index * 35)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    return display_frame


def resolve_distance_calibration_path(calibration_file=DEFAULT_DISTANCE_CALIBRATION_FILE):
    """Resolve relative calibration paths against this file's directory."""
    if os.path.isabs(calibration_file):
        return calibration_file
    return os.path.join(os.path.dirname(__file__), calibration_file)


def get_distance_calibration_resolution(calibration_file=DEFAULT_DISTANCE_CALIBRATION_FILE):
    """Return the saved calibration resolution, or None if unavailable."""
    calibration_path = resolve_distance_calibration_path(calibration_file)
    if not os.path.isfile(calibration_path):
        return None

    try:
        with open(calibration_path, encoding="utf-8") as calibration_handle:
            calibration_data = json.load(calibration_handle)
    except (OSError, json.JSONDecodeError):
        return None

    saved_resolution = calibration_data.get("resolution")
    if not isinstance(saved_resolution, list) or len(saved_resolution) != 2:
        return None

    try:
        return (int(saved_resolution[0]), int(saved_resolution[1]))
    except (TypeError, ValueError):
        return None


def detect_orange_ball(frame_bgr):
    """Return the strongest orange-ball detection plus its radial pixel offset."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    orange_mask = cv2.inRange(hsv, ORANGE_LOWER_BOUND, ORANGE_UPPER_BOUND)
    orange_contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    valid_detections = []
    for contour in orange_contours:
        area = cv2.contourArea(contour)
        if area <= MIN_CONTOUR_AREA:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        if width * height >= MAX_BOUNDING_BOX_AREA:
            continue
        valid_detections.append((contour, x, y, width, height, area))

    if not valid_detections:
        return None

    contour, x, y, width, height, area = max(valid_detections, key=lambda item: item[3] * item[4])
    centre_x = x + (width / 2.0)
    centre_y = y + (height / 2.0)
    frame_centre_x = frame_bgr.shape[1] / 2.0
    frame_centre_y = frame_bgr.shape[0] / 2.0
    radial_pixels = math.hypot(centre_x - frame_centre_x, centre_y - frame_centre_y)

    return {
        "contour": contour,
        "bbox": (x, y, width, height),
        "centre": (centre_x, centre_y),
        "frame_centre": (frame_centre_x, frame_centre_y),
        "radial_pixels": radial_pixels,
        "bearing_deg": calculate_ball_bearing_deg(
            centre_x,
            centre_y,
            frame_bgr.shape[1],
            frame_bgr.shape[0],
        ),
        "contour_area": area,
        "bounding_box_area": width * height,
    }


def calculate_ball_bearing_deg(centre_x, centre_y, frame_width, frame_height):
    """Match the bearing convention used by camera.py."""
    bearing_rad = math.atan2(centre_y - (frame_height / 2.0), centre_x - (frame_width / 2.0))
    return math.degrees(bearing_rad) - 90.0


def apply_camera_bearing_offset(bearing_deg):
    """Match camera.py's additional 180 degree offset."""
    return bearing_deg + 180.0


def _calculate_rmse(actual_values, predicted_values):
    if not actual_values:
        return 0.0
    squared_errors = [(actual - predicted) ** 2 for actual, predicted in zip(actual_values, predicted_values)]
    return math.sqrt(sum(squared_errors) / len(squared_errors))


def _leave_one_out_rmse(samples, degree):
    if len(samples) < degree + 2:
        return None

    predicted_values = []
    actual_values = []
    for sample_index, held_out_sample in enumerate(samples):
        training_samples = samples[:sample_index] + samples[sample_index + 1 :]
        radial_values = [sample["radial_pixels"] for sample in training_samples]
        distance_values = [sample["distance_mm"] for sample in training_samples]
        coefficients = np.polyfit(radial_values, distance_values, degree)
        prediction = float(np.polyval(coefficients, held_out_sample["radial_pixels"]))
        predicted_values.append(prediction)
        actual_values.append(held_out_sample["distance_mm"])
    return _calculate_rmse(actual_values, predicted_values)


def fit_distance_calibration(samples, max_degree=3):
    """Fit several polynomial models and choose the best one by validation error."""
    if len(samples) < 2:
        raise ValueError("Need at least 2 samples to fit a distance model.")

    max_degree = max(1, min(int(max_degree), len(samples) - 1))
    radial_values = [sample["radial_pixels"] for sample in samples]
    distance_values = [sample["distance_mm"] for sample in samples]

    candidate_models = []
    for degree in range(1, max_degree + 1):
        coefficients = np.polyfit(radial_values, distance_values, degree)
        training_predictions = [float(np.polyval(coefficients, radial)) for radial in radial_values]
        candidate_models.append(
            {
                "degree": degree,
                "coefficients": [float(value) for value in coefficients.tolist()],
                "training_rmse_mm": _calculate_rmse(distance_values, training_predictions),
                "leave_one_out_rmse_mm": _leave_one_out_rmse(samples, degree),
            }
        )

    comparable_models = [model for model in candidate_models if model["leave_one_out_rmse_mm"] is not None]
    if comparable_models:
        selected_model = min(comparable_models, key=lambda model: model["leave_one_out_rmse_mm"])
        selection_reason = "lowest leave-one-out RMSE"
    else:
        selected_model = min(candidate_models, key=lambda model: model["training_rmse_mm"])
        selection_reason = "lowest training RMSE"

    return {
        "model_type": "polynomial",
        "selected_degree": selected_model["degree"],
        "coefficients": selected_model["coefficients"],
        "selection_reason": selection_reason,
        "fit_metrics": candidate_models,
        "sample_count": len(samples),
        "radial_pixel_range": [
            float(min(radial_values)),
            float(max(radial_values)),
        ],
        "distance_mm_range": [
            float(min(distance_values)),
            float(max(distance_values)),
        ],
    }


def save_distance_calibration(samples, resolution, calibration_file=DEFAULT_DISTANCE_CALIBRATION_FILE, max_degree=3):
    """Fit a model from samples and save it to disk as JSON."""
    fit_result = fit_distance_calibration(samples, max_degree=max_degree)
    captured_samples = [sample for sample in samples if sample.get("captured", False)]
    capture_calibration = {
        "sample_count": len(captured_samples),
        "samples": [
            {
                "distance_mm": float(sample["distance_mm"]),
                "bearing_deg": float(sample["bearing_deg"]),
                "radial_pixels": float(sample["radial_pixels"]),
                "centre_x": float(sample["centre_x"]),
                "centre_y": float(sample["centre_y"]),
            }
            for sample in captured_samples
        ],
    }
    if captured_samples:
        capture_distances = [float(sample["distance_mm"]) for sample in captured_samples]
        capture_bearings = [float(sample["bearing_deg"]) for sample in captured_samples]
        capture_abs_bearings = [abs(bearing) for bearing in capture_bearings]
        capture_calibration["distance_mm_range"] = [min(capture_distances), max(capture_distances)]
        capture_calibration["bearing_deg_range"] = [min(capture_bearings), max(capture_bearings)]
        capture_calibration["abs_bearing_deg_range"] = [min(capture_abs_bearings), max(capture_abs_bearings)]
        capture_calibration["max_distance_mm"] = max(capture_distances)
        capture_calibration["max_abs_bearing_deg"] = max(capture_abs_bearings)

    calibration_data = {
        "version": 1,
        "resolution": [int(resolution[0]), int(resolution[1])],
        "frame_centre": [resolution[0] / 2.0, resolution[1] / 2.0],
        "input_feature": "radial_pixels_from_frame_centre",
        "distance_units": "mm",
        "model": fit_result,
        "capture_calibration": capture_calibration,
        "samples": [
            {
                "distance_mm": float(sample["distance_mm"]),
                "radial_pixels": float(sample["radial_pixels"]),
                "centre_x": float(sample["centre_x"]),
                "centre_y": float(sample["centre_y"]),
                "bounding_box_area": float(sample["bounding_box_area"]),
                "captured": bool(sample.get("captured", False)),
                **(
                    {"bearing_deg": float(sample["bearing_deg"])}
                    if sample.get("captured", False)
                    else {}
                ),
            }
            for sample in samples
        ],
    }

    calibration_path = resolve_distance_calibration_path(calibration_file)
    temp_path = f"{calibration_path}.tmp"
    with open(temp_path, "w", encoding="utf-8") as calibration_handle:
        json.dump(calibration_data, calibration_handle, indent=2)
        calibration_handle.flush()
        os.fsync(calibration_handle.fileno())
    os.replace(temp_path, calibration_path)
    return calibration_data, calibration_path


def load_distance_calibration(resolution, calibration_file=DEFAULT_DISTANCE_CALIBRATION_FILE):
    """Load a saved calibration file if it matches the current camera resolution."""
    calibration_path = resolve_distance_calibration_path(calibration_file)
    if not os.path.isfile(calibration_path):
        return None

    try:
        with open(calibration_path, encoding="utf-8") as calibration_handle:
            calibration_data = json.load(calibration_handle)
    except (OSError, json.JSONDecodeError):
        return None

    saved_resolution = calibration_data.get("resolution")
    if list(saved_resolution or []) != [int(resolution[0]), int(resolution[1])]:
        return None

    model = calibration_data.get("model")
    if not isinstance(model, dict):
        return None
    if model.get("model_type") != "polynomial":
        return None

    coefficients = model.get("coefficients")
    if not isinstance(coefficients, list) or not coefficients:
        return None
    return calibration_data


def predict_distance_from_calibration(calibration_data, radial_pixels):
    """Predict distance in mm from radial pixels using the saved polynomial model."""
    if calibration_data is None or radial_pixels is None:
        return None

    model = calibration_data.get("model", {})
    coefficients = model.get("coefficients")
    radial_range = model.get("radial_pixel_range")
    if not isinstance(coefficients, list) or not coefficients:
        return None
    if not isinstance(radial_range, list) or len(radial_range) != 2:
        return None

    clamped_radial = min(max(float(radial_pixels), float(radial_range[0])), float(radial_range[1]))
    predicted_distance = float(np.polyval(coefficients, clamped_radial))
    if predicted_distance < 0:
        return None
    return predicted_distance


def _format_polynomial(coefficients):
    degree = len(coefficients) - 1
    terms = []
    for index, coefficient in enumerate(coefficients):
        power = degree - index
        if abs(coefficient) < 1e-12:
            continue
        if power == 0:
            terms.append(f"{coefficient:.8f}")
        elif power == 1:
            terms.append(f"{coefficient:.8f} * r")
        else:
            terms.append(f"{coefficient:.8f} * r^{power}")
    return " + ".join(terms) if terms else "0.0"


def run_interactive_calibration(
    resolution=(2000, 2000),
    frame_rate=60,
    calibration_file=DEFAULT_DISTANCE_CALIBRATION_FILE,
    stream_port=8000,
):
    """Capture samples in the terminal and save a radial-pixels-to-distance fit."""
    from picamera2 import Picamera2

    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(main={"size": resolution, "format": "RGB888"}))
    picam2.controls.FrameRate = frame_rate
    picam2.controls.AnalogueGain = 2.0
    picam2.start()

    samples = []
    output = StreamingOutput()
    stream_server = StreamingServer(output, ("", stream_port), StreamingHandler)
    stream_thread = threading.Thread(target=stream_server.serve_forever, daemon=True)
    stream_thread.start()

    print("Ball distance calibration")
    print(f"Stream available at http://localhost:{stream_port}/stream.mjpg")
    print("Press Enter to capture a fresh frame.")
    print("Type a distance in mm to save a sample.")
    print("Add # to the distance to mark the sample as captured.")
    print("Commands: r = remove last sample, f = fit and save, q = quit")

    try:
        while True:
            frame = picam2.capture_array()
            detection = detect_orange_ball(frame)
            display_frame = _draw_status_overlay(frame, detection, samples)
            success, encoded_frame = cv2.imencode(".jpg", display_frame)
            if success:
                output.write(encoded_frame.tobytes())
            print()
            print(f"Samples: {len(samples)}")
            if detection is not None:
                centre_x, centre_y = detection["centre"]
                radial_pixels = detection["radial_pixels"]
                bearing_deg = apply_camera_bearing_offset(detection["bearing_deg"])
                print(
                    "Ball detected:",
                    f"radial={radial_pixels:.2f} px,",
                    f"bearing={bearing_deg:.2f} deg,",
                    f"centre=({centre_x:.1f}, {centre_y:.1f})",
                )
            else:
                print("Ball not detected in the current frame.")

            command = input(
                "Distance in mm, blank refresh, or command [r/f/q]: "
            ).strip()

            if command == "":
                continue

            if command.lower() == "q":
                break

            if command.lower() == "r":
                if samples:
                    removed_sample = samples.pop()
                    print(
                        "Removed sample:",
                        f"distance={removed_sample['distance_mm']:.2f} mm,",
                        f"radial={removed_sample['radial_pixels']:.2f} px,",
                        f"captured={removed_sample.get('captured', False)}",
                    )
                else:
                    print("No samples to remove.")
                continue

            if command.lower() == "f":
                if len(samples) < 2:
                    print("Need at least 2 samples before fitting.")
                    continue

                calibration_data, calibration_path = save_distance_calibration(
                    samples=samples,
                    resolution=resolution,
                    calibration_file=calibration_file,
                )
                selected_model = calibration_data["model"]
                print(f"Saved calibration to {calibration_path}")
                print(
                    "Selected model:",
                    f"degree={selected_model['selected_degree']},",
                    f"reason={selected_model['selection_reason']}",
                )
                print("distance_mm =", _format_polynomial(selected_model["coefficients"]))
                capture_calibration = calibration_data["capture_calibration"]
                print(f"Captured samples saved: {capture_calibration['sample_count']}")
                if capture_calibration["sample_count"] > 0:
                    print(
                        "Capture thresholds:",
                        f"max_distance_mm={capture_calibration['max_distance_mm']:.2f},",
                        f"max_abs_bearing_deg={capture_calibration['max_abs_bearing_deg']:.2f}",
                    )
                for candidate_model in selected_model["fit_metrics"]:
                    print(
                        f"degree {candidate_model['degree']}:",
                        f"training_rmse={candidate_model['training_rmse_mm']:.2f} mm,",
                        f"leave_one_out_rmse={candidate_model['leave_one_out_rmse_mm']}",
                    )
                continue

            if detection is None:
                print("Ball not detected. Move the ball into view before saving a sample.")
                continue

            captured = "#" in command
            normalized_distance_text = command.replace("#", "").strip()
            try:
                distance_mm = float(normalized_distance_text)
            except ValueError:
                print("Invalid input. Enter a distance in mm, or use r, f, q.")
                continue

            centre_x, centre_y = detection["centre"]
            sample = {
                "distance_mm": distance_mm,
                "radial_pixels": float(detection["radial_pixels"]),
                "centre_x": float(centre_x),
                "centre_y": float(centre_y),
                "bounding_box_area": float(detection["bounding_box_area"]),
                "captured": captured,
            }
            if captured:
                sample["bearing_deg"] = float(
                    apply_camera_bearing_offset(detection["bearing_deg"])
                )
            samples.append(sample)
            print_parts = [
                "Saved sample:",
                f"distance={distance_mm:.2f} mm,",
                f"radial={sample['radial_pixels']:.2f} px,",
                f"centre=({sample['centre_x']:.1f}, {sample['centre_y']:.1f}),",
                f"captured={captured}",
            ]
            if captured:
                print_parts.append(f"bearing={sample['bearing_deg']:.2f} deg")
            print(*print_parts)
    finally:
        stream_server.shutdown()
        stream_server.server_close()
        picam2.stop()


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Capture orange-ball samples at known distances, then fit and save a "
            "distance_mm = f(radial_pixels_from_frame_centre) calibration model."
        )
    )
    parser.add_argument("--width", type=int, default=2000, help="Camera frame width in pixels.")
    parser.add_argument("--height", type=int, default=2000, help="Camera frame height in pixels.")
    parser.add_argument("--frame-rate", type=int, default=60, help="Camera frame rate.")
    parser.add_argument(
        "--output",
        default=DEFAULT_DISTANCE_CALIBRATION_FILE,
        help="Calibration JSON filename to write.",
    )
    parser.add_argument(
        "--stream-port",
        type=int,
        default=8000,
        help="HTTP port for the MJPEG calibration stream.",
    )
    args = parser.parse_args()

    run_interactive_calibration(
        resolution=(args.width, args.height),
        frame_rate=args.frame_rate,
        calibration_file=args.output,
        stream_port=args.stream_port,
    )


if __name__ == "__main__":
    main()
