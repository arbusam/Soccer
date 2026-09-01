import importlib
import sys
import threading
import types
import unittest.mock

import numpy as np
import pytest


def _import_camera_without_picamera_hardware():
    cv2 = types.ModuleType("cv2")
    cv2.COLOR_BGR2RGB = 4

    def cvt_color(frame, _code):
        return frame

    cv2.cvtColor = cvt_color
    picamera2 = types.ModuleType("picamera2")
    picamera2.Picamera2 = object
    encoders = types.ModuleType("picamera2.encoders")
    encoders.H264Encoder = object
    encoders.JpegEncoder = object
    outputs = types.ModuleType("picamera2.outputs")
    outputs.FileOutput = object
    outputs.PyavOutput = object
    request = types.ModuleType("picamera2.request")
    request.MappedArray = object
    modules = {
        "cv2": cv2,
        "picamera2": picamera2,
        "picamera2.encoders": encoders,
        "picamera2.outputs": outputs,
        "picamera2.request": request,
    }
    with unittest.mock.patch.dict(sys.modules, modules):
        sys.modules.pop("lib.camera", None)
        return importlib.import_module("lib.camera")


def inference_metadata_identifies_exact_source_capture():
    camera_module = _import_camera_without_picamera_hardware()
    camera = camera_module.Camera.__new__(camera_module.Camera)
    camera._infer_stop = threading.Event()
    camera._is_shutting_down = False
    camera._latest_lock = threading.Lock()
    camera._latest_seq = 42
    camera._latest_buf = np.zeros((8, 8, 3), dtype=np.uint8)
    camera._latest_sensor_timestamp_ns = 1_500_000_000
    camera._first_sensor_timestamp_ns = 1_000_000_000
    camera._video_start_elapsed_s = 2.0
    camera.session_epoch_monotonic = 10.0
    camera._measurement_lock = threading.Lock()
    camera._last_detection = None
    camera._bearing = None
    camera._distance = None
    camera._frame_id = 0
    camera.distance_calibration = None
    camera._distance_calibration_warning_logged = True
    detection = {
        "bbox": (1, 2, 3, 4),
        "centre": (2.5, 4.0),
        "radial_pixels": 1.0,
        "confidence": 0.75,
        "polygon": None,
    }
    camera._detect_ball = lambda _frame: detection
    events = []

    def collect(event):
        events.append(event)
        camera._infer_stop.set()

    camera.detection_callback = collect
    camera._infer_loop()

    assert events[0]["capture_sequence"] == 42
    assert events[0]["sensor_timestamp_ns"] == 1_500_000_000
    assert events[0]["video_time_s"] == pytest.approx(0.5)
    assert events[0]["elapsed_s"] == pytest.approx(2.5)
    assert events[0]["detection"] is detection
