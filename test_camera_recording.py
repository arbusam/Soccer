import importlib
import sys
import threading
import types
import unittest
from unittest.mock import patch

import numpy as np


def import_camera_without_picamera_hardware():
    cv2 = types.ModuleType("cv2")
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
    with patch.dict(sys.modules, modules):
        sys.modules.pop("camera", None)
        return importlib.import_module("camera")


class CameraRecordingTests(unittest.TestCase):
    def test_inference_metadata_identifies_exact_source_capture(self):
        camera_module = import_camera_without_picamera_hardware()
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

        self.assertEqual(events[0]["capture_sequence"], 42)
        self.assertEqual(events[0]["sensor_timestamp_ns"], 1_500_000_000)
        self.assertAlmostEqual(events[0]["video_time_s"], 0.5)
        self.assertAlmostEqual(events[0]["elapsed_s"], 2.5)
        self.assertIs(events[0]["detection"], detection)


if __name__ == "__main__":
    unittest.main()
