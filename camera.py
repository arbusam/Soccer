import io
import logging
import socketserver
import asyncio
import threading
from http import server
from pathlib import Path
from threading import Condition

import cv2
import numpy as np
from ball_distance_calibration import (
    DEFAULT_DISTANCE_CALIBRATION_FILE,
    calculate_ball_bearing_deg,
    get_distance_calibration_resolution,
    load_distance_calibration,
    predict_distance_from_calibration,
)
# change to picamzero
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
from picamera2.request import MappedArray
from hailo_ball import HailoBallDetector


DEFAULT_BALL_MODEL_PATH = Path(__file__).resolve().parent / "open-soccer-detect-n_hailo_model"


def _resolve_model_path(model_path):
    model_path = Path(model_path)
    if model_path.is_absolute():
        return model_path
    return Path(__file__).resolve().parent / model_path

class Camera:
    def __init__(
        self,
        PORT,
        resolution=None,
        frame_rate=30,
        distance_calibration_file=DEFAULT_DISTANCE_CALIBRATION_FILE,
        ball_model_path=DEFAULT_BALL_MODEL_PATH,
        ball_confidence=0.25,
    ):
        if resolution is None:
            resolution = get_distance_calibration_resolution(distance_calibration_file) or (
                2000,
                2000,
            )
        self.PORT = PORT
        self.resolution = resolution
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_video_configuration(main={"size": resolution, "format": 'RGB888'}))
        self.picam2.controls.FrameRate = frame_rate
        self.forward_angle = 0  # Add forward angle property
        # self.picam2.controls.ExposureTime = 30000
        self.picam2.controls.AnalogueGain = 2.0
        self.output = self.StreamingOutput()
        self.server = None
        self.server_thread = None
        self.user_callback = None
        self._bearing = None
        self._distance = None
        self._frame_id = 0
        self._measurement_lock = threading.Lock()
        self._capture_started = False
        self._recording = False
        self._server_started = False
        self._is_shutting_down = False
        self.ball_model_path = _resolve_model_path(ball_model_path)
        self.ball_confidence = ball_confidence
        self.ball_model = HailoBallDetector(
            self.ball_model_path,
            conf=ball_confidence,
        )
        self.calibrated = True
        self.upperbound = 0
        self.lowerbound = 0
        self.colours = []
        self.distance_calibration = load_distance_calibration(
            resolution=resolution,
            calibration_file=distance_calibration_file,
        )
        self._distance_calibration_warning_logged = False
        if self.distance_calibration is None:
            logging.warning(
                "Ball distance calibration file '%s' was not loaded; distance estimates will be unavailable.",
                distance_calibration_file,
            )

        # Enable detection callback by default
        self.picam2.pre_callback = self._proxy_callback

        # # print camera modes
        # print(self.picam2.sensor_modes)
            
        # Define async tasks
        self.async_tasks = [self.run_server]

    @property
    def stream_enabled(self):
        return self._server_started

    class StreamingOutput(io.BufferedIOBase):
        def __init__(self):
            self.frame = None
            self.condition = Condition()

        def write(self, buf):
            with self.condition:
                self.frame = buf
                self.condition.notify_all()

    class StreamingHandler(server.BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path in ('/', '/index.html'):
                self.serve_index()
            elif self.path.startswith('/stream.mjpg'):
                self.serve_stream()
            else:
                self.send_error(404)

        def serve_index(self):
            content = (
                "<html><head><title>Camera Preview</title></head>"
                "<body><img src='/stream.mjpg' /></body></html>"
            ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)

        def serve_stream(self):
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with self.server.camera.output.condition:
                        self.server.camera.output.condition.wait()
                        frame = self.server.camera.output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))

    class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
        allow_reuse_address = True
        daemon_threads = True

        def __init__(self, camera, *args, **kwargs):
            self.camera = camera
            super().__init__(*args, **kwargs)

    @property
    def bearing(self):
        if self._is_shutting_down:
            return None
        with self._measurement_lock:
            return self._bearing

    @property
    def distance(self):
        if self._is_shutting_down:
            return None
        with self._measurement_lock:
            return self._distance

    @property
    def frame_id(self):
        with self._measurement_lock:
            return self._frame_id

    def get_measurement(self):
        if self._is_shutting_down:
            with self._measurement_lock:
                return self._frame_id, None, None
        with self._measurement_lock:
            return self._frame_id, self._bearing, self._distance

    def set_callback(self, callback_function):
        self.user_callback = callback_function
        self.picam2.pre_callback = self._proxy_callback

    def _detect_ball(self, frame):
        return self.ball_model.best_ball(frame)

    def _start_capture(self, enable_stream=False):
        if self._capture_started:
            return

        if enable_stream:
            # JPEG encode frames into the MJPEG buffer used by the HTTP preview.
            self.picam2.start_recording(JpegEncoder(), FileOutput(self.output))
            self._recording = True
        else:
            # Capture-only path: run the detection callback without JPEG encoding.
            self.picam2.start()
            self._recording = False
        self._capture_started = True

    def _start_http_server(self):
        if self._server_started:
            return

        address = ("", self.PORT)
        self.server = self.StreamingServer(self, address, self.StreamingHandler)
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        self._server_started = True
        print(f"Camera preview available at http://localhost:{self.PORT}/")
        print(f"Raw MJPEG stream available at http://localhost:{self.PORT}/stream.mjpg")

    def start(self):
        """Start camera capture so bearing updates in callback (no MJPEG preview)."""
        self._start_capture(enable_stream=False)

    def start_stream(self):
        """Start camera capture and the MJPEG HTTP stream."""
        self._start_capture(enable_stream=True)
        self._start_http_server()

    def _proxy_callback(self, request):
        if self._is_shutting_down:
            return
        try:
            with MappedArray(request, "main") as m:
                if self._is_shutting_down:
                    return

                preview_active = self._server_started
                needs_hsv = (not self.calibrated) or self.user_callback is not None
                hsv = cv2.cvtColor(m.array, cv2.COLOR_BGR2HSV) if needs_hsv else None

                if self.calibrated:
                    detection = self._detect_ball(m.array)
                    if detection is not None:
                        x, y, w, h = detection["bbox"]
                        centre_x, centre_y = detection["centre"]
                        if preview_active:
                            if detection["polygon"] is not None:
                                points = detection["polygon"].astype(np.int32).reshape(-1, 1, 2)
                                cv2.polylines(m.array, [points], True, (0, 165, 255), 2)
                            else:
                                cv2.rectangle(m.array, (x, y), (x + w, y + h), (0, 165, 255), 2)
                            cv2.circle(
                                m.array,
                                (int(round(centre_x)), int(round(centre_y))),
                                5,
                                (255, 255, 255),
                                -1,
                            )

                        # Find the bearing of the ball centre from the centre of the image.
                        new_bearing = calculate_ball_bearing_deg(
                            centre_x,
                            centre_y,
                            m.array.shape[1],
                            m.array.shape[0],
                        )
                        new_bearing += 180
                        new_distance = predict_distance_from_calibration(
                            self.distance_calibration,
                            detection["radial_pixels"],
                        )
                        if new_distance is None:
                            if (
                                self.distance_calibration is None
                                and not self._distance_calibration_warning_logged
                            ):
                                logging.warning(
                                    "Ball detected, but no valid distance calibration is loaded."
                                )
                                self._distance_calibration_warning_logged = True
                    else:
                        new_bearing = None
                        new_distance = None
                    with self._measurement_lock:
                        self._bearing = new_bearing
                        self._distance = new_distance
                        self._frame_id += 1

                    if self.user_callback:
                        self.user_callback(m.array, hsv)
                else:
                    # Only run for calibration, when self.calibrated is false
                    center_y = m.array.shape[0] - 250
                    center_x = m.array.shape[1] - 1200

                    if preview_active:
                        cv2.line(
                            m.array,
                            (center_x, center_y - 10),
                            (center_x, center_y + 10),
                            (0, 0, 255),
                            2,
                        )
                        cv2.line(
                            m.array,
                            (center_x - 10, center_y),
                            (center_x + 10, center_y),
                            (0, 0, 255),
                            2,
                        )

                    # Check for colour values in certain range
                    pixel_colour = hsv[center_y, center_x]
                    if 10 < pixel_colour[0] < 25 and 100 < pixel_colour[1] < 255 and 100 < pixel_colour[2] < 255:
                        if (pixel_colour[0], pixel_colour[1], pixel_colour[2]) not in self.colours:
                            self.colours.append((pixel_colour[0], pixel_colour[1], pixel_colour[2]))

                    # Add detected colour to text file
                    with open("calibrate_camera.txt", "w") as c:
                        for i in self.colours:
                            c.write(str(i[0]) + " " + str(i[1]) + " " + str(i[2]))
                            c.write("\n")
                    with self._measurement_lock:
                        self._bearing = None
                        self._distance = None
                        self._frame_id += 1

        except Exception as e:
            logging.warning("Camera callback error (may be during shutdown): %s", e)

    async def run_server(self):
        """Async task to run the camera server"""
        self.start_stream()

        try:
            # Keep the task alive
            while True:
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            self.stop()

    def stop(self):
        if self._is_shutting_down:
            return
        self._is_shutting_down = True

        if not self._capture_started and not self._server_started:
            print("Camera stopped")
            return

        try:
            self.picam2.pre_callback = None
        except Exception as e:
            logging.warning("Error clearing camera callback: %s", e)

        if self.server:
            try:
                self.server.shutdown()
                self.server.server_close()
            except Exception as e:
                logging.warning("Error shutting down camera HTTP server: %s", e)
            self.server = None
        self.server_thread = None
        self._server_started = False
        if self._capture_started:
            try:
                if self._recording:
                    self.picam2.stop_recording()
                else:
                    self.picam2.stop()
            except Exception as e:
                logging.warning("Error stopping camera capture: %s", e)
            self._capture_started = False
            self._recording = False
        try:
            self.ball_model.close()
        except Exception as e:
            logging.warning("Error closing Hailo detector: %s", e)
        print("Camera stopped")

async def main():
    camera = Camera(PORT=8000, resolution=(2000, 2000), frame_rate=60)
    await camera.run_server()

if __name__ == "__main__":
    asyncio.run(main())