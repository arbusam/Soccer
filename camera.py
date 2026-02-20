import io
import logging
import socketserver
from http import server
from threading import Condition
import cv2
import threading
import asyncio
import math
import numpy as np
# change to picamzero
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
from picamera2.request import MappedArray

class Camera:
    def __init__(self, PORT, resolution=(640, 640), frame_rate=30):
        self.PORT = PORT
        self.resolution = resolution
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_video_configuration(main={"size": resolution, "format": 'RGB888'}))
        self.picam2.controls.FrameRate = frame_rate
        self.forward_angle = 0  # Add forward angle property
        # self.picam2.controls.ExposureTime = 30000
        self.picam2.controls.AnalogueGain = 15.0
        self.output = self.StreamingOutput()
        self.server = None
        self.server_thread = None
        self.user_callback = None
        self._bearing = None
        self._distance = None
        self._capture_started = False
        self._server_started = False
        self._is_shutting_down = False

        # Enable color detection callback by default
        self.picam2.pre_callback = self._proxy_callback

        # # print camera modes
        # print(self.picam2.sensor_modes)
            
        # Define async tasks
        self.async_tasks = [self.run_server]

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
            if self.path.startswith('/stream.mjpg'):
                self.serve_stream()

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
        return self._bearing

    @property
    def distance(self):
        if self._is_shutting_down:
            return None
        return self._distance

    def set_callback(self, callback_function):
        self.user_callback = callback_function
        self.picam2.pre_callback = self._proxy_callback

    def _start_capture(self):
        if self._capture_started:
            return

        self.picam2.start_recording(JpegEncoder(), FileOutput(self.output))
        self._capture_started = True

    def _start_http_server(self):
        if self._server_started:
            return

        address = ("", self.PORT)
        self.server = self.StreamingServer(self, address, self.StreamingHandler)
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        self._server_started = True
        print(f"Stream available at http://localhost:{self.PORT}/stream.mjpg")

    def start(self):
        """Start camera capture so bearing updates in callback."""
        self._start_capture()

    def _proxy_callback(self, request):
        if self._is_shutting_down:
            return
        try:
            with MappedArray(request, "main") as m:
                if self._is_shutting_down:
                    return
                # Convert the image to HSV
                hsv = cv2.cvtColor(m.array, cv2.COLOR_BGR2HSV)

                # Print HSV value at the center of the frame for tuning/debugging.
                # center_y = m.array.shape[0] // 2
                # center_x = m.array.shape[1] // 2
                # print(f"Center HSV: {hsv[center_y, center_x]}")
                #
                # # Draw a small crosshair in the center of the image
                # cv2.line(m.array, (m.array.shape[1] // 2, m.array.shape[0] // 2 - 10), (m.array.shape[1] // 2, m.array.shape[0] // 2 + 10), (0, 0, 255), 2)
                # cv2.line(m.array, (m.array.shape[1] // 2 - 10, m.array.shape[0] // 2), (m.array.shape[1] // 2 + 10, m.array.shape[0] // 2), (0, 0, 255), 2)

                orangeLowerBound = np.array([0, 200, 140])
                orangeUpperBound = np.array([15, 255, 255])

                orangeMask = cv2.inRange(hsv, orangeLowerBound, orangeUpperBound)
                orangeContours, _ = cv2.findContours(orangeMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for contour in orangeContours:
                    area = cv2.contourArea(contour)
                    if area > 200:
                        x, y, w, h = cv2.boundingRect(contour)
                        cv2.drawContours(m.array, [contour], -1, (0, 165, 255), 2)
                # print(redContours)
                # Select the biggest contour with bounding-box area (w*h) less than 10000
                validContours = []
                for c in orangeContours:
                    x_tmp, y_tmp, w_tmp, h_tmp = cv2.boundingRect(c)
                    if w_tmp * h_tmp < 50000:
                        validContours.append(c)
                biggestContour = max(
                    validContours,
                    key=lambda c: (lambda wh: wh[0] * wh[1])(cv2.boundingRect(c)[2:4])
                ) if validContours else None
                if biggestContour is not None:
                    x, y, w, h = cv2.boundingRect(biggestContour)
                    cv2.rectangle(m.array, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    # print(f"Biggest contour area: {w*h}")
                    # A = -220d + 11300
                    # d = (11300 - A) / 220
                    self._distance = (11300 - w*h) / 220
                    biggestContourAreaCentre = (x + w // 2, y + h // 2)

                    # Find the bearing of biggestContourAreaCentre from the centre of the image
                    bearing_rad = math.atan2(biggestContourAreaCentre[1] - m.array.shape[0] // 2, biggestContourAreaCentre[0] - m.array.shape[1] // 2)
                    self._bearing = math.degrees(bearing_rad) - 90
                    print(f"Distance: {self._distance}, Bearing: {self._bearing}")

                    # need to find focal length of camera
                    # bw, fl = 0.42, 0
                    # distance = (bw * fl) / w
                else:
                    self._bearing = None
                    self._distance = None
                # print(f"Bearing: {self._bearing}")

                # Call the user-specified callback with both the original and HSV arrays
                if self.user_callback:
                    self.user_callback(m.array, hsv)
        except Exception as e:
            logging.warning("Camera callback error (may be during shutdown): %s", e)

    async def run_server(self):
        """Async task to run the camera server"""
        self._start_capture()
        self._start_http_server()

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
                self.picam2.stop_recording()
            except Exception as e:
                logging.warning("Error stopping camera recording: %s", e)
            self._capture_started = False
        print("Camera stopped")

async def main():
    camera = Camera(PORT=8000, resolution=(2000, 2000), frame_rate=60)
    await camera.run_server()

if __name__ == "__main__":
    asyncio.run(main())