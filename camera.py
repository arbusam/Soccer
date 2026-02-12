import io
import logging
import socketserver
from http import server
from threading import Condition
import cv2
import threading
import asyncio
import math
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
        self.output = self.StreamingOutput()
        self.server = None
        self.user_callback = None
        
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

    def set_callback(self, callback_function):
        self.user_callback = callback_function
        self.picam2.pre_callback = self._proxy_callback

    def _proxy_callback(self, request):
        with MappedArray(request, "main") as m:
            # Convert the image to HSV
            hsv = cv2.cvtColor(m.array, cv2.COLOR_BGR2HSV)

            # # Print the colour of the pixel at the center of the image
            # print(hsv[m.array.shape[0] // 2, m.array.shape[1] // 2])
            
            # # Draw a small crosshair in the center of the image
            # cv2.line(m.array, (m.array.shape[1] // 2, m.array.shape[0] // 2 - 10), (m.array.shape[1] // 2, m.array.shape[0] // 2 + 10), (0, 0, 255), 2)
            # cv2.line(m.array, (m.array.shape[1] // 2 - 10, m.array.shape[0] // 2), (m.array.shape[1] // 2 + 10, m.array.shape[0] // 2), (0, 0, 255), 2)

            redLowerBound = (0, 185, 140)
            redUpperBound = (10, 210, 200)

            redMask = cv2.inRange(hsv, redLowerBound, redUpperBound)
            redContours, _ = cv2.findContours(redMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in redContours:
                area = cv2.contourArea(contour)
                if area > 200:
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.drawContours(m.array, [contour], -1, (0, 0, 255), 2)
            # print(redContours)
            biggestContour = max(redContours, key=cv2.contourArea) if redContours else None
            if biggestContour is not None:
                x, y, w, h = cv2.boundingRect(biggestContour)
                cv2.rectangle(m.array, (x, y), (x + w, y + h), (0, 0, 255), 2)
            
                biggestContourAreaCentre = (x + w // 2, y + h // 2)
                
                # Find the bearing of biggestContourAreaCentre from the centre of the image
                bearing = math.atan2(biggestContourAreaCentre[1] - m.array.shape[0] // 2, biggestContourAreaCentre[0] - m.array.shape[1] // 2)
                bearing = math.degrees(bearing)
                print(bearing)
                

            # Call the user-specified callback with both the original and HSV arrays
            if self.user_callback:
                self.user_callback(m.array, hsv)

    async def run_server(self):
        """Async task to run the camera server"""
        self.picam2.start_recording(JpegEncoder(), FileOutput(self.output))
        address = ('', self.PORT)
        self.server = self.StreamingServer(self, address, self.StreamingHandler)
        print(f"Stream available at http://localhost:{self.PORT}/stream.mjpg")
        
        # Run server in a thread since it's blocking
        server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        server_thread.start()
        
        try:
            # Keep the task alive
            while True:
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            self.stop()

    def stop(self):
        if self.server:
            self.server.shutdown()
            self.server.server_close()
        self.picam2.stop_recording()
        print("Camera server stopped")

async def main():
    camera = Camera(PORT=8000, resolution=(640, 420), frame_rate=60)
    await camera.run_server()

if __name__ == "__main__":
    asyncio.run(main())