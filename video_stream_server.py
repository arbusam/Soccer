#!/usr/bin/env python3
"""
Video streaming server for Raspberry Pi Camera.

Streams live video from the connected Pi camera to a web browser using MJPEG.
Access the stream at http://localhost:5000

Note: picamera2 is typically installed via apt on Raspberry Pi OS:
    sudo apt update
    sudo apt install -y python3-picamera2

Usage: python3 video_stream_server.py
"""

from flask import Flask, Response, render_template_string
from picamera2 import Picamera2
from PIL import Image
import io
import time

app = Flask(__name__)

# Initialize camera
picam2 = Picamera2()
camera_config = picam2.create_video_configuration(
    main={"size": (1280, 720)},  # HD resolution
    lores={"size": (640, 480)},  # Lower resolution for streaming
    controls={"FrameRate": 30}
)
picam2.configure(camera_config)
picam2.start()


def generate_frames():
    """Generator function that yields video frames."""
    while True:
        # Capture frame from camera (using lores stream for better performance)
        frame = picam2.capture_array("lores")
        
        # Convert numpy array to JPEG bytes
        img = Image.fromarray(frame)
        jpeg_buffer = io.BytesIO()
        img.save(jpeg_buffer, format='JPEG', quality=85)
        jpeg_bytes = jpeg_buffer.getvalue()
        
        # Yield frame in MJPEG format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg_bytes + b'\r\n')


@app.route('/')
def index():
    """Serve the main HTML page."""
    html = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Pi Camera Live Stream</title>
        <style>
            body {
                font-family: Arial, sans-serif;
                display: flex;
                flex-direction: column;
                align-items: center;
                justify-content: center;
                min-height: 100vh;
                margin: 0;
                background-color: #1a1a1a;
                color: #ffffff;
            }
            h1 {
                margin-bottom: 20px;
            }
            #video-stream {
                border: 2px solid #333;
                border-radius: 8px;
                box-shadow: 0 4px 8px rgba(0, 0, 0, 0.3);
                max-width: 100%;
                height: auto;
            }
            .info {
                margin-top: 20px;
                padding: 10px;
                background-color: #2a2a2a;
                border-radius: 4px;
                font-size: 14px;
            }
        </style>
    </head>
    <body>
        <h1>Raspberry Pi Camera Live Stream</h1>
        <img id="video-stream" src="/video_feed" alt="Video Stream">
        <div class="info">
            <p>Streaming live video from Raspberry Pi Camera</p>
            <p>Resolution: 1280x720 | Frame Rate: ~30 FPS</p>
        </div>
    </body>
    </html>
    """
    return render_template_string(html)


@app.route('/video_feed')
def video_feed():
    """Video streaming route."""
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


def cleanup():
    """Cleanup camera resources."""
    picam2.stop()


if __name__ == '__main__':
    try:
        print("Starting video streaming server...")
        print("Access the stream at http://localhost:5000")
        print("Press Ctrl+C to stop")
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\nStopping server...")
    finally:
        cleanup()
