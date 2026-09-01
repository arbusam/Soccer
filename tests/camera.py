#!/usr/bin/env python3
"""Print the ball bearing and distance reported by lib/camera.py."""

import argparse
import time

from lib.camera import Camera


def parse_args():
    parser = argparse.ArgumentParser(
        description="Start the camera and print ball bearing/distance measurements."
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8000,
        help="HTTP stream port passed to Camera (default: 8000).",
    )
    parser.add_argument(
        "--resolution",
        nargs=2,
        type=int,
        metavar=("WIDTH", "HEIGHT"),
        default=(2000, 2000),
        help="Camera resolution as WIDTH HEIGHT (default: 2000 2000).",
    )
    parser.add_argument(
        "--frame-rate",
        type=int,
        default=90,
        help="Camera frame rate (default: 90).",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.05,
        help="Polling interval in seconds (default: 0.05).",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    resolution = tuple(args.resolution)

    camera = Camera(
        PORT=args.port,
        resolution=resolution,
        frame_rate=args.frame_rate,
    )

    print("Starting camera...")
    print(f"Resolution: {resolution[0]} x {resolution[1]}")
    print(f"Frame rate: {args.frame_rate} FPS")
    print("Printing ball measurements (Ctrl+C to stop):\n")

    camera.start()
    last_frame_id = camera.frame_id

    try:
        while True:
            frame_id, ball_angle, ball_distance = camera.get_measurement()
            if frame_id != last_frame_id:
                last_frame_id = frame_id
                if ball_angle is None or ball_distance is None:
                    print("Ball: not detected")
                else:
                    print(
                        f"Ball angle: {ball_angle:.2f} deg, "
                        f"distance: {ball_distance:.1f} mm"
                    )
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        camera.stop()


if __name__ == "__main__":
    main()
