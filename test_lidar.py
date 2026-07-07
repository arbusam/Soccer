#!/usr/bin/env python3
"""
Test script for LIDAR MCL localization (lidar_module).

Connects to the LIDAR, starts localization, and prints the bot's (x, y, yaw).
Runs continuously until interrupted with Ctrl+C.
"""

import sys
import time
import lidar

# Default pitch dimensions in mm (same as defence.py)
PITCH_X = 2430
PITCH_Y = 1820
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 460800


def main():
    print("LIDAR localization test")
    print("=" * 40)
    print(f"Port: {LIDAR_PORT}, baud: {LIDAR_BAUDRATE}")
    print(f"Pitch: {PITCH_X} x {PITCH_Y} mm")
    print()

    try:
        lidar.init(LIDAR_PORT, LIDAR_BAUDRATE)
    except Exception as e:
        print(f"Failed to initialize LIDAR: {e}", file=sys.stderr)
        sys.exit(1)

    print("Waiting for first scan...")
    while not lidar.is_scan_ready():
        time.sleep(0.1)
    print("Scan ready.")

    lidar.start_coordinates(PITCH_X, PITCH_Y)

    print("Waiting for first pose estimate...")
    while not lidar.is_coordinates_ready():
        time.sleep(0.1)
    print("Pose ready. Printing (Ctrl+C to stop):\n")

    try:
        while True:
            x, y, yaw, confidence = lidar.get_pose()
            if x is not None and y is not None and yaw is not None:
                print(f"x = {x:.1f} mm, y = {y:.1f} mm, yaw = {yaw:.1f}°, conf = {confidence:.2f}")
            else:
                print("No confident pose")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nInterrupted by user")

    print("Shutting down LIDAR...")
    lidar.shutdown()
    print("Done.")


if __name__ == "__main__":
    main()
