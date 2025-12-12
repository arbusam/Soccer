#!/usr/bin/env python3
"""
Test script for the LIDAR pybind11 module.

Connects to the LIDAR and prints the distance at angle closest to 0 degrees.
Runs continuously until interrupted.
"""

import time
import lidar

def main():
    print("LIDAR Test Script")
    print("=" * 40)
    
    # Initialize LIDAR
    lidar_port = "/dev/ttyUSB0"
    lidar_baudrate = 460800
    print(f"Initializing LIDAR on {lidar_port} at {lidar_baudrate} baud...")
    try:
        lidar.init(lidar_port, lidar_baudrate)
    except Exception as e:
        print(f"Failed to initialize LIDAR: {e}")
        return
    
    print("LIDAR initialized successfully!")
    print()
    
    # Wait for first scan (use Ctrl+C to stop if needed)
    print("Waiting for first scan data...")
    while not lidar.is_scan_ready():
        time.sleep(0.1)
    
    print("Scan data ready!")
    print()
    
    print("Running (Ctrl+C to stop)")
    print("-" * 40)
    
    # Main loop
    try:
        while True:
            # Get distance at 0 degrees (forward)
            distance_0 = lidar.get_distance_at_angle(0)
            
            # Also get a few other directions for context
            distance_90 = lidar.get_distance_at_angle(90)   # Right
            distance_180 = lidar.get_distance_at_angle(180) # Back
            distance_270 = lidar.get_distance_at_angle(270) # Left
            
            scan_count = lidar.get_scan_count()
            
            # Print in a nice format
            print(f"\rPoints: {scan_count:4d} | "
                  f"0°: {distance_0:7.1f}mm | "
                  f"90°: {distance_90:7.1f}mm | "
                  f"180°: {distance_180:7.1f}mm | "
                  f"270°: {distance_270:7.1f}mm", end="", flush=True)
            
            time.sleep(0.1)  # 10 Hz update rate
            
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    # Cleanup
    print("\nShutting down LIDAR...")
    lidar.shutdown()
    print("Done!")

if __name__ == "__main__":
    main()

