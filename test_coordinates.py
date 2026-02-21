"""Print the result of get_coordinates for testing. Requires LIDAR to be connected."""
import sys
import time

from defence import get_coordinates, LIDAR_PORT, LIDAR_BAUDRATE


def main():
    import lidar

    # Pass in yaw with command line argument
    # python test_coordinates.py 90
    yaw = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0

    print(f"Initializing LIDAR on {LIDAR_PORT} at {LIDAR_BAUDRATE} baud...")
    try:
        lidar.init(LIDAR_PORT, LIDAR_BAUDRATE)
    except Exception as e:
        print(f"Failed to initialize LIDAR: {e}", file=sys.stderr)
        sys.exit(1)

    print("Waiting for first scan...")
    while not lidar.is_scan_ready():
        time.sleep(0.1)

    x_pos, y_pos = get_coordinates(yaw)
    print(f"yaw={yaw}")
    print(f"x_pos={x_pos}")
    print(f"y_pos={y_pos}")


if __name__ == "__main__":
    main()
