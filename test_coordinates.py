"""Print the result of get_coordinates for testing. Requires LIDAR to be connected."""
import sys
import time

from defence import LIDAR_PORT, LIDAR_BAUDRATE


def main():
    import lidar

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

    lidar.start_coordinates(2430, 1820)
    lidar.set_yaw(yaw)

    print("Waiting for coordinate estimate...")
    while not lidar.is_coordinates_ready():
        time.sleep(0.1)

    x_pos, y_pos = lidar.get_coordinates()
    print(f"yaw={yaw}")
    print(f"x_pos={x_pos}")
    print(f"y_pos={y_pos}")

    lidar.shutdown()


if __name__ == "__main__":
    main()
