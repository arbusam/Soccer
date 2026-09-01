"""Stream real measured world state for ``simulate.py --connect``."""

import math
import select
import sys
import time

from lib import lidar, send_log
from lib.camera import Camera
from lib.imu import IMU
from striker import BALL_TIMEOUT, CAMERA_PORT, LIDAR_BAUDRATE, LIDAR_PORT

PORT = send_log.PORT
PITCH_WIDTH = 2430
PITCH_HEIGHT = 1820


def imu_yaw_to_relative_yaw(imu_yaw, startup_yaw):
    """Match the startup-relative yaw conversion used by defence.py."""
    return ((float(startup_yaw) - float(imu_yaw) + 180.0) % 360.0) - 180.0


def _enter_pressed():
    if not sys.stdin.isatty():
        return False
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if not ready:
        return False
    sys.stdin.readline()
    return True


def capture_startup_yaw(imu, sample_count=25, sample_interval=0.02):
    print("Stabilizing IMU yaw reference...")
    sin_sum = 0.0
    cos_sum = 0.0
    samples = 0
    while samples < sample_count:
        yaw_sample = imu.get_yaw()
        if yaw_sample is not None:
            yaw_rad = math.radians(yaw_sample)
            sin_sum += math.sin(yaw_rad)
            cos_sum += math.cos(yaw_rad)
            samples += 1
        time.sleep(sample_interval)
    return math.degrees(math.atan2(sin_sum, cos_sum))


def format_log_line(x_pos, y_pos, yaw_relative, ball_x, ball_y, other_bot_positions):
    values = [x_pos, y_pos, yaw_relative, ball_x, ball_y]
    for other_x, other_y in other_bot_positions:
        values.extend((other_x, other_y))
    return ",".join("None" if value is None else str(value) for value in values)


def main():
    camera = None
    imu = None

    send_log.start_server_background()
    time.sleep(0.05)
    print(f"Websocket log server running on ws://0.0.0.0:{PORT}")
    print(f"Connect with: python simulate.py --connect 127.0.0.1:{PORT}")

    try:
        print(f"Initializing LIDAR on {LIDAR_PORT} at {LIDAR_BAUDRATE} baud...")
        try:
            lidar.init(LIDAR_PORT, LIDAR_BAUDRATE)
        except Exception as exc:
            raise RuntimeError(f"Failed to initialize LIDAR: {exc}") from exc

        print("Waiting for first scan data...")
        while not lidar.is_scan_ready():
            if _enter_pressed():
                print("Shutdown requested, exiting.")
                return
            time.sleep(0.1)

        camera = Camera(CAMERA_PORT, resolution=(2000, 2000), frame_rate=90)
        camera.start()
        imu = IMU()

        startup_yaw = capture_startup_yaw(imu)
        print(f"Startup yaw reference set to {startup_yaw:.6f} deg")
        yaw_world = imu.get_yaw()
        if yaw_world is not None:
            lidar.set_imu_yaw(imu_yaw_to_relative_yaw(yaw_world, startup_yaw))

        lidar.start_coordinates(PITCH_WIDTH, PITCH_HEIGHT)

        print("Waiting for first coordinate estimate...")
        while not lidar.is_coordinates_ready():
            if _enter_pressed():
                print("Shutdown requested, exiting.")
                return
            yaw_world = imu.get_yaw()
            if yaw_world is not None:
                lidar.set_imu_yaw(imu_yaw_to_relative_yaw(yaw_world, startup_yaw))
            time.sleep(0.1)

        print("Streaming measured positions. Press Enter to shut down.")

        ball_dx = 0.0
        ball_dy = 0.0
        last_ball_update = time.time()
        last_ball_x = None
        last_ball_y = None
        last_camera_frame_id = camera.frame_id

        while True:
            if _enter_pressed():
                print("Shutdown requested, exiting.")
                break

            yaw_world = imu.get_yaw()
            if yaw_world is None:
                time.sleep(0.01)
                continue

            yaw_relative = imu_yaw_to_relative_yaw(yaw_world, startup_yaw)
            lidar.set_imu_yaw(yaw_relative)
            lidar.predict_odometry(0.0, 0.0, 0.0, 0.01)

            x_pos, y_pos, mcl_yaw, _confidence = lidar.get_pose()
            if mcl_yaw is not None:
                yaw_relative = mcl_yaw
            elif x_pos is None or y_pos is None:
                time.sleep(0.01)
                continue

            camera_frame_id, ball_direction, ball_distance = camera.get_measurement()
            has_new_camera_frame = camera_frame_id != last_camera_frame_id
            last_camera_frame_id = camera_frame_id

            if has_new_camera_frame:
                if (
                    ball_distance is not None
                    and ball_direction is not None
                    and x_pos is not None
                    and y_pos is not None
                ):
                    ball_x = x_pos + ball_distance * math.cos(math.radians(ball_direction))
                    ball_y = y_pos + ball_distance * math.sin(math.radians(ball_direction))
                else:
                    ball_x = None
                    ball_y = None
            else:
                ball_x = None
                ball_y = None

            now = time.time()
            if ball_x is not None and ball_y is not None:
                dt = now - last_ball_update
                if last_ball_x is not None and last_ball_y is not None and dt > 0:
                    ball_dx = (ball_x - last_ball_x) / dt
                    ball_dy = (ball_y - last_ball_y) / dt
                last_ball_x = ball_x
                last_ball_y = ball_y
                last_ball_update = now
            elif (
                last_ball_x is not None
                and last_ball_y is not None
                and now - last_ball_update < BALL_TIMEOUT
            ):
                dt_lost = now - last_ball_update
                ball_x = last_ball_x + ball_dx * dt_lost
                ball_y = last_ball_y + ball_dy * dt_lost
            else:
                ball_x = None
                ball_y = None

            send_log.update_latest_log(
                format_log_line(
                    x_pos,
                    y_pos,
                    yaw_relative,
                    ball_x,
                    ball_y,
                    [],
                )
            )
            time.sleep(0.01)
    finally:
        if camera is not None:
            try:
                camera.stop()
            except Exception as exc:
                print(f"Warning: failed to stop camera cleanly: {exc}")
        if imu is not None:
            try:
                imu.close()
            except Exception as exc:
                print(f"Warning: failed to close IMU cleanly: {exc}")
        try:
            lidar.shutdown()
        except Exception as exc:
            print(f"Warning: failed to shut down lidar cleanly: {exc}")


if __name__ == "__main__":
    main()
