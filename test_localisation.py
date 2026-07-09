#!/usr/bin/env python3
"""Drive to user-specified field coordinates using LIDAR MCL pose feedback.

Interactive test: enter a target (x, y) in mm and the robot drives there while
printing its localized position. Requires LIDAR, IMU, and motors.
"""

import math
import time

import lidar
from imu import IMU
from movement import (
    LidarVelocityEstimator,
    MotorCommunicationError,
    MovementController,
    compute_wheel_odometry_trust,
    imu_yaw_to_relative_yaw,
)

TARGET_TOLERANCE_MM = 10
MAX_SPEED_MM_S = 600
SLOW_RADIUS_MM = 300
LOOP_DELAY_SECONDS = 0.02

WHEEL_DIAMETER = 50
MAX_YAW_RPM = 100
MAX_MOTOR_RPM = 400
YAW_CORRECT_THRESHOLD = 3

LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 460800
PITCH_X = 2430
PITCH_Y = 1820
I2C_ADDRESSES = [28, 32, 31, 30]


def capture_startup_yaw(imu, sample_count=25, sample_interval=0.02):
    """Average a short burst of IMU samples so startup yaw is not just the first reading."""
    print("Stabilizing IMU yaw reference...")
    sin_sum = 0.0
    cos_sum = 0.0
    samples = 0
    while samples < sample_count:
        yaw = imu.get_yaw()
        if yaw is not None:
            yaw_rad = math.radians(yaw)
            sin_sum += math.sin(yaw_rad)
            cos_sum += math.cos(yaw_rad)
            samples += 1
        time.sleep(sample_interval)
    return math.degrees(math.atan2(sin_sum, cos_sum))


def get_yaw(imu, startup_yaw, mcl_yaw):
    """Prefer MCL yaw when available, otherwise use startup-relative IMU yaw."""
    if mcl_yaw is not None:
        return mcl_yaw
    imu_yaw = imu.get_yaw()
    if imu_yaw is None:
        return None
    return imu_yaw_to_relative_yaw(imu_yaw, startup_yaw)


def get_position(lidar_module):
    """Return the latest confident (x, y, yaw) pose, or None if unavailable."""
    x_pos, y_pos, yaw, _confidence = lidar_module.get_pose()
    if x_pos is None or y_pos is None or yaw is None:
        return None
    return x_pos, y_pos, yaw


def predict_odometry(
    lidar_module,
    movement_controller,
    imu,
    lidar_velocity,
    yaw_deg,
    last_pose_time,
):
    """Feed wheel and gyro measurements into the MCL motion model."""
    now = time.monotonic()
    dt = now - last_pose_time
    omega = 0.0
    gyro_z = imu.get_gyro_z_deg_s()
    if gyro_z is not None:
        omega = gyro_z

    vx, vy = 0.0, 0.0
    if movement_controller is not None and yaw_deg is not None:
        vx_wheel, vy_wheel = movement_controller.get_measured_body_velocity_mm_s(yaw_deg)
        lidar_vx, lidar_vy = lidar_velocity.get_body_velocity(yaw_deg)
        trust = compute_wheel_odometry_trust(
            vx_wheel,
            vy_wheel,
            lidar_vx,
            lidar_vy,
            lidar_velocity.is_fresh(now),
        )
        vx = trust * vx_wheel
        vy = trust * vy_wheel

    lidar_module.predict_odometry(vx, vy, omega, dt)
    return now


def drive_to_target(
    target_x,
    target_y,
    lidar_module,
    movement_controller,
    imu,
    startup_yaw,
    lidar_velocity,
    last_pose_time,
    last_mcl_yaw,
):
    """Drive toward (target_x, target_y) using localized pose feedback."""
    while True:
        now = time.monotonic()
        yaw_for_odom = last_mcl_yaw if last_mcl_yaw is not None else 0.0
        last_pose_time = predict_odometry(
            lidar_module,
            movement_controller,
            imu,
            lidar_velocity,
            yaw_for_odom,
            last_pose_time,
        )

        pose = get_position(lidar_module)
        if pose is None:
            time.sleep(LOOP_DELAY_SECONDS)
            continue

        current_x, current_y, mcl_yaw = pose
        yaw = get_yaw(imu, startup_yaw, mcl_yaw)
        if yaw is None:
            time.sleep(LOOP_DELAY_SECONDS)
            continue

        last_mcl_yaw = mcl_yaw
        lidar_velocity.update(current_x, current_y, yaw, now)

        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.hypot(dx, dy)
        if distance <= TARGET_TOLERANCE_MM:
            break

        direction = math.degrees(math.atan2(dy, dx))
        speed = min(MAX_SPEED_MM_S, distance / SLOW_RADIUS_MM * MAX_SPEED_MM_S)
        movement_controller.move(direction, speed, yaw, 1.0, yaw)
        time.sleep(LOOP_DELAY_SECONDS)

    movement_controller.stop()
    return last_pose_time, last_mcl_yaw


def main():
    imu = None
    movement_controller = None
    try:
        print(f"Initializing LIDAR on {LIDAR_PORT} at {LIDAR_BAUDRATE} baud...")
        lidar.init(LIDAR_PORT, LIDAR_BAUDRATE)

        print("Waiting for first scan...")
        while not lidar.is_scan_ready():
            time.sleep(0.1)

        lidar.start_coordinates(PITCH_X, PITCH_Y)

        print("Waiting for first pose estimate...")
        while not lidar.is_coordinates_ready():
            time.sleep(0.1)

        print("Initializing IMU...")
        imu = IMU()
        startup_yaw = capture_startup_yaw(imu)
        print(f"Startup yaw reference set to {startup_yaw:.1f} deg")

        print(f"Initializing motors at I2C addresses: {I2C_ADDRESSES}")
        movement_controller = MovementController.from_i2c_addresses(
            I2C_ADDRESSES,
            WHEEL_DIAMETER,
            MAX_YAW_RPM,
            MAX_MOTOR_RPM,
            YAW_CORRECT_THRESHOLD,
        )

        lidar_velocity = LidarVelocityEstimator()
        last_pose_time = time.monotonic()
        last_mcl_yaw = None
        print("Enter target coordinates in mm. Press Ctrl+C to quit.")

        while True:
            target_x = float(input("What x position to move to? "))
            target_y = float(input("What y position to move to? "))

            pose = get_position(lidar)
            if pose is not None:
                print(f"Current position: ({pose[0]:.1f}, {pose[1]:.1f}), yaw={pose[2]:.1f} deg")
            else:
                print("Current position: unavailable")

            last_pose_time, last_mcl_yaw = drive_to_target(
                target_x,
                target_y,
                lidar,
                movement_controller,
                imu,
                startup_yaw,
                lidar_velocity,
                last_pose_time,
                last_mcl_yaw,
            )
            print(f"Reached target ({target_x:.1f}, {target_y:.1f})")
    except KeyboardInterrupt:
        print("\nStopping test.")
    except MotorCommunicationError as exc:
        print(exc)
        raise
    finally:
        if movement_controller is not None:
            movement_controller.stop()
        if imu is not None:
            imu.close()
        try:
            lidar.shutdown()
        except Exception as exc:
            print(f"Warning: failed to shut down lidar cleanly: {exc}")


if __name__ == "__main__":
    main()
