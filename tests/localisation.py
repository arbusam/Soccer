#!/usr/bin/env python3
"""Drive to user-specified field coordinates using LIDAR MCL pose feedback.

Interactive test: enter a target (x, y) in mm and the robot drives there while
printing its localized position. Requires LIDAR, IMU, and motors.

With ``--no-move``, motors are skipped and the script only prints / streams the
localized pose (still needs LIDAR and IMU).

With ``-s`` / ``--stream``, poses are pushed to the websocket log server for live
viewing with ``python simulate.py --connect 127.0.0.1:8765``.
"""

import argparse
import math
import queue
import threading
import time

from lib import lidar
from lib.imu import IMU
from lib.movement import (
    LidarVelocityEstimator,
    MotorCommunicationError,
    MovementController,
    compute_wheel_odometry_trust,
    imu_yaw_to_relative_yaw,
)

TARGET_TOLERANCE_MM = 10
MAX_SPEED_MM_S = 500
SLOW_RADIUS_MM = 300
LOOP_DELAY_SECONDS = 0.02
STATUS_PRINT_INTERVAL_S = 0.2

WHEEL_DIAMETER = 50
MAX_YAW_RPM = 100
MAX_MOTOR_RPM = 400
YAW_CORRECT_THRESHOLD = 3

LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 460800
PITCH_X = 2430
PITCH_Y = 1820
I2C_ADDRESSES = [31,29,32,28]


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Drive to user-specified field coordinates using LIDAR MCL pose feedback."
        )
    )
    parser.add_argument(
        "-s",
        "--stream",
        action="store_true",
        help="Enable websocket live pose streaming for simulate.py --connect.",
    )
    parser.add_argument(
        "--no-move",
        action="store_true",
        help="Disable motors; only print/stream localized pose.",
    )
    return parser.parse_args()


def format_pose_log_line(x_pos, y_pos, yaw):
    """CSV line matching simulate.py parse_log_frame (x, y, yaw, ball_x, ball_y)."""
    values = [x_pos, y_pos, yaw, None, None]
    return ",".join("None" if value is None else str(value) for value in values)


def stream_pose(stream_enabled, send_log_module, x_pos, y_pos, yaw):
    if not stream_enabled or send_log_module is None:
        return
    if x_pos is None or y_pos is None or yaw is None:
        return
    send_log_module.update_latest_log(format_pose_log_line(x_pos, y_pos, yaw))


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


def print_localisation_status(lidar_module):
    """Print pose, confidence, and whether MCL is currently using LIDAR scans."""
    x_pos, y_pos, yaw, confidence, ok = lidar_module.get_coordinates_info()
    lidar_used = lidar_module.scan_updates_enabled()
    lidar_label = "LIDAR" if lidar_used else "odom-only"
    if ok and x_pos is not None and y_pos is not None and yaw is not None:
        print(
            f"pose=({x_pos:.1f}, {y_pos:.1f}) yaw={yaw:.1f} deg "
            f"confidence={confidence:.2f} source={lidar_label}"
        )
    else:
        print(
            f"pose=unavailable confidence={confidence:.2f} source={lidar_label}"
        )


def print_scan_correction_if_new(lidar_module, last_sequence):
    """If a new LIDAR scan update landed, print odometry vs LIDAR correction error.

    When confidence drops below the tracking threshold, MCL stops recording a
    correction, so also print a one-line collapse notice once per scan.

    Returns the latest correction sequence number.
    """
    (
        sequence,
        pred_x,
        pred_y,
        pred_yaw,
        corr_x,
        corr_y,
        corr_yaw,
        error_mm,
        yaw_error_deg,
        valid,
    ) = lidar_module.get_last_scan_correction()
    _x, _y, _yaw, confidence, ok = lidar_module.get_coordinates_info()
    generation = lidar_module.get_scan_generation()
    last_collapse_generation = getattr(
        print_scan_correction_if_new, "_collapse_generation", 0
    )

    if not ok and generation != last_collapse_generation:
        recovery = lidar_module.get_recovery_status()
        scan_quality, baseline, bad_scans, global_fraction, _valid = recovery
        confidence_s = "n/a" if confidence is None else f"{confidence:.2f}"
        print(
            f"scan#{generation} confidence collapsed "
            f"confidence={confidence_s} quality={scan_quality:.2f}/{baseline:.2f} "
            f"bad_scans={bad_scans} global={100.0 * global_fraction:.0f}%"
        )
        print_scan_correction_if_new._collapse_generation = generation

    if not valid or sequence == last_sequence:
        return last_sequence

    recovery = lidar_module.get_recovery_status()
    scan_quality, baseline, bad_scans, global_fraction, _valid = recovery
    confidence_s = "n/a" if confidence is None else f"{confidence:.2f}"
    print(
        f"scan#{sequence} odom_error={error_mm:.1f} mm "
        f"yaw_error={yaw_error_deg:+.1f} deg confidence={confidence_s} "
        f"quality={scan_quality:.2f}/{baseline:.2f} "
        f"bad_scans={bad_scans} global={100.0 * global_fraction:.0f}% | "
        f"odom=({pred_x:.1f}, {pred_y:.1f}, {pred_yaw:.1f}) -> "
        f"lidar=({corr_x:.1f}, {corr_y:.1f}, {corr_yaw:.1f})"
    )
    return sequence


def feed_imu_yaw_prior(lidar_module, imu, startup_yaw):
    """Push startup-relative IMU yaw into MCL as a soft heading prior."""
    imu_yaw = imu.get_yaw()
    if imu_yaw is None:
        return
    lidar_module.set_imu_yaw(imu_yaw_to_relative_yaw(imu_yaw, startup_yaw))


def predict_odometry(
    lidar_module,
    movement_controller,
    imu,
    startup_yaw,
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
    feed_imu_yaw_prior(lidar_module, imu, startup_yaw)

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
    last_scan_sequence=0,
    stream_enabled=False,
    send_log_module=None,
):
    """Drive toward (target_x, target_y) using localized pose feedback."""
    # last_status_print = 0.0
    while True:
        now = time.monotonic()
        yaw_for_odom = last_mcl_yaw if last_mcl_yaw is not None else 0.0
        last_pose_time = predict_odometry(
            lidar_module,
            movement_controller,
            imu,
            startup_yaw,
            lidar_velocity,
            yaw_for_odom,
            last_pose_time,
        )
        last_scan_sequence = print_scan_correction_if_new(
            lidar_module, last_scan_sequence
        )

        # if now - last_status_print >= STATUS_PRINT_INTERVAL_S:
        #     print_localisation_status(lidar_module)
        #     last_status_print = now

        pose = get_position(lidar_module)
        if pose is None:
            # Pause safely without terminating the controller's drive thread.
            movement_controller.move(
                0.0, 0.0, yaw_for_odom, 0.0, yaw_for_odom
            )
            time.sleep(LOOP_DELAY_SECONDS)
            continue

        current_x, current_y, mcl_yaw = pose
        yaw = get_yaw(imu, startup_yaw, mcl_yaw)
        if yaw is None:
            # Pause safely without terminating the controller's drive thread.
            movement_controller.move(
                0.0, 0.0, yaw_for_odom, 0.0, yaw_for_odom
            )
            time.sleep(LOOP_DELAY_SECONDS)
            continue

        last_mcl_yaw = mcl_yaw
        lidar_velocity.update(current_x, current_y, yaw, now)
        stream_pose(stream_enabled, send_log_module, current_x, current_y, yaw)

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
    return last_pose_time, last_mcl_yaw, last_scan_sequence


def _read_target_coordinates(result_queue):
    """Blocking stdin prompt; runs on a worker thread so MCL can keep updating."""
    try:
        target_x = float(input("What x position to move to? "))
        target_y = float(input("What y position to move to? "))
        result_queue.put(("ok", target_x, target_y))
    except ValueError as exc:
        result_queue.put(("error", exc))
    except EOFError as exc:
        result_queue.put(("error", exc))


def wait_for_target_while_localising(
    lidar_module,
    imu,
    startup_yaw,
    lidar_velocity,
    last_pose_time,
    last_mcl_yaw,
    last_scan_sequence=0,
    stream_enabled=False,
    send_log_module=None,
    movement_controller=None,
):
    """Keep feeding MCL until the user enters a target (x, y).

    ``input()`` blocks the calling thread, which would otherwise pause IMU/odom
    updates and stall particle-filter convergence. Prompt on a side thread and
    keep running the usual localisation loop here until a target arrives.
    """
    result_queue = queue.Queue()
    prompt_thread = threading.Thread(
        target=_read_target_coordinates,
        args=(result_queue,),
        daemon=True,
    )
    prompt_thread.start()

    # last_status_print = 0.0
    while True:
        try:
            status, *payload = result_queue.get_nowait()
        except queue.Empty:
            status = None

        if status == "ok":
            target_x, target_y = payload
            return (
                target_x,
                target_y,
                last_pose_time,
                last_mcl_yaw,
                last_scan_sequence,
            )
        if status == "error":
            raise payload[0]

        now = time.monotonic()
        yaw_for_odom = last_mcl_yaw if last_mcl_yaw is not None else 0.0
        last_pose_time = predict_odometry(
            lidar_module,
            movement_controller,
            imu,
            startup_yaw,
            lidar_velocity,
            yaw_for_odom,
            last_pose_time,
        )
        last_scan_sequence = print_scan_correction_if_new(
            lidar_module, last_scan_sequence
        )

        # if now - last_status_print >= STATUS_PRINT_INTERVAL_S:
        #     print_localisation_status(lidar_module)
        #     last_status_print = now

        pose = get_position(lidar_module)
        if pose is not None:
            current_x, current_y, mcl_yaw = pose
            yaw = get_yaw(imu, startup_yaw, mcl_yaw)
            if yaw is not None:
                last_mcl_yaw = mcl_yaw
                lidar_velocity.update(current_x, current_y, yaw, now)
                stream_pose(
                    stream_enabled, send_log_module, current_x, current_y, yaw
                )

        time.sleep(LOOP_DELAY_SECONDS)


def monitor_pose(
    lidar_module,
    imu,
    startup_yaw,
    lidar_velocity,
    last_pose_time,
    last_mcl_yaw,
    last_scan_sequence=0,
    stream_enabled=False,
    send_log_module=None,
):
    """Print and optionally stream localized pose without commanding motors."""
    # last_status_print = 0.0
    while True:
        now = time.monotonic()
        yaw_for_odom = last_mcl_yaw if last_mcl_yaw is not None else 0.0
        last_pose_time = predict_odometry(
            lidar_module,
            None,
            imu,
            startup_yaw,
            lidar_velocity,
            yaw_for_odom,
            last_pose_time,
        )
        last_scan_sequence = print_scan_correction_if_new(
            lidar_module, last_scan_sequence
        )

        # if now - last_status_print >= STATUS_PRINT_INTERVAL_S:
        #     print_localisation_status(lidar_module)
        #     last_status_print = now

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
        stream_pose(stream_enabled, send_log_module, current_x, current_y, yaw)
        time.sleep(LOOP_DELAY_SECONDS)


def main():
    args = parse_args()
    send_log_module = None
    if args.stream:
        from lib import send_log

        send_log_module = send_log
        send_log.start_server_background()
        time.sleep(0.05)
        print(f"Websocket log server running on ws://0.0.0.0:{send_log.PORT}")
        print(f"Connect with: python simulate.py --connect 127.0.0.1:{send_log.PORT}")

    imu = None
    movement_controller = None
    try:
        print(f"Initializing LIDAR on {LIDAR_PORT} at {LIDAR_BAUDRATE} baud...")
        lidar.init(LIDAR_PORT, LIDAR_BAUDRATE)

        print("Waiting for first scan...")
        while not lidar.is_scan_ready():
            time.sleep(0.1)

        print("Initializing IMU...")
        imu = IMU()
        startup_yaw = capture_startup_yaw(imu)
        print(f"Startup yaw reference set to {startup_yaw:.1f} deg")
        feed_imu_yaw_prior(lidar, imu, startup_yaw)

        lidar.start_coordinates(PITCH_X, PITCH_Y)

        print("Waiting for first pose estimate...")
        last_pose_time = time.monotonic()
        # last_status_print = 0.0
        while not lidar.is_coordinates_ready():
            now = time.monotonic()
            omega = 0.0
            gyro_z = imu.get_gyro_z_deg_s()
            if gyro_z is not None:
                omega = gyro_z
            feed_imu_yaw_prior(lidar, imu, startup_yaw)
            # Zero translation still applies process noise so particles can explore
            # after resampling; without this the filter often never reaches confidence.
            lidar.predict_odometry(0.0, 0.0, omega, now - last_pose_time)
            last_pose_time = now
            # if now - last_status_print >= STATUS_PRINT_INTERVAL_S:
            #     print_localisation_status(lidar)
            #     print(f"  scan_points={lidar.get_scan_count()}")
            #     last_status_print = now
            time.sleep(0.1)

        lidar_velocity = LidarVelocityEstimator()
        last_mcl_yaw = None
        last_scan_sequence = 0

        if args.no_move:
            print("Movement disabled (--no-move). Monitoring pose. Press Ctrl+C to quit.")
            monitor_pose(
                lidar,
                imu,
                startup_yaw,
                lidar_velocity,
                last_pose_time,
                last_mcl_yaw,
                last_scan_sequence,
                stream_enabled=args.stream,
                send_log_module=send_log_module,
            )
        else:
            print(f"Initializing motors at I2C addresses: {I2C_ADDRESSES}")
            movement_controller = MovementController.from_i2c_addresses(
                I2C_ADDRESSES,
                WHEEL_DIAMETER,
                MAX_YAW_RPM,
                MAX_MOTOR_RPM,
                YAW_CORRECT_THRESHOLD,
            )

            print(
                "Enter target coordinates in mm. Localisation keeps running "
                "while you type. Press Ctrl+C to quit."
            )
            while True:
                (
                    target_x,
                    target_y,
                    last_pose_time,
                    last_mcl_yaw,
                    last_scan_sequence,
                ) = wait_for_target_while_localising(
                    lidar,
                    imu,
                    startup_yaw,
                    lidar_velocity,
                    last_pose_time,
                    last_mcl_yaw,
                    last_scan_sequence,
                    stream_enabled=args.stream,
                    send_log_module=send_log_module,
                    movement_controller=movement_controller,
                )

                # print_localisation_status(lidar)
                pose = get_position(lidar)
                if pose is not None:
                    stream_pose(args.stream, send_log_module, pose[0], pose[1], pose[2])

                last_pose_time, last_mcl_yaw, last_scan_sequence = drive_to_target(
                    target_x,
                    target_y,
                    lidar,
                    movement_controller,
                    imu,
                    startup_yaw,
                    lidar_velocity,
                    last_pose_time,
                    last_mcl_yaw,
                    last_scan_sequence,
                    stream_enabled=args.stream,
                    send_log_module=send_log_module,
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
