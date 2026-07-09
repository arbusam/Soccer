import board
import argparse
import time
import select
import sys
import math
import lidar
import switch
import defence
from imu import IMU
from movement import (
    LidarVelocityEstimator,
    MotorCommunicationError,
    MovementController,
    compute_wheel_odometry_trust,
)
from camera import Camera
from kicker import Kicker
from tof import ToF
from enum import Enum

WHEEL_DIAMETER = 50 # mm, used to convert mm/s to RPM
MAX_YAW_RPM = 100 # Maximum rpm that can be added or subtracted from the wheel speeds to correct yaw

TOF_ADDRESS = 0x50
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 460800

MAX_MOTOR_RPM = 1000  # Max translation ~2618 mm/s at 50 mm wheels; driver hardware max ~1984 RPM (~5194 mm/s)
YAW_CORRECT_THRESHOLD = 3 # deg, threshold of allowable yaw error.

CAMERA_PORT = 8000
I2C_ADDRESSES = [28, 32, 31, 30]

BALL_CAPTURED_DISTANCE = 27 # mm, distance from the ToF to the ball to consider it captured
BALL_TIMEOUT = 1 # seconds, time to extrapolate the ball position from velocity without assuming 'lost' state.

class BotMode(Enum):
    DEFENCE = 1
    GOALIE = 2
    STRIKER = 3

bot_mode = BotMode.DEFENCE
mode_switch = switch.Switch(board.D16)
pause_switch = switch.Switch(board.D21)
if mode_switch.read():
    bot_mode = BotMode.GOALIE
else:
    bot_mode = BotMode.DEFENCE
print(bot_mode)

run = False

parser = argparse.ArgumentParser(
    description="Run defence controller with optional live websocket streaming."
)
parser.add_argument(
    "-s",
    "--stream",
    action="store_true",
    help="Enable websocket live log streaming for simulate.py --connect.",
)
args = parser.parse_args()

if args.stream:
    import send_log
    # Start websocket log server in the background (it runs its own asyncio loop).
    send_log.start_server_background()
    time.sleep(0.05)

camera = None
kicker = None
movement_controller = None
imu = None
last_pose_time = None

def enter_pressed():
    if not sys.stdin.isatty():
        return False

    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if not ready:
        return False

    sys.stdin.readline()
    return True


try:
    kicker = Kicker(board.D26, 0.1)
    tof = ToF(address=TOF_ADDRESS)
    print(f"Initializing LIDAR on {LIDAR_PORT} at {LIDAR_BAUDRATE} baud...")
    try:
        lidar.init(LIDAR_PORT, LIDAR_BAUDRATE)
    except Exception as e:
        raise RuntimeError(f"Failed to initialize LIDAR: {e}")

    print("LIDAR initialized successfully!")
    print()

    print("Waiting for first scan data...")
    while not lidar.is_scan_ready():
        if enter_pressed():
            print("Shutdown requested, exiting.")
            raise KeyboardInterrupt
        time.sleep(0.1)

    lidar.start_coordinates(2430, 1820)

    print("Waiting for first pose estimate...")
    while not lidar.is_coordinates_ready():
        if enter_pressed():
            print("Shutdown requested, exiting.")
            raise KeyboardInterrupt
        time.sleep(0.1)

    print("Initializing IMU...")
    imu = IMU()

    camera = Camera(CAMERA_PORT, resolution=(2000, 2000), frame_rate=60)
    camera.start_stream()

    print(f"Initializing motors at I2C addresses: {I2C_ADDRESSES}")
    movement_controller = MovementController.from_i2c_addresses(
        I2C_ADDRESSES,
        WHEEL_DIAMETER,
        MAX_YAW_RPM,
        MAX_MOTOR_RPM,
        YAW_CORRECT_THRESHOLD,
    )
    print("Press Enter to shut down.")
    steering_state = False

    ball_dx = 0
    ball_dy = 0
    last_ball_update = time.time()
    last_ball_x = None
    last_ball_y = None
    last_camera_frame_id = camera.frame_id
    last_pose_time = time.monotonic()
    last_mcl_yaw = None
    lidar_velocity = LidarVelocityEstimator()

    while True:
        if pause_switch.read():
            run = not run
            last_pose_time = time.monotonic()
            while pause_switch.read():
                time.sleep(0.01)
            if not run:
                time.sleep(0.5)
                steering_state = False
        if run:
            if enter_pressed():
                print("Shutdown requested, exiting.")
                break

            now_pose = time.monotonic()
            dt_pose = now_pose - last_pose_time
            last_pose_time = now_pose
            omega = 0.0
            if imu is not None:
                gyro_z = imu.get_gyro_z_deg_s()
                if gyro_z is not None:
                    omega = gyro_z
            vx, vy = 0.0, 0.0
            if movement_controller is not None:
                yaw_for_odom = last_mcl_yaw if last_mcl_yaw is not None else 0.0
                vx_wheel, vy_wheel = movement_controller.get_measured_body_velocity_mm_s(
                    yaw_for_odom
                )
                lidar_vx, lidar_vy = lidar_velocity.get_body_velocity(yaw_for_odom)
                trust = compute_wheel_odometry_trust(
                    vx_wheel,
                    vy_wheel,
                    lidar_vx,
                    lidar_vy,
                    lidar_velocity.is_fresh(now_pose),
                )
                vx = trust * vx_wheel
                vy = trust * vy_wheel
            lidar.predict_odometry(vx, vy, omega, dt_pose)

            x_pos, y_pos, yaw, _confidence = lidar.get_pose()
            if x_pos is not None and y_pos is not None and yaw is not None:
                lidar_velocity.update(x_pos, y_pos, yaw, now_pose)
            if yaw is not None:
                last_mcl_yaw = yaw
            if x_pos is None or y_pos is None or yaw is None:
                time.sleep(0.01)
                continue

            camera_frame_id, ball_direction, ball_distance = camera.get_measurement()
            has_new_camera_frame = camera_frame_id != last_camera_frame_id
            last_camera_frame_id = camera_frame_id
            if has_new_camera_frame:
                ball_x = x_pos + ball_distance * math.cos(math.radians(ball_direction)) if ball_distance is not None and ball_direction is not None else None
                ball_y = y_pos + ball_distance * math.sin(math.radians(ball_direction)) if ball_distance is not None and ball_direction is not None else None
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
            distance_to_ball = tof.read()
            if distance_to_ball is not None and distance_to_ball < BALL_CAPTURED_DISTANCE:
                ball_captured = True
                ball_x = x_pos + 150 * math.cos(math.radians(yaw))
                ball_y = y_pos + 150 * math.sin(math.radians(yaw))
            else:
                ball_captured = False
            if args.stream:
                log_values = [x_pos, y_pos, yaw, ball_x, ball_y]
                send_log.update_latest_log(
                    ",".join("None" if value is None else str(value) for value in log_values)
                )
            if bot_mode == BotMode.DEFENCE:
                direction, speed, rotation, steering_state, kick = defence.defence(
                    x_pos,
                    y_pos,
                    yaw,
                    ball_x,
                    ball_y,
                    ball_captured,
                    steering_state=steering_state,
                    friendly_bot_positions=[],
                    enemy_bot_positions=[],
                )
            else:
                direction, speed, rotation, kick = defence.goalie(
                    x_pos,
                    y_pos,
                    yaw,
                    ball_x,
                    ball_y,
                    ball_captured,
                    friendly_bot_positions=[],
                    enemy_bot_positions=[],
                )
            if kick:
                kicker.kick()
            try:
                movement_controller.move(direction, speed, rotation, 1.0, yaw)
            except MotorCommunicationError as exc:
                print(exc)
                raise
        else:
            if mode_switch.read():
                bot_mode = BotMode.GOALIE
            else:
                bot_mode = BotMode.DEFENCE
            time.sleep(0.01)
            if movement_controller is not None:
                movement_controller.stop()

finally:
    if movement_controller is not None:
        try:
            movement_controller.stop()
        except Exception as exc:
            print(f"Warning: failed to stop motors cleanly: {exc}")
    if kicker is not None:
        try:
            kicker.deinit()
        except Exception as exc:
            print(f"Warning: failed to deinitialize kicker cleanly: {exc}")
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
