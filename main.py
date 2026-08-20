import argparse
import math
import select
import sys
import threading
import time
from enum import Enum
from pathlib import Path

import board
import lidar

import defence
import striker
import switch
from break_beam import Breakbeam
from camera import Camera
from communication import Peer
from imu import IMU
from kicker import Kicker
from movement import (
    LidarVelocityEstimator,
    MotorCommunicationError,
    MovementController,
    compute_wheel_odometry_trust,
    imu_yaw_to_relative_yaw,
)
from recording_session import RecordingSession

LOG_FPS = 30 # How often the bot state is written to the log file
FPS_REPORT_INTERVAL = 1.0 # seconds, how often the FPS is printed to the console when --fps is used
PEER_PORT = 5005
ENABLE_COMMUNICATION = False # Use communication.py to communicate between bots.
USE_PAUSE = False # Whether to pause the bot when the pause switch is pressed. Set to False for debugging.

WHEEL_DIAMETER = 50 # mm, used to convert between motor RPM and robot mm/s
MAX_YAW_RPM = 100 # Maximum rpm that can be added or subtracted from the wheel speeds to correct yaw

LIDAR_PORT = "/dev/ttyUSB0" # LIDAR port. Usually "/dev/ttyUSB0"
LIDAR_BAUDRATE = 460800

MAX_MOTOR_RPM = 1000  # This converts to a maximum linear translation of ~2618 mm/s with 50 mm wheels; driver hardware max is ~1984 RPM (~5194 mm/s)
YAW_CORRECT_THRESHOLD = 3 # deg, threshold of allowable yaw error.

CAMERA_PORT = 8000 # Port used for streaming the camera feed for debugging.
CAMERA_RESOLUTION = (640, 640)
CAMERA_FPS = 90

BALL_TIMEOUT = 0.5 # seconds, maximum time for which the ball position can be extrapolated from velocity without assuming 'lost' state.

CONFIG_PATH = Path(__file__).resolve().parent / "config.txt" # Path to the config file (see example_config.txt)

# Enum for bot mode. Depending on game state, the bot can change between these modes.
# Each bot includes a mode switch, which can set the default mode to two of these, depending on the switch's position.
# These default modes can be set in the config file.
class BotMode(Enum):
    DEFENCE = 1
    GOALIE = 2
    STRIKER = 3


def load_config(path: Path = CONFIG_PATH) -> tuple[list[int], BotMode, BotMode]:
    # Load all settings from the config file

    # If the config file is not found, raise an error
    if not path.is_file():
        raise FileNotFoundError(
            f"Missing {path.name}. Copy example_config.txt to config.txt and edit as needed."
        )

    # Parse the config file into a dictionary following these rules:
    # - Lines starting with # are ignored as comments
    # - Any key-value pair must be in the format "key=value"
    values: dict[str, str] = {}
    with path.open(encoding="utf-8") as config_file:
        for raw_line in config_file:
            line = raw_line.split("#", 1)[0].strip()
            if not line or "=" not in line:
                continue
            key, value = line.split("=", 1)
            values[key.strip().lower()] = value.strip()

    # If included in the config file, parse the motor I2C addresses as a comma-separated list of integers
    # Motor I2C addresses follow the order: back left, back right, front right, front left, dribbler (optional)
    try:
        i2c_addresses = [int(part.strip()) for part in values["i2c_addresses"].split(",")]
    except (KeyError, ValueError) as exc:
        raise ValueError(
            f"{path.name}: i2c_addresses must be a comma-separated list of integers"
        ) from exc

    valid_modes = ", ".join(mode.name for mode in BotMode) # Generate a comma-separated string of the bot modes for error messages``

    def parse_mode(key: str) -> BotMode:
        # Parse the mode from the config file for a given key into a BotMode enum value
        try:
            return BotMode[values[key].upper()]
        except KeyError as exc:
            raise ValueError(
                f"{path.name}: {key} must be one of: {valid_modes}"
            ) from exc

    return i2c_addresses, parse_mode("mode_switch_off"), parse_mode("mode_switch_on") # Return the motor I2C addresses and the two default modes for the mode switches

# Load the config into the global variables
I2C_ADDRESSES, MODE_SWITCH_OFF, MODE_SWITCH_ON = load_config()

# Initialize the mode switch and pause switch
# TODO: Make these constants at the top
mode_switch = switch.Switch(board.D16)
pause_switch = switch.Switch(board.D21)
bot_mode = MODE_SWITCH_ON if mode_switch.read() else MODE_SWITCH_OFF

run = not USE_PAUSE

parser = argparse.ArgumentParser(
    description="Run defence controller with optional live streaming and log recording."
)
parser.add_argument(
    "-s",
    "--stream",
    action="store_true",
    help="Enable websocket live log streaming for simulate.py --connect.",
)
parser.add_argument(
    "--camera-stream",
    action="store_true",
    help="Enable the camera MJPEG HTTP preview (adds JPEG encode + overlay draw cost).",
)
parser.add_argument(
    "-l",
    "--save-log",
    metavar="PATH",
    help=f"Write game log lines to PATH at {LOG_FPS} FPS for playback with simulate.py.",
)
parser.add_argument(
    "--record-session",
    metavar="DIRECTORY",
    help=(
        "Record synchronized video, controller state, and model detections in DIRECTORY."
    ),
)
parser.add_argument(
    "--fps",
    action="store_true",
    help="Print logic-loop and background-thread rates once per second.",
)
args = parser.parse_args()
if args.record_session is not None and args.camera_stream:
    parser.error("--record-session cannot be combined with --camera-stream")


class FpsMonitor:
    """Sample monotonic counters and print Hz once per reporting interval."""

    def __init__(self, interval_s=FPS_REPORT_INTERVAL):
        self._interval_s = interval_s
        self._sources: list[tuple[str, object]] = []
        self._last_counts: dict[str, int] = {}
        self._last_t = time.monotonic()

    def add(self, name: str, getter) -> None:
        self._sources.append((name, getter))
        self._last_counts[name] = int(getter())

    def maybe_print(self) -> None:
        now = time.monotonic()
        elapsed = now - self._last_t
        if elapsed < self._interval_s:
            return
        parts = []
        for name, getter in self._sources:
            count = int(getter())
            rate = (count - self._last_counts[name]) / elapsed
            self._last_counts[name] = count
            parts.append(f"{name}={rate:.1f}")
        self._last_t = now
        print("FPS " + " ".join(parts), flush=True)


if args.stream:
    import send_log
    # Start websocket log server in the background (it runs its own asyncio loop).
    send_log.start_server_background()
    time.sleep(0.05)

log_recorder_stop = threading.Event()
log_recorder_thread = None
_latest_log_line = None
_latest_log_lock = threading.Lock()
_log_write_count = 0
_log_write_count_lock = threading.Lock()
_logic_loop_count = 0


def update_latest_log_snapshot(log_line: str) -> None:
    global _latest_log_line
    with _latest_log_lock:
        _latest_log_line = log_line


def _log_recorder_loop(path: str) -> None:
    global _log_write_count
    interval = 1.0 / LOG_FPS
    next_write = time.monotonic()
    try:
        with open(path, "w", encoding="utf-8") as log_file_handle:
            while not log_recorder_stop.is_set():
                now = time.monotonic()
                if now < next_write:
                    time.sleep(min(next_write - now, 0.005))
                    continue
                next_write += interval
                if next_write < now:
                    # Avoid catch-up bursts after a stall.
                    next_write = now + interval
                with _latest_log_lock:
                    line = _latest_log_line
                if line is not None:
                    log_file_handle.write(line + "\n")
                    log_file_handle.flush()
                    with _log_write_count_lock:
                        _log_write_count += 1
    except Exception as exc:
        print(f"Warning: log recorder stopped with error: {exc}")


def get_log_write_count() -> int:
    with _log_write_count_lock:
        return _log_write_count


if args.save_log is not None:
    log_recorder_thread = threading.Thread(
        target=_log_recorder_loop,
        args=(args.save_log,),
        daemon=True,
        name="log-recorder",
    )
    log_recorder_thread.start()
    print(f"Saving game log to {args.save_log} at {LOG_FPS} FPS")

camera = None
kicker = None
movement_controller = None
imu = None
peer = None
recording_session = None
last_pose_time = None

def enter_pressed():
    if not sys.stdin.isatty():
        return False

    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if not ready:
        return False

    sys.stdin.readline()
    return True


def capture_startup_yaw(imu_sensor, sample_count=25, sample_interval=0.02):
    """Average a short burst of IMU samples so startup yaw is not just the first reading."""
    print("Stabilizing IMU yaw reference...")
    sin_sum = 0.0
    cos_sum = 0.0
    samples = 0
    while samples < sample_count:
        yaw_sample = imu_sensor.get_yaw()
        if yaw_sample is not None:
            yaw_rad = math.radians(yaw_sample)
            sin_sum += math.sin(yaw_rad)
            cos_sum += math.cos(yaw_rad)
            samples += 1
        time.sleep(sample_interval)
    return math.degrees(math.atan2(sin_sum, cos_sum))


def feed_imu_yaw_prior(imu_sensor, startup_yaw):
    """Push startup-relative IMU yaw into MCL as a soft heading prior."""
    imu_yaw = imu_sensor.get_yaw()
    if imu_yaw is None:
        return
    lidar.set_imu_yaw(imu_yaw_to_relative_yaw(imu_yaw, startup_yaw))


try:
    kicker = Kicker(board.D26, 0.02)
    break_beam = Breakbeam(board.D14)
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

    print("Initializing IMU...")
    imu = IMU()
    startup_yaw = capture_startup_yaw(imu)
    print(f"Startup yaw reference set to {startup_yaw:.6f} deg")
    feed_imu_yaw_prior(imu, startup_yaw)

    lidar.start_coordinates(2430, 1820)

    print("Waiting for first pose estimate...")
    while not lidar.is_coordinates_ready():
        if enter_pressed():
            print("Shutdown requested, exiting.")
            raise KeyboardInterrupt
        feed_imu_yaw_prior(imu, startup_yaw)
        time.sleep(0.1)

    if args.record_session is not None:
        recording_session = RecordingSession(
            args.record_session,
            resolution=CAMERA_RESOLUTION,
            requested_fps=CAMERA_FPS,
        )
        recording_session.update_metadata(
            {
                "bot_mode_at_start": bot_mode.name,
                "performance_note": (
                    "Use --fps with and without --record-session to compare camera_cap, "
                    "camera_infer, and logic rates on this robot."
                ),
            }
        )
        print(f"Recording synchronized session to {recording_session.directory}")

    camera = Camera(
        CAMERA_PORT,
        resolution=CAMERA_RESOLUTION,
        frame_rate=CAMERA_FPS,
        recording_path=(
            recording_session.video_path if recording_session is not None else None
        ),
        session_epoch_monotonic=(
            recording_session.epoch_monotonic
            if recording_session is not None
            else None
        ),
        detection_callback=(
            recording_session.record_detection
            if recording_session is not None
            else None
        ),
    )
    if args.camera_stream:
        camera.start_stream()
    else:
        camera.start()
        print("Camera preview disabled (pass --camera-stream to enable MJPEG stream)")

    print(f"Initializing motors at I2C addresses: {I2C_ADDRESSES}")
    movement_controller = MovementController.from_i2c_addresses(
        I2C_ADDRESSES,
        WHEEL_DIAMETER,
        MAX_YAW_RPM,
        MAX_MOTOR_RPM,
        YAW_CORRECT_THRESHOLD,
    )
    if ENABLE_COMMUNICATION:
        peer = Peer(port=PEER_PORT)
        peer.start()
        print(f"Peer communication started on UDP port {PEER_PORT} (bot_id={peer.bot_id})")
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

    fps_monitor = None
    if args.fps:
        fps_monitor = FpsMonitor()
        fps_monitor.add("logic", lambda: _logic_loop_count)
        fps_monitor.add("camera_cap", lambda: camera.capture_count)
        fps_monitor.add("camera_infer", lambda: camera.infer_count)
        fps_monitor.add("drive", lambda: movement_controller.loop_count)
        fps_monitor.add("imu", lambda: imu.update_count)
        if hasattr(lidar, "get_scan_generation"):
            fps_monitor.add("lidar_scan", lidar.get_scan_generation)
        if hasattr(lidar, "get_mcl_update_count"):
            fps_monitor.add("lidar_mcl", lidar.get_mcl_update_count)
        if peer is not None:
            fps_monitor.add("peer_rx", lambda: peer.receive_count)
        if log_recorder_thread is not None:
            fps_monitor.add("log", get_log_write_count)
        if recording_session is not None:
            fps_monitor.add("session_game", lambda: recording_session.game_writer.written)
            fps_monitor.add(
                "session_detection",
                lambda: recording_session.detection_writer.written,
            )
        print("FPS monitoring enabled")

    while True:
        if fps_monitor is not None:
            fps_monitor.maybe_print()
        if USE_PAUSE and pause_switch.read():
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

            _logic_loop_count += 1

            now_pose = time.monotonic()
            dt_pose = now_pose - last_pose_time
            last_pose_time = now_pose
            omega = 0.0
            if imu is not None:
                gyro_z = imu.get_gyro_z_deg_s()
                if gyro_z is not None:
                    omega = gyro_z
                feed_imu_yaw_prior(imu, startup_yaw)
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
            if break_beam.read():
                ball_captured = True
                ball_x = x_pos + 100 * math.cos(math.radians(yaw))
                ball_y = y_pos + 100 * math.sin(math.radians(yaw))
            else:
                ball_captured = False

            peer_msg = None
            if peer is not None:
                peer.send(
                    {
                        "x": x_pos,
                        "y": y_pos,
                        "yaw": yaw,
                        "mode": bot_mode.name,
                        "ball_x": ball_x,
                        "ball_y": ball_y,
                    }
                )
                peer_msg = peer.receive()
            if (
                peer_msg is not None
                and peer_msg.get("x") is not None
                and peer_msg.get("y") is not None
            ):
                friendly_bot_positions = [(peer_msg["x"], peer_msg["y"])]
            else:
                friendly_bot_positions = []
            if (
                ball_x is None
                and ball_y is None
                and peer_msg is not None
                and peer_msg.get("ball_x") is not None
                and peer_msg.get("ball_y") is not None
            ):
                ball_x = peer_msg["ball_x"]
                ball_y = peer_msg["ball_y"]

            if bot_mode == BotMode.DEFENCE:
                direction, speed, rotation, steering_state, kick, dribbler = defence.defence(
                    x_pos,
                    y_pos,
                    yaw,
                    ball_x,
                    ball_y,
                    ball_captured,
                    steering_state=steering_state,
                    friendly_bot_positions=friendly_bot_positions,
                    enemy_bot_positions=[],
                )
            elif bot_mode == BotMode.STRIKER:
                direction, speed, rotation, steering_state, kick, dribbler = striker.striker(
                    x_pos,
                    y_pos,
                    yaw,
                    ball_x,
                    ball_y,
                    ball_captured,
                    steering_state=steering_state,
                )
            elif bot_mode == BotMode.GOALIE:
                direction, speed, rotation, kick, dribbler = defence.goalie(
                    x_pos,
                    y_pos,
                    yaw,
                    ball_x,
                    ball_y,
                    ball_captured,
                    friendly_bot_positions=friendly_bot_positions,
                    enemy_bot_positions=[],
                )
                steering_state = False
            if (
                args.stream
                or log_recorder_thread is not None
                or recording_session is not None
            ):
                log_values = [
                    x_pos,
                    y_pos,
                    yaw,
                    ball_x,
                    ball_y,
                    ball_captured,
                    bot_mode.name,
                    steering_state,
                    direction,
                    speed,
                    rotation,
                    kick,
                    dribbler,
                ]
                log_line = ",".join(
                    "None" if value is None else str(value) for value in log_values
                )
                if args.stream:
                    send_log.update_latest_log(log_line)
                if log_recorder_thread is not None:
                    update_latest_log_snapshot(log_line)
                if recording_session is not None:
                    recording_session.record_game(log_values)
            if kick:
                kicker.kick()
            try:
                movement_controller.move(direction, speed, rotation, 1.0, yaw, dribbler)
            except MotorCommunicationError as exc:
                print(exc)
                raise
        else:
            bot_mode = MODE_SWITCH_ON if mode_switch.read() else MODE_SWITCH_OFF
            time.sleep(0.01)
            if movement_controller is not None:
                movement_controller.stop()

finally:
    if log_recorder_thread is not None:
        log_recorder_stop.set()
        log_recorder_thread.join(timeout=1.0)
    if peer is not None:
        try:
            peer.stop()
        except Exception as exc:
            print(f"Warning: failed to stop peer communication cleanly: {exc}")
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
    if recording_session is not None:
        try:
            if camera is not None:
                recording_session.update_metadata(camera.recording_info)
            recording_session.close()
            dropped_game = recording_session.game_writer.dropped
            dropped_detection = recording_session.detection_writer.dropped
            print(
                "Session saved "
                f"(dropped game rows={dropped_game}, "
                f"detection rows={dropped_detection})"
            )
        except Exception as exc:
            print(f"Warning: failed to finalize recording session cleanly: {exc}")
    if imu is not None:
        try:
            imu.close()
        except Exception as exc:
            print(f"Warning: failed to close IMU cleanly: {exc}")
    try:
        lidar.shutdown()
    except Exception as exc:
        print(f"Warning: failed to shut down lidar cleanly: {exc}")
