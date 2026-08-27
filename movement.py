import json
import math
import os
import sys
import threading
import time

import board
import busio
from steelbar_powerful_bldc_driver import PowerfulBLDCDriver

# Initialises the i2c bus
i2c = busio.I2C(board.SCL, board.SDA)

# Conversion factor from RPM to motor speed units.
# Formula: rpm * 7 (pole pairs) * 36 (36:1 gear ratio) / 60 (seconds per minute) / 2^-16 (electrical revolutions per second)
RPM_TO_MOTOR_SPEED = 275251.2
MAX_VELOCITY_CHANGE_PER_SEC = 8000.0 # Max change in movement vector per second (mm/s/s)
# Fixed-rate drive loop so control-loop stalls do not inflate accel dt or pause yaw correction.
DRIVE_LOOP_HZ = 50.0
DRIVE_LOOP_INTERVAL_S = 1.0 / DRIVE_LOOP_HZ
# Cap one-step accel so a late wake never applies more than ~2 control periods of ramp.
MAX_DRIVE_DT_S = DRIVE_LOOP_INTERVAL_S * 2.0

DRIBBLER_RPM = 1000
DRIBBLER_MOTOR_SPEED = int(DRIBBLER_RPM * RPM_TO_MOTOR_SPEED)
MAX_MOTORS = 8


class MotorCommunicationError(RuntimeError):
    """Raised when an I2C write to a motor driver fails."""


def wrap_angle_deg(angle):
    """Wrap an angle to the shortest signed equivalent in [-180, 180)."""
    return ((float(angle) + 180.0) % 360.0) - 180.0


def imu_yaw_to_relative_yaw(imu_yaw, startup_yaw):
    """Convert raw IMU yaw into the project's clockwise-positive startup-relative frame."""
    return wrap_angle_deg(float(startup_yaw) - float(imu_yaw))


def _resolve_calibration_path(calibration_file):
    """Resolve relative calibration files against this module's directory."""
    if os.path.isabs(calibration_file):
        return calibration_file
    return os.path.join(os.path.dirname(__file__), calibration_file)


def _load_calibration_data(calibration_file):
    """Load and validate motor calibration JSON from disk."""
    calibration_path = _resolve_calibration_path(calibration_file)
    if not os.path.isfile(calibration_path):
        print(f"Error: calibration file '{calibration_path}' not found. Run calibrate.py first to create it.")
        sys.exit()

    if os.path.getsize(calibration_path) == 0:
        print(
            f"Error: calibration file '{calibration_path}' is empty. "
            "Run calibrate.py to regenerate it before starting the motors."
        )
        sys.exit()

    try:
        with open(calibration_path, encoding="utf-8") as f:
            cal_data = json.load(f)
    except json.JSONDecodeError as exc:
        print(
            f"Error: calibration file '{calibration_path}' is not valid JSON "
            f"(line {exc.lineno}, column {exc.colno}). Run calibrate.py to regenerate it."
        )
        sys.exit()

    if not isinstance(cal_data, dict) or "motors" not in cal_data or not isinstance(cal_data["motors"], list):
        print(
            f"Error: calibration file '{calibration_path}' has an unexpected format. "
            "Run calibrate.py to regenerate it."
        )
        sys.exit()

    return cal_data


def get_motors_for_calibration(i2c_addresses):
    """Create motor driver objects and set PID/limits (no calibration or FOC). Input: list of i2c addresses. Returns: tuple of (motors, motor_count, normalized_addresses)."""
    try:
        normalized_addresses = [int(addr) for addr in i2c_addresses]
    except (TypeError, ValueError):
        print("Error invalid i2c address list, please verify inputs and try again.")
        sys.exit()

    motor_count = len(normalized_addresses)
    # motors[0:4] = drive wheels; motors[4] = optional dribbler
    if motor_count == 0 or motor_count > MAX_MOTORS:
        print("Error motor count out of range, please reboot microcontroller to try again.")
        sys.exit()

    motors = [None] * motor_count

    # Creates a motor driver object for each address (PowerfulBLDCDriver)
    for setup_motor_count, address in enumerate(normalized_addresses):
        if address <= 7 or address >= 120:
            print("Error invalid i2c address, please reboot microcontroller to try again.")
            sys.exit()
        motors[setup_motor_count] = PowerfulBLDCDriver(i2c, address)
        print(f"The firmware version of motor driver number {setup_motor_count} is: {motors[setup_motor_count].get_firmware_version()}")
        if motors[setup_motor_count].get_firmware_version() != 3:
            print("Error unsupported motor driver version, please check for updates, maybe check wiring and i2c configuration, reboot microcontroller to try again.")
            sys.exit()

    for setup_motor_count in range(motor_count):
        motors[setup_motor_count].set_current_limit_foc(524288)  # set current limit to 8 amp (only works in FOC mode)
        motors[setup_motor_count].set_id_pid_constants(1500, 200)
        motors[setup_motor_count].set_iq_pid_constants(1500, 200)
        motors[setup_motor_count].set_speed_pid_constants(4e-2, 4e-4, 3e-2)  # Constants valid for FOC and Robomaster M2006 P36 motor only
        motors[setup_motor_count].set_position_pid_constants(275, 0, 0)
        motors[setup_motor_count].set_position_region_boundary(250000)
        motors[setup_motor_count].set_speed_limit(546133333)
    return motors, motor_count, normalized_addresses


def init_motors(i2c_addresses, calibration_file="calibration_data.json"):
    """Initialises the motors and returns the motor objects and motor modes"""
    motors, motor_count, _normalized_addresses = get_motors_for_calibration(i2c_addresses)
    motor_modes = [None] * motor_count

    # Loads the calibration data from the file. Calibration file is created by running calibrate.py
    cal_data = _load_calibration_data(calibration_file)
    # Checks if the calibration file has the correct number of motors
    if len(cal_data["motors"]) < motor_count:
        print(f"Error: calibration file has {len(cal_data['motors'])} motor(s), but {motor_count} motor(s) were requested.")
        sys.exit()
    # Sets the calibration constants to each motor (drive wheels and optional dribbler)
    for setup_motor_count in range(motor_count):
        motor_cal = cal_data["motors"][setup_motor_count]
        motors[setup_motor_count].set_ELECANGLEOFFSET(motor_cal["elecangleoffset"])
        motors[setup_motor_count].set_SINCOSCENTRE(motor_cal["sincoscentre"])
        motors[setup_motor_count].configure_operating_mode_and_sensor(3, 1)  # configure FOC mode and sin/cos encoder
        motors[setup_motor_count].configure_command_mode(12)  # configure speed command mode
        motor_modes[setup_motor_count] = 12
    return motors, motor_modes


class MovementController:
    """Own the drive motor state and convert movement commands into motor speeds.

    Accel ramping and yaw correction run on a fixed-rate background thread so a
    stalled caller (slow LIDAR/camera/strategy) cannot inflate accel dt or pause
    heading correction. ``move()`` only updates the latest command targets.
    """

    def __init__(
        self,
        motors,
        motor_modes,
        diameter,
        max_yaw_rpm,
        max_rpm,
        yaw_correct_threshold,
    ):
        drive_motors = motors[:4]
        if any(motor is None for motor in drive_motors):
            raise ValueError(
                "MovementController requires 4 initialized drive motors in motors[0:4]."
            )

        self.motors = motors
        self.motor_modes = motor_modes
        self.diameter = diameter
        self.max_yaw_rpm = max_yaw_rpm
        self.max_rpm = max_rpm
        self.yaw_correct_threshold = yaw_correct_threshold

        self._command_lock = threading.Lock()
        self._i2c_lock = threading.Lock()
        self._current_lock = threading.Lock()
        self._error_lock = threading.Lock()

        self._target_direction = 0.0
        self._target_speed = 0.0
        self._target_rotation = 0.0
        self._target_rotation_speed = 0.0
        self._target_yaw = 0.0
        self._target_dribbler = 0

        self._current_direction = 0.0
        self._current_speed = 0.0
        self._last_error = None
        self._loop_count = 0
        self._loop_count_lock = threading.Lock()

        self._running = True
        self._thread = threading.Thread(
            target=self._drive_loop,
            name="movement-drive",
            daemon=True,
        )
        self._thread.start()

    @property
    def loop_count(self):
        with self._loop_count_lock:
            return self._loop_count

    @property
    def current_direction(self):
        with self._current_lock:
            return self._current_direction

    @current_direction.setter
    def current_direction(self, value):
        with self._current_lock:
            self._current_direction = float(value)

    @property
    def current_speed(self):
        with self._current_lock:
            return self._current_speed

    @current_speed.setter
    def current_speed(self, value):
        with self._current_lock:
            self._current_speed = float(value)

    @classmethod
    def from_i2c_addresses(
        cls,
        i2c_addresses,
        diameter,
        max_yaw_rpm,
        max_rpm,
        yaw_correct_threshold,
        calibration_file="calibration_data.json",
    ):
        """Initialise motors and return a ready-to-use movement controller."""
        motors, motor_modes = init_motors(i2c_addresses, calibration_file=calibration_file)
        return cls(
            motors,
            motor_modes,
            diameter,
            max_yaw_rpm,
            max_rpm,
            yaw_correct_threshold,
        )

    def move(self, direction, speed, rotation, rotation_speed, yaw, dribbler=0):
        """Update drive targets; accel and yaw correction continue on the drive thread."""
        self._raise_pending_error()
        with self._command_lock:
            self._target_direction = float(direction)
            self._target_speed = float(speed)
            self._target_rotation = float(rotation)
            self._target_rotation_speed = float(rotation_speed)
            self._target_yaw = float(yaw)
            self._target_dribbler = int(dribbler)

    def get_measured_body_velocity_mm_s(self, yaw_deg):
        """Estimate robot-body translation speed (mm/s) from measured wheel RPMs."""
        self._raise_pending_error()
        drive_motors = self.motors[:4]
        with self._i2c_lock:
            rpms = read_wheel_rpms(drive_motors)
        wheel_mm_s = wheel_rpms_to_linear_mm_s(rpms, self.diameter)
        return measured_wheel_speeds_to_body_velocity_mm_s(wheel_mm_s, yaw_deg)

    def stop(self):
        """Stop the drive thread and set speed to 0 on all owned motors."""
        self._running = False
        if self._thread.is_alive() and threading.current_thread() is not self._thread:
            self._thread.join(timeout=1.0)

        with self._command_lock:
            self._target_speed = 0.0
            self._target_rotation_speed = 0.0
            self._target_dribbler = 0
        with self._current_lock:
            self._current_speed = 0.0

        with self._i2c_lock:
            for index, motor in enumerate(self.motors):
                if motor is not None:
                    _set_motor_speed(motor, 0, index, ignore_errors=True)

    def _raise_pending_error(self):
        with self._error_lock:
            error = self._last_error
            self._last_error = None
        if error is not None:
            raise error

    def _set_pending_error(self, error):
        with self._error_lock:
            self._last_error = error

    def _snapshot_command(self):
        with self._command_lock:
            return (
                self._target_direction,
                self._target_speed,
                self._target_rotation,
                self._target_rotation_speed,
                self._target_yaw,
                self._target_dribbler,
            )

    def _drive_loop(self):
        next_tick = time.monotonic()
        last_update_time = next_tick
        while self._running:
            now = time.monotonic()
            if now < next_tick:
                time.sleep(min(next_tick - now, 0.005))
                continue

            next_tick += DRIVE_LOOP_INTERVAL_S
            if next_tick < now:
                # Avoid catch-up bursts after the scheduler or I2C stalls.
                next_tick = now + DRIVE_LOOP_INTERVAL_S

            dt = min(now - last_update_time, MAX_DRIVE_DT_S)
            last_update_time = now
            if dt <= 0.0:
                continue

            with self._loop_count_lock:
                self._loop_count += 1

            (
                direction,
                speed,
                rotation,
                rotation_speed,
                yaw,
                dribbler,
            ) = self._snapshot_command()

            with self._current_lock:
                current_direction = self._current_direction
                current_speed = self._current_speed

            dx = math.cos(math.radians(current_direction)) * current_speed
            dy = math.sin(math.radians(current_direction)) * current_speed
            target_dx = math.cos(math.radians(direction)) * speed
            target_dy = math.sin(math.radians(direction)) * speed

            delta_dx = target_dx - dx
            delta_dy = target_dy - dy
            delta_magnitude = math.hypot(delta_dx, delta_dy)

            max_step = MAX_VELOCITY_CHANGE_PER_SEC * dt
            if delta_magnitude > max_step and delta_magnitude > 0.0:
                scale = max_step / delta_magnitude
                delta_dx *= scale
                delta_dy *= scale

            new_dx = dx + delta_dx
            new_dy = dy + delta_dy
            new_direction = math.degrees(math.atan2(new_dy, new_dx))
            new_speed = math.hypot(new_dx, new_dy)

            with self._current_lock:
                self._current_direction = new_direction
                self._current_speed = new_speed

            a_speed, b_speed, c_speed, d_speed = _calculate_drive_rpms(
                new_direction,
                new_speed,
                rotation,
                rotation_speed,
                yaw,
                self.diameter,
                self.max_yaw_rpm,
                self.max_rpm,
                self.yaw_correct_threshold,
            )

            a_val = int(a_speed * RPM_TO_MOTOR_SPEED)
            b_val = int(b_speed * RPM_TO_MOTOR_SPEED)
            c_val = int(c_speed * RPM_TO_MOTOR_SPEED)
            d_val = int(d_speed * RPM_TO_MOTOR_SPEED)
            drive_motors = self.motors[:4]

            try:
                with self._i2c_lock:
                    _set_motor_speed(drive_motors[0], a_val, 0)
                    _set_motor_speed(drive_motors[1], b_val, 1)
                    _set_motor_speed(drive_motors[2], c_val, 2)
                    _set_motor_speed(drive_motors[3], d_val, 3)
                    if len(self.motors) > 4 and self.motors[4] is not None:
                        dribbler_speed = DRIBBLER_MOTOR_SPEED * dribbler 
                        _set_motor_speed(
                            self.motors[4], dribbler_speed, 4, ignore_errors=True
                        )
            except MotorCommunicationError as exc:
                self._set_pending_error(exc)


# Used by calibrate.py to calibrate the motors and save the results to the calibration file to be re used.
def calibrate_motors(motors, motor_count, i2c_addresses, calibration_file="calibration_data.json"):
    """Run physical calibration on each motor and save results to a JSON file."""
    calibration_path = _resolve_calibration_path(calibration_file)
    cal_data = {"motors": []}
    for setup_motor_count in range(motor_count):
        motors[setup_motor_count].configure_operating_mode_and_sensor(15, 1)  # calibration mode and sin/cos encoder
        motors[setup_motor_count].configure_command_mode(15)  # calibration mode
        motors[setup_motor_count].set_calibration_options(300, 2097152, 50000, 500000)
        motors[setup_motor_count].start_calibration()
        print(f"Starting calibration of motor {setup_motor_count}")
        while not motors[setup_motor_count].is_calibration_finished():
            print(".", end="")
            sys.stdout.flush()
            time.sleep(0.5)
        print()
        elecangleoffset = motors[setup_motor_count].get_calibration_ELECANGLEOFFSET()
        sincoscentre = motors[setup_motor_count].get_calibration_SINCOSCENTRE()
        print(f"ELECANGLEOFFSET: {elecangleoffset}")
        print(f"SINCOSCENTRE: {sincoscentre}")
        cal_data["motors"].append({
            "address": int(i2c_addresses[setup_motor_count]),
            "elecangleoffset": elecangleoffset,
            "sincoscentre": sincoscentre,
        })
    temp_calibration_path = f"{calibration_path}.tmp"
    with open(temp_calibration_path, "w", encoding="utf-8") as f:
        json.dump(cal_data, f, indent=2)
        f.flush()
        os.fsync(f.fileno())
    os.replace(temp_calibration_path, calibration_path)
    print(f"Calibration data saved to {calibration_path}")
    return cal_data


def _get_motor_address(motor):
    """Best-effort lookup of the driver's I2C address for diagnostics."""
    for attr in ("address", "device_address"):
        address = getattr(motor, attr, None)
        if address is not None:
            return address
    i2c_device = getattr(motor, "_i2c_device", None)
    return getattr(i2c_device, "device_address", None)


def _set_motor_speed(motor, speed, motor_index, *, ignore_errors=False):
    """Write a speed command and optionally suppress communication failures."""
    try:
        motor.set_speed(speed)
    except OSError as exc:
        address = _get_motor_address(motor)
        details = f"motor {motor_index}"
        if address is not None:
            details += f" (I2C address {address})"
        message = f"I2C communication failed while writing speed to {details}: {exc}"
        if ignore_errors:
            print(f"Warning: {message}")
            return False
        raise MotorCommunicationError(message) from exc
    return True


def _clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def _calculate_drive_rpms(
    direction,
    speed,
    rotation,
    rotation_speed,
    yaw,
    diameter,
    max_yaw_rpm,
    max_rpm,
    yaw_correct_threshold,
):
    """Convert global translation and heading targets into per-wheel RPM values."""
    direction = float(direction)
    speed = float(speed)
    rotation = float(rotation)
    rotation_speed = _clamp(float(rotation_speed), 0.0, 1.0)
    yaw = float(yaw)
    diameter = float(diameter)
    max_yaw_rpm = abs(float(max_yaw_rpm))
    max_rpm = abs(float(max_rpm))
    yaw_correct_threshold = abs(float(yaw_correct_threshold))

    if diameter <= 0:
        raise ValueError("move() requires a positive wheel diameter.")

    # Positive yaw error means the robot must rotate clockwise to match the target heading.
    yaw_error = wrap_angle_deg(rotation - yaw)
    if abs(yaw_error) <= yaw_correct_threshold or rotation_speed == 0.0 or max_yaw_rpm == 0.0:
        yaw_correction_rpm = 0.0
    else:
        yaw_correction_rpm = _clamp(
            (yaw_error / 60.0) * max_yaw_rpm * rotation_speed,
            -max_yaw_rpm,
            max_yaw_rpm,
        )

    # Convert the global move direction into the robot's local frame.
    # In this project 0 = forward and +90 = right, so a positive robot yaw means
    # the requested global direction must be shifted left in the robot frame.
    local_direction = wrap_angle_deg(yaw - direction) + 45.0
    local_direction_rad = math.radians(local_direction)
    wheel_multipliers = (
        -math.sin(local_direction_rad),  # Back left wheel
        math.cos(local_direction_rad),  # Back right wheel
        math.sin(local_direction_rad),   # Front right wheel
        -math.cos(local_direction_rad),   # Front left wheel
    )

    mmps_to_rpm = 60.0 / (diameter * math.pi)
    translation_rpms = [multiplier * speed * mmps_to_rpm for multiplier in wheel_multipliers]

    # Reserve headroom for yaw correction so the robot keeps rotating toward the target
    # without distorting the requested global translation direction.
    available_translation_rpm = max(max_rpm - abs(yaw_correction_rpm), 0.0)
    max_translation_rpm = max((abs(rpm) for rpm in translation_rpms), default=0.0)
    if max_translation_rpm > available_translation_rpm and max_translation_rpm > 0.0:
        scale = available_translation_rpm / max_translation_rpm
        translation_rpms = [rpm * scale for rpm in translation_rpms]

    # Subtract the common-mode RPM so a positive (clockwise) yaw error
    # actually turns the robot clockwise. Adding it spun anticlockwise.
    return tuple(
        _clamp(rpm - yaw_correction_rpm, -max_rpm, max_rpm)
        for rpm in translation_rpms
    )


def read_wheel_rpms(motors):
    """Read the current speed of each drive motor in RPM."""
    rpms = []
    for motor_index, motor in enumerate(motors):
        if motor is None:
            rpms.append(0.0)
            continue
        motor.update_quick_data_readout()
        if hasattr(motor, "get_speed_QDR"):
            speed_units = motor.get_speed_QDR()
        else:
            speed_units = motor.get_qdr_speed()
        # Firmware QDR exposes the signed speed register as uint32.
        if speed_units >= 2**31:
            speed_units -= 2**32
        rpms.append(
            speed_units
            / RPM_TO_MOTOR_SPEED
        )
    return rpms


def wheel_rpms_to_linear_mm_s(rpms, diameter):
    """Convert wheel RPM values to linear speeds at the wheel contact point."""
    mm_per_rev = math.pi * diameter
    return [rpm * mm_per_rev / 60.0 for rpm in rpms]


def measured_wheel_speeds_to_body_velocity_mm_s(wheel_mm_s, yaw_deg):
    """Invert X-drive translation kinematics into body-frame (vx forward, vy left).

    Rotation adds the same speed to all four wheels while the translation
    multipliers sum to zero, so subtracting the mean removes the rotation
    component using measurements only.
    """
    if len(wheel_mm_s) < 4:
        return 0.0, 0.0

    # If the bot is only translating, without rotation, the average wheel speed will be 0
    # This is because opposite wheels spin in opposite directions
    # However, if on average the wheels are spinning in a direction, then the bot is spinning in that direction
    rotation_mm_s = sum(wheel_mm_s[:4]) / 4.0
    corrected = [speed - rotation_mm_s for speed in wheel_mm_s[:4]] # This is the speed of the bot relative to the ground
    sin_term = (corrected[2] - corrected[0]) * 0.5 # This is the speed of the bot relative to the ground in the y direction
    cos_term = (corrected[1] - corrected[3]) * 0.5 # This is the speed of the bot relative to the ground in the x direction
    translation_speed = math.hypot(sin_term, cos_term)
    if translation_speed < 1e-3:
        return 0.0, 0.0

    local_direction_rad = math.atan2(sin_term, cos_term)
    direction_deg = wrap_angle_deg(yaw_deg - math.degrees(local_direction_rad) + 45.0)
    direction_rad = math.radians(direction_deg)
    global_vx = translation_speed * math.cos(direction_rad)
    global_vy = translation_speed * math.sin(direction_rad)
    return global_velocity_to_body_mm_s(global_vx, global_vy, yaw_deg)


def global_velocity_to_body_mm_s(global_vx, global_vy, yaw_deg):
    """Convert a global-frame velocity into body-frame (vx forward, vy left)."""
    yaw_rad = math.radians(yaw_deg)
    body_vx = global_vx * math.cos(yaw_rad) + global_vy * math.sin(yaw_rad)
    body_vy = global_vx * math.sin(yaw_rad) - global_vy * math.cos(yaw_rad)
    return body_vx, body_vy


WHEEL_SPEED_SLIP_MIN_MM_S = 50.0
SLIP_RATIO_THRESHOLD = 0.5
LIDAR_VELOCITY_MAX_AGE_S = 0.15
LIDAR_VELOCITY_MIN_DT_S = 0.02
LIDAR_POSE_UPDATE_MIN_MM = 2.0


class LidarVelocityEstimator:
    """Estimate body velocity from successive confident LIDAR pose updates.

    Velocity is smoothed in the global (field) frame, which does not rotate
    with the robot, so consecutive samples can be safely blended even when
    yaw changes between updates. The result is converted to the body frame
    on read using the caller's current yaw.
    """

    def __init__(self, smoothing=0.3):
        self._smoothing = smoothing
        self._last_x = None
        self._last_y = None
        self._last_time = None
        self._global_vx = 0.0
        self._global_vy = 0.0
        self._last_update_time = None

    def update(self, x, y, yaw, now):
        """Record a new pose sample and refresh the smoothed global velocity."""
        if self._last_x is None:
            self._last_x = x
            self._last_y = y
            self._last_time = now
            return

        dt = now - self._last_time
        if dt < LIDAR_VELOCITY_MIN_DT_S:
            return

        delta_mm = math.hypot(x - self._last_x, y - self._last_y)
        if delta_mm < LIDAR_POSE_UPDATE_MIN_MM:
            return

        global_vx = (x - self._last_x) / dt
        global_vy = (y - self._last_y) / dt

        alpha = self._smoothing
        self._global_vx = alpha * global_vx + (1.0 - alpha) * self._global_vx
        self._global_vy = alpha * global_vy + (1.0 - alpha) * self._global_vy
        self._last_update_time = now
        self._last_x = x
        self._last_y = y
        self._last_time = now

    def get_body_velocity(self, yaw_deg):
        """Return the smoothed velocity in the body frame for the given yaw."""
        return global_velocity_to_body_mm_s(self._global_vx, self._global_vy, yaw_deg)

    def is_fresh(self, now):
        if self._last_update_time is None:
            return False
        return (now - self._last_update_time) <= LIDAR_VELOCITY_MAX_AGE_S


def compute_wheel_odometry_trust(wheel_vx, wheel_vy, lidar_vx, lidar_vy, lidar_fresh):
    """Return 0..1 trust in wheel translation velocity based on LIDAR agreement."""
    if not lidar_fresh:
        return 1.0

    wheel_speed = math.hypot(wheel_vx, wheel_vy)
    if wheel_speed < WHEEL_SPEED_SLIP_MIN_MM_S:
        return 1.0

    lidar_speed = math.hypot(lidar_vx, lidar_vy)
    slip_ratio = abs(wheel_speed - lidar_speed) / wheel_speed
    if slip_ratio >= SLIP_RATIO_THRESHOLD:
        return 0.0
    return 1.0 - (slip_ratio / SLIP_RATIO_THRESHOLD)


def stop_all_motors(motors):
    """Set speed to 0 on all non-None motors. Call on exit to avoid runaway motors."""
    for index, m in enumerate(motors):
        if m is not None:
            _set_motor_speed(m, 0, index, ignore_errors=True)


def _prompt_i2c_addresses():
    """Gets i2c addresses from the user."""
    print("Please enter the number of motor drivers you want to control:")
    tempuint32 = int(input())
    if tempuint32 == 0 or tempuint32 > MAX_MOTORS:
        print("Error motor count out of range, please reboot microcontroller to try again.")
        sys.exit()

    addresses = []
    setup_motor_count = 0
    while setup_motor_count < tempuint32:
        print(f"Please enter the i2c address of motor driver number {setup_motor_count}:")
        address = int(input())
        if address <= 7 or address >= 120:
            print("Error invalid i2c address, please reboot microcontroller to try again.")
            sys.exit()
        addresses.append(address)
        setup_motor_count += 1
    return addresses

# Should not be used, only used for testing initialisation.
if __name__ == "__main__":
    init_motors(_prompt_i2c_addresses())