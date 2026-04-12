import json
import math
import os
import sys
import time
from steelbar_powerful_bldc_driver import PowerfulBLDCDriver
import board
import busio

# Initialises the i2c bus
i2c = busio.I2C(board.SCL, board.SDA)

# Conversion factor from RPM to motor speed units.
# Formula: rpm * 7 (pole pairs) * 36 (36:1 gear ratio) / 60 (seconds per minute) / 2^-16 (electrical revolutions per second)
RPM_TO_MOTOR_SPEED = 275251.2
DEFAULT_DIRECTION_SLEW_RATE_DEG_PER_SEC = 1000.0

_actual_direction_deg = None
_last_direction_update_time = None


class MotorCommunicationError(RuntimeError):
    """Raised when an I2C write to a motor driver fails."""


def wrap_angle_deg(angle):
    """Wrap an angle to the shortest signed equivalent in [-180, 180)."""
    return ((float(angle) + 180.0) % 360.0) - 180.0


def _shortest_angle_delta_deg(current_angle, target_angle):
    """Return the shortest signed delta from current to target.

    Exact 180 degree differences keep the sign from the caller's requested
    change so 0 -> 180 ramps upward instead of always choosing -180.
    """
    raw_delta = float(target_angle) - float(current_angle)
    wrapped_delta = wrap_angle_deg(raw_delta)
    if math.isclose(wrapped_delta, -180.0, abs_tol=1e-9) and raw_delta > 0.0:
        return 180.0
    return wrapped_delta


def reset_move_direction_state():
    """Forget the smoothed translation direction so the next call snaps to target."""
    global _actual_direction_deg, _last_direction_update_time
    _actual_direction_deg = None
    _last_direction_update_time = None


def _get_smoothed_direction(target_direction, direction_slew_rate_deg_per_sec):
    """Rate-limit translation direction changes between successive move() calls."""
    global _actual_direction_deg, _last_direction_update_time

    target_direction = float(target_direction)
    direction_slew_rate_deg_per_sec = abs(float(direction_slew_rate_deg_per_sec))
    now = time.monotonic()

    if (
        _actual_direction_deg is None
        or _last_direction_update_time is None
        or direction_slew_rate_deg_per_sec == 0.0
    ):
        _actual_direction_deg = target_direction
        _last_direction_update_time = now
        return _actual_direction_deg

    elapsed = max(0.0, now - _last_direction_update_time)
    _last_direction_update_time = now

    max_step = direction_slew_rate_deg_per_sec * elapsed
    direction_error = _shortest_angle_delta_deg(_actual_direction_deg, target_direction)
    if abs(direction_error) <= max_step:
        _actual_direction_deg = target_direction
    else:
        direction_step = _clamp(direction_error, -max_step, max_step)
        _actual_direction_deg = wrap_angle_deg(_actual_direction_deg + direction_step)
    return _actual_direction_deg


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
        quit()

    if os.path.getsize(calibration_path) == 0:
        print(
            f"Error: calibration file '{calibration_path}' is empty. "
            "Run calibrate.py to regenerate it before starting the motors."
        )
        quit()

    try:
        with open(calibration_path, encoding="utf-8") as f:
            cal_data = json.load(f)
    except json.JSONDecodeError as exc:
        print(
            f"Error: calibration file '{calibration_path}' is not valid JSON "
            f"(line {exc.lineno}, column {exc.colno}). Run calibrate.py to regenerate it."
        )
        quit()

    if not isinstance(cal_data, dict) or "motors" not in cal_data or not isinstance(cal_data["motors"], list):
        print(
            f"Error: calibration file '{calibration_path}' has an unexpected format. "
            "Run calibrate.py to regenerate it."
        )
        quit()

    return cal_data


def get_motors_for_calibration(i2c_addresses):
    """Create motor driver objects and set PID/limits (no calibration or FOC). Input: list of i2c addresses. Returns: tuple of (motors, motor_count, normalized_addresses)."""
    try:
        normalized_addresses = [int(addr) for addr in i2c_addresses]
    except (TypeError, ValueError):
        print("Error invalid i2c address list, please verify inputs and try again.")
        quit()

    motor_count = len(normalized_addresses)
    # Max of 4 motors
    if motor_count == 0 or motor_count > 4:
        print("Error motor count out of range, please reboot microcontroller to try again.")
        quit()

    # Creates an empty array of length 4
    motors = [None] * 4

    # Creates a motor driver object for each address (PowerfulBLDCDriver)
    for setup_motor_count, address in enumerate(normalized_addresses):
        if address <= 7 or address >= 120:
            print("Error invalid i2c address, please reboot microcontroller to try again.")
            quit()
        motors[setup_motor_count] = PowerfulBLDCDriver(i2c, address)
        print(f"The firmware version of motor driver number {setup_motor_count} is: {motors[setup_motor_count].get_firmware_version()}")
        if motors[setup_motor_count].get_firmware_version() != 3:
            print("Error unsupported motor driver version, please check for updates, maybe check wiring and i2c configuration, reboot microcontroller to try again.")
            quit()

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
    motors, motor_count, normalized_addresses = get_motors_for_calibration(i2c_addresses)
    motor_modes = [None] * 4

    # Loads the calibration data from the file. Calibration file is created by running calibrate.py
    cal_data = _load_calibration_data(calibration_file)
    # Checks if the calibration file has the correct number of motors
    if len(cal_data["motors"]) < motor_count:
        print(f"Error: calibration file has {len(cal_data['motors'])} motor(s), but {motor_count} motor(s) were requested.")
        quit()
    # Sets the calibration constants to each motor
    for setup_motor_count in range(motor_count):
        motor_cal = cal_data["motors"][setup_motor_count]
        motors[setup_motor_count].set_ELECANGLEOFFSET(motor_cal["elecangleoffset"])
        motors[setup_motor_count].set_SINCOSCENTRE(motor_cal["sincoscentre"])
        motors[setup_motor_count].configure_operating_mode_and_sensor(3, 1)  # configure FOC mode and sin/cos encoder
        motors[setup_motor_count].configure_command_mode(12)  # configure speed command mode
        motor_modes[setup_motor_count] = 12
    reset_move_direction_state()
    return motors, motor_modes


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
        -math.cos(local_direction_rad),  # Back right wheel
        math.sin(local_direction_rad),   # Front right wheel
        math.cos(local_direction_rad),   # Front left wheel
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

    return tuple(
        _clamp(rpm - yaw_correction_rpm, -max_rpm, max_rpm)
        for rpm in translation_rpms
    )

# Inputs:
# direction: int - global movement direction in degrees, where 0 is startup-forward and 90 is startup-right.
# speed: int - translation speed of the robot in mm/s
# rotation: int - desired robot heading in degrees in the same frame as direction and yaw.
# rotation_speed: float - the strength of the yaw correction in 0.0-1.0
# yaw: int - measured robot heading in degrees in the same frame as direction and rotation.
# motors: list - the list of motor objects
# motor_modes: list - the list of motor modes
# diameter: int - the diameter of the wheels in mm. Used to convert bot speed to motor rpm.
# max_yaw_rpm: int - the maximum yaw rpm. Used to limit the yaw correction speed.
# max_rpm: int - the maximum rpm for the motors. Used to limit the motor speed.
# yaw_correct_threshold: int - the threshold for the yaw correction in degrees. If yaw error is less than this, no correction is applied. This is to prevent overcorrection.
# direction_slew_rate_deg_per_sec: float - maximum change in translation direction per second. 180 by default.
def move(
    direction,
    speed,
    rotation,
    rotation_speed,
    yaw,
    motors,
    motor_modes,
    diameter,
    max_yaw_rpm,
    max_rpm,
    yaw_correct_threshold,
    direction_slew_rate_deg_per_sec=DEFAULT_DIRECTION_SLEW_RATE_DEG_PER_SEC,
):
    # Gets the first 4 motors from the list of motors
    drive_motors = motors[:4]
    # Checks if any of the motors are None
    if any(motor is None for motor in drive_motors):
        raise ValueError("move() requires 4 initialized drive motors in motors[0:4].")

    actual_direction = _get_smoothed_direction(direction, direction_slew_rate_deg_per_sec)

    a_speed, b_speed, c_speed, d_speed = _calculate_drive_rpms(
        actual_direction,
        speed,
        rotation,
        rotation_speed,
        yaw,
        diameter,
        max_yaw_rpm,
        max_rpm,
        yaw_correct_threshold,
    )

    # Uses the formula to convert rpm to motor speed units.
    a_val = int(a_speed * RPM_TO_MOTOR_SPEED)
    b_val = int(b_speed * RPM_TO_MOTOR_SPEED)
    c_val = int(c_speed * RPM_TO_MOTOR_SPEED)
    d_val = int(d_speed * RPM_TO_MOTOR_SPEED)

    # Sends the desired speed to the motor drivers.
    _set_motor_speed(drive_motors[0], a_val, 0)
    _set_motor_speed(drive_motors[1], b_val, 1)
    _set_motor_speed(drive_motors[2], c_val, 2)
    _set_motor_speed(drive_motors[3], d_val, 3)


def stop_all_motors(motors):
    """Set speed to 0 on all non-None motors. Call on exit to avoid runaway motors."""
    for index, m in enumerate(motors):
        if m is not None:
            _set_motor_speed(m, 0, index, ignore_errors=True)
    reset_move_direction_state()


# TODO: Automatically use i2c addresses using a fixed array in order: [28, 32, 31, 30]
def _prompt_i2c_addresses():
    """Gets i2c addresses from the user."""
    print("Please enter the number of motor drivers you want to control:")
    tempuint32 = int(input())
    if tempuint32 == 0 or tempuint32 > 8:
        print("Error motor count out of range, please reboot microcontroller to try again.")
        quit()

    addresses = []
    setup_motor_count = 0
    while setup_motor_count < tempuint32:
        print(f"Please enter the i2c address of motor driver number {setup_motor_count}:")
        address = int(input())
        if address <= 7 or address >= 120:
            print("Error invalid i2c address, please reboot microcontroller to try again.")
            quit()
        addresses.append(address)
        setup_motor_count += 1
    return addresses

# Should not be used, only used for testing initialisation.
if __name__ == "__main__":
    init_motors(_prompt_i2c_addresses())