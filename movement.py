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


class MotorCommunicationError(RuntimeError):
    """Raised when an I2C write to a motor driver fails."""


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

# Inputs:
# direction: int - the direction of the robot in degrees. Must use the same heading reference frame as rotation and yaw.
# speed: int - the speed of the robot in mm/s
# rotation: int - the desired rotation of the robot in degrees. Must use the same heading reference frame as direction and yaw.
# rotation_speed: float - the strength of the yaw correction in 0.0-1.0
# yaw: int - the measured yaw of the robot in degrees, in the same heading reference frame as direction and rotation.
# motors: list - the list of motor objects
# motor_modes: list - the list of motor modes
# diameter: int - the diameter of the wheels in mm. Used to convert bot speed to motor rpm.
# max_yaw_rpm: int - the maximum yaw rpm. Used to limit the yaw correction speed.
# max_rpm: int - the maximum rpm for the motors. Used to limit the motor speed.
# yaw_correct_threshold: int - the threshold for the yaw correction in degrees. If yaw error is less than this, no correction is applied. This is to prevent overcorrection.
def move(direction, speed, rotation, rotation_speed, yaw, motors, motor_modes, diameter, max_yaw_rpm, max_rpm, yaw_correct_threshold):
    # Ensures integer parameters are integers
    direction = int(direction)
    speed = int(speed)
    rotation = float(rotation)
    rotation_speed = float(rotation_speed)
    yaw = float(yaw)
    max_yaw_rpm = int(max_yaw_rpm)
    max_rpm = int(max_rpm)
    yaw_correct_threshold = int(yaw_correct_threshold)
    
    # Gets the first 4 motors from the list of motors
    drive_motors = motors[:4]
    # Checks if any of the motors are None
    if any(motor is None for motor in drive_motors):
        raise ValueError("move() requires 4 initialized drive motors in motors[0:4].")

    # Signed shortest-angle error from current yaw to target rotation.
    # Example: yaw=10, rotation=0 -> error=-10, so the controller turns back toward 0.
    yaw_error = ((rotation - yaw + 180) % 360) - 180

    rotation_speed = max(0.0, min(rotation_speed, 1.0))
    if abs(yaw_error) > yaw_correct_threshold:
        # Map heading error to a yaw RPM request (60 deg error -> max_yaw_rpm)
        yaw_correct_rpm_component = max_yaw_rpm * (yaw_error / 60.0) * rotation_speed
    else:
        yaw_correct_rpm_component = 0.0

    # Local direction is from the robot's perspective, where 0 is forward and 90 is right.
    local_direction = direction - yaw + 45
    a_mult = -math.sin(math.radians(local_direction)) # Back left wheel
    b_mult = -math.cos(math.radians(local_direction)) # Back right wheel
    c_mult = math.sin(math.radians(local_direction)) # Front right wheel
    d_mult = math.cos(math.radians(local_direction)) # Front left wheel

    # Values in mm/s
    a_value = a_mult * speed
    b_value = b_mult * speed
    c_value = c_mult * speed
    d_value = d_mult * speed

    # Values in rpm
    mmps_to_rpm = 60.0 / (diameter * math.pi)
    a_speed = a_value * mmps_to_rpm
    b_speed = b_value * mmps_to_rpm
    c_speed = c_value * mmps_to_rpm
    d_speed = d_value * mmps_to_rpm

    # Finds the maximum speed of any individual motor in rpm.
    # 1e-6 is added to avoid division by zero.
    max_trans_rpm = max(abs(a_speed), abs(b_speed), abs(c_speed), abs(d_speed), 1e-6)
    if max_trans_rpm > max_rpm:
        # Scales all motor speeds down so the maximum speed is equal to max_rpm, while preserving the relative speeds of the motors so direction is the same.
        scale = max_rpm / max_trans_rpm
        a_speed *= scale
        b_speed *= scale
        c_speed *= scale
        d_speed *= scale

    # Clamp to max yaw correction component rpm.
    yaw_correct_rpm_component = max(min(yaw_correct_rpm_component, max_yaw_rpm), -max_yaw_rpm)

    # The following block calculates how many rpm are left for the yaw correction component between the max rpm and desired rpm.
    # Upper bound is the highest positive value, lower bound is the lowest negative value.
    bounds = []
    for base in (a_speed, b_speed, c_speed, d_speed):
        upper = max_rpm - base
        lower = -max_rpm - base
        bounds.append((lower, upper))

    lower_bound = max(b[0] for b in bounds)
    upper_bound = min(b[1] for b in bounds)
    # Clamps yaw correction component rpm to the bounds.
    yaw_correction_rpm = max(min(yaw_correct_rpm_component, upper_bound), lower_bound)

    # Adds the yaw correction component to the motor speeds.
    a_speed += yaw_correction_rpm
    b_speed += yaw_correction_rpm
    c_speed += yaw_correction_rpm
    d_speed += yaw_correction_rpm

    # Clamp between -max_rpm to max_rpm
    a_speed = max(min(a_speed, max_rpm), -max_rpm)
    b_speed = max(min(b_speed, max_rpm), -max_rpm)
    c_speed = max(min(c_speed, max_rpm), -max_rpm)
    d_speed = max(min(d_speed, max_rpm), -max_rpm)
    # print(f"a_speed: {a_speed}, b_speed: {b_speed}, c_speed: {c_speed}, d_speed: {d_speed}")

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

    # for motor in drive_motors:
    #     motor.update_quick_data_readout()
    
    # print(drive_motors[0].get_speed_QDR())


def stop_all_motors(motors):
    """Set speed to 0 on all non-None motors. Call on exit to avoid runaway motors."""
    for index, m in enumerate(motors):
        if m is not None:
            _set_motor_speed(m, 0, index, ignore_errors=True)


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