import math
import sys
import time 
from steelbar_powerful_bldc_driver import PowerfulBLDCDriver
import board
import busio

i2c = busio.I2C(board.SCL, board.SDA)

def init_motors(i2c_addresses):
    try:
        normalized_addresses = [int(addr) for addr in i2c_addresses]
    except (TypeError, ValueError):
        print("Error invalid i2c address list, please verify inputs and try again.")
        quit()

    motor_count = len(normalized_addresses)
    if motor_count == 0 or motor_count > 8:
        print("Error motor count out of range, please reboot microcontroller to try again.")
        quit()

    motors = [None] * 8
    motor_modes = [None] * 8

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
        motors[setup_motor_count].set_current_limit_foc(65536)  # set current limit to 1 amp (only works in FOC mode)
        motors[setup_motor_count].set_id_pid_constants(1500, 200)
        motors[setup_motor_count].set_iq_pid_constants(1500, 200)
        motors[setup_motor_count].set_speed_pid_constants(4e-2, 4e-4, 3e-2)  # Constants valid for FOC and Robomaster M2006 P36 motor only, see tuning constants document for more details
        motors[setup_motor_count].set_position_pid_constants(275, 0, 0)
        motors[setup_motor_count].set_position_region_boundary(250000)
        motors[setup_motor_count].set_speed_limit(10000000)
        
        motors[setup_motor_count].configure_operating_mode_and_sensor(15, 1)  # configure calibration mode and sin/cos encoder
        motors[setup_motor_count].configure_command_mode(15)  # configure calibration mode
        motors[setup_motor_count].set_calibration_options(300, 2097152, 50000, 500000)  # set calibration voltage to 300/3399*vcc volts, speed to 2097152/65536 elecangle/s, settling time to 50000/50000 seconds, calibration time to 500000/50000 seconds
        
        motors[setup_motor_count].start_calibration()  # start the calibration
        print(f"Starting calibration of motor {setup_motor_count}")
        while not motors[setup_motor_count].is_calibration_finished():  # wait for the calibration to finish, do not call any other motor driver functions while calibration is ongoing
            print(".", end="")
            sys.stdout.flush()
            time.sleep(0.5)
        print()  # print out the calibration results
        print(f"ELECANGLEOFFSET: {motors[setup_motor_count].get_calibration_ELECANGLEOFFSET()}")
        print(f"SINCOSCENTRE: {motors[setup_motor_count].get_calibration_SINCOSCENTRE()}")

        motors[setup_motor_count].configure_operating_mode_and_sensor(3, 1)  # configure FOC mode and sin/cos encoder
        motors[setup_motor_count].configure_command_mode(12)  # configure speed command mode
        motor_modes[setup_motor_count] = 12
    return motors, motor_modes

def move(direction, speed, rotation, yaw, motors, motor_modes, diameter, lever_arm, max_yaw_rpm, max_rpm, yaw_correct_threshold):
    yaw_error = ((rotation - yaw + 180) % 360) - 180

    if abs(yaw_error) > yaw_correct_threshold:
        # Map heading error to a yaw RPM request (60 deg error -> max_yaw_rpm)
        yaw_correct_rpm_component = max_yaw_rpm * (yaw_error / 60.0)
    else:
        yaw_correct_rpm_component = 0.0

    local_direction = direction - yaw - 45
    a_mult = math.sin(math.radians(local_direction))
    b_mult = math.cos(math.radians(local_direction))
    c_mult = -math.sin(math.radians(local_direction))
    d_mult = -math.cos(math.radians(local_direction))

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

    max_trans_rpm = max(abs(a_speed), abs(b_speed), abs(c_speed), abs(d_speed), 1e-6)
    if max_trans_rpm > max_rpm:
        scale = max_rpm / max_trans_rpm
        a_speed *= scale
        b_speed *= scale
        c_speed *= scale
        d_speed *= scale

    # Clamp to max motor rpm
    yaw_correct_rpm_component = max(min(yaw_correct_rpm_component, max_yaw_rpm), -max_yaw_rpm)

    bounds = []
    for base, k in ((a_speed, 1), (b_speed, -1), (c_speed, 1), (d_speed, -1)):
        if k == 1:
            upper = max_rpm - base
            lower = -max_rpm - base
        else:
            upper = max_rpm + base
            lower = base - max_rpm
        bounds.append((lower, upper))

    lower_bound = max(b[0] for b in bounds)
    upper_bound = min(b[1] for b in bounds)
    w_cmd = max(min(yaw_correct_rpm_component, upper_bound), lower_bound)

    a_speed += w_cmd
    c_speed += w_cmd
    b_speed -= w_cmd
    d_speed -= w_cmd

    # Clamp between -max_rpm to max_rpm
    a_speed = max(min(a_speed, max_rpm), -max_rpm)
    b_speed = max(min(b_speed, max_rpm), -max_rpm)
    c_speed = max(min(c_speed, max_rpm), -max_rpm)
    d_speed = max(min(d_speed, max_rpm), -max_rpm)

    motors[0].set_speed(a_speed)
    motors[1].set_speed(b_speed)
    motors[2].set_speed(c_speed)
    motors[3].set_speed(d_speed)

    for motor in motors:
        motor.update_quick_data_readout()
    
    print(motors[0].get_speed_QDR())

def _prompt_i2c_addresses():
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

if __name__ == "__main__":
    init_motors(_prompt_i2c_addresses())