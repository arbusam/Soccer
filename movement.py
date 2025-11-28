import math
import sys
import time 
from steelbar_powerful_bldc_driver import PowerfulBLDCDriver
import board
import busio

i2c = busio.I2C(board.SCL, board.SDA)

def init_motors():
    motors = [None] * 8
    motor_modes = [None] * 8
    print("Please enter the number of motor drivers you want to control:")
    tempuint32 = int(input())
    if tempuint32 == 0 or tempuint32 > 8:
        print("Error motor count out of range, please reboot microcontroller to try again.")
        quit()
    motor_count = tempuint32

    setup_motor_count = 0
    while setup_motor_count < motor_count:
        print(f"Please enter the i2c address of motor driver number {setup_motor_count}:")
        tempuint32 = int(input())
        if tempuint32 <= 7 or tempuint32 >= 120:
            print("Error invalid i2c address, please reboot microcontroller to try again.")
            quit()
        motors[setup_motor_count] = PowerfulBLDCDriver(i2c, tempuint32)
        
        print(f"The firmware version of motor driver number {setup_motor_count} is: {motors[setup_motor_count].get_firmware_version()}")
        if motors[setup_motor_count].get_firmware_version() != 3:
            print("Error unsupported motor driver version, please check for updates, maybe check wiring and i2c configuration, reboot microcontroller to try again.")
            quit()

        setup_motor_count += 1

    setup_motor_count = 0
    while setup_motor_count < motor_count:
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
        
        setup_motor_count += 1
    return motors, motor_modes

# TODO: Add dynamic yaw correction. Ensure global movement vector is maintained.
def move(direction, speed, rotation, motors, motor_modes, diameter): # degrees, mm/s
    direction -= 45
    a_mult = math.sin(math.radians(direction))
    b_mult = math.cos(math.radians(direction))
    c_mult = -math.sin(math.radians(direction))
    d_mult = -math.cos(math.radians(direction))

    # Values in mm/s
    a_value = int(a_mult * speed)
    b_value = int(b_mult * speed)
    c_value = int(c_mult * speed)
    d_value = int(d_mult * speed)

    # Values in rpm
    a_speed = a_value / (diameter * math.pi) * 60
    b_speed = b_value / (diameter * math.pi) * 60
    c_speed = c_value / (diameter * math.pi) * 60
    d_speed = d_value / (diameter * math.pi) * 60

    motors[0].set_speed(motor_modes[0], a_speed)
    motors[1].set_speed(motor_modes[1], b_speed)
    motors[2].set_speed(motor_modes[2], c_speed)
    motors[3].set_speed(motor_modes[3], d_speed)

    for motor in motors:
        motor.update_quick_data_readout()
    
    print(motors[0].get_speed_QDR())

if __name__ == "__main__":
    init_motors()