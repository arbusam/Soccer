"""
Initialise motors via init_motors, prompt for which motor to test, then ramp set speed
from 100,000 in steps of 100,000 while measuring actual speed. Stop when measured speed
plateaus (no increase for 3 consecutive steps) and print the maximum measured speed.
"""

from __future__ import annotations

import sys
import time

from lib.movement import init_motors

START_SPEED = 100_000
SPEED_INCREMENT = 100_000
SETTLE_SECONDS = 0.5
PLATEAU_STALL_COUNT = 3


def _prompt_i2c_addresses():
    print("Please enter the number of motor drivers you want to control:")
    tempuint32 = int(input())
    if tempuint32 == 0 or tempuint32 > 8:
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


def _prompt_motor_index(motors) -> int:
    valid_indices = [i for i, m in enumerate(motors) if m is not None]
    if not valid_indices:
        print("Error no initialized motors.")
        sys.exit()
    print(f"Initialized motor indices: {valid_indices}")
    while True:
        print("Enter the motor index to test:")
        try:
            idx = int(input())
        except (ValueError, EOFError):
            print("Invalid input. Please enter an integer.")
            continue
        if idx in valid_indices:
            return idx
        print(f"Error index must be one of {valid_indices}. Try again.")


def _stop_motor(motor) -> None:
    try:
        motor.set_speed(0)
    except Exception:
        pass


def _get_measured_speed(motor):
    motor.update_quick_data_readout()
    if hasattr(motor, "get_speed_QDR"):
        return motor.get_speed_QDR()
    return motor.get_qdr_speed()


def main(argv: list[str]) -> int:
    motors, _motor_modes = init_motors(_prompt_i2c_addresses())
    motor_index = _prompt_motor_index(motors)
    motor = motors[motor_index]

    current_speed = START_SPEED
    max_measured = None
    stall_count = 0

    print(f"Ramping set speed from {START_SPEED}, increment {SPEED_INCREMENT}. Plateau after {PLATEAU_STALL_COUNT} non-increasing readings.")
    try:
        while True:
            motor.set_speed(current_speed)
            time.sleep(SETTLE_SECONDS)
            measured = _get_measured_speed(motor)
            if max_measured is None:
                max_measured = measured
            if measured > max_measured:
                max_measured = measured
                stall_count = 0
            else:
                stall_count += 1
            print(f"  set_speed={current_speed}  measured={measured}  max_measured={max_measured}  stall_count={stall_count}")
            if stall_count >= PLATEAU_STALL_COUNT:
                break
            current_speed += SPEED_INCREMENT
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received; stopping...")
    finally:
        _stop_motor(motor)
        if max_measured is not None:
            print(f"Maximum measured speed: {max_measured}")
        else:
            print("No speed measurements taken.")
        print("Stopped. Exiting.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
