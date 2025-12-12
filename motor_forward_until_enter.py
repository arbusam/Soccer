"""
Initialise motors (same prompt/init flow as defence.py/controller_movement.py),
run them forward at a fixed RPM until Enter is pressed, then stop and exit.
"""

from __future__ import annotations

import argparse
import sys

from movement import init_motors


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


def _iter_initialized_motors(motors):
    # `init_motors` returns a fixed-size list with None entries for unused slots.
    for m in motors:
        if m is not None:
            yield m


def _stop_all(motors) -> None:
    for m in _iter_initialized_motors(motors):
        try:
            m.set_speed(0)
        except Exception:
            # Best-effort stop; still continue stopping others.
            pass


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Run motors forward at a fixed RPM until Enter is pressed.")
    parser.add_argument("--rpm", type=int, default=100, help="Motor RPM to command (default: 100).")
    args = parser.parse_args(argv)

    motors, _motor_modes = init_motors(_prompt_i2c_addresses())

    rpm = int(args.rpm)
    print(f"Commanding forward speed: {rpm} rpm")
    print("Press Enter to stop...")

    try:
        for m in _iter_initialized_motors(motors):
            m.set_speed(rpm)
        input()
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received; stopping...")
    finally:
        _stop_all(motors)
        print("Stopped. Exiting.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))


