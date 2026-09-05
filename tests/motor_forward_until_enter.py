"""
Initialise motors from config.txt, run them forward at a fixed RPM until Enter
is pressed, then stop and exit.
"""

from __future__ import annotations

import argparse
import sys

from lib.config import load_config
from lib.movement import init_motors


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
    parser.add_argument("--rpm", type=int, default=100000, help="Motor RPM to command (default: 100).")
    args = parser.parse_args(argv)

    i2c_addresses = load_config().i2c_addresses
    print(f"Initializing motors at I2C addresses: {i2c_addresses}")
    motors, _motor_modes = init_motors(i2c_addresses)

    rpm = int(args.rpm)
    print(f"Commanding forward speed: {rpm} rpm")
    print("Press Enter to stop...")

    try:
        # Command specific motors:
        motors_list = list(_iter_initialized_motors(motors))
        if len(motors_list) > 0:
            motors_list[0].set_speed(-rpm)
        if len(motors_list) > 1:
            motors_list[1].set_speed(-rpm)
        if len(motors_list) > 2:
            motors_list[2].set_speed(rpm)
        if len(motors_list) > 3:
            motors_list[3].set_speed(rpm)
        input()
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received; stopping...")
    finally:
        _stop_all(motors)
        print("Stopped. Exiting.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))


