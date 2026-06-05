from __future__ import annotations

import argparse
import sys
import time

from movement import MotorCommunicationError, MovementController

WHEEL_DIAMETER = 50  # mm
MAX_YAW_RPM = 100
MAX_MOTOR_RPM = 400
YAW_CORRECT_THRESHOLD = 3  # deg

I2C_ADDRESSES = [28, 32, 31, 30]
SQUARE_DIRECTIONS = (0, 90, 180, 270)
DEFAULT_SIDE_SECONDS = 1.0
DEFAULT_SPEED = 100  # mm/s
COMMAND_INTERVAL = 0.05


def _run_side(direction, duration, speed, movement_controller):
    print(f"Moving at {direction} degrees for {duration:.2f} seconds")
    end_time = time.monotonic() + duration
    while time.monotonic() < end_time:
        movement_controller.move(direction, speed, 0, 0.0, 0)
        time.sleep(COMMAND_INTERVAL)


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Move the robot in a one-second-per-side square.")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED, help="Translation speed in mm/s.")
    parser.add_argument(
        "--side-seconds",
        type=float,
        default=DEFAULT_SIDE_SECONDS,
        help="Seconds to drive each side of the square.",
    )
    args = parser.parse_args(argv)

    if args.speed < 0:
        parser.error("--speed must be non-negative")
    if args.side_seconds <= 0:
        parser.error("--side-seconds must be positive")

    movement_controller = None
    try:
        print(f"Initializing motors at I2C addresses: {I2C_ADDRESSES}")
        movement_controller = MovementController.from_i2c_addresses(
            I2C_ADDRESSES,
            WHEEL_DIAMETER,
            MAX_YAW_RPM,
            MAX_MOTOR_RPM,
            YAW_CORRECT_THRESHOLD,
        )
        print("Running square path. Press Ctrl+C to stop.")
        for direction in SQUARE_DIRECTIONS:
            _run_side(direction, args.side_seconds, args.speed, movement_controller)
    except KeyboardInterrupt:
        print("\nStopping square path.")
    except MotorCommunicationError as exc:
        print(exc)
        raise
    finally:
        if movement_controller is not None:
            movement_controller.stop()
            print("Motors stopped.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
