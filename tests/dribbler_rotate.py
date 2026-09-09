import time

from lib.hardware_controller import MotorCommunicationError

from lib.hardware_test_utils import create_hardware, set_startup_yaw

COMMAND_INTERVAL = 0.05
# Keep a clockwise yaw error so the bot spins in place instead of holding a heading.
CLOCKWISE_ROTATION = 90.0


def main():
    hardware = None
    try:
        hardware = create_hardware(max_motor_rpm=400)
        set_startup_yaw(hardware)
        print("Dribbler on; rotating clockwise. Press Ctrl+C to stop.")
        while True:
            hardware.move(
                0,
                0,
                CLOCKWISE_ROTATION,
                1.0,
                1,
            )
            time.sleep(COMMAND_INTERVAL)
    except KeyboardInterrupt:
        print("\nStopping test.")
    except MotorCommunicationError as exc:
        print(exc)
        raise
    finally:
        if hardware is not None:
            hardware.stop()


if __name__ == "__main__":
    main()
