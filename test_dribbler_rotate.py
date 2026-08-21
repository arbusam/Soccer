import time

from movement import MotorCommunicationError, MovementController

WHEEL_DIAMETER = 50
MAX_YAW_RPM = 100
MAX_MOTOR_RPM = 400
YAW_CORRECT_THRESHOLD = 3

# Drive wheels then dribbler (same slot order as MovementController).
I2C_ADDRESSES = [31, 29, 32, 28, 27]
COMMAND_INTERVAL = 0.05
# Keep a clockwise yaw error so the bot spins in place instead of holding a heading.
CLOCKWISE_ROTATION = 90.0


def main():
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
        print("Dribbler on; rotating clockwise. Press Ctrl+C to stop.")
        while True:
            movement_controller.move(
                0,
                0,
                CLOCKWISE_ROTATION,
                1.0,
                0,
                True,
            )
            time.sleep(COMMAND_INTERVAL)
    except KeyboardInterrupt:
        print("\nStopping test.")
    except MotorCommunicationError as exc:
        print(exc)
        raise
    finally:
        if movement_controller is not None:
            movement_controller.stop()


if __name__ == "__main__":
    main()
