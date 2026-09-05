from lib.config import load_config
from lib.movement import MovementController

WHEEL_DIAMETER = 50  # mm
MAX_YAW_RPM = 100
MAX_MOTOR_RPM = 1000
YAW_CORRECT_THRESHOLD = 3  # deg


def main() -> None:
    i2c_addresses = load_config().i2c_addresses
    print(f"Initializing motors at I2C addresses: {i2c_addresses}")
    movement_controller = MovementController.from_i2c_addresses(
        i2c_addresses,
        WHEEL_DIAMETER,
        MAX_YAW_RPM,
        MAX_MOTOR_RPM,
        YAW_CORRECT_THRESHOLD,
    )
    movement_controller.stop()
    print("Motors stopped.")


if __name__ == "__main__":
    main()
