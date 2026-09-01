from pathlib import Path

from lib.movement import MovementController

WHEEL_DIAMETER = 50  # mm
MAX_YAW_RPM = 100
MAX_MOTOR_RPM = 1000
YAW_CORRECT_THRESHOLD = 3  # deg

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.txt"


def load_i2c_addresses(path: Path = CONFIG_PATH) -> list[int]:
    if not path.is_file():
        raise FileNotFoundError(
            f"Missing {path.name}. Copy example_config.txt to config.txt and edit as needed."
        )

    values: dict[str, str] = {}
    with path.open(encoding="utf-8") as config_file:
        for raw_line in config_file:
            line = raw_line.split("#", 1)[0].strip()
            if not line or "=" not in line:
                continue
            key, value = line.split("=", 1)
            values[key.strip().lower()] = value.strip()

    try:
        return [int(part.strip()) for part in values["i2c_addresses"].split(",")]
    except (KeyError, ValueError) as exc:
        raise ValueError(
            f"{path.name}: i2c_addresses must be a comma-separated list of integers"
        ) from exc


def main() -> None:
    i2c_addresses = load_i2c_addresses()
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
