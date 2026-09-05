import math
import time

from lib.config import load_config
from lib.imu import IMU
from lib.kicker import Kicker
from lib.movement import (
    MotorCommunicationError,
    MovementController,
    imu_yaw_to_relative_yaw,
)

WHEEL_DIAMETER = 50
MAX_YAW_RPM = 100
MAX_MOTOR_RPM = 400
YAW_CORRECT_THRESHOLD = 3

TEST_DIRECTION = 0
TEST_SPEED = 100
TEST_ROTATION = 0
TEST_ROTATION_SPEED = 1.0
COMMAND_INTERVAL = 0.05

PULSE_S = 0.02


def capture_startup_yaw(imu, sample_count=25, sample_interval=0.02):
    """Average a short burst of IMU samples so startup yaw is not just the first reading."""
    print("Stabilizing IMU yaw reference...")
    sin_sum = 0.0
    cos_sum = 0.0
    samples = 0
    while samples < sample_count:
        yaw = imu.get_yaw()
        if yaw is not None:
            yaw_rad = math.radians(yaw)
            sin_sum += math.sin(yaw_rad)
            cos_sum += math.cos(yaw_rad)
            samples += 1
        time.sleep(sample_interval)
    return math.degrees(math.atan2(sin_sum, cos_sum))


def main():
    imu = None
    movement_controller = None
    startup_yaw = None
    try:
        print("Initializing IMU...")
        imu = IMU()
        print("Waiting for first yaw reading...")
        yaw = None
        while yaw is None:
            yaw = imu.get_yaw()
            if yaw is None:
                time.sleep(0.01)

        config = load_config()
        print(f"Initializing motors at I2C addresses: {config.i2c_addresses}")
        movement_controller = MovementController.from_i2c_addresses(
            config.i2c_addresses,
            WHEEL_DIAMETER,
            MAX_YAW_RPM,
            MAX_MOTOR_RPM,
            YAW_CORRECT_THRESHOLD,
        )
        startup_yaw = capture_startup_yaw(imu)
        print(f"Startup yaw reference set to {startup_yaw:.6f} deg")
        print(
            "Running move() test with "
            f"direction={TEST_DIRECTION}, speed={TEST_SPEED}, rotation={TEST_ROTATION}, using live IMU yaw"
        )
        print("Press Ctrl+C to stop.")
        l = 0
        kicker = Kicker(config.kicker_pin, PULSE_S)
        while True:
            if l < 60:
                dir = 1
            elif l < 120:
                dir = -1
            else:
                dir = -1
                kicker.kick()
                time.sleep(PULSE_S + 0.1)
                kicker.deinit()
            yaw = imu.get_yaw()
            if yaw is None:
                time.sleep(0.01)
                continue
            yaw_relative = imu_yaw_to_relative_yaw(yaw, startup_yaw)
            vx, vy = movement_controller.get_measured_body_velocity_mm_s(yaw_relative)
            measured_speed = math.hypot(vx, vy)
            print(
                f"Yaw: {yaw:.6f} deg (relative {yaw_relative:.6f} deg) | "
                f"measured {measured_speed:.1f} mm/s "
                f"(vx={vx:.1f} forward, vy={vy:.1f} left)"
            )
            movement_controller.move(
                TEST_DIRECTION,
                TEST_SPEED,
                TEST_ROTATION,
                TEST_ROTATION_SPEED,
                yaw_relative,
                dir
            )
            l += 1
            time.sleep(COMMAND_INTERVAL)
    except KeyboardInterrupt:
        print("\nStopping test.")
    except MotorCommunicationError as exc:
        print(exc)
        raise
    finally:
        if imu is not None:
            imu.close()
        if movement_controller is not None:
            movement_controller.stop()


if __name__ == "__main__":
    main()
