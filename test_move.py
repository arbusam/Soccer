import math
import time

from imu import IMU
from movement import MotorCommunicationError, init_motors, move, stop_all_motors

WHEEL_DIAMETER = 50
MAX_YAW_RPM = 100
MAX_MOTOR_RPM = 400
YAW_CORRECT_THRESHOLD = 3

I2C_ADDRESSES = [28, 32, 31, 30]
TEST_DIRECTION = 0
TEST_SPEED = 100
TEST_ROTATION = 0
TEST_ROTATION_SPEED = 1.0
COMMAND_INTERVAL = 0.05


def wrap_angle_deg(angle):
    return ((angle + 180) % 360) - 180


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
    motors = []
    motor_modes = []
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

        print(f"Initializing motors at I2C addresses: {I2C_ADDRESSES}")
        motors, motor_modes = init_motors(I2C_ADDRESSES)
        startup_yaw = capture_startup_yaw(imu)
        print(f"Startup yaw reference set to {startup_yaw:.6f} deg")
        print(
            "Running move() test with "
            f"direction={TEST_DIRECTION}, speed={TEST_SPEED}, rotation={TEST_ROTATION}, using live IMU yaw"
        )
        print("Press Ctrl+C to stop.")

        while True:
            yaw = imu.get_yaw()
            if yaw is None:
                time.sleep(0.01)
                continue
            yaw_relative = wrap_angle_deg(yaw - startup_yaw)
            print(f"Yaw: {yaw:.6f} deg (relative {yaw_relative:.6f} deg)")
            move(
                TEST_DIRECTION,
                TEST_SPEED,
                TEST_ROTATION,
                TEST_ROTATION_SPEED,
                yaw_relative,
                motors,
                motor_modes,
                WHEEL_DIAMETER,
                MAX_YAW_RPM,
                MAX_MOTOR_RPM,
                YAW_CORRECT_THRESHOLD,
            )
            time.sleep(COMMAND_INTERVAL)
    except KeyboardInterrupt:
        print("\nStopping test.")
    except MotorCommunicationError as exc:
        print(exc)
        raise
    finally:
        if imu is not None:
            imu.close()
        if motors:
            stop_all_motors(motors)


if __name__ == "__main__":
    main()
