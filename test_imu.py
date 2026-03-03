import time
from imu import IMU

imu = IMU()

try:
    while True:
        time.sleep(0.5)
        yaw = imu.get_yaw()
        if yaw is None:
            print("Yaw: waiting for quaternion...")
        else:
            print(f"Yaw: {yaw:.6f} deg")
except KeyboardInterrupt:
    pass
finally:
    imu.close()
