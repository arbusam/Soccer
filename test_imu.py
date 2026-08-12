import time

from imu import IMU

imu = IMU()

try:
    while True:
        time.sleep(0.5)
        yaw = imu.get_yaw()
        gyro_z = imu.get_gyro_z_deg_s()
        if yaw is None:
            print("Yaw: waiting for quaternion...")
        else:
            print(f"Yaw: {yaw:.6f} deg")
        if gyro_z is None:
            print("Gyro Z: waiting for gyro report...")
        else:
            print(f"Gyro Z: {gyro_z:.3f} deg/s")
except KeyboardInterrupt:
    pass
finally:
    imu.close()
