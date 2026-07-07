import math
import threading
import time

import board
import busio

from adafruit_bno08x import BNO_REPORT_GYROSCOPE, BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C


class IMU:
    def __init__(self, poll_interval=0.01):
        self._poll_interval = poll_interval
        self._lock = threading.Lock()
        self._running = True
        self._latest_quaternion = None
        self._latest_yaw = None
        self._latest_gyro = None

        i2c = busio.I2C(board.SCL, board.SDA)
        self._bno = BNO08X_I2C(i2c)
        self._bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        self._bno.enable_feature(BNO_REPORT_GYROSCOPE)

        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()

    def _update_loop(self):
        while self._running:
            try:
                quat_i, quat_j, quat_k, quat_real = self._bno.quaternion
                yaw = self._quaternion_to_yaw_degrees(quat_i, quat_j, quat_k, quat_real)
                gyro = self._bno.gyro
                with self._lock:
                    self._latest_quaternion = (quat_i, quat_j, quat_k, quat_real)
                    self._latest_yaw = yaw
                    self._latest_gyro = gyro
            except Exception:
                # Keep the updater alive if a read occasionally fails.
                pass
            time.sleep(self._poll_interval)

    @staticmethod
    def _quaternion_to_yaw_degrees(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def get_latest_quaternion(self):
        with self._lock:
            return self._latest_quaternion

    def get_yaw(self):
        with self._lock:
            return self._latest_yaw

    def get_gyro_z_deg_s(self):
        """Z-axis turn rate in project frame (deg/s, clockwise positive).

        The BNO08x is mounted upside down (sensor +Z points down). For that
        orientation, positive gyro_z matches clockwise rotation in the project
        heading frame used by MCL and movement.
        """
        with self._lock:
            if self._latest_gyro is None:
                return None
            _, _, gyro_z = self._latest_gyro
        return math.degrees(gyro_z)

    def close(self):
        self._running = False
        self._thread.join(timeout=1.0)
