"""Process-wide I2C bus and transaction lock.

All devices on the Raspberry Pi's primary I2C bus must use these shared
objects so background threads cannot overlap multi-transaction device access.
"""

import threading

import board
import busio

_lock = threading.RLock()
_bus = None


def get_shared_i2c_bus():
    """Return the process-wide Blinka I2C bus instance."""
    global _bus
    with _lock:
        if _bus is None:
            _bus = busio.I2C(board.SCL, board.SDA)
        return _bus


def get_shared_i2c_lock():
    """Return the lock that serializes application-level I2C operations."""
    return _lock
