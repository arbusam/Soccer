"""Background reader for the SteelBar time-of-flight sensor."""

import threading
import time

try:
    from smbus2 import SMBus, i2c_msg
except ImportError as exc:
    raise ImportError(
        "tof.py requires smbus2. Install it with `pip install smbus2`."
    ) from exc


class ToF:
    def __init__(self, address=0x50, bus_number=1, poll_interval=0.001):
        self._address = address
        self._poll_interval = poll_interval
        self._lock = threading.Lock()
        self._running = True
        self._last_sequence = None
        self._latest_distance = None
        self._bus = SMBus(bus_number)

        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()

    def _read_sensor(self):
        write = i2c_msg.write(self._address, [0x10])
        read = i2c_msg.read(self._address, 5)
        self._bus.i2c_rdwr(write, read)
        data = list(read)

        if len(data) != 5:
            raise RuntimeError(f"Expected 5 bytes from ToF sensor, got {len(data)}")

        sequence = data[0]
        distance = int.from_bytes(bytes(data[1:5]), byteorder="little", signed=True)
        return sequence, distance

    def _update_loop(self):
        while self._running:
            try:
                sequence, distance = self._read_next_measurement()
                if distance is None:
                    continue
                with self._lock:
                    self._latest_distance = distance
            except Exception:
                # Keep the background reader alive if an I2C read occasionally fails.
                time.sleep(self._poll_interval)

    def _read_next_measurement(self):
        while self._running:
            sequence, distance = self._read_sensor()
            changed = sequence != self._last_sequence
            self._last_sequence = sequence
            if changed:
                return sequence, distance
            time.sleep(self._poll_interval)
        return None, None

    def read(self):
        with self._lock:
            return self._latest_distance

    def close(self):
        self._running = False
        self._thread.join(timeout=1.0)
        self._bus.close()
