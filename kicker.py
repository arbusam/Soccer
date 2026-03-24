"""Solenoid kicker: pulse a GPIO pin high for a fixed duration."""

import threading
import time

import digitalio


class Kicker:
    def __init__(self, pin, pulse: float) -> None:
        self._pulse = pulse
        self._cooldown_s = 0.5
        self._pin = digitalio.DigitalInOut(pin)
        self._pin.direction = digitalio.Direction.OUTPUT
        self._pin.value = False
        self._state_lock = threading.Lock()
        self._kick_thread = None
        self._last_kick_started_at = float("-inf")
        self._kicking = False

    def _run_kick(self, pin) -> None:
        try:
            pin.value = True
            time.sleep(self._pulse)
            pin.value = False
        finally:
            with self._state_lock:
                self._kicking = False

    def deinit(self) -> None:
        with self._state_lock:
            kick_thread = self._kick_thread

        if kick_thread is not None and kick_thread.is_alive():
            kick_thread.join()

        with self._state_lock:
            if self._pin is not None:
                self._pin.value = False
                self._pin.deinit()
                self._pin = None
            self._kick_thread = None
            self._kicking = False

    def kick(self) -> None:
        with self._state_lock:
            if self._pin is None:
                return

            now = time.monotonic()
            if self._kicking or now - self._last_kick_started_at < self._cooldown_s:
                return

            self._kicking = True
            self._last_kick_started_at = now
            kick_thread = threading.Thread(target=self._run_kick, args=(self._pin,), daemon=True)
            self._kick_thread = kick_thread
            kick_thread.start()
