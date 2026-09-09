"""Pulse the configured kicker through the native hardware controller."""

import time

from lib.hardware_test_utils import create_hardware


def main():
    hardware = create_hardware(kicker=True)
    try:
        hardware.move(0, 0, 0, 0, kick=True)
        time.sleep(0.1)
    finally:
        hardware.stop()


if __name__ == "__main__":
    main()
