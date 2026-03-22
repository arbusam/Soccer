#!/usr/bin/env python3
"""
Pulse GPIO 26 high for 0.1 s, then low (e.g. solenoid kick test on Raspberry Pi).
"""

import time

import board
import digitalio

SOLENOID_PIN = board.D26
PULSE_S = 0.1


def main():
    pin = digitalio.DigitalInOut(SOLENOID_PIN)
    pin.direction = digitalio.Direction.OUTPUT
    try:
        pin.value = True
        time.sleep(PULSE_S)
        pin.value = False
    finally:
        pin.deinit()


if __name__ == "__main__":
    main()
