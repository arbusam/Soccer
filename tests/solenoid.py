#!/usr/bin/env python3
"""
Pulse GPIO 26 high for 0.1 s, then low (e.g. solenoid kick test on Raspberry Pi).
"""

import time

import board
from lib.kicker import Kicker

SOLENOID_PIN = board.D21
PULSE_S = 0.02


def main():
    kicker = Kicker(SOLENOID_PIN, PULSE_S)
    kicker.kick()
    time.sleep(PULSE_S + 0.1)
    kicker.deinit()


if __name__ == "__main__":
    main()
