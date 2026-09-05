#!/usr/bin/env python3
"""Pulse the configured kicker pin for a short kick, then release it."""

import time

from lib.config import load_config
from lib.kicker import Kicker

PULSE_S = 0.02


def main():
    kicker = Kicker(load_config().kicker_pin, PULSE_S)
    kicker.kick()
    time.sleep(PULSE_S + 0.1)
    kicker.deinit()


if __name__ == "__main__":
    main()
