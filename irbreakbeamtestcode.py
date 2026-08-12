import time

import board
import digitalio

# Setup the sensor on GPIO 14 (Mapped to board.D14)
break_beam = digitalio.DigitalInOut(board.D14)
break_beam.direction = digitalio.Direction.INPUT
break_beam.pull = digitalio.Pull.UP # Must use pull-up resistor

print("IR Breakbeam Sensor Test Initialized.")
print("Waiting for beam to be broken...")

try:
    while True:
        # .value is True when the beam is unbroken (Solid)
        # .value is False when the beam is broken (Interrupted)
        if not break_beam.value:
            print("Beam is BROKEN!")
            time.sleep(0.5) # Debounce/delay
        else:
            print("Beam is Solid (unbroken).")
            time.sleep(0.5)
except KeyboardInterrupt:
    print("Test stopped.")
