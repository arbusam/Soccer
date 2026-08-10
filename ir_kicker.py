import board
import digitalio
from kicker import Kicker

# Setup the sensor on GPIO 14 (Mapped to board.D14)
break_beam = digitalio.DigitalInOut(board.D14)
break_beam.direction = digitalio.Direction.INPUT
break_beam.pull = digitalio.Pull.UP # Must use pull-up resistor

print("IR Breakbeam Sensor Test Initialized.")
print("Waiting for beam to be broken...")
kicker = Kicker(board.D27, 0.1)

try:
    while True:
        # .value is True when the beam is unbroken (Solid)
        # .value is False when the beam is broken (Interrupted)
        if not break_beam.value:
            print("Beam is BROKEN!")
            kicker.kick()
        else:
            print("Beam is Solid (unbroken).")
except KeyboardInterrupt:
    print("Test stopped.")
