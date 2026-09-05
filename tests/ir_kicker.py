from lib.break_beam import Breakbeam
from lib.config import load_config
from lib.kicker import Kicker

config = load_config()
break_beam = Breakbeam(config.break_beam_pin)
kicker = Kicker(config.kicker_pin, 0.1)

print("IR Breakbeam Sensor Test Initialized.")
print("Waiting for beam to be broken...")

try:
    while True:
        if break_beam.read():
            print("Beam is BROKEN!")
            kicker.kick()
        else:
            print("Beam is Solid (unbroken).")
except KeyboardInterrupt:
    print("Test stopped.")
