import time

from lib import switch as switch_module
from lib.config import load_config

switch = switch_module.Switch(load_config().mode_switch_pin)

while True:
    if switch.read():
        print("Switch pressed")
    time.sleep(0.1)
