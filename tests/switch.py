import time

import board
from lib import switch as switch_module

switch = switch_module.Switch(board.D16)

while True:
    if switch.read():
        print("Switch pressed")
    time.sleep(0.1)