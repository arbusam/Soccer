import time

import board

import switch

switch = switch.Switch(board.D16)

while True:
    if switch.read():
        print("Switch pressed")
    time.sleep(0.1)