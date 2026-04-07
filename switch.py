import board
import digitalio
import time

switch = digitalio.DigitalInOut(board.D16)
switch.direction = digitalio.Direction.INPUT
switch.pull = digitalio.Pull.UP

while True:
    if not switch.value:
        print("Switch pressed")
    time.sleep(0.1)