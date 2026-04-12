import digitalio

class Switch:
    switch = None
    def __init__(self, pin):
        self.switch = digitalio.DigitalInOut(pin)
        self.switch.direction = digitalio.Direction.INPUT
        self.switch.pull = digitalio.Pull.UP

    def read(self):
        return not self.switch.value