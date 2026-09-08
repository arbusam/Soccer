
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306


class Display:
    def __init__(self, rotate=1):
        self.serial = i2c(port=1, address=0x3C)
        self.device = ssd1306(self.serial, rotate=rotate)
    
    def draw_text(self, text):
        with canvas(self.device) as draw:
            draw.text((0, 0), text, fill="white")