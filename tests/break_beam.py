from lib.break_beam import Breakbeam
from lib.config import load_config

break_beam = Breakbeam(load_config().break_beam_pin)

while True:
    print(break_beam.read())
