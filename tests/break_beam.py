import board
from lib.break_beam import Breakbeam

break_beam = Breakbeam(board.D17)

while True:
    print(break_beam.read())