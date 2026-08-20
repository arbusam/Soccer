from lib.break_beam import BreakBeam
import board

break_beam = BreakBeam(board.D17)

while True:
    print(break_beam.read())