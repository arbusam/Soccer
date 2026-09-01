import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from tof import ToF

tof = ToF()

try:
    while True:
        time.sleep(0.5)
        distance = tof.read()
        if distance is None:
            print("ToF: waiting for reading...")
        else:
            print(f"ToF: {distance} mm")
except KeyboardInterrupt:
    pass
finally:
    tof.close()
