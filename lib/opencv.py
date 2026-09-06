import copy
import json
import logging
from pathlib import Path

import cv2

DEFAULT_THRESHOLDS = {
    "blue": {"lower": [100, 240, 100], "upper": [120, 255, 255]},
    "yellow": {"lower": [20, 100, 100], "upper": [30, 255, 255]},
}
THRESHOLDS_FILE = Path(__file__).resolve().parent.parent / "goal_thresholds.json"


def validate_thresholds(bounds):
    if not isinstance(bounds, dict) or set(bounds) != {"blue", "yellow"}:
        raise ValueError("Expected blue and yellow HSV bounds")
    for colour in bounds.values():
        if not isinstance(colour, dict) or set(colour) != {"lower", "upper"}:
            raise ValueError("Expected lower and upper HSV bounds")
        for side in ("lower", "upper"):
            values = colour[side]
            if not isinstance(values, (list, tuple)) or len(values) != 3:
                raise ValueError("HSV bounds need three integers")
            if any(type(v) is not int or not 0 <= v <= limit
                   for v, limit in zip(values, (179, 255, 255), strict=True)):
                raise ValueError("HSV ranges: H 0–179, S/V 0–255")
        if any(a > b for a, b in zip(colour["lower"], colour["upper"], strict=True)):
            raise ValueError("Each lower bound must be <= its upper bound")
    return copy.deepcopy(bounds)


def load_thresholds(path=THRESHOLDS_FILE):
    try:
        return validate_thresholds(json.loads(Path(path).read_text()))
    except FileNotFoundError:
        return copy.deepcopy(DEFAULT_THRESHOLDS)
    except (ValueError, TypeError, OSError) as exc:
        logging.getLogger(__name__).warning("Invalid goal thresholds: %s", exc)
        return copy.deepcopy(DEFAULT_THRESHOLDS)


class OpenCV:
    def __init__(self, thresholds=None):
        self.bounding_boxes = []
        self.thresholds = load_thresholds() if thresholds is None else validate_thresholds(thresholds)

    def mask(self, image_hsv, blue):
        bounds = self.thresholds["blue" if blue else "yellow"]
        return cv2.inRange(image_hsv, tuple(bounds["lower"]), tuple(bounds["upper"]))

    def process_image(self, image_hsv, blue):
        contours, _ = cv2.findContours(
            self.mask(image_hsv, blue), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        return contours


# Test by passing in an image
if __name__ == "__main__":
    import sys
    import time

    if len(sys.argv) < 3:
        print("Usage: python lib/opencv.py <image_path> <blue|yellow>")
        sys.exit(1)

    image_path = sys.argv[1]
    color_arg = sys.argv[2].lower()
    if color_arg == "blue":
        blue = True
    elif color_arg == "yellow":
        blue = False
    else:
        print("Choose 'blue' or 'yellow' as the second argument.")
        sys.exit(1)

    image_bgr = cv2.imread(image_path)
    if image_bgr is None:
        print(f"Failed to load image: {image_path}")
        sys.exit(1)

    image_hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)

    opencv = OpenCV()
    start_time = time.perf_counter()
    contours = opencv.process_image(image_hsv, blue)
    elapsed_ms = (time.perf_counter() - start_time) * 1000
    print(f"Processing took {elapsed_ms:.2f} ms")

    overlay = image_bgr.copy()
    cv2.drawContours(overlay, contours, -1, (0, 255, 0), cv2.FILLED)
    preview = cv2.addWeighted(image_bgr, 0.6, overlay, 0.4, 0)
    window_name = "Colour selection"

    def show_pixel_hsv(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            hue, saturation, value = image_hsv[y, x]
            print(
                f"Pixel ({x}, {y}) HSV: "
                f"({int(hue)}, {int(saturation)}, {int(value)})"
            )

    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, show_pixel_hsv)
    cv2.imshow(window_name, preview)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
