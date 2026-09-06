import cv2


class OpenCV:
    def __init__(self):
        self.bounding_boxes = []

    def process_image(self, image_hsv, blue):
        blue_lower = (100, 240, 100)
        blue_upper = (120, 255, 255)
        yellow_lower = (20, 100, 100)
        yellow_upper = (30, 255, 255)

        if blue:
            mask = cv2.inRange(image_hsv, blue_lower, blue_upper)
        else:
            mask = cv2.inRange(image_hsv, yellow_lower, yellow_upper)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
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
