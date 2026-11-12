import cv2
import numpy as np

from table_detector import detect_table_zone
from junction_detector import detect_y_junction


def make_skeleton(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    mask = cv2.adaptiveThreshold(
        gray, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        31, 10
    )

    mask = cv2.morphologyEx(
        mask, cv2.MORPH_OPEN,
        np.ones((3, 3), np.uint8)
    )

    skeleton = np.zeros_like(mask)
    element = cv2.getStructuringElement(
        cv2.MORPH_CROSS, (3, 3)
    )

    while cv2.countNonZero(mask) > 0:
        eroded = cv2.erode(mask, element)
        opened = cv2.dilate(eroded, element)
        skeleton = cv2.bitwise_or(
            skeleton, cv2.subtract(mask, opened)
        )
        mask = eroded

    return skeleton


tests = [
    ("table_circle_original.jpg", "TABLE_ZONE"),
    ("table_circle_test_2.jpg", "TABLE_ZONE"),
    ("path_branches_labeled.jpg", "JUNCTION"),
]

for filename, expected in tests:
    image = cv2.imread(filename)

    if image is None:
        raise RuntimeError(f"Missing image: {filename}")

    table = detect_table_zone(image)

    if table:
        detected = "TABLE_ZONE"
    else:
        skeleton = make_skeleton(image)
        junction = detect_y_junction(skeleton)
        detected = "JUNCTION" if junction else "UNKNOWN"

    result = "PASS" if detected == expected else "FAIL"

    print(
        f"{filename}: "
        f"Expected={expected}, "
        f"Detected={detected}, "
        f"{result}"
    )
