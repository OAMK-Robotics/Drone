import cv2
import numpy as np

def detect_table_zone(image):
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

    contours, _ = cv2.findContours(
        mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
    )

    candidates = []

    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)

        if h > 0 and 7000 < area < 18000 and 0.85 < w / h < 1.2:
            candidates.append((area, x, y, w, h))

    return candidates


tests = [
    ("table_circle_original.jpg", True),
    ("table_circle_test_2.jpg", True),
    ("path_branches_labeled.jpg", False),
]

passed = 0

for filename, expected in tests:
    image = cv2.imread(filename)

    if image is None:
        raise RuntimeError(f"Missing image: {filename}")

    candidates = detect_table_zone(image)
    detected = len(candidates) > 0
    success = detected == expected

    if success:
        passed += 1

    print(
        f"{filename}: "
        f"detected={detected}, "
        f"candidates={len(candidates)}, "
        f"{'PASS' if success else 'FAIL'}"
    )

print(f"Offline detection tests: {passed}/{len(tests)} PASSED")
