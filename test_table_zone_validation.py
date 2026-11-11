import cv2
import numpy as np

image = cv2.imread("table_circle_test_2.jpg")

if image is None:
    raise RuntimeError("Test image not found")

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

result = image.copy()
candidates = []

for contour in contours:
    area = cv2.contourArea(contour)
    x, y, w, h = cv2.boundingRect(contour)

    if 7000 < area < 18000 and 0.85 < w / h < 1.2:
        candidates.append((area, x, y, w, h))

        cv2.drawContours(
            result, [contour], -1,
            (0, 255, 0), 2
        )

        cv2.rectangle(
            result, (x, y), (x+w, y+h),
            (0, 0, 255), 2
        )

print("Candidates:", len(candidates))

for item in candidates:
    print("Area, X, Y, W, H:", item)

cv2.imwrite("table_zone_validation.jpg", result)
