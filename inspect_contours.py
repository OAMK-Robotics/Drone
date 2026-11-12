import cv2
import numpy as np

mask = cv2.imread("table_circle_mask.jpg", 0)

if mask is None:
    raise RuntimeError("Mask not found")

contours, _ = cv2.findContours(
    mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
)

results = []

for contour in contours:
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)

    if perimeter == 0:
        continue

    x, y, w, h = cv2.boundingRect(contour)

    results.append((
        round(area),
        round(4 * np.pi * area / perimeter**2, 3),
        round(w / h, 3),
        (x, y, w, h)
    ))

print("Total contours:", len(contours))
print("Largest 10 contours:")
print("Area | Circularity | Aspect ratio | Bounding box")

for row in sorted(results, reverse=True)[:10]:
    print(*row, sep=" | ")
