import cv2
import numpy as np

image = cv2.imread("table_circle_original.jpg")
mask = cv2.imread("table_circle_mask.jpg", 0)

if image is None or mask is None:
    raise RuntimeError("Required images not found")

# Find connected white regions in the mask
contours, _ = cv2.findContours(
    mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
)

result = image.copy()
candidate_count = 0

for contour in contours:
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)

    if area < 500 or perimeter == 0:
        continue

    circularity = 4 * np.pi * area / (perimeter ** 2)

    x, y, w, h = cv2.boundingRect(contour)
    aspect_ratio = w / h

    # Preliminary shape filters
    if 0.5 < aspect_ratio < 1.8 and circularity > 0.15:
        candidate_count += 1

        cv2.drawContours(
            result, [contour], -1, (0, 0, 255), 2
        )

        cv2.rectangle(
            result, (x, y), (x + w, y + h),
            (0, 255, 0), 2
        )

        print(
            f"Candidate {candidate_count}: "
            f"Area={area:.0f}, "
            f"Circularity={circularity:.2f}, "
            f"AspectRatio={aspect_ratio:.2f}"
        )

cv2.imwrite("table_shape_detection.jpg", result)

print("Total shape candidates:", candidate_count)
print("Saved: table_shape_detection.jpg")
