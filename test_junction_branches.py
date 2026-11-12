import cv2
import numpy as np

image = cv2.imread("path_skeleton.jpg", 0)

if image is None:
    raise RuntimeError("Skeleton not found")

binary = (image > 127).astype(np.uint8)

# J4 is a manually selected hypothesis, NOT an automatic result.
cx, cy = 144, 138
radius = 45

# Inspect three regions outside the immediate junction
regions = {
    "TABLE_A": (cx-radius, cy-radius, cx-10, cy-10),
    "TABLE_B": (cx-radius, cy+10, cx-10, cy+radius),
    "ARM": (cx+15, cy-12, cx+radius, cy+12)
}

result = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

for name, (x1, y1, x2, y2) in regions.items():
    region = binary[y1:y2, x1:x2]
    pixels = int(region.sum())

    detected = pixels >= 8

    print(f"{name}: pixels={pixels}, detected={detected}")

    color = (0, 255, 0) if detected else (0, 0, 255)

    cv2.rectangle(result, (x1, y1), (x2, y2), color, 1)

cv2.circle(result, (cx, cy), 6, (0, 255, 255), 2)

cv2.imwrite("junction_branch_check.jpg", result)
print("Saved: junction_branch_check.jpg")
