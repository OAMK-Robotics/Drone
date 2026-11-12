import cv2
import numpy as np

image = cv2.imread("path_skeleton.jpg", 0)

if image is None:
    raise RuntimeError("Skeleton image not found")

binary = (image > 127).astype(np.uint8)
height, width = binary.shape

# Count neighboring skeleton pixels
kernel = np.ones((3, 3), dtype=np.uint8)
kernel[1, 1] = 0

neighbors = cv2.filter2D(
    binary, cv2.CV_16S, kernel
)

# Candidate junction pixels have 3+ neighbors
junction_pixels = (
    (binary == 1) & (neighbors >= 3)
).astype(np.uint8) * 255

# Restrict search to central image region
roi = np.zeros_like(junction_pixels)
roi[height//4:3*height//4,
    width//4:3*width//4] = 255

junction_pixels = cv2.bitwise_and(
    junction_pixels, roi
)

# Combine nearby junction pixels into groups
expanded = cv2.dilate(
    junction_pixels,
    np.ones((7, 7), np.uint8)
)

count, labels, stats, centers = (
    cv2.connectedComponentsWithStats(expanded)
)

result = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

candidates = 0

for i in range(1, count):
    x, y, w, h, area = stats[i]

    if area < 5:
        continue

    cx, cy = centers[i]
    center = (round(cx), round(cy))

    candidates += 1

    cv2.circle(result, center, 12, (0, 0, 255), 2)
    cv2.putText(
        result, f"J{candidates}",
        (center[0] + 12, center[1] - 8),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4, (0, 255, 255), 1
    )

    print(f"Candidate J{candidates}: {center}")

cv2.imwrite("junction_candidates.jpg", result)

print("Total junction candidates:", candidates)
print("Saved: junction_candidates.jpg")
