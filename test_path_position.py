import cv2
import numpy as np

image = cv2.imread("downward_frame_4_cropped.jpg")
mask = cv2.imread("path_cleaned.jpg", 0)

if image is None or mask is None:
    raise RuntimeError("Required images not found")

height, width = mask.shape

# Inspect a horizontal region near image center
y = height // 2
row = mask[y]

# Find groups of white pixels
points = np.where(row > 127)[0]
groups = np.split(points, np.where(np.diff(points) > 1)[0] + 1)

# Keep only segments with reasonable width
segments = [
    (int(g[0]), int(g[-1]))
    for g in groups
    if len(g) >= 3
]

# Draw camera center and detected segments
cv2.line(image, (width // 2, 0), (width // 2, height - 1),
         (0, 255, 0), 1)

cv2.line(image, (0, y), (width - 1, y),
         (255, 0, 0), 1)

for x1, x2 in segments:
    center = (x1 + x2) // 2
    cv2.circle(image, (center, y), 5, (0, 0, 255), -1)
    print("Detected line segment:", x1, "-", x2, "Center:", center)

cv2.imwrite("path_position_test.jpg", image)
print("Saved: path_position_test.jpg")
