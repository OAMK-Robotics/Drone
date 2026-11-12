import cv2
import numpy as np

mask = cv2.imread("path_adaptive_mask.jpg", cv2.IMREAD_GRAYSCALE)

if mask is None:
    raise RuntimeError("Mask not found")

# Remove small isolated regions
kernel = np.ones((3, 3), np.uint8)

cleaned = cv2.morphologyEx(
    mask,
    cv2.MORPH_OPEN,
    kernel
)

cv2.imwrite("path_cleaned.jpg", cleaned)

print("Saved: path_cleaned.jpg")
