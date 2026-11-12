import cv2
import numpy as np

mask = cv2.imread("new_path_mask.jpg", cv2.IMREAD_GRAYSCALE)

if mask is None:
    raise RuntimeError("new_path_mask.jpg not found")

# Convert mask to binary
binary = (mask > 127).astype(np.uint8) * 255

# Skeletonization using standard OpenCV
skeleton = np.zeros_like(binary)
element = cv2.getStructuringElement(
    cv2.MORPH_CROSS, (3, 3)
)

while cv2.countNonZero(binary) > 0:
    eroded = cv2.erode(binary, element)
    opened = cv2.dilate(eroded, element)

    difference = cv2.subtract(binary, opened)
    skeleton = cv2.bitwise_or(skeleton, difference)

    binary = eroded

cv2.imwrite("path_skeleton.jpg", skeleton)

print("Skeleton saved: path_skeleton.jpg")
