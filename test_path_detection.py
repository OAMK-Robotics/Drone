import cv2

image = cv2.imread("downward_frame_4_cropped.jpg")

if image is None:
    raise RuntimeError("Test image not found")

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Detect dark regions
_, mask = cv2.threshold(
    gray, 75, 255, cv2.THRESH_BINARY_INV
)

# Save results for inspection
cv2.imwrite("path_detection_mask.jpg", mask)

print("Image dimensions:", image.shape)
print("Mask saved: path_detection_mask.jpg")
