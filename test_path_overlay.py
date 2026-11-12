import cv2

image = cv2.imread("new_path_original.jpg")
mask = cv2.imread("new_path_mask.jpg", 0)

if image is None or mask is None:
    raise RuntimeError("Required image missing")

# Detect candidate path boundaries
edges = cv2.Canny(mask, 50, 150)

# Overlay detected edges in red
result = image.copy()
result[edges > 0] = (0, 0, 255)

# Mark image center
h, w = image.shape[:2]
cv2.drawMarker(
    result, (w // 2, h // 2),
    (0, 255, 0),
    cv2.MARKER_CROSS, 18, 2
)

cv2.imwrite("path_overlay.jpg", result)

print("Saved: path_overlay.jpg")
