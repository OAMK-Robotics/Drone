import cv2

image = cv2.imread("downward_frame_4_cropped.jpg")
if image is None:
    raise RuntimeError("Image not found")

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
gray = cv2.GaussianBlur(gray, (5, 5), 0)

mask = cv2.adaptiveThreshold(
    gray, 255,
    cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
    cv2.THRESH_BINARY_INV,
    31, 10
)

cv2.imwrite("path_adaptive_mask.jpg", mask)
print("Saved: path_adaptive_mask.jpg")
