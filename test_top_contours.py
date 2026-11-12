import cv2

image = cv2.imread("table_circle_original.jpg")
mask = cv2.imread("table_circle_mask.jpg", 0)

if image is None or mask is None:
    raise RuntimeError("Required images not found")

contours, _ = cv2.findContours(
    mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
)

largest = sorted(
    contours,
    key=cv2.contourArea,
    reverse=True
)[:3]

colors = [
    (0, 0, 255),   # Red
    (0, 255, 0),   # Green
    (255, 0, 0)    # Blue
]

result = image.copy()

for i, contour in enumerate(largest):
    area = cv2.contourArea(contour)

    cv2.drawContours(
        result, [contour], -1, colors[i], 2
    )

    x, y, w, h = cv2.boundingRect(contour)

    print(
        f"Candidate {i+1}: "
        f"Area={area:.0f}, "
        f"Box=({x},{y},{w},{h})"
    )

cv2.rectangle(result, (0, 0), (319, 239), (255, 255, 255), 1)

cv2.imwrite("table_contours_top3.jpg", result)

print("Saved: table_contours_top3.jpg")
