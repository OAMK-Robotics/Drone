import cv2

image = cv2.imread("table_circle_original.jpg")
mask = cv2.imread("table_circle_mask.jpg", cv2.IMREAD_GRAYSCALE)

if image is None or mask is None:
    raise RuntimeError("Required images not found")

# Extract contours from the binary mask
contours, _ = cv2.findContours(
    mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
)

result = image.copy()
candidates = 0

for contour in contours:
    area = cv2.contourArea(contour)

    # Filter out small noise
    if area < 500:
        continue

    candidates += 1

    # Draw candidate contours
    cv2.drawContours(result, [contour], -1, (0, 0, 255), 2)

    x, y, w, h = cv2.boundingRect(contour)

    cv2.rectangle(
        result, (x, y), (x + w, y + h),
        (0, 255, 0), 1
    )

    print(
        f"Candidate {candidates}: "
        f"area={area:.0f}, "
        f"bounding box=({x},{y},{w},{h})"
    )

cv2.imwrite("table_zone_candidates.jpg", result)

print("Total candidates:", candidates)
print("Saved: table_zone_candidates.jpg")
