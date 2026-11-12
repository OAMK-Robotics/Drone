import cv2

image = cv2.imread("new_path_original.jpg")

if image is None:
    raise RuntimeError("Image not found")

# Approximate junction from current test image
junction = (190, 135)

cv2.circle(image, junction, 8, (0, 255, 255), 2)

# Mark expected branches
cv2.putText(image, "TABLE A", (15, 35),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

cv2.putText(image, "TABLE B", (15, 220),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

cv2.putText(image, "ARM - NO ENTRY", (195, 120),
            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

cv2.imwrite("path_branches_labeled.jpg", image)

print("Saved: path_branches_labeled.jpg")
