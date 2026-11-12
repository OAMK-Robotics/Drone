import cv2
import numpy as np

img = cv2.imread("path_skeleton.jpg", 0)

if img is None:
    raise RuntimeError("Skeleton image missing")

binary = (img > 127).astype(np.uint8)

# Bridge small gaps for this diagnostic only
connected = cv2.dilate(
    binary, np.ones((3, 3), np.uint8)
)

# Find connected components
count, labels, stats, centers = cv2.connectedComponentsWithStats(
    connected, connectivity=8
)

# Find component nearest the proposed junction
cx, cy = 144, 138
ys, xs = np.where(connected > 0)

if len(xs) == 0:
    raise RuntimeError("No path pixels found")

nearest = np.argmin((xs-cx)**2 + (ys-cy)**2)
component_id = labels[ys[nearest], xs[nearest]]

# Isolate only the junction-connected region
component = (labels == component_id).astype(np.uint8)

regions = {
    "TABLE_A": (99, 93, 134, 128),
    "TABLE_B": (99, 148, 134, 183),
    "ARM": (159, 126, 189, 150)
}

result = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

for name, (x1, y1, x2, y2) in regions.items():
    pixels = int(component[y1:y2, x1:x2].sum())
    reached = pixels >= 8

    print(f"{name}: connected_pixels={pixels}, reached={reached}")

    color = (0, 255, 0) if reached else (0, 0, 255)
    cv2.rectangle(result, (x1, y1), (x2, y2), color, 1)

cv2.circle(result, (cx, cy), 7, (0, 255, 255), 2)

cv2.imwrite("junction_connectivity.jpg", result)
print("Saved: junction_connectivity.jpg")
