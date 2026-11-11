import cv2
import numpy as np

img = cv2.imread("path_skeleton.jpg", 0)
if img is None:
    raise RuntimeError("Skeleton image missing")

binary = (img > 127).astype(np.uint8)
h, w = binary.shape

# Automatically find junction candidates
kernel = np.ones((3, 3), np.uint8)
kernel[1, 1] = 0
neighbors = cv2.filter2D(binary, cv2.CV_16S, kernel)

candidate_mask = (
    (binary == 1) & (neighbors >= 3)
).astype(np.uint8) * 255

roi = np.zeros_like(candidate_mask)
roi[h//4:3*h//4, w//4:3*w//4] = 255
candidate_mask = cv2.bitwise_and(candidate_mask, roi)

expanded = cv2.dilate(
    candidate_mask, np.ones((7, 7), np.uint8)
)
count, labels, stats, centers = cv2.connectedComponentsWithStats(expanded)

# Connected path region for scoring
path = cv2.dilate(binary, np.ones((3, 3), np.uint8))

yy, xx = np.indices((h, w))
results = []

for i in range(1, count):
    if stats[i, cv2.CC_STAT_AREA] < 5:
        continue

    cx, cy = centers[i]
    dx, dy = xx - cx, yy - cy
    distance = np.hypot(dx, dy)

    ring = (distance >= 20) & (distance <= 50)

    directions = {
        "TABLE_A": (dx < -10) & (dy < -10),
        "TABLE_B": (dx < -10) & (dy > 10),
        "ARM": (dx > 15) & (np.abs(dy) < dx * 0.3)
    }

    scores = {
        name: int(np.count_nonzero(path[ring & region]))
        for name, region in directions.items()
    }

    score = min(scores.values())
    results.append((score, round(cx), round(cy), scores))

if not results:
    raise RuntimeError("No junction candidates found")

results.sort(reverse=True)

result = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

for score, cx, cy, scores in results:
    print(f"Candidate ({cx},{cy}): score={score}, branches={scores}")

best = results[0]
score, cx, cy, scores = best

cv2.circle(result, (cx, cy), 10, (0, 255, 0), 2)
cv2.putText(
    result, "BEST CANDIDATE",
    (max(0, cx - 75), max(15, cy - 15)),
    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1
)

cv2.imwrite("automatic_junction.jpg", result)

print("Selected junction:", (cx, cy))
print("Saved: automatic_junction.jpg")
