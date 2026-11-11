import cv2
import numpy as np


def detect_y_junction(skeleton):
    if skeleton is None:
        raise ValueError("Skeleton image missing")

    binary = (skeleton > 127).astype(np.uint8)
    h, w = binary.shape

    kernel = np.ones((3, 3), np.uint8)
    kernel[1, 1] = 0

    neighbors = cv2.filter2D(binary, cv2.CV_16S, kernel)

    candidates = (
        (binary == 1) & (neighbors >= 3)
    ).astype(np.uint8) * 255

    roi = np.zeros_like(candidates)
    roi[h//4:3*h//4, w//4:3*w//4] = 255
    candidates = cv2.bitwise_and(candidates, roi)

    expanded = cv2.dilate(
        candidates, np.ones((7, 7), np.uint8)
    )

    count, labels, stats, centers = (
        cv2.connectedComponentsWithStats(expanded)
    )

    path = cv2.dilate(
        binary, np.ones((3, 3), np.uint8)
    )

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
        results.append((score, round(cx), round(cy)))

    if not results:
        return None

    results.sort(reverse=True)
    score, cx, cy = results[0]

    if score <= 0:
        return None

    return (cx, cy, score)
