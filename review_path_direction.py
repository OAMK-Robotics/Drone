import cv2
import numpy as np
from pathlib import Path

folder = Path("captures/final_dataset")
tiles = []

for file in sorted(folder.glob("*.jpg")):
    frame = cv2.imread(str(file))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    blackhat = cv2.morphologyEx(
        gray, cv2.MORPH_BLACKHAT,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (31, 31))
    )

    _, mask = cv2.threshold(
        blackhat, 0, 255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )

    mask = cv2.morphologyEx(
        mask, cv2.MORPH_OPEN,
        np.ones((3, 3), np.uint8)
    )

    skeleton = np.zeros_like(mask)
    element = cv2.getStructuringElement(
        cv2.MORPH_CROSS, (3, 3)
    )

    while cv2.countNonZero(mask):
        eroded = cv2.erode(mask, element)
        opened = cv2.dilate(eroded, element)
        skeleton = cv2.bitwise_or(
            skeleton, cv2.subtract(mask, opened)
        )
        mask = eroded

    result = frame.copy()
    result[skeleton > 0] = (0, 255, 255)

    h, w = frame.shape[:2]
    center = np.array([w // 2, h // 2])

    cv2.drawMarker(
        result, tuple(center), (0, 255, 0),
        cv2.MARKER_CROSS, 15, 1
    )

    ys, xs = np.where(skeleton > 0)
    status = "NO PATH"

    if len(xs):
        points = np.column_stack((xs, ys)).astype(float)
        distance = np.linalg.norm(points - center, axis=1)
        nearest_index = np.argmin(distance)
        nearest = points[nearest_index]

        if distance[nearest_index] <= 60:
            local = points[
                np.linalg.norm(points - nearest, axis=1) <= 30
            ]

            cv2.circle(
                result, tuple(nearest.astype(int)),
                4, (0, 0, 255), -1
            )

            if len(local) >= 10:
                covariance = np.cov(local.T)
                values, vectors = np.linalg.eigh(covariance)

                direction = vectors[:, np.argmax(values)]
                ratio = values[-1] / max(values[0], 1e-6)

                if ratio >= 3:
                    p1 = nearest - direction * 35
                    p2 = nearest + direction * 35

                    cv2.line(
                        result,
                        tuple(p1.astype(int)),
                        tuple(p2.astype(int)),
                        (255, 255, 0), 2
                    )
                    status = "TANGENT"
                else:
                    status = "AMBIGUOUS"
            else:
                status = "INSUFFICIENT"

    tile = np.full((275, 320, 3), 235, dtype=np.uint8)
    tile[30:270] = result

    cv2.putText(
        tile, f"{file.stem} | {status}", (5, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.43, (0, 0, 0), 1
    )
    tiles.append(tile)
    print(file.stem, status)

cols = 5
rows = (len(tiles) + cols - 1) // cols
sheet = np.full(
    (rows * 275, cols * 320, 3),
    235, dtype=np.uint8
)

for i, tile in enumerate(tiles):
    r, c = divmod(i, cols)
    sheet[r*275:(r+1)*275, c*320:(c+1)*320] = tile

output = "captures/path_direction_review.jpg"
cv2.imwrite(output, sheet)

print("Processed:", len(tiles))
print("Saved:", output)
