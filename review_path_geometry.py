import cv2
import numpy as np
from pathlib import Path

src = Path("captures/final_dataset")
tiles = []
detected = 0

for file in sorted(src.glob("*.jpg")):
    frame = cv2.imread(str(file))
    if frame is None:
        continue

    gray = cv2.GaussianBlur(
        cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0
    )
    blackhat = cv2.morphologyEx(
        gray, cv2.MORPH_BLACKHAT,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (31, 31))
    )
    _, mask = cv2.threshold(
        blackhat, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)
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

    overlay = frame.copy()
    overlay[skeleton > 0] = (0, 255, 255)

    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    cv2.drawMarker(
        overlay, (cx, cy), (0, 255, 0),
        cv2.MARKER_CROSS, 12, 1
    )

    ys, xs = np.where(skeleton > 0)
    if len(xs):
        distances = (xs - cx)**2 + (ys - cy)**2
        nearest = int(np.argmin(distances))
        x, y = int(xs[nearest]), int(ys[nearest])
        distance = float(np.sqrt(distances[nearest]))

        if distance <= 60:
            cv2.circle(overlay, (x, y), 5, (0, 0, 255), 2)
            detected += 1

    tile = np.full((270, 320, 3), 230, dtype=np.uint8)
    tile[30:270] = overlay
    cv2.putText(
        tile, file.stem, (5, 20),
        cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 1
    )
    tiles.append(tile)

cols = 5
rows = (len(tiles) + cols - 1) // cols
sheet = np.full(
    (rows * 270, cols * 320, 3), 230, dtype=np.uint8
)

for i, tile in enumerate(tiles):
    r, c = divmod(i, cols)
    sheet[r*270:(r+1)*270, c*320:(c+1)*320] = tile

output = "captures/path_geometry_review.jpg"
cv2.imwrite(output, sheet)
print("Images:", len(tiles))
print("Near-centre candidates:", detected)
print("Saved:", output)
