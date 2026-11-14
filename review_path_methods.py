import cv2
import numpy as np
from pathlib import Path

tiles = []
files = sorted(Path("captures/final_dataset").glob("*.jpg"))

for file in files:
    img = cv2.imread(str(file))
    gray = cv2.GaussianBlur(
        cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (5, 5), 0
    )

    adaptive = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV, 31, 10
    )
    adaptive = cv2.morphologyEx(
        adaptive, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)
    )

    blackhat = cv2.morphologyEx(
        gray, cv2.MORPH_BLACKHAT,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (31, 31))
    )
    _, blackhat = cv2.threshold(
        blackhat, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )

    tile = np.full((150, 495, 3), 235, dtype=np.uint8)
    for j, image in enumerate([img, adaptive, blackhat]):
        if image.ndim == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        tile[25:145, j*165:j*165+160] = cv2.resize(image, (160, 120))

    cv2.putText(tile, file.stem, (5, 17),
                cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 0), 1)
    tiles.append(tile)

cols = 3
rows = (len(tiles) + cols - 1) // cols
sheet = np.full((rows*150, cols*495, 3), 235, dtype=np.uint8)

for i, tile in enumerate(tiles):
    y, x = divmod(i, cols)
    sheet[y*150:(y+1)*150, x*495:(x+1)*495] = tile

output = "captures/path_methods_review.jpg"
cv2.imwrite(output, sheet)
print("Processed:", len(tiles))
print("Saved:", output)
