import cv2
import numpy as np
from pathlib import Path

src = Path("captures/final_dataset")
tiles = []

for p in sorted(src.glob("*.jpg")):
    image = cv2.imread(str(p))
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, 75, 255, cv2.THRESH_BINARY_INV)

    tile = np.full((145, 330, 3), 240, dtype=np.uint8)
    tile[20:140, 0:160] = cv2.resize(image, (160, 120))
    tile[20:140, 170:330] = cv2.resize(
        cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), (160, 120)
    )
    cv2.putText(tile, p.stem, (5, 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
    tiles.append(tile)

cols = 4
rows = (len(tiles) + cols - 1) // cols
sheet = np.full((rows * 145, cols * 330, 3), 240, dtype=np.uint8)

for i, tile in enumerate(tiles):
    row, col = divmod(i, cols)
    sheet[row*145:(row+1)*145, col*330:(col+1)*330] = tile

output = "captures/threshold75_review.jpg"
cv2.imwrite(output, sheet)
print("Processed:", len(tiles))
print("Saved:", output)
