import cv2
import numpy as np
from pathlib import Path

sources = [
    ("A", Path("captures/algorithm")),
    ("B", Path("captures/algorithm_extra_fixed")),
]

tiles = []

for prefix, folder in sources:
    for path in sorted(folder.glob("*.jpg")):
        img = cv2.imread(str(path))

        if img is None:
            print("Unreadable:", path)
            continue

        # First dataset contains raw frames.
        # Extract the documented downward-camera region.
        if prefix == "A":
            if img.shape[0] < 240 or img.shape[1] < 320:
                print("Unexpected dimensions:", path, img.shape)
                continue
            img = img[:240, :320]

        img = cv2.resize(img, (320, 240))

        canvas = np.full((275, 340, 3), 255, dtype=np.uint8)
        canvas[25:265, 10:330] = img

        label = f"{prefix}-{path.stem}"
        cv2.putText(
            canvas, label, (10, 19),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55, (0, 0, 0), 1
        )

        tiles.append(canvas)

if not tiles:
    raise RuntimeError("No valid images found")

columns = 5
rows = (len(tiles) + columns - 1) // columns

sheet = np.full(
    (rows * 275, columns * 340, 3),
    235, dtype=np.uint8
)

for i, tile in enumerate(tiles):
    row, col = divmod(i, columns)
    sheet[row*275:(row+1)*275,
          col*340:(col+1)*340] = tile

output = Path("captures/algorithm_contact_sheet.jpg")
cv2.imwrite(str(output), sheet)

print("Images:", len(tiles))
print("Saved:", output)
