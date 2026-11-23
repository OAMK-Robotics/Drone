import cv2
import numpy as np
from pathlib import Path

root = Path(".")

names = [
    "downward_frame_1.jpg",
    "downward_frame_2.jpg",
    "downward_frame_3.jpg",
    "downward_frame_4.jpg",
    "downward_frame_5.jpg",
    "new_path_original.jpg",
    "table_circle_original.jpg",
    "table_circle_test_2.jpg",
    "straight_test.jpg",
    "video_recovery_test.jpg",
    "downward_test.jpg",
    "downward_test2.jpg",
    "captures/junction_raw_01.png",
]

tiles = []

for i, name in enumerate(names, 1):
    path = root / name
    img = cv2.imread(str(path))

    if img is None:
        print("Missing or unreadable:", name)
        continue

    # Preview only. Do not modify or automatically crop originals.
    original_shape = img.shape
    preview = cv2.resize(img, (320, 240))

    tile = np.full((285, 340, 3), 255, dtype=np.uint8)
    tile[35:275, 10:330] = preview

    cv2.putText(
        tile, f"{i:02d} {path.name[:27]}",
        (8, 23), cv2.FONT_HERSHEY_SIMPLEX,
        0.45, (0, 0, 0), 1
    )

    tiles.append(tile)
    print(i, name, original_shape)

cols = 4
rows = (len(tiles) + cols - 1) // cols

sheet = np.full(
    (rows * 285, cols * 340, 3),
    235, dtype=np.uint8
)

for i, tile in enumerate(tiles):
    row, col = divmod(i, cols)
    sheet[row*285:(row+1)*285,
          col*340:(col+1)*340] = tile

output = Path("captures/older_images_contact_sheet.jpg")
cv2.imwrite(str(output), sheet)

print("Contact sheet:", output)
print("Candidate images:", len(tiles))
