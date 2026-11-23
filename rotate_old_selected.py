import cv2
from pathlib import Path

folder = Path("captures/older_selected")
files = ["old_07.jpg", "old_08.jpg", "old_09.jpg", "old_10.jpg"]

for name in files:
    path = folder / name
    img = cv2.imread(str(path))

    if img is None:
        print("SKIPPED:", name)
        continue

    rotated = cv2.rotate(img, cv2.ROTATE_180)
    saved = cv2.imwrite(str(path), rotated)
    print(f"{name} -> rotated, saved={saved}")

print("Done.")
