import cv2
from pathlib import Path

a = Path("captures/algorithm")
b = Path("captures/algorithm_extra_fixed")
out = Path("captures/final_dataset")
out.mkdir(parents=True, exist_ok=True)

count = 0

for prefix, folder in [("A", a), ("B", b)]:
    for file in sorted(folder.glob("*.jpg")):
        img = cv2.imread(str(file))

        if img is None or img.shape[0] < 240 or img.shape[1] < 320:
            print("SKIPPED:", file)
            continue

        if prefix == "A":
            img = img[0:240, 0:320]
            img = cv2.rotate(img, cv2.ROTATE_180)
        else:
            img = img[0:240, 0:320]

        target = out / f"{prefix}_{file.name}"

        if cv2.imwrite(str(target), img):
            count += 1
            print("Saved:", target.name)

print(f"TOTAL: {count}/30")
