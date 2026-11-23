import cv2
from pathlib import Path

src = Path("captures/algorithm_extra")
dst = Path("captures/algorithm_extra_fixed")
dst.mkdir(parents=True, exist_ok=True)

for file in sorted(src.glob("*.jpg")):
    img = cv2.imread(str(file))

    if img is None:
        print("SKIPPED:", file.name)
        continue

    h, w = img.shape[:2]

    if h < 240 or w < 320:
        print("INVALID SIZE:", file.name, img.shape)
        continue

    # 1. Extract the downward-camera image FIRST.
    cropped = img[0:240, 0:320]

    # 2. Rotate the valid image 180 degrees.
    corrected = cv2.rotate(cropped, cv2.ROTATE_180)

    filename = dst / file.name
    cv2.imwrite(str(filename), corrected)
    print("FIXED:", file.name, corrected.shape)

print("DONE")
