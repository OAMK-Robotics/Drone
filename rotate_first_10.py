from pathlib import Path
import cv2

src = Path("captures/algorithm")
dst = Path("captures/algorithm_rotated")
dst.mkdir(parents=True, exist_ok=True)

for i in range(1, 11):
    name = f"path_{i:02d}.jpg"
    in_file = src / name
    out_file = dst / name

    img = cv2.imread(str(in_file))
    if img is None:
        print("Missing:", in_file)
        continue

    rotated = cv2.rotate(img, cv2.ROTATE_180)
    ok = cv2.imwrite(str(out_file), rotated)
    print(f"{name} -> rotated, saved={ok}")

print("Done.")
