import cv2
from pathlib import Path

out = Path("captures/older_selected")
out.mkdir(parents=True, exist_ok=True)

selected = {
    "07": "table_circle_original.jpg",
    "08": "table_circle_test_2.jpg",
    "09": "straight_test.jpg",
    "10": "video_recovery_test.jpg"
}

for number, name in selected.items():
    img = cv2.imread(name)

    if img is None:
        print("SKIPPED:", name)
        continue

    if number in ("09", "10"):
        if img.shape[0] < 240 or img.shape[1] < 320:
            print("INVALID SIZE:", name, img.shape)
            continue

        # Extract valid downward camera region first.
        img = img[:240, :320]

        # Mild denoising; preserve path edges.
        img = cv2.medianBlur(img, 3)

    target = out / f"old_{number}.jpg"

    saved = cv2.imwrite(str(target), img)
    print(target.name, "saved:", saved, "shape:", img.shape)

print("DONE")
