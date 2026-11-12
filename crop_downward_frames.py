import cv2
from pathlib import Path

for path in sorted(Path(".").glob("downward_frame_*.jpg")):
    frame = cv2.imread(str(path))

    if frame is None or frame.shape[0] < 240 or frame.shape[1] < 320:
        print("SKIP:", path.name)
        continue

    cropped = frame[0:240, 0:320].copy()

    output = path.with_name(path.stem + "_cropped.jpg")
    cv2.imwrite(str(output), cropped)

    print("Saved:", output.name, cropped.shape)
