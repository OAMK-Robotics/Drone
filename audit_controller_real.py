import csv
from pathlib import Path
from collections import Counter
import cv2
from hover.junction_marker_detector import detect_junction_marker
from hover.junction_hover_controller import JunctionHoverController

folders = [
    Path("captures/drift_20_30cm_20261123_113843"),
    Path("captures/drift_20_30cm_20261123_114430"),
]

for folder in folders:
    controller = JunctionHoverController(target_center=(160, 120))
    counts = Counter()
    with (folder / "observations.csv").open(newline="") as f:
        rows = list(csv.DictReader(f))
    for row in rows:
        if row["event"] != "FRAME" or not row["image"]:
            continue
        frame = cv2.imread(str(folder / row["image"]))
        if frame is None:
            raise RuntimeError("Missing image: " + row["image"])
        marker = detect_junction_marker(frame)
        status, rx, ry = controller.update(marker, phase=row["phase"])
        counts[status] += 1
        if rx != 0 or ry != 0:
            raise RuntimeError("Unexpected correction request during observation")
    print(folder.name, dict(counts))
    print("Reference:", controller.reference, "Latched:", controller.latched)
print("OFFLINE ONLY. NO MOTOR COMMANDS.")
