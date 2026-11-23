import csv, cv2
from pathlib import Path
from hover.dual_marker_detector import detect_both
from hover.dual_marker_pose import camera_relative_pose
for name in ("drift_20_30cm_20261123_113843", "drift_20_30cm_20261123_114430"):
    folder = Path("captures") / name
    with (folder / "observations.csv").open(newline="") as f:
        rows = list(csv.DictReader(f))
    rows = [r for r in rows if r["event"] == "FRAME" and r["sdk_height_cm"] == "70" and r["image"]]
    if not rows: raise RuntimeError("No 70cm samples")
    start = float(rows[0]["t_s"])
    poses = []
    for r in rows:
        if float(r["t_s"]) - start > 1.6: break
        frame = cv2.imread(str(folder / r["image"]))
        if frame is None: raise RuntimeError("Missing image")
        pose = camera_relative_pose(detect_both(frame))
        if pose is not None: poses.append((float(r["t_s"]), pose))
    if len(poses) < 2: raise RuntimeError("Insufficient paired detections")
    dt = poses[-1][0] - poses[0][0]
    du = (poses[-1][1][0] - poses[0][1][0]) / dt
    dv = (poses[-1][1][1] - poses[0][1][1]) / dt
    print(name, "pairs:", len(poses), "du/s:", round(du,4), "dv/s:", round(dv,4))
print("OFFLINE ONLY")
