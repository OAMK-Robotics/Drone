import cv2
from .junction_marker_detector import DETECTOR
def detect_both(frame):
    if frame is None or frame.size == 0: return {}
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    corners, ids, _ = DETECTOR.detectMarkers(gray)
    if ids is None: return {}
    return {int(i): {"id": int(i), "center": tuple(map(float, c[0].mean(axis=0))), "corners": c[0].tolist()} for i, c in zip(ids.flatten(), corners) if int(i) in (23, 25)}
