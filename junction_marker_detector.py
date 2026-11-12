import cv2
import numpy as np

DICTIONARY = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)

DETECTOR = cv2.aruco.ArucoDetector(
    DICTIONARY,
    cv2.aruco.DetectorParameters()
)


def detect_junction_marker(frame):
    """Return ID 23 marker information, or None."""

    if frame is None or frame.size == 0:
        return None

    if len(frame.shape) == 3:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        gray = frame

    corners, ids, _ = DETECTOR.detectMarkers(gray)

    if ids is None:
        return None

    for i, marker_id in enumerate(ids.flatten()):
        if marker_id != 23:
            continue

        points = corners[i][0]
        center = np.mean(points, axis=0)

        return {
            "id": 23,
            "center": (
                float(center[0]),
                float(center[1])
            ),
            "corners": points.tolist()
        }

    return None
