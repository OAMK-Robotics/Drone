import cv2
import numpy as np
from landmark_detector import detect_landmarks, dictionary

def build_observation(frame, path_label="UNKNOWN"):
    return {
        "path_label": path_label,
        "landmarks": detect_landmarks(frame)
    }

if __name__ == "__main__":
    for marker_id, expected in [
        (23, "HOME"),
        (24, "BRANCH_A"),
        (25, "BRANCH_B")
    ]:
        frame = np.full((240, 320), 255, dtype=np.uint8)
        marker = cv2.aruco.generateImageMarker(
            dictionary, marker_id, 80
        )
        frame[80:160, 120:200] = marker

        result = build_observation(frame, "JUNCTION")
        names = [m["name"] for m in result["landmarks"]]

        assert result["path_label"] == "JUNCTION"
        assert names == [expected]

        print(f"PASS: path=JUNCTION, landmark={expected}")

    print("ALL OBSERVATION TESTS PASSED")
