import cv2
import numpy as np

LANDMARKS = {
    23: "HOME",
    24: "BRANCH_A",
    25: "BRANCH_B",
}

dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)

detector = cv2.aruco.ArucoDetector(
    dictionary,
    cv2.aruco.DetectorParameters()
)


def detect_landmarks(frame):
    if frame is None or frame.size == 0:
        return []

    gray = (
        cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if frame.ndim == 3 else frame
    )

    corners, ids, _ = detector.detectMarkers(gray)

    if ids is None:
        return []

    results = []

    for index, marker_id in enumerate(ids.flatten()):
        marker_id = int(marker_id)

        if marker_id not in LANDMARKS:
            continue

        center = corners[index][0].mean(axis=0)

        results.append({
            "id": marker_id,
            "name": LANDMARKS[marker_id],
            "center": tuple(map(float, center))
        })

    return results


if __name__ == "__main__":
    for marker_id in [23, 24, 25, 5]:
        frame = np.full((240, 320), 255, dtype=np.uint8)

        marker = cv2.aruco.generateImageMarker(
            dictionary, marker_id, 80
        )

        frame[80:160, 120:200] = marker

        result = detect_landmarks(frame)
        detected = [item["id"] for item in result]

        expected = (
            [marker_id] if marker_id in LANDMARKS else []
        )

        assert detected == expected, (
            f"Marker {marker_id}: expected {expected}, got {detected}"
        )

        print(f"PASS: Marker {marker_id} -> {result}")

    print("ALL 4 TESTS PASSED")
