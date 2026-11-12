import cv2
import numpy as np

dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)

detector = cv2.aruco.ArucoDetector(
    dictionary,
    cv2.aruco.DetectorParameters()
)

marker = cv2.aruco.generateImageMarker(
    dictionary, 23, 400
)

print("Camera resolution: 320 x 240")
print("Expected marker ID: 23")
print("-" * 45)

for size in [24, 32, 48, 64]:
    for angle in [0, 25]:

        small = cv2.resize(
            marker, (size, size),
            interpolation=cv2.INTER_NEAREST
        )

        padding = size // 2
        patch_size = size + 2 * padding
        patch = np.full(
            (patch_size, patch_size), 255,
            dtype=np.uint8
        )
        patch[
            padding:padding + size,
            padding:padding + size
        ] = small

        matrix = cv2.getRotationMatrix2D(
            (patch_size / 2, patch_size / 2),
            angle, 1
        )

        patch = cv2.warpAffine(
            patch, matrix,
            (patch_size, patch_size),
            borderValue=255
        )

        frame = np.full(
            (240, 320), 255, dtype=np.uint8
        )

        x = (320 - patch_size) // 2
        y = (240 - patch_size) // 2

        frame[
            y:y + patch_size,
            x:x + patch_size
        ] = patch

        corners, ids, rejected = detector.detectMarkers(frame)

        found = (
            ids.flatten().tolist()
            if ids is not None else []
        )

        result = "PASS" if 23 in found else "FAIL"

        print(
            f"Size: {size:2d}px | "
            f"Rotation: {angle:2d} | {result}"
        )
