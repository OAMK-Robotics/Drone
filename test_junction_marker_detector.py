import cv2
import numpy as np
from junction_marker_detector import detect_junction_marker

dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)

def make_frame(marker_id):
    frame = np.full((240, 320), 255, dtype=np.uint8)
    marker = cv2.aruco.generateImageMarker(
        dictionary, marker_id, 48
    )
    frame[96:144, 136:184] = marker
    return frame


# Test 1: Correct junction marker
result = detect_junction_marker(make_frame(23))
assert result is not None and result["id"] == 23
print("PASS: Junction ID 23 detected")

# Test 2: Different marker ID
result = detect_junction_marker(make_frame(5))
assert result is None
print("PASS: Wrong ID rejected")

# Test 3: Blank tabletop
blank = np.full((240, 320), 255, dtype=np.uint8)
assert detect_junction_marker(blank) is None
print("PASS: Blank image rejected")

print("Junction marker tests: 3/3 PASSED")
