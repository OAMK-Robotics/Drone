import cv2
import numpy as np

from vision_label_adapter import resolve_vision_label
from navigation_controller import NavigationController


dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)


def make_frame(marker_id=None):
    frame = np.full(
        (240, 320, 3), 255, dtype=np.uint8
    )

    if marker_id is not None:
        marker = cv2.aruco.generateImageMarker(
            dictionary, marker_id, 48
        )
        frame[96:144, 136:184] = cv2.cvtColor(
            marker, cv2.COLOR_GRAY2BGR
        )

    return frame


# Test 1: Main junction identified
assert resolve_vision_label(
    make_frame(23), "JUNCTION"
) == "MAIN_JUNCTION"
print("PASS: ArUco 23 produces MAIN_JUNCTION")


# Test 2: Wrong marker does not confirm junction
assert resolve_vision_label(
    make_frame(5), "JUNCTION"
) == "JUNCTION"
print("PASS: Wrong marker does not confirm junction")


# Test 3: No marker preserves UNKNOWN
assert resolve_vision_label(
    make_frame(), "UNKNOWN"
) == "UNKNOWN"
print("PASS: No marker preserves UNKNOWN")


# Test 4: Full offline controller integration
controller = NavigationController()

assert controller.update("JUNCTION") == "SELECT_TABLE_A"
assert controller.update("TABLE_ZONE", 7) == "RETURN_TO_JUNCTION"

false_junction = resolve_vision_label(
    make_frame(5), "JUNCTION"
)

assert controller.update(false_junction) == "RETURN_TO_JUNCTION"

main_junction = resolve_vision_label(
    make_frame(23), "JUNCTION"
)

assert controller.update(main_junction) == "SELECT_TABLE_B"

print("PASS: Camera marker -> Controller -> Table B")
print("Integration tests: 4/4 PASSED")
