import cv2
import numpy as np

from table_detector import detect_table_zone
from junction_detector import detect_y_junction

# Blank white tabletop, without black tape
image = np.full((240, 320, 3), 255, dtype=np.uint8)

table = detect_table_zone(image)

# No detected black path -> empty skeleton
skeleton = np.zeros((240, 320), dtype=np.uint8)
junction = detect_y_junction(skeleton)

detected = (
    "TABLE_ZONE" if table
    else "JUNCTION" if junction
    else "UNKNOWN"
)

assert detected == "UNKNOWN", (
    f"Expected UNKNOWN, got {detected}"
)

print("Blank tabletop:", detected)
print("Safety test: PASS")
