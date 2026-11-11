import cv2
import numpy as np

ref = cv2.imread("references/table_zone_reference.jpg", 0)
test = cv2.imread("table_circle_test_2.jpg", 0)

if ref is None or test is None:
    raise RuntimeError("Images not found")

# Approximate pad regions based on the two captured images.
# These are test-specific coordinates, not a universal detector.
ref_mask = np.full(ref.shape, 255, dtype=np.uint8)
test_mask = np.full(test.shape, 255, dtype=np.uint8)

cv2.rectangle(ref_mask, (55, 45), (165, 155), 0, -1)
cv2.rectangle(test_mask, (45, 40), (165, 155), 0, -1)

orb = cv2.ORB_create(nfeatures=1000)

kp1, des1 = orb.detectAndCompute(ref, ref_mask)
kp2, des2 = orb.detectAndCompute(test, test_mask)

if des1 is None or des2 is None:
    print("No features found outside Mission Pad.")
else:
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    pairs = matcher.knnMatch(des1, des2, k=2)

    good = [
        m for pair in pairs if len(pair) == 2
        for m, n in [pair] if m.distance < 0.75 * n.distance
    ]

    print("Matches outside Pad:", len(good))

    result = cv2.drawMatches(
        ref, kp1, test, kp2, good[:40], None,
        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
    )

    cv2.imwrite("orb_without_pad.jpg", result)
    print("Saved: orb_without_pad.jpg")
