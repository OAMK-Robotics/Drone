import cv2

ref = cv2.imread("references/table_zone_reference.jpg", 0)
test = cv2.imread("table_circle_test_2.jpg", 0)

if ref is None or test is None:
    raise RuntimeError("Reference or test image missing")

orb = cv2.ORB_create(nfeatures=1000)

kp1, des1 = orb.detectAndCompute(ref, None)
kp2, des2 = orb.detectAndCompute(test, None)

matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
pairs = matcher.knnMatch(des1, des2, k=2)

good = [
    m for pair in pairs if len(pair) == 2
    for m, n in [pair] if m.distance < 0.75 * n.distance
]

result = cv2.drawMatches(
    ref, kp1, test, kp2,
    good[:40], None,
    flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
)

cv2.imwrite("orb_matches.jpg", result)

print("Good matches:", len(good))
print("Saved: orb_matches.jpg")
