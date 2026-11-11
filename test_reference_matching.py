import cv2

reference = cv2.imread("references/table_zone_reference.jpg", 0)

orb = cv2.ORB_create(nfeatures=1000)
kp1, des1 = orb.detectAndCompute(reference, None)

images = [
    "table_circle_original.jpg",
    "new_path_original.jpg",
    "table_circle_test_2.jpg"
]

for filename in images:
    image = cv2.imread(filename, 0)

    if image is None:
        print(filename, "NOT FOUND")
        continue

    kp2, des2 = orb.detectAndCompute(image, None)

    if des1 is None or des2 is None:
        print(filename, "No descriptors")
        continue

    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    pairs = matcher.knnMatch(des1, des2, k=2)

    good = [
        m for pair in pairs if len(pair) == 2
        for m, n in [pair] if m.distance < 0.75 * n.distance
    ]

    print(filename, "Good ORB matches:", len(good))


