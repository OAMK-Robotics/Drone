import cv2
from table_detector import detect_table_zone

tests = [
    ("table_circle_original.jpg", True),
    ("table_circle_test_2.jpg", True),
    ("path_branches_labeled.jpg", False),
]

passed = 0

for filename, expected in tests:
    image = cv2.imread(filename)

    if image is None:
        raise RuntimeError(f"Missing image: {filename}")

    candidates = detect_table_zone(image)
    detected = len(candidates) > 0

    success = detected == expected
    passed += success

    print(
        f"{filename}: "
        f"candidates={len(candidates)}, "
        f"{'PASS' if success else 'FAIL'}"
    )

print(f"Reusable module tests: {passed}/{len(tests)} PASSED")
