"""Offline ArUco image-drift measurement for Tello takeoff recordings.

Does not connect to any drone, import djitellopy, or send commands.
Requires: opencv-contrib-python (cv2.aruco), Python >=3.9.

Usage:
  python vision_drift_offline.py --self-test
  python vision_drift_offline.py captures/takeoff40_20261116_143116
"""
import argparse
import csv
import math
import statistics
from pathlib import Path

import cv2

MARKERS = (23, 25)
REFERENCE_AFTER_SECONDS = 2.5
MIN_MARKER_SIDE_PX = 9.0
MAX_REFERENCE_SIDE_PX = 32.0
MAX_MARKER_DISAGREEMENT_PX = 25.0
DEADBAND_PX = 4.0
MAX_IMAGE_SHIFT_PX = 8.0


def clamp(value, bound):
    return max(-bound, min(bound, value))


def expected_center(reference_center, reference_side, current_side, image_center):
    """Assuming only altitude changes, project stored reference around image principal point.

    Ideal pinhole projection on a planar surface, assuming no yaw/tilt/translation.
    Outputs pixel location expected at changed marker scale. Not an inertial estimate.
    """
    scale = current_side / reference_side
    return (image_center[0] + scale * (reference_center[0] - image_center[0]),
            image_center[1] + scale * (reference_center[1] - image_center[1]))


def score_error(dx, dy, deadband=DEADBAND_PX, limit=MAX_IMAGE_SHIFT_PX):
    """Bounded *image* shift request in pixels; NOT a Tello RC command."""
    return (0.0 if abs(dx) <= deadband else round(clamp(-dx, limit), 1),
            0.0 if abs(dy) <= deadband else round(clamp(-dy, limit), 1))


def image_measurements(frame, detector):
    corners, ids, _ = detector.detectMarkers(frame)
    if ids is None:
        return {}
    out = {}
    for i, marker_id in enumerate(ids.flatten()):
        marker_id = int(marker_id)
        if marker_id not in MARKERS:
            continue
        pts = corners[i][0]
        sides = [float(math.hypot(*(pts[(j + 1) % 4] - pts[j]))) for j in range(4)]
        size = statistics.median(sides)
        if size < MIN_MARKER_SIDE_PX:
            continue
        out[marker_id] = (float(pts[:, 0].mean()), float(pts[:, 1].mean()), size)
    return out


def evaluate(t, detections, references, shape):
    """Return diagnostic row and update anchors only when first confidently observed.

    Anchors are deliberately acquired after initial takeoff, when markers become small.
    This cannot reconstruct drift before anchors are established.
    """
    cy, cx = shape[0] / 2.0, shape[1] / 2.0
    for mid, (x, y, size) in detections.items():
        if mid not in references and t >= REFERENCE_AFTER_SECONDS and size <= MAX_REFERENCE_SIDE_PX:
            references[mid] = (x, y, size)

    residuals = []
    for mid, (x, y, size) in detections.items():
        if mid not in references:
            continue
        rx, ry, rs = references[mid]
        ex, ey = expected_center((rx, ry), rs, size, (cx, cy))
        residuals.append((mid, x - ex, y - ey))

    result = {'markers': ','.join(map(str, sorted(detections))),
              'anchors': ','.join(map(str, sorted(references))),
              'dx_px': '', 'dy_px': '', 'image_shift_x_px': 0,
              'image_shift_y_px': 0, 'status': ''}
    if not detections:
        result['status'] = 'NO_MARKERS_NEUTRAL'
    elif not residuals:
        result['status'] = 'WAITING_FOR_REFERENCE'
    elif len(residuals) >= 2 and math.hypot(residuals[0][1] - residuals[1][1],
                                             residuals[0][2] - residuals[1][2]) > MAX_MARKER_DISAGREEMENT_PX:
        result['status'] = 'MARKERS_DISAGREE_NEUTRAL'
    else:
        dx = statistics.median(a[1] for a in residuals)
        dy = statistics.median(a[2] for a in residuals)
        sx, sy = score_error(dx, dy)
        result.update(dx_px=round(dx, 1), dy_px=round(dy, 1),
                      image_shift_x_px=sx, image_shift_y_px=sy,
                      status='TWO_MARKERS' if len(residuals) >= 2 else 'SINGLE_MARKER_LOW_CONFIDENCE')
    return result


def self_test():
    p = expected_center((100, 90), 12, 18, (160, 120))
    assert p == (70.0, 75.0), p
    assert score_error(30, -20) == (-8.0, 8.0)
    assert score_error(3, -1) == (0.0, 0.0)
    refs = {}
    anchor = evaluate(3.0, {23: (100, 90, 12), 25: (180, 200, 12)}, refs, (240, 320, 3))
    assert anchor['status'] == 'TWO_MARKERS' and anchor['dx_px'] == 0, anchor
    translated = evaluate(4.0, {23: (92, 94, 12), 25: (172, 204, 12)}, refs, (240, 320, 3))
    assert translated['dx_px'] == -8 and translated['dy_px'] == 4, translated
    assert translated['image_shift_x_px'] == 8 and translated['image_shift_y_px'] == 0, translated
    scaled = evaluate(5.0, {23: (70, 75, 18), 25: (190, 240, 18)}, refs, (240, 320, 3))
    assert scaled['dx_px'] == 0 and scaled['dy_px'] == 0, scaled
    lost = evaluate(6.0, {}, refs, (240, 320, 3))
    assert lost['status'] == 'NO_MARKERS_NEUTRAL' and lost['image_shift_x_px'] == 0, lost
    conflict = evaluate(7.0, {23: (20, 90, 12), 25: (180, 200, 12)}, refs, (240, 320, 3))
    assert conflict['status'] == 'MARKERS_DISAGREE_NEUTRAL', conflict
    print('PASS: reference acquisition, altitude scale correction, bounded scores, deadband, marker loss, disagreement')
    print('NO DRONE CONNECTION. NO FLIGHT.')


def main(folder):
    folder = Path(folder)
    csv_path = folder / 'frames.csv'
    if not csv_path.is_file():
        raise SystemExit(f'Missing flight CSV: {csv_path}')
    with csv_path.open(newline='', encoding='utf-8-sig') as f:
        rows = list(csv.DictReader(f))
    required = {'filename', 'seconds_from_takeoff_start', 'phase'}
    if not rows or not required.issubset(rows[0]):
        raise SystemExit(f'CSV is empty or missing columns: {required}')
    if not hasattr(cv2, 'aruco'):
        raise SystemExit('cv2.aruco unavailable. Install opencv-contrib-python.')
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
        cv2.aruco.DetectorParameters())
    references, output, errors = {}, [], []
    for row in rows:
        filename = Path(row['filename'])
        if filename.is_absolute() or '..' in filename.parts:
            raise SystemExit('Unsafe frame filename in CSV')
        img = cv2.imread(str(folder / filename))
        if img is None:
            raise SystemExit(f'Missing/unreadable frame: {folder / filename}')
        t = float(row['seconds_from_takeoff_start'])
        result = evaluate(t, image_measurements(img, detector), references, img.shape)
        out = {'time_s': round(t, 3), 'phase': row['phase'], 'filename': row['filename'], **result}
        output.append(out)
        if result['dx_px'] != '':
            errors.append((t, result['dx_px'], result['dy_px']))
    output_path = folder / 'drift_offline_report.csv'
    with output_path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=list(output[0]))
        writer.writeheader()
        writer.writerows(output)
    print(f'Analyzed: {len(output)} images; useful relative measurements: {len(errors)}')
    print(f'Reference markers (acquired after {REFERENCE_AFTER_SECONDS}s): {sorted(references)}')
    if errors:
        print('FIRST relative result (t, dx, dy):', errors[0])
        print('LAST relative result  (t, dx, dy):', errors[-1])
        print('NOTE: Scores are IMAGE SHIFT REQUESTS, NOT motor/drone RC commands.')
    for key in sorted(set(row['status'] for row in output)):
        print(f'  {key}: {sum(row["status"] == key for row in output)} frames')
    print('Saved:', output_path)
    print('NO DRONE CONNECTION. NO FLIGHT.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', nargs='?', help='Captured flight folder with frames.csv')
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test()
    elif args.folder:
        main(args.folder)
    else:
        parser.print_help()
