"""Offline-only black-tape geometry audit for Tello downward-camera frames.

Run from the Drone repository root:
    python offline_path_geometry.py

No djitellopy import, network access, motor commands, or repository file changes.
This is a diagnostic baseline, NOT a flight controller.
"""
from __future__ import annotations

import csv
import math
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

INPUT = Path('captures/final_dataset')
OUTPUT = Path('captures/offline_path_geometry')
EXPECTED_SHAPE = (240, 320)
MAX_CENTRE_DISTANCE = 60.0
LOCAL_RADIUS = 30.0
MIN_LOCAL_PIXELS = 10
MIN_LINEARITY = 3.0


def skeletonize(mask: np.ndarray) -> np.ndarray:
    """OpenCV-only morphological skeleton; mask must be binary uint8."""
    work = mask.copy()
    result = np.zeros_like(mask)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    while cv2.countNonZero(work):
        eroded = cv2.erode(work, element)
        opened = cv2.dilate(eroded, element)
        result = cv2.bitwise_or(result, cv2.subtract(work, opened))
        work = eroded
    return result


def analyze(frame: np.ndarray) -> tuple[dict, np.ndarray]:
    """Return geometry candidates and a diagnostic image; never commands."""
    if frame is None or frame.ndim != 3 or frame.shape[:2] != EXPECTED_SHAPE:
        raise ValueError('Expected readable BGR camera image of shape 240x320')

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    blackhat = cv2.morphologyEx(
        gray, cv2.MORPH_BLACKHAT,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (31, 31)),
    )
    _, mask = cv2.threshold(
        blackhat, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)
    )
    skeleton = skeletonize(mask)

    view = frame.copy()
    view[skeleton > 0] = (0, 255, 255)  # yellow: centreline candidates
    height, width = frame.shape[:2]
    cx, cy = width // 2, height // 2
    cv2.drawMarker(view, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 15, 1)

    data = {
        'status': 'NO_PATH',
        'nearest_distance_px': None,
        'x_error_px': None,
        'y_error_px': None,
        'axis_angle_deg': None,
        'linearity_ratio': None,
        'skeleton_pixels': int(cv2.countNonZero(skeleton)),
    }

    ys, xs = np.where(skeleton > 0)
    if len(xs) == 0:
        return data, view

    points = np.column_stack((xs, ys)).astype(np.float64)
    distances = np.linalg.norm(points - [cx, cy], axis=1)
    nearest_index = int(np.argmin(distances))
    nearest = points[nearest_index]
    dist = float(distances[nearest_index])
    data['nearest_distance_px'] = round(dist, 2)
    if dist > MAX_CENTRE_DISTANCE:
        data['status'] = 'NO_PATH_NEAR_CENTRE'
        return data, view

    x, y = map(int, nearest)
    data['x_error_px'] = x - cx
    data['y_error_px'] = y - cy
    cv2.circle(view, (x, y), 5, (0, 0, 255), 2)

    local = points[np.linalg.norm(points - nearest, axis=1) <= LOCAL_RADIUS]
    if len(local) < MIN_LOCAL_PIXELS:
        data['status'] = 'INSUFFICIENT_LOCAL_PATH'
        return data, view

    covariance = np.cov(local.T)
    values, vectors = np.linalg.eigh(covariance)
    ratio = float(values[-1] / max(values[0], 1e-6))
    data['linearity_ratio'] = round(ratio, 2)
    if ratio < MIN_LINEARITY:
        data['status'] = 'AMBIGUOUS_JUNCTION_OR_CURVE'
        return data, view

    direction = vectors[:, -1]
    # Fix PCA's arbitrary sign only for consistent DISPLAY; this is NOT travel direction.
    if direction[1] > 0 or (abs(direction[1]) < 1e-9 and direction[0] < 0):
        direction = -direction
    angle = math.degrees(math.atan2(float(direction[0]), float(-direction[1])))
    data['status'] = 'AXIS_CANDIDATE'
    data['axis_angle_deg'] = round(angle, 2)
    p1 = nearest - direction * 35
    p2 = nearest + direction * 35
    cv2.line(view, tuple(p1.astype(int)), tuple(p2.astype(int)), (255, 255, 0), 2)
    return data, view


def main() -> None:
    files = sorted(INPUT.glob('*.jpg'))
    if not files:
        raise SystemExit(f'No JPEG images found in {INPUT.resolve()}')
    OUTPUT.mkdir(parents=True, exist_ok=True)
    rows = []
    counts = Counter()
    thumbnails = []
    for path in files:
        frame = cv2.imread(str(path))
        try:
            data, overlay = analyze(frame)
        except ValueError as exc:
            rows.append({'file': path.name, 'status': f'INVALID: {exc}'})
            counts['INVALID'] += 1
            continue
        rows.append({'file': path.name, **data})
        counts[data['status']] += 1
        label = f"{path.stem}  {data['status']}"
        tile = np.full((263, 320, 3), 245, dtype=np.uint8)
        tile[23:, :] = overlay
        cv2.putText(tile, label[:43], (3, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.39, (0, 0, 0), 1)
        thumbnails.append(tile)

    fields = ['file', 'status', 'nearest_distance_px', 'x_error_px',
              'y_error_px', 'axis_angle_deg', 'linearity_ratio', 'skeleton_pixels']
    with (OUTPUT / 'geometry_results.csv').open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    if thumbnails:
        cols = 5
        nrows = (len(thumbnails) + cols - 1) // cols
        sheet = np.full((nrows * 263, cols * 320, 3), 235, dtype=np.uint8)
        for idx, tile in enumerate(thumbnails):
            row, col = divmod(idx, cols)
            sheet[row*263:(row+1)*263, col*320:(col+1)*320] = tile
        cv2.imwrite(str(OUTPUT / 'geometry_review.jpg'), sheet)

    print('Processed:', len(rows))
    for status, count in sorted(counts.items()):
        print(f'{status}: {count}')
    print('CSV:', OUTPUT / 'geometry_results.csv')
    print('Contact sheet:', OUTPUT / 'geometry_review.jpg')
    print('IMPORTANT: AXIS_CANDIDATE is not an identified travel direction or branch.')


if __name__ == '__main__':
    main()
