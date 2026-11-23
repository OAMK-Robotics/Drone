"""Determine Tello downward-camera axis signs by moving the DISARMED drone by hand.

NO takeoff, landing, move, send_rc_control, or motor commands are present.
Usage: python calibrate_camera_axes_no_flight.py
       python calibrate_camera_axes_no_flight.py --self-test

Two ArUco markers (23 and 25) must be visible at all four poses. Keep
altitude and yaw as constant as possible during manual translations.
"""
from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import time

import cv2
import numpy as np

HOST = '192.168.1.63'
MARKERS = (23, 25)
SAMPLE_COUNT = 5
SHIFT_CM = 5.0  # approximate user-applied manual displacement; not motor calibration
CROP_H, CROP_W = 240, 320


def detector_new():
    if not hasattr(cv2, 'aruco'):
        raise RuntimeError('cv2.aruco missing; install opencv-contrib-python')
    return cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
        cv2.aruco.DetectorParameters())


def image_from_reader(reader):
    source = reader.frame
    if source is None or source.ndim != 3 or source.shape[0] < CROP_H or source.shape[1] < CROP_W:
        raise RuntimeError('Invalid camera frame shape')
    # Same camera crop and 180-degree rotation as the successful flight dataset.
    return cv2.rotate(cv2.cvtColor(source[:CROP_H, :CROP_W].copy(), cv2.COLOR_RGB2BGR), cv2.ROTATE_180)


def markers_in_image(frame, detector):
    corners, ids, _ = detector.detectMarkers(frame)
    if ids is None:
        return {}
    result = {}
    for i, marker_id in enumerate(ids.flatten()):
        if int(marker_id) in MARKERS:
            pts = corners[i][0]
            result[int(marker_id)] = {
                'xy': np.mean(pts, axis=0).astype(float),
                'size': float(np.mean(np.linalg.norm(pts - np.roll(pts, -1, axis=0), axis=1))),
            }
    return result


def capture(label, reader, detector, out):
    samples = []
    last_img = None
    for _ in range(SAMPLE_COUNT):
        img = image_from_reader(reader)
        if img.std() < 8 or img.mean() < 10:
            raise RuntimeError('Camera image appears corrupted or blank')
        det = markers_in_image(img, detector)
        if set(det) != set(MARKERS):
            raise RuntimeError(f'{label}: need markers 23 AND 25; saw {sorted(det)}. Change view and rerun.')
        samples.append(det)
        last_img = img
        time.sleep(0.125)
    measurement = {}
    for mid in MARKERS:
        positions = np.array([s[mid]['xy'] for s in samples])
        spread = float(np.max(np.linalg.norm(positions - np.median(positions, axis=0), axis=1)))
        if spread > 7:
            raise RuntimeError(f'{label}: marker {mid} moving during sampling ({spread:.1f}px). Hold drone steady and rerun.')
        measurement[mid] = {
            'xy': np.median(positions, axis=0),
            'size': float(np.median([s[mid]['size'] for s in samples])),
        }
    path = out / f'{label}.png'
    if not cv2.imwrite(str(path), last_img):
        raise RuntimeError(f'Cannot save {path}')
    print(f'{label}: ' + ' '.join(f'{m}=({measurement[m]["xy"][0]:.1f},{measurement[m]["xy"][1]:.1f})' for m in MARKERS))
    return measurement


def measured_shift(before, after, label):
    shifts = []
    for mid in MARKERS:
        ratio = after[mid]['size'] / max(before[mid]['size'], 1e-6)
        if not 0.8 <= ratio <= 1.2:
            raise RuntimeError(f'{label}: marker {mid} changed size {ratio:.2f}x (height/perspective changed). Calibration rejected.')
        shifts.append(after[mid]['xy'] - before[mid]['xy'])
    disagreement = float(np.linalg.norm(shifts[0] - shifts[1]))
    if disagreement > 12:
        raise RuntimeError(f'{label}: markers moved differently by {disagreement:.1f}px (yaw or perspective). Calibration rejected.')
    avg = np.mean(shifts, axis=0)
    if np.linalg.norm(avg) < 5:
        raise RuntimeError(f'{label}: movement too small in image ({np.linalg.norm(avg):.1f}px). Calibration rejected.')
    return avg


def analyze(baseline, right, back, forward):
    reset_delta = np.mean([back[m]['xy'] - baseline[m]['xy'] for m in MARKERS], axis=0)
    if np.linalg.norm(reset_delta) > 10:
        raise RuntimeError(f'Return to baseline failed ({np.linalg.norm(reset_delta):.1f}px). Calibration rejected.')
    right_shift = measured_shift(baseline, right, 'RIGHT')
    forward_shift = measured_shift(back, forward, 'FORWARD')
    jacobian = np.array([[right_shift[0], forward_shift[0]], [right_shift[1], forward_shift[1]]])
    cond = float(np.linalg.cond(jacobian))
    if not np.isfinite(cond) or cond > 6:
        raise RuntimeError(f'Camera axes insufficiently distinct (condition {cond:.1f}). Calibration rejected.')
    return right_shift, forward_shift, cond


def self_test():
    def pose(points, shift=(0, 0), scale=1.0):
        return {m: {'xy': np.array(x, dtype=float) + shift, 'size': 17.0 * scale} for m, x in points.items()}
    points = {23: (80, 80), 25: (180, 170)}
    b = pose(points)
    r = pose(points, (-15, 0))
    c = pose(points, (1, -1))
    f = pose(points, (0, 16))
    dx_r, dx_f, cond = analyze(b, r, c, f)
    assert np.allclose(dx_r, [-15, 0]) and np.allclose(dx_f, [-1, 17])
    assert cond < 6
    try:
        analyze(b, r, pose(points, (30, 0)), f)
        raise AssertionError('Reset check not enforced')
    except RuntimeError:
        pass
    try:
        analyze(b, pose(points, (-15, 0), 1.5), c, f)
        raise AssertionError('Size change check not enforced')
    except RuntimeError:
        pass
    print('PASS: manual-motion axis mapping, reset consistency, scale rejection, no flight commands')


def main():
    # Import isolated to make --self-test run without a drone package or network.
    from djitellopy import Tello
    folder = Path('captures') / ('camera_axis_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    folder.mkdir(parents=True, exist_ok=False)
    drone = Tello(host=HOST)
    connected, streaming = False, False
    try:
        print('NO FLIGHT. No motor commands. Keep propellers STOPPED throughout.')
        print('Manually hold/support drone at about 40 cm above markers 23 and 25.')
        print('Maintain same height and heading; move the whole drone, not the markers.')
        drone.connect()
        connected = True
        print('Connected. Battery:', drone.get_battery(), '%')
        drone.set_video_direction(Tello.CAMERA_DOWNWARD)
        drone.streamon()
        streaming = True
        reader = drone.get_frame_read()
        time.sleep(1.0)
        detector = detector_new()
        input('Hold at BASELINE with BOTH markers visible. Press ENTER to capture (or Ctrl+C to stop): ')
        baseline = capture('baseline', reader, detector, folder)
        input(f'Move drone approximately {SHIFT_CM:g} cm RIGHT (its own right), no turning. Press ENTER: ')
        right = capture('right', reader, detector, folder)
        input('Return drone to BASELINE. Press ENTER: ')
        back = capture('baseline_return', reader, detector, folder)
        input(f'Move drone approximately {SHIFT_CM:g} cm FORWARD (its own nose direction), no turning. Press ENTER: ')
        forward = capture('forward', reader, detector, folder)
        dr, df, condition = analyze(baseline, right, back, forward)
        print(f'RIGHT manual movement -> marker shift dx={dr[0]:+.1f}px, dy={dr[1]:+.1f}px')
        print(f'FORWARD manual movement -> marker shift dx={df[0]:+.1f}px, dy={df[1]:+.1f}px')
        print(f'Axis condition number: {condition:.2f} (<= 6 accepted)')
        print('CAMERA AXES CALIBRATED IN IMAGE SPACE. Live RC magnitudes NOT calibrated.')
        with (folder / 'calibration.txt').open('w', encoding='utf-8') as f:
            f.write(f'RIGHT manual shift (px) = {dr.tolist()}\n')
            f.write(f'FORWARD manual shift (px) = {df.tolist()}\n')
            f.write(f'Axis condition = {condition:.3f}\n')
        print('Saved:', folder)
    finally:
        if streaming:
            try:
                drone.streamoff()
            except Exception as exc:
                print('Stream cleanup warning:', exc)
        if connected:
            try:
                drone.end()
            except Exception:
                pass
        print('NO FLIGHT occurred.')


if __name__ == '__main__':
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument('--self-test', action='store_true')
    opts = args.parse_args()
    if opts.self_test:
        self_test()
    else:
        main()
