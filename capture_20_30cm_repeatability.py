"""Record real Tello takeoff -> one controlled SDK descent toward 30 cm -> short hover -> land.

This is a DATA COLLECTION flight, NOT position stabilization or a guaranteed 30 cm flight.
Ryze Tello auto-takeoff altitude/speed cannot be set with this SDK sequence.
No horizontal RC commands, no path following. Observe aircraft and stop if leaving table.

OFFLINE: python -B capture_30cm_drift.py --self-test
REAL:    python -B capture_30cm_drift.py --fly --accept-takeoff-overshoot
"""
from __future__ import annotations

import argparse
import csv
import statistics
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

HOST = '192.168.1.63'  # Lab network used previously
BATTERY_MIN = 40
TARGET_CM = 25  # SDK-reported target; NOT verified physical clearance
ACCEPT_MIN_CM = 20
ACCEPT_MAX_CM = 30
SETTLE_AFTER_DESCENT_S = 1.6
OBSERVE_TARGET_S = 2.0
FRAME_PERIOD_S = 0.10
MAX_AIRBORNE_S = 23.0


def orient(frame):
    if not isinstance(frame, np.ndarray) or frame.shape != (720, 320, 3):
        raise ValueError(f'Unexpected Tello frame: {getattr(frame, "shape", None)}')
    return cv2.rotate(cv2.cvtColor(frame[:240, :320].copy(), cv2.COLOR_RGB2BGR), cv2.ROTATE_180)


def markers(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    found = {}
    if ids is None:
        return found
    for pts, id_arr in zip(corners, ids.flatten()):
        ident = int(id_arr)
        if ident not in (23, 24, 25):
            continue
        p = pts.reshape((4, 2))
        centre = p.mean(axis=0)
        side = float(np.mean([np.linalg.norm(p[(i + 1) % 4] - p[i]) for i in range(4)]))
        angle_deg = float(np.degrees(np.arctan2(p[1, 1] - p[0, 1], p[1, 0] - p[0, 0])))
        found[ident] = (float(centre[0]), float(centre[1]), side, angle_deg)
    return found


def height_window(drone, wait=time.sleep):
    values = []
    for _ in range(5):
        h = drone.get_height()
        if isinstance(h, bool) or not isinstance(h, (int, float)) or not 0 < h < 300:
            raise RuntimeError(f'Invalid SDK height reading {h!r}')
        values.append(float(h))
        wait(0.16)
    if max(values) - min(values) > 10:
        raise RuntimeError(f'Unstable height readings: {values}')
    return values


def descent_step(readings):
    """Return one legal SDK down distance, or None when already close to target.

    No repeated blind descent or non-SDK-sized movement is allowed.
    """
    h = statistics.median(readings)
    if ACCEPT_MIN_CM <= h <= ACCEPT_MAX_CM:
        return None
    if not 50 <= h <= 70:
        raise RuntimeError(f'No bounded descent plan from SDK {h:.0f} cm')
    step = int(round(h - TARGET_CM))
    if not 20 <= step <= 40:
        raise RuntimeError(f'Descent distance {step} cm outside SDK 20..40 guard')
    return step


def self_test():
    x = np.zeros((720, 320, 3), dtype=np.uint8)
    x[0, 0] = (10, 20, 30)
    oriented = orient(x)
    assert oriented.shape == (240, 320, 3)
    assert tuple(oriented[239, 319]) == (30, 20, 10)
    assert descent_step([60, 60, 60, 60, 60]) == 35
    assert descent_step([65] * 5) == 40
    assert descent_step([30] * 5) is None
    for v in ([40] * 5, [80] * 5, [25] * 5):
        try:
            if v[0] == 25:
                assert descent_step(v) is None
            else:
                descent_step(v)
                raise AssertionError(f'Expected rejection {v}')
        except RuntimeError:
            pass
    class Fake:
        def __init__(self):
            self.n = 0
        def get_height(self):
            self.n += 1
            return 30
    assert height_window(Fake(), wait=lambda _: None) == [30.0] * 5
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    canvas = np.full((240, 320, 3), 255, dtype=np.uint8)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        symbol = cv2.aruco.generateImageMarker(dictionary, 23, 56)
    else:
        symbol = cv2.aruco.drawMarker(dictionary, 23, 56)
    canvas[82:138, 132:188] = cv2.cvtColor(symbol, cv2.COLOR_GRAY2BGR)
    assert 23 in markers(canvas), 'Marker detection self-test failed'
    print('PASS: 180-degree camera rotation; real ArUco 23 detector; telemetry validation; single 25 cm SDK descent target.')
    print('SELF-TEST ONLY. NO DJITELLOPY IMPORT, WIFI, FLIGHT OR MOTOR COMMANDS.')


def fly():
    from djitellopy import Tello

    out = Path('captures') / f'drift_20_30cm_{datetime.now():%Y%m%d_%H%M%S}'
    out.mkdir(parents=True, exist_ok=False)
    csvfile = out / 'observations.csv'
    drone = Tello(host=HOST)
    started = time.monotonic()
    takeoff_at = [None]
    stop = threading.Event()
    phase = ['PREFLIGHT']
    records = []
    errors = []
    lock = threading.Lock()
    reader = None
    connected = streaming = attempted_takeoff = False
    worker = None

    def log(event, detail=''):
        with lock:
            records.append({'t_s': round(time.monotonic() - started, 3),
                            'takeoff_t_s': '' if takeoff_at[0] is None else round(time.monotonic() - takeoff_at[0], 3),
                            'phase': phase[0], 'event': event, 'sdk_height_cm': '',
                            'marker23_x': '', 'marker23_y': '', 'marker23_side_px': '', 'marker23_angle_deg': '',
                            'marker25_x': '', 'marker25_y': '', 'marker25_side_px': '', 'marker25_angle_deg': '',
                            'image': '', 'detail': detail})
        print(f'EVENT {event}: {detail}', flush=True)

    def capture():
        idx = 0
        while not stop.is_set():
            beginning = time.monotonic()
            try:
                frame = orient(reader.frame)
                found = markers(frame)
                fname = f'{idx:04d}.png'
                if not cv2.imwrite(str(out / fname), frame):
                    raise RuntimeError(f'Could not save {fname}')
                try:
                    height = drone.get_height()  # cached SDK value, not physical height
                except Exception:
                    height = ''
                row = {'t_s': round(beginning - started, 3),
                       'takeoff_t_s': '' if takeoff_at[0] is None else round(beginning - takeoff_at[0], 3),
                       'phase': phase[0], 'event': 'FRAME', 'sdk_height_cm': height,
                       'marker23_x': '', 'marker23_y': '', 'marker23_side_px': '', 'marker23_angle_deg': '',
                       'marker25_x': '', 'marker25_y': '', 'marker25_side_px': '', 'marker25_angle_deg': '',
                       'image': fname, 'detail': 'rotated_180'}
                for ident in (23, 25):
                    if ident in found:
                        x, y, side, angle = found[ident]
                        row.update({f'marker{ident}_x': round(x, 2), f'marker{ident}_y': round(y, 2),
                                    f'marker{ident}_side_px': round(side, 2),
                                    f'marker{ident}_angle_deg': round(angle, 2)})
                with lock:
                    records.append(row)
                if idx % 10 == 0:
                    print(f'FRAME {idx} {phase[0]} SDK_H={height} M23={found.get(23, None)} M25={found.get(25, None)}', flush=True)
                idx += 1
            except Exception as exc:
                errors.append(repr(exc))
                stop.set()
                print('CAMERA ERROR:', repr(exc), flush=True)
                return
            stop.wait(max(0.0, FRAME_PERIOD_S - (time.monotonic() - beginning)))

    def ensure_capture_and_time():
        if errors or stop.is_set() or worker is None or not worker.is_alive():
            raise RuntimeError('Camera capture worker failed')
        if takeoff_at[0] is not None and time.monotonic() - takeoff_at[0] > MAX_AIRBORNE_S:
            raise RuntimeError('Airborne time budget reached')

    def observe(seconds, label):
        phase[0] = label
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            ensure_capture_and_time()
            time.sleep(0.05)

    try:
        drone.connect()
        connected = True
        battery = drone.get_battery()
        log('BATTERY', str(battery))
        if battery < BATTERY_MIN:
            raise RuntimeError('Battery <40%: no takeoff')
        drone.set_video_direction(Tello.CAMERA_DOWNWARD)
        drone.streamon()
        streaming = True
        reader = drone.get_frame_read()
        image = None
        for _ in range(30):
            time.sleep(0.2)
            try:
                candidate = orient(reader.frame)
            except (ValueError, TypeError):
                continue
            if candidate.std() >= 10 and candidate.max() >= 15:
                image = candidate
                break
        if image is None:
            raise RuntimeError('No valid downward video: no takeoff')
        cv2.imwrite(str(out / 'preflight.png'), image)
        print('DATASET:', out, flush=True)
        print('ONE FLIGHT: record takeoff, ONE descent toward SDK 25cm, ~2s neutral hover, land.', flush=True)
        print('No left/right/forward/back motor commands. Position is NOT stabilized.', flush=True)
        print('Takeoff can rise above 30-40cm; no absolute altitude guarantee.', flush=True)
        answer = input('On the dedicated table, after checking edge clearance, type FLY-20-30-DATA: ').strip()
        if answer != 'FLY-20-30-DATA':
            log('CANCELLED', 'no takeoff')
            return
        phase[0] = 'GROUND'
        worker = threading.Thread(target=capture, name='drift-recorder', daemon=True)
        worker.start()
        time.sleep(0.5)
        with lock:
            n0 = len([r for r in records if r['event'] == 'FRAME'])
        if errors or n0 < 3:
            raise RuntimeError('Camera recording did not start: no takeoff')
        phase[0] = 'TAKEOFF'
        takeoff_at[0] = time.monotonic()
        attempted_takeoff = True
        log('TAKEOFF_SENT')
        drone.takeoff()
        log('TAKEOFF_ACK')
        ensure_capture_and_time()
        phase[0] = 'HEIGHT_ASSESSMENT'
        h_before = height_window(drone)
        log('HEIGHT_BEFORE', str(h_before))
        step = descent_step(h_before)
        if step is not None:
            phase[0] = 'DESCENT'
            log('DESCENT_SENT', f'down {step} (SDK estimated height {statistics.median(h_before):.0f})')
            drone.move_down(step)
            log('DESCENT_ACK')
        observe(SETTLE_AFTER_DESCENT_S, 'HEIGHT_SETTLE')
        phase[0] = 'HEIGHT_VERIFY'
        h_after = height_window(drone)
        log('HEIGHT_AFTER', str(h_after))
        if not all(ACCEPT_MIN_CM <= h <= ACCEPT_MAX_CM for h in h_after):
            raise RuntimeError(f'20-30cm SDK target not verified: {h_after}; recording kept; land')
        print('SDK near-30cm verified (NOT physical height):', h_after, flush=True)
        observe(OBSERVE_TARGET_S, '20_30CM_NEUTRAL_HOVER')
        log('CAPTURE_COMPLETE', '2 seconds at SDK 20-30cm; no horizontal commands')
    except KeyboardInterrupt:
        log('OPERATOR_INTERRUPT', 'attempt landing')
    except Exception as exc:
        log('STOP', str(exc))
    finally:
        if attempted_takeoff:
            phase[0] = 'LANDING'
            try:
                # Neutral is the only RC command used; no horizontal pulses.
                drone.send_rc_control(0, 0, 0, 0)
                log('NEUTRAL_SENT')
            except Exception as exc:
                log('NEUTRAL_ERROR', repr(exc))
            try:
                log('LANDING_SENT')
                drone.land()
                log('LANDING_ACK', 'visual confirmation required')
            except Exception as exc:
                log('LANDING_ERROR', repr(exc))
        stop.set()
        if worker is not None:
            worker.join(timeout=4)
            if worker.is_alive():
                print('WARNING: recording thread still alive', flush=True)
        if streaming:
            try:
                drone.streamoff()
            except Exception as exc:
                print('streamoff error:', repr(exc), flush=True)
        if connected:
            try:
                drone.end()
            except Exception:
                pass
        with lock:
            saved = list(records)
        fields = ['t_s','takeoff_t_s','phase','event','sdk_height_cm',
                  'marker23_x','marker23_y','marker23_side_px','marker23_angle_deg',
                  'marker25_x','marker25_y','marker25_side_px','marker25_angle_deg',
                  'image','detail']
        with csvfile.open('w', encoding='utf-8', newline='') as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            w.writerows(saved)
        print('FRAME COUNT:', sum(r['event']=='FRAME' for r in saved), flush=True)
        print('MARKER23:', sum(r['event']=='FRAME' and r['marker23_x']!='' for r in saved), flush=True)
        print('MARKER25:', sum(r['event']=='FRAME' and r['marker25_x']!='' for r in saved), flush=True)
        print('CSV:', csvfile, flush=True)
        print('DATASET:', out, flush=True)
        print('REAL DRIFT CAPTURE ONLY. NO AUTONOMOUS CORRECTION.', flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument('--self-test', action='store_true')
    mode.add_argument('--fly', action='store_true')
    ap.add_argument('--accept-takeoff-overshoot', action='store_true')
    args = ap.parse_args()
    if args.self_test:
        self_test()
    elif not args.accept_takeoff_overshoot:
        print('REFUSED: Auto-takeoff height/speed cannot be set. Use --accept-takeoff-overshoot only if that is acceptable.')
    else:
        fly()


if __name__ == '__main__':
    main()
