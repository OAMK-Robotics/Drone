"""Record Tello Marker 23 camera observations during takeoff and a brief hover.

From the project root:
  python -B record_marker23_ascent.py --self-test  # no Tello import, no network
  python -B record_marker23_ascent.py --fly        # ACTUAL TAKEOFF AND LANDING

This tool RECORDS observations only: no horizontal RC corrections or movement commands.
It is NOT a closed-loop flight controller or a substitute for safe flight supervision.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
import threading
import time

import cv2
import numpy as np

IP = '192.168.1.63'  # Verify at each lab session
BATTERY_MIN = 40
FRAME_SHAPE = (720, 320, 3)
CROP_HEIGHT = 240
CAPTURE_INTERVAL_S = 0.15
HOVER_AFTER_TAKEOFF_S = 4.0


def oriented_image(rgb: np.ndarray) -> np.ndarray:
    if rgb is None or getattr(rgb, 'shape', None) != FRAME_SHAPE:
        raise RuntimeError(f'Unexpected camera frame shape: {getattr(rgb, "shape", None)}')
    # The verified camera adapter: valid 240-row region, RGB to BGR, 180-degree rotation.
    return cv2.rotate(cv2.cvtColor(rgb[:CROP_HEIGHT, :320].copy(), cv2.COLOR_RGB2BGR), cv2.ROTATE_180)


def marker_center(image: np.ndarray):
    # Import belongs to the project root; safe for self-test without drone package.
    from hover.junction_marker_detector import detect_junction_marker
    marker = detect_junction_marker(image)
    if marker is None:
        return None
    return tuple(float(v) for v in marker['center'])


def self_test():
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(d, 23, 80)
    oriented = np.full((240, 320, 3), 255, dtype=np.uint8)
    oriented[80:160, 120:200] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    rgb = np.zeros(FRAME_SHAPE, dtype=np.uint8)
    rgb[:240] = cv2.cvtColor(cv2.rotate(oriented, cv2.ROTATE_180), cv2.COLOR_BGR2RGB)
    actual = oriented_image(rgb)
    assert np.array_equal(actual, oriented), 'Camera orientation regression'
    xy = marker_center(actual)
    assert xy is not None and abs(xy[0]-160) < 2 and abs(xy[1]-120) < 2, xy
    try:
        oriented_image(np.zeros((240, 320, 3), dtype=np.uint8))
    except RuntimeError:
        pass
    else:
        raise AssertionError('Unexpected frame shape accepted')
    print('PASS: verified 720x320 RGB crop, 180-degree rotation, Marker 23 and invalid-frame rejection')
    print('SELF-TEST ONLY: NO DJITELLOPY IMPORT, NO NETWORK, NO TAKEOFF, NO MOTOR COMMANDS.')


def fly():
    from djitellopy import Tello

    out = Path('captures') / f'ascent_marker23_{datetime.now():%Y%m%d_%H%M%S}'
    out.mkdir(parents=True, exist_ok=False)
    print('DATASET:', out, flush=True)
    drone = Tello(host=IP)
    connected = streaming = takeoff_attempted = False
    stop = threading.Event()
    records = []
    errors = []
    thread = None
    clock_start = time.monotonic()
    phase = ['GROUND']
    reader = None

    def log_event(name, value=''):
        stamp = round(time.monotonic()-clock_start, 3)
        print(f'EVENT {stamp:.3f}s: {name} {value}', flush=True)
        records.append(dict(time_s=stamp, phase=name, image='', marker_x='', marker_y='', variation='', detail=str(value)))

    def capture_worker():
        n = 0
        while not stop.is_set():
            started = time.monotonic()
            try:
                source = reader.frame
                if source is None:
                    raise RuntimeError('No frame')
                image = oriented_image(source)
                variation = float(image.std())
                xy = marker_center(image)
                name = f'frame_{n:04d}.png'
                if not cv2.imwrite(str(out/name), image):
                    raise RuntimeError(f'Cannot save {name}')
                stamp = round(started-clock_start, 3)
                records.append(dict(time_s=stamp, phase=phase[0], image=name,
                                    marker_x='' if xy is None else round(xy[0],2),
                                    marker_y='' if xy is None else round(xy[1],2),
                                    variation=round(variation,2), detail=''))
                print(f'{stamp:6.2f}s {phase[0]:12s} frame={n:03d} marker={xy}',flush=True)
                n += 1
            except Exception as exc:
                errors.append(f'{round(started-clock_start,3)}s {exc}')
                print('CAPTURE ERROR:', errors[-1], flush=True)
                stop.set()
            stop.wait(max(0.01,CAPTURE_INTERVAL_S-(time.monotonic()-started)))

    try:
        print('REAL FLIGHT SCRIPT: TAKEOFF + SHORT HOVER + LAND, NO HORIZONTAL RC COMMANDS.', flush=True)
        drone.connect()
        connected = True
        battery = drone.get_battery()
        print('BATTERY:',battery,'%', flush=True)
        if battery < BATTERY_MIN:
            raise RuntimeError('Battery below 40%. Cancelled.')
        drone.set_video_direction(Tello.CAMERA_DOWNWARD)
        drone.streamon()
        streaming = True
        reader = drone.get_frame_read()
        preview = None
        for _ in range(30):
            time.sleep(0.25)
            f = reader.frame
            try:
                image = oriented_image(f)
            except RuntimeError:
                continue
            if float(image.std()) >= 10 and float(image.max()) > 15:
                preview = image
                break
        if preview is None:
            raise RuntimeError('Camera invalid or blank; no takeoff.')
        if not cv2.imwrite(str(out/'preflight.png'), preview):
            raise RuntimeError('Could not save preflight image')
        print('PREFLIGHT IMAGE:',out/'preflight.png',flush=True)
        print('PREFLIGHT MARKER:',marker_center(preview), '(None is normal when camera is too close to floor)',flush=True)
        print('Use ONLY a large clear indoor floor area; verify visibly clear of obstacles/people.',flush=True)
        if input('After inspecting camera and area, type FLY-ASCENT for real takeoff: ').strip() != 'FLY-ASCENT':
            print('CANCELLED: no takeoff.',flush=True)
            return
        # Start capture before takeoff; worker continues while SDK takeoff() blocks.
        thread = threading.Thread(target=capture_worker, name='camera-recording', daemon=True)
        thread.start()
        time.sleep(0.5)
        if errors or not any(r['image'] for r in records):
            raise RuntimeError('Camera recorder not producing images: no takeoff.')
        phase[0] = 'TAKEOFF'
        log_event('TAKEOFF_SENT')
        takeoff_attempted = True
        drone.takeoff()
        log_event('TAKEOFF_ACKNOWLEDGED')
        phase[0] = 'HOVER'
        # Height is not a reliable substitute for camera/visual safety.
        print('REPORTED HEIGHT (approximate):',drone.get_height(),'cm',flush=True)
        stop.wait(HOVER_AFTER_TAKEOFF_S)
        if errors:
            print('Camera recording stopped due to error; landing.',flush=True)
        phase[0] = 'LANDING'
    except KeyboardInterrupt:
        print('OPERATOR INTERRUPT: attempting landing if takeoff was initiated.',flush=True)
    except Exception as exc:
        print('STOP:',str(exc),'— attempting landing if takeoff was initiated.',flush=True)
    finally:
        if takeoff_attempted:
            # Attempt land even if the prior SDK command or capture failed.
            try:
                log_event('LANDING_SENT')
                drone.land()
                log_event('LANDING_ACKNOWLEDGED')
                print('LAND command acknowledged; this is not proof of physical touchdown.',flush=True)
            except Exception as exc:
                print('CRITICAL: Landing SDK command failed:',exc,flush=True)
        # Stop recording BEFORE asking the operator to confirm touchdown.
        # Otherwise capture_worker keeps printing while input() is blocked.
        stop.set()
        if thread is not None:
            thread.join(timeout=4)
            if thread.is_alive():
                print('WARNING: capture thread did not stop within 4 seconds.',flush=True)
        if takeoff_attempted:
            # Physical touchdown must still be verified visually; land ACK is not proof.
            try:
                input('After VISUALLY confirming actual touchdown, press Enter: ')
            except (EOFError,KeyboardInterrupt):
                print('WARNING: operator confirmation unavailable; verify physical landing before leaving.',flush=True)
        if streaming:
            try:
                drone.streamoff()
            except Exception as exc:
                print('WARNING: video shutdown:',exc,flush=True)
        if connected:
            try:
                drone.end()
            except Exception as exc:
                print('WARNING: connection cleanup:',exc,flush=True)
        path = out/'observations.csv'
        try:
            with path.open('w',encoding='utf-8',newline='') as fh:
                writer=csv.DictWriter(fh,fieldnames=['time_s','phase','image','marker_x','marker_y','variation','detail'])
                writer.writeheader()
                writer.writerows(records)
            measurements = [r for r in records if r['image']]
            tracked = [r for r in measurements if r['marker_x']!='']
            print(f'SUMMARY: {len(tracked)}/{len(measurements)} frames with Marker 23',flush=True)
            if tracked:
                print('FIRST MARKER:',tracked[0]['time_s'],'s',tracked[0]['marker_x'],tracked[0]['marker_y'],flush=True)
                print('LAST MARKER:',tracked[-1]['time_s'],'s',tracked[-1]['marker_x'],tracked[-1]['marker_y'],flush=True)
            print('CSV:',path,flush=True)
            print('DATASET:',out,flush=True)
        except Exception as exc:
            print('WARNING: Failed to save CSV:',exc,flush=True)
        print('NO HORIZONTAL RC COMMANDS OR AUTOMATIC DRIFT CORRECTION.',flush=True)


if __name__ == '__main__':
    args=argparse.ArgumentParser(description=__doc__)
    args.add_argument('--self-test',action='store_true')
    args.add_argument('--fly',action='store_true')
    a=args.parse_args()
    if a.self_test == a.fly:
        args.error('Choose exactly one: --self-test or --fly')
    if a.self_test:
        self_test()
    else:
        fly()
