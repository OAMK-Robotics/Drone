"""Guarded Tello motor-axis evidence capture, with image samples DURING RC pulses.

OFFLINE: python -B collect_motor_axis_streaming.py --self-test
REAL FLIGHT (ONLY AFTER OFFLINE PASS AND CLEAR LARGE INDOOR AREA):
    python -B collect_motor_axis_streaming.py --fly --axis right
    python -B collect_motor_axis_streaming.py --fly --axis forward

Version 2: always neutral after final pulse, then observe a full post-pulse
window; require six consecutive valid frames before reverse command.
Record measurements BEFORE slower disk writes to reduce stale-frame race.
Not autonomous stabilization and not collision avoidance. Operator visually supervises.
"""
from __future__ import annotations

import argparse
from collections import deque
import csv
from datetime import datetime
from pathlib import Path
import statistics
import threading
import time

import cv2
import numpy as np

HOST = '192.168.1.63'    # Verified on university lab network; verify in a new network.
BATTERY_MIN = 40
CROP_W, CROP_H = 320, 240
FRAME_INTERVAL_S = 0.10
PULSE_SEND_COUNT = 3
PULSE_INTERVAL_S = 0.15
RC_SPEED = 10
MAX_AIRBORNE_S = 25.0
MAX_FRAME_AGE_S = 0.40
WINDOW_S = 0.90
MIN_VALID = 4
MAX_MISSES_IN_WINDOW = 2
MAX_ERROR_X = 45
MAX_ERROR_Y = 42
MAX_POINT_SPREAD_PX = 15


def orient(rgb):
    if not isinstance(rgb, np.ndarray) or rgb.shape != (720, 320, 3):
        raise RuntimeError(f'Bad camera frame shape: {getattr(rgb, "shape", None)}')
    return cv2.rotate(cv2.cvtColor(rgb[:CROP_H, :CROP_W].copy(), cv2.COLOR_RGB2BGR), cv2.ROTATE_180)


def rc_vector(axis, sign):
    if axis == 'right':
        return (RC_SPEED * sign, 0, 0, 0)
    if axis == 'forward':
        return (0, RC_SPEED * sign, 0, 0)
    raise ValueError('Invalid axis')


def marker_quality(samples, now):
    """Takeoff/hover gate. Samples: (timestamp_monotonic, center_or_None)."""
    recent = [(t, xy) for t, xy in samples if now - WINDOW_S <= t <= now + 0.001]
    if not recent or now - recent[-1][0] > MAX_FRAME_AGE_S:
        return False, 'Camera observations missing or stale'
    # Require latest actual observation to contain the marker; no blind pulse.
    if recent[-1][1] is None:
        return False, 'Marker 23 missing in latest frame'
    valid = [xy for _, xy in recent if xy is not None]
    if len(valid) < MIN_VALID or len(recent) - len(valid) > MAX_MISSES_IN_WINDOW:
        return False, f'Insufficient Marker 23 tracking: {len(valid)}/{len(recent)}'
    x = statistics.median(p[0] for p in valid)
    y = statistics.median(p[1] for p in valid)
    if abs(x - 160) > MAX_ERROR_X or abs(y - 120) > MAX_ERROR_Y:
        return False, f'Unsafe marker centrality: ({x:.1f},{y:.1f})'
    spread = max(((px - x)**2 + (py - y)**2)**0.5 for px, py in valid)
    if spread > MAX_POINT_SPREAD_PX:
        return False, f'Marker motion/spread too high: {spread:.1f}px'
    # Also check latest point, not only median: prevents late boundary excursions.
    lx, ly = valid[-1]
    if abs(lx - 160) > MAX_ERROR_X or abs(ly - 120) > MAX_ERROR_Y:
        return False, f'Latest marker outside permitted centre box: ({lx:.1f},{ly:.1f})'
    return True, f'{len(valid)}/{len(recent)} valid; median=({x:.1f},{y:.1f}); spread={spread:.1f}px'


def reverse_stability(samples, now):
    """Fail closed unless last six samples are fresh, detected, and central."""
    tail = list(samples)[-6:]
    if len(tail) != 6:
        return False, 'fewer than six recent samples'
    if any(point is None or t > now + 0.001 or now - t > MAX_FRAME_AGE_S + 0.3
           for t, point in tail):
        return False, 'missing or stale marker within six latest samples'
    if any(abs(point[0] - 160) > 35 or abs(point[1] - 120) > 32
           for _, point in tail):
        return False, 'one of last six marker positions close to centrality boundary'
    return True, 'six consecutive fresh and central marker detections'


def short_pulse(drone, vector, check, event, sleep=time.sleep):
    """Bounded pulse. Before EACH positive RC send, check fresh vision. Always neutral."""
    event('PULSE_START', str(vector))
    try:
        for i in range(PULSE_SEND_COUNT):
            check()  # raises on stale/missing/out-of-bounds tracking
            drone.send_rc_control(*vector)
            event('PULSE_COMMAND', f'{i + 1}/{PULSE_SEND_COUNT} {vector}')
            sleep(PULSE_INTERVAL_S)
        # The last RC interval is complete; immediately neutralize before
        # post-pulse observation. A single failed detection in this final
        # interval cannot authorize more movement: the next command (reverse)
        # requires a separate, stricter stable-tracking gate.
    finally:
        try:
            drone.send_rc_control(0, 0, 0, 0)
        except Exception as exc:
            event('CRITICAL_NEUTRAL_FAILURE', repr(exc))
            print('CRITICAL: Neutral RC failed:', exc, flush=True)
        event('PULSE_END', 'neutral command attempted')


def run_live(axis):
    # Network-enabled imports ONLY inside this function, not the self-test.
    from djitellopy import Tello
    from hover.junction_marker_detector import detect_junction_marker

    out = Path('captures') / f'streaming_v2_axis_{axis}_{datetime.now():%Y%m%d_%H%M%S}'
    out.mkdir(parents=True, exist_ok=False)
    outfile = out / 'observations.csv'
    fh = outfile.open('w', encoding='utf-8', newline='')
    writer = csv.writer(fh)
    writer.writerow(['elapsed_s','phase','event','marker23_x','marker23_y','image','details'])
    started = time.monotonic()
    lock = threading.Lock()
    recent = deque(maxlen=30)
    stop = threading.Event()
    errors = []
    state = {'phase': 'GROUND', 'count': 0}
    drone = Tello(host=HOST)
    connected = streaming = takeoff_attempted = False
    thread = None
    airborne_at = None

    def event(name, details='', point=None, image=''):
        with lock:
            pt = point if point is not None else ('', '')
            writer.writerow([f'{time.monotonic()-started:.3f}', state['phase'], name,
                             pt[0], pt[1], image, details])
            fh.flush()

    def set_phase(value):
        with lock:
            state['phase'] = value

    def capture_worker(reader):
        while not stop.is_set():
            loop_start = time.monotonic()
            try:
                image = orient(reader.frame)
                marker = detect_junction_marker(image)
                point = tuple(float(x) for x in marker['center']) if marker else None
                with lock:
                    i = state['count']
                    state['count'] += 1
                    phase = state['phase']
                filename = f'{i:04d}_{phase.lower()}.png'
                # Update control evidence as soon as detection completes.
                # Disk I/O must not make fresh camera data appear stale.
                measured_at = time.monotonic()
                with lock:
                    recent.append((measured_at, point))
                    writer.writerow([f'{measured_at-started:.3f}', phase, 'FRAME',
                                     '' if point is None else point[0],
                                     '' if point is None else point[1], filename, 'rotated_180'])
                    fh.flush()
                if not cv2.imwrite(str(out / filename), image):
                    raise RuntimeError('Could not save video sample')
                # Summarize once per ~1 s; no infinite terminal spam.
                if i % 10 == 0:
                    print(f'FRAME {i} phase={phase} marker23={point}',flush=True)
            except Exception as exc:
                with lock:
                    errors.append(repr(exc))
                stop.set()
                print('CRITICAL CAMERA ERROR:', exc, flush=True)
                return
            stop.wait(max(0, FRAME_INTERVAL_S - (time.monotonic() - loop_start)))

    def check():
        if stop.is_set() or errors or thread is None or not thread.is_alive():
            raise RuntimeError('Camera recorder failed or stopped')
        if airborne_at is not None and time.monotonic()-airborne_at > MAX_AIRBORNE_S:
            raise RuntimeError('Airborne budget exceeded')
        with lock:
            snapshot = list(recent)
        good, reason = marker_quality(snapshot, time.monotonic())
        if not good:
            raise RuntimeError('PULSE CANCELLED: '+reason)
        return reason

    def observe(seconds, label):
        set_phase(label)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if stop.is_set() or errors:
                raise RuntimeError('Video capture stopped')
            if airborne_at is not None and time.monotonic()-airborne_at > MAX_AIRBORNE_S:
                raise RuntimeError('Airborne budget exceeded')
            time.sleep(0.05)
        good = check()
        # Before allowing a NEW direction after a pulse, require at least
        # six consecutive good, fresh frames (no missing Marker 23). The
        # condition is deliberately stricter than the standard quality gate.
        if label == 'AFTER_POSITIVE':
            with lock:
                snapshot = list(recent)
            stable, reason = reverse_stability(snapshot, time.monotonic())
            if not stable:
                raise RuntimeError('No reverse pulse: '+reason)
            event('REVERSE_STABILITY_PASS',reason)
        event('GATE_PASS',good)
        print('GATE',label,'PASS',good,flush=True)

    try:
        print('EXPERIMENT V2: continuous camera with independent neutral/post-pulse gates. NOT stabilization.',flush=True)
        drone.connect()
        connected = True
        battery = drone.get_battery()
        print('BATTERY:',battery,'%',flush=True)
        if battery < BATTERY_MIN:
            raise RuntimeError('Low battery: no takeoff')
        drone.set_video_direction(Tello.CAMERA_DOWNWARD)
        drone.streamon()
        streaming = True
        reader = drone.get_frame_read()
        preview = None
        for _ in range(24):
            time.sleep(0.25)
            try:
                candidate = orient(reader.frame)
            except RuntimeError:
                continue
            if float(candidate.std()) >= 10 and float(candidate.max()) >= 15:
                preview = candidate
                break
        if preview is None:
            raise RuntimeError('Camera blank/invalid: no takeoff')
        if not cv2.imwrite(str(out / 'preflight.png'), preview):
            raise RuntimeError('Cannot save preflight image')
        print('PREFLIGHT:',out/'preflight.png', '(marker may be invisible on ground)',flush=True)
        print('Clear LARGE indoor floor area, unobstructed in BOTH axis directions; keep people away.',flush=True)
        print('Operator must watch aircraft and be ready for emergency intervention.',flush=True)
        approval = input(f'Type STREAM-{axis.upper()} to take off and send up to two short {axis} pulses: ').strip()
        if approval != f'STREAM-{axis.upper()}':
            print('CANCELLED: no flight',flush=True)
            return
        # Capture begins BEFORE takeoff() blocks; continue across all phases.
        thread = threading.Thread(target=capture_worker, args=(reader,), name='axis-camera', daemon=True)
        thread.start()
        time.sleep(0.35)
        if stop.is_set() or state['count'] < 2:
            raise RuntimeError('Recorder did not start: no takeoff')
        set_phase('TAKEOFF')
        takeoff_attempted = True
        airborne_at = time.monotonic()
        event('TAKEOFF_SENT')
        drone.takeoff()
        event('TAKEOFF_ACK')
        print('Takeoff command acknowledged; observing marker (not height telemetry).',flush=True)
        # Two matched neutral windows before acting; initial acceptance requires centrality.
        observe(1.0, 'NEUTRAL_1')
        observe(1.0, 'NEUTRAL_2')
        event('PRE_PULSE_GATE',check())
        print('Starting small positive pulse; monitoring marker DURING each RC interval.',flush=True)
        set_phase('POSITIVE_PULSE')
        short_pulse(drone, rc_vector(axis, +1), check, event)
        observe(1.0, 'AFTER_POSITIVE')
        # Reverse command is not automatic if marker is near any safety boundary.
        print('Positive-pulse observation passed. Checking reverse-pulse safety.',flush=True)
        event('PRE_REVERSE_GATE',check())
        set_phase('NEGATIVE_PULSE')
        short_pulse(drone, rc_vector(axis, -1), check, event)
        observe(1.0, 'AFTER_NEGATIVE')
        print('COMPLETE: all vision safety gates passed; inspect CSV and frames.',flush=True)
        event('COMPLETE')
    except KeyboardInterrupt:
        print('INTERRUPTED: attempting neutral and landing',flush=True)
        event('INTERRUPTED')
    except Exception as exc:
        print('STOP:',exc,flush=True)
        event('STOP',str(exc))
    finally:
        if takeoff_attempted:
            set_phase('LANDING')
            try:
                drone.send_rc_control(0,0,0,0)
                event('FINAL_NEUTRAL_SENT')
            except Exception as exc:
                print('CRITICAL: could not send final neutral:',exc,flush=True)
                event('FINAL_NEUTRAL_FAILURE',repr(exc))
            try:
                event('LANDING_SENT')
                drone.land()
                event('LANDING_ACK')
                print('LAND ACKNOWLEDGED. VISUALLY confirm actual touchdown.',flush=True)
            except Exception as exc:
                event('LANDING_ERROR',repr(exc))
                print('CRITICAL: landing failed:',exc,flush=True)
        # Terminate recording BEFORE waiting for operator input; avoids old infinite-print bug.
        stop.set()
        if thread is not None:
            thread.join(timeout=4)
            if thread.is_alive():
                print('WARNING: camera worker has not stopped.',flush=True)
        if streaming:
            try:
                drone.streamoff()
            except Exception as exc:
                print('WARNING: stream shutdown error:',exc,flush=True)
        if connected:
            try:
                drone.end()
            except Exception as exc:
                print('WARNING: end() failed:',exc,flush=True)
        fh.close()
        print('CSV:',outfile,flush=True)
        print('DATASET:',out,flush=True)
        print('NO AUTONOMOUS DRIFT CORRECTION. Motor directions still need independent validation.',flush=True)
        if takeoff_attempted:
            print('OPERATOR: check propellers stopped and drone actually landed.',flush=True)


def self_test():
    # Reject wrong camera shape, and verify actual 180-degree pixel coordinate mapping.
    img = np.zeros((720,320,3),dtype=np.uint8)
    img[0,0] = [1,2,3] # RGB at top-left -> BGR at bottom-right of 240x320 crop
    out = orient(img)
    assert out.shape == (240,320,3)
    assert out[239,319].tolist() == [3,2,1]
    for axis in ('right','forward'):
        assert rc_vector(axis,1) != rc_vector(axis,-1)
    now = 10.0
    good = [(9.2+i*0.1,(159.0,120.0)) for i in range(9)]
    assert marker_quality(good,now)[0], marker_quality(good,now)
    assert not marker_quality([(9.95,(159.0,51.1))]*6,now)[0]  # old unsafe trial
    assert not marker_quality([(9.95,None)]*6,now)[0]
    assert not marker_quality([(7,(160,120))]*6,now)[0]  # stale
    assert not marker_quality([(9.70,(160,120))]*5 + [(9.95,None)],now)[0]
    class FakeDrone:
        def __init__(self):
            self.calls=[]
        def send_rc_control(self,*v):
            self.calls.append(tuple(v))
    for axis in ('right','forward'):
        for sign in (-1,1):
            fake=FakeDrone()
            short_pulse(fake,rc_vector(axis,sign),lambda:'safe',lambda *a:None,sleep=lambda x:None)
            assert fake.calls == [rc_vector(axis,sign)]*PULSE_SEND_COUNT + [(0,0,0,0)]
    fake=FakeDrone()
    checks=[0]
    def fail_mid_pulse():
        checks[0]+=1
        if checks[0]>=2:
            raise RuntimeError('marker lost')
    try:
        short_pulse(fake,rc_vector('right',1),fail_mid_pulse,lambda *a:None,sleep=lambda x:None)
    except RuntimeError:
        pass
    else:
        raise AssertionError('Missing marker must stop pulse')
    assert fake.calls == [(10,0,0,0),(0,0,0,0)],fake.calls
    # Regression: final pulse need not call check again AFTER the final send;
    # neutral must be attempted; further RC is gated by the subsequent observe.
    fake=FakeDrone()
    calls=[0]
    def final_interval_observer():
        calls[0] += 1
        if calls[0] > PULSE_SEND_COUNT:
            raise AssertionError('Post-interval check happened before neutralization')
    short_pulse(fake,(10,0,0,0),final_interval_observer,lambda *a:None,sleep=lambda x:None)
    assert calls[0] == PULSE_SEND_COUNT and fake.calls == [(10,0,0,0)]*PULSE_SEND_COUNT+[(0,0,0,0)]
    print('PASS: last RC interval always neutralizes before separate post-pulse review.')
    now=10.0
    good_tail=[(now-0.50 + i*0.10,(160.0,120.0)) for i in range(6)]
    assert reverse_stability(good_tail,now)[0]
    assert not reverse_stability(good_tail[:5],now)[0]
    assert not reverse_stability(good_tail[:3]+[(9.8,None)]+good_tail[4:],now)[0]
    assert not reverse_stability([(8.0,(160.0,120.0))]*6,now)[0]
    assert not reverse_stability(good_tail[:5]+[(9.95,(160.0,80.0))],now)[0]
    print('PASS: six-frame post-pulse reacquisition/stability gates reject missing, stale or off-centre samples.')
    print('PASS: camera RGB crop/180 rotation; RC signs; strict centrality/freshness;')
    print('PASS: mocked positive/negative pulses; marker-loss stops further RC and attempts neutral.')
    print('SELF-TEST ONLY. NO DJITELLOPY IMPORT, NETWORK, TAKEOFF OR MOTOR COMMANDS.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--self-test',action='store_true')
    mode.add_argument('--fly',action='store_true')
    parser.add_argument('--axis',choices=('right','forward'))
    args=parser.parse_args()
    if args.self_test:
        self_test()
    else:
        if not args.axis:
            parser.error('--fly requires --axis')
        run_live(args.axis)


if __name__=='__main__':
    main()
