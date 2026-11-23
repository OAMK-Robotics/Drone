"""V6: Marker-23 junction tracking experiment, 30-40 cm SDK operating window.

IMPORTANT: V5 tracked Marker 25 at the branch, rather than Marker 23 at the Y junction.
Tracking Marker 23 does NOT localize table edges and does NOT stabilize flight.
This is a guarded data-collection experiment; no automatic course corrections.

OFFLINE: python -B collect_motor_axis_streaming_v6.py --self-test

IMPORTANT: DJI/Ryze Tello's SDK takeoff is automatic and may rise above 40 cm.
This program REFUSES flight by default. Explicitly allowing takeoff overshoot
is necessary to permit a low-height (30-40 cm) AFTER-takeoff calibration.
SDK height telemetry is NOT an independently verified physical height sensor.
Do not fly if 40 cm is a hard limit at every instant or if adequate space and
visual supervision are unavailable. NOT obstacle avoidance or stabilization.
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
# SDK height may report a transient old value just after move_down() ACK.
# Observe descent settling BEFORE assessing a fresh five-sample height window.
HEIGHT_SETTLE_S = 1.5
MAX_FRAME_AGE_S = 0.40
WINDOW_S = 0.90
MIN_VALID = 4
MAX_MISSES_IN_WINDOW = 2
MAX_ERROR_X = 45
MAX_ERROR_Y = 42
MAX_POINT_SPREAD_PX = 15
# 30-40 cm operating-height target, NOT a guaranteed takeoff ceiling.
HEIGHT_MIN_CM = 30
HEIGHT_MAX_CM = 40
HEIGHT_SAMPLE_COUNT = 5
HEIGHT_SAMPLE_INTERVAL_S = 0.14
MAX_HEIGHT_SPREAD_CM = 6
# We only attempt ONE SDK 'down x' maneuver after auto-takeoff, and only
# for an initial measured altitude in this conservative range.
DESCENT_START_MIN_CM = 50
DESCENT_START_MAX_CM = 75
DESCENT_TARGET_CM = 35
SECONDARY_MARKER_ID = 25
PRIMARY_TRACKING_ID = 23
# Marker 25 is near the table edge in supplied physical photos.
# Image centering of Marker 25 IS NOT proof of edge clearance.
TRACKING_MARGIN_X = 28
TRACKING_MARGIN_Y = 28


def orient(rgb):
    if not isinstance(rgb, np.ndarray) or rgb.shape != (720, 320, 3):
        raise RuntimeError(f'Bad camera frame shape: {getattr(rgb, "shape", None)}')
    return cv2.rotate(cv2.cvtColor(rgb[:CROP_H, :CROP_W].copy(), cv2.COLOR_RGB2BGR), cv2.ROTATE_180)


def detect_marker25(image):
    """Detect Marker 25 as the flight-tracking reference.

    This is image tracking only, NOT a measurement of table-edge clearance.
    Marker 23 remains logged independently for analysis.
    """
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    if ids is None:
        return None
    for idx, detected_id in enumerate(ids.flatten().tolist()):
        if detected_id == SECONDARY_MARKER_ID:
            points = corners[idx].reshape(4, 2)
            center = points.mean(axis=0)
            return float(center[0]), float(center[1])
    return None


def settled_height_window(height_samples, elapsed_since_descent_ack):
    """Use a fresh window collected AFTER a settling delay, never an old sample.

    This is only a logical gate for cached SDK height estimates; neither it
    nor a time delay proves the drone's true physical altitude or stability.
    """
    if elapsed_since_descent_ack < HEIGHT_SETTLE_S:
        return False, 'Descent settling period incomplete'
    return in_target_height(height_samples)


def rc_vector(axis, sign):
    if axis == 'right':
        return (RC_SPEED * sign, 0, 0, 0)
    if axis == 'forward':
        return (0, RC_SPEED * sign, 0, 0)
    raise ValueError('Invalid axis')


def marker_quality(samples, now):
    """Tracking gate for Marker 23; NOT a physical tabletop edge check."""
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
    if abs(x - 160) > TRACKING_MARGIN_X or abs(y - 120) > TRACKING_MARGIN_Y:
        return False, f'Unsafe marker centrality: ({x:.1f},{y:.1f})'
    spread = max(((px - x)**2 + (py - y)**2)**0.5 for px, py in valid)
    if spread > MAX_POINT_SPREAD_PX:
        return False, f'Marker motion/spread too high: {spread:.1f}px'
    # Also check latest point, not only median: prevents late boundary excursions.
    lx, ly = valid[-1]
    if abs(lx - 160) > TRACKING_MARGIN_X or abs(ly - 120) > TRACKING_MARGIN_Y:
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
    if any(abs(point[0] - 160) > TRACKING_MARGIN_X or abs(point[1] - 120) > TRACKING_MARGIN_Y
           for _, point in tail):
        return False, 'one of last six marker positions close to centrality boundary'
    return True, 'six consecutive fresh and central marker detections'


def height_quality(heights):
    """Validate a window of SDK telemetry. Does NOT prove actual altitude."""
    if len(heights) != HEIGHT_SAMPLE_COUNT:
        return False, f'Expected {HEIGHT_SAMPLE_COUNT} height measurements'
    if any(isinstance(h, bool) or not isinstance(h, (int, float)) or not 0 < h < 300
           for h in heights):
        return False, 'Missing or implausible SDK height telemetry'
    spread = max(heights) - min(heights)
    if spread > MAX_HEIGHT_SPREAD_CM:
        return False, f'Unstable SDK height: spread {spread:.1f} cm'
    return True, f'median {statistics.median(heights):.1f} cm; spread {spread:.1f} cm'


def in_target_height(heights):
    good, reason = height_quality(heights)
    if not good:
        return False, reason
    if not all(HEIGHT_MIN_CM <= h <= HEIGHT_MAX_CM for h in heights):
        return False, f'Outside 30-40 cm SDK target: {heights}'
    return True, reason


def descent_distance(heights):
    """One SDK move_down from 50-75cm only. No descent on bad telemetry."""
    good, reason = height_quality(heights)
    if not good:
        raise RuntimeError('Descent forbidden: '+reason)
    mid = statistics.median(heights)
    if not DESCENT_START_MIN_CM <= mid <= DESCENT_START_MAX_CM:
        raise RuntimeError(f'No safe prevalidated descent for SDK height {mid:.1f} cm')
    # Documented SDK minimum step is 20cm; do not attempt smaller blind moves.
    return max(20, min(40, int(round(mid - DESCENT_TARGET_CM))))


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
            raise RuntimeError('RC neutralization failed: no further pulses') from exc
        event('PULSE_END', 'neutral command sent')


def run_live(axis, permit_auto_takeoff_overshoot=False, enable_table_pulses=False):
    # Refuse BEFORE drone initialization/network if user requires strict max 40cm.
    # Height telemetry and one post-takeoff descent cannot prevent auto-takeoff overshoot.
    if not permit_auto_takeoff_overshoot:
        print('REFUSED BEFORE NETWORK: Tello auto-takeoff may exceed 40 cm.',flush=True)
        print('V5 cannot enforce an absolute 40-cm ceiling. No flight or motor commands.',flush=True)
        return
    # Network-enabled imports ONLY inside this function, not the self-test.
    from djitellopy import Tello
    from hover.junction_marker_detector import detect_junction_marker

    out = Path('captures') / f'streaming_v5_marker25_{axis}_{datetime.now():%Y%m%d_%H%M%S}'
    out.mkdir(parents=True, exist_ok=False)
    outfile = out / 'observations.csv'
    fh = outfile.open('w', encoding='utf-8', newline='')
    writer = csv.writer(fh)
    writer.writerow(['elapsed_s','phase','event','marker23_x','marker23_y','marker25_x','marker25_y','sdk_height_cm','image','details'])
    started = time.monotonic()
    lock = threading.Lock()
    recent = deque(maxlen=30)
    recent_heights = deque(maxlen=30)
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
                             pt[0], pt[1], '', '', '', image, details])
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
                marker25 = detect_marker25(image)
                with lock:
                    i = state['count']
                    state['count'] += 1
                    phase = state['phase']
                filename = f'{i:04d}_{phase.lower()}.png'
                # Update control evidence as soon as detection completes.
                # Disk I/O must not make fresh camera data appear stale.
                measured_at = time.monotonic()
                try:
                    sdk_height = drone.get_height()   # Cached SDK telemetry, NOT ground truth.
                except Exception:
                    sdk_height = ''
                with lock:
                    recent.append((measured_at, point))
                    recent_heights.append((measured_at, sdk_height))
                    writer.writerow([f'{measured_at-started:.3f}', phase, 'FRAME',
                                     '' if point is None else point[0],
                                     '' if point is None else point[1],
                                     '' if marker25 is None else marker25[0],
                                     '' if marker25 is None else marker25[1],
                                     sdk_height, filename, 'rotated_180'])
                    fh.flush()
                if not cv2.imwrite(str(out / filename), image):
                    raise RuntimeError('Could not save video sample')
                # Summarize once per ~1 s; no infinite terminal spam.
                if i % 10 == 0:
                    print(f'FRAME {i} phase={phase} marker23={point} marker25={marker25}',flush=True)
            except Exception as exc:
                with lock:
                    errors.append(repr(exc))
                stop.set()
                print('CRITICAL CAMERA ERROR:', exc, flush=True)
                return
            stop.wait(max(0, FRAME_INTERVAL_S - (time.monotonic() - loop_start)))

    def check_airborne_budget():
        if airborne_at is not None and time.monotonic() - airborne_at >= MAX_AIRBORNE_S:
            raise RuntimeError('Airborne budget exceeded; land without pulses')

    def settle_after_descent():
        # No motor pulse or extra descent during this period. Still collecting
        # camera frames and height estimates continuously in background.
        set_phase('HEIGHT_SETTLE')
        event('HEIGHT_SETTLE_START', f'{HEIGHT_SETTLE_S:.1f}s after descent ACK')
        deadline = time.monotonic() + HEIGHT_SETTLE_S
        while time.monotonic() < deadline:
            check_airborne_budget()
            if stop.is_set() or errors:
                raise RuntimeError('Video capture stopped during settling')
            time.sleep(0.05)
        event('HEIGHT_SETTLE_DONE')

    def sample_heights():
        vals = []
        for _ in range(HEIGHT_SAMPLE_COUNT):
            check_airborne_budget()
            if stop.is_set() or errors:
                raise RuntimeError('Recorder stopped during SDK height assessment')
            try:
                vals.append(drone.get_height())
            except Exception as exc:
                raise RuntimeError('SDK height unavailable: '+str(exc)) from exc
            time.sleep(HEIGHT_SAMPLE_INTERVAL_S)
        return vals

    def check_marker23_only():
        # Height-independent: stop before moving down if the junction cannot be
        # tracked centrally and consistently. Still not a physical geofence.
        if stop.is_set() or errors or thread is None or not thread.is_alive():
            raise RuntimeError('PRE-DESCENT: Camera recorder failed or stopped')
        with lock:
            snapshot = list(recent)
        good, reason = marker_quality(snapshot, time.monotonic())
        if not good:
            raise RuntimeError('PRE-DESCENT: Marker 23 tracking unsafe: ' + reason)
        return reason

    def check():
        # Require an actual RECENT window of independently sampled timestamps.
        # All are still SDK telemetry estimates, not verified physical height.
        with lock:
            h_samples = list(recent_heights)[-HEIGHT_SAMPLE_COUNT:]
        now = time.monotonic()
        if (len(h_samples) != HEIGHT_SAMPLE_COUNT
                or now - h_samples[-1][0] > MAX_FRAME_AGE_S
                or h_samples[-1][0] - h_samples[0][0] < 0.30
                or any(not 0 <= now - t <= 0.9 for t, _ in h_samples)):
            raise RuntimeError('HEIGHT GATE: insufficient fresh telemetry frames')
        good_h, reason_h = in_target_height([h for _, h in h_samples])
        if not good_h:
            raise RuntimeError('HEIGHT GATE: '+reason_h)
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
            check()  # Abort as soon as tracking/height is unsafe; do not wait 1s.
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
        print('EXPERIMENT V6: 30-40 cm SDK target; Marker 23 primary; Marker 25 logged; no autonomous correction.',flush=True)
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
        print('Marker 23 is the junction target; its image position does NOT imply tabletop clearance.',flush=True)
        print('STOP if limited to an exposed tabletop: this code has NO table-edge geofence.',flush=True)
        print('Use only with independently verified safe clearance in ALL intended directions.',flush=True)
        print('Operator must watch aircraft and be ready for emergency intervention.',flush=True)
        approval = input(f'WARNING: auto-takeoff MAY EXCEED 40cm. Type OVERSHOOT-ACCEPT-{axis.upper()} if you accept this and area is clear: ').strip()
        if approval != f'OVERSHOOT-ACCEPT-{axis.upper()}':
            print('CANCELLED: no flight',flush=True)
            return
        if enable_table_pulses:
            print('WARNING: Marker 23 centering cannot detect table edge. No vision-based geofence.',flush=True)
            if input('Only with independently verified physical clearance in the intended direction, type TABLE-PULSE-CONSENT: ').strip() != 'TABLE-PULSE-CONSENT':
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
        print('Takeoff ACK. Height now assessed via SDK only; no physical-height guarantee.',flush=True)
        set_phase('HEIGHT_ASSESSMENT')
        before = sample_heights()
        good, why = height_quality(before)
        event('SDK_HEIGHT_BEFORE', str(before))
        if not good:
            raise RuntimeError('Height assessment failed: '+why)
        descended = False
        # Original V5 waited until the END of descent and neutral observation to
        # ask whether its tracked marker remained visible. Fail closed earlier.
        pre_desc = check_marker23_only()
        event('PRE_DESCENT_MARKER23_GATE_PASS', pre_desc)
        if not in_target_height(before)[0]:
            distance = descent_distance(before)  # raises without sending if not 50-75cm
            print(f'Attempting ONE SDK move_down({distance}) based on approximate telemetry; not a guaranteed 35cm.',flush=True)
            event('DESCENT_SENT', f'down {distance} from {before}')
            set_phase('DESCENT')
            drone.move_down(distance)
            descent_acked_at = time.monotonic()
            event('DESCENT_ACK')
            descended = True
            settle_after_descent()
        else:
            descent_acked_at = None
            # Already in target; still observe to reduce racing with takeoff.
            settle_after_descent()
        set_phase('HEIGHT_VERIFY')
        after = sample_heights()
        event('SDK_HEIGHT_AFTER_SETTLING', str(after))
        elapsed_settle = time.monotonic()-descent_acked_at if descended else HEIGHT_SETTLE_S
        good, why = settled_height_window(after, elapsed_settle)
        if not good:
            raise RuntimeError('LOW-HEIGHT TARGET FAILED, NO MOTOR PULSES: '+why)
        print('SDK low-height gate PASS after settling:',after,'(not independent physical measurement)',flush=True)
        # Two neutral windows with height checks before horizontal action.
        observe(1.0, 'NEUTRAL_1')
        observe(1.0, 'NEUTRAL_2')
        event('PRE_PULSE_GATE',check())
        if not enable_table_pulses:
            print('OBSERVATION COMPLETE: Marker 23 neutral data collected. No horizontal pulses (default).',flush=True)
            event('OBSERVATION_COMPLETE', 'No motor pulses; table boundary unknown')
            return
        print('Starting small positive pulse; monitoring Marker 23 DURING each RC interval.',flush=True)
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
            except KeyboardInterrupt:
                # Previously this escaped `except Exception` and abandoned cleanup.
                # No second unverified land command: landing status is UNKNOWN.
                event('LANDING_INTERRUPTED', 'Ctrl+C during drone.land(); touchdown unconfirmed')
                print('CRITICAL: landing interrupted, status UNKNOWN; visually verify aircraft.',flush=True)
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
        print('NO AUTONOMOUS DRIFT CORRECTION. SDK HEIGHT NOT PHYSICAL GROUND TRUTH. Flight safety unverified.',flush=True)
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
    assert in_target_height([35,34,35,36,35])[0]
    assert not in_target_height([30,35,42,34,35])[0]
    assert not in_target_height([0,35,35,35,35])[0]
    assert not in_target_height([35,35,35,35,60])[0]
    assert descent_distance([60,61,59,60,60]) == 25
    for bad in ([45,45,45,45,45],[85,85,85,85,85],[0,0,0,0,0], [60,60,60,60,90]):
        try:
            descent_distance(bad)
        except RuntimeError:
            pass
        else:
            raise AssertionError('Unsafe descent must be rejected')
    # Regression based on V3 flight: [40,30,30,30,30] right after descent ACK.
    # It must be rejected, and a fresh stable window accepted ONLY after settling.
    assert not settled_height_window([40,30,30,30,30], 0.1)[0]
    assert not settled_height_window([30]*5, HEIGHT_SETTLE_S-0.1)[0]
    assert settled_height_window([30]*5, HEIGHT_SETTLE_S+0.1)[0]
    assert not settled_height_window([40,30,30,30,30], HEIGHT_SETTLE_S+0.1)[0]
    assert not settled_height_window([25]*5, HEIGHT_SETTLE_S+0.1)[0]

    # Marker 23 is the primary 2D tracking reference after height settling.
    # It is NOT proof of a safe distance from the physical table edge.
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    canvas = np.full((240,320,3),255,dtype=np.uint8)
    if hasattr(cv2.aruco,'generateImageMarker'):
        tile=cv2.aruco.generateImageMarker(dictionary,25,70)
    else:
        tile=cv2.aruco.drawMarker(dictionary,25,70)
    canvas[80:150,110:180] = cv2.cvtColor(tile,cv2.COLOR_GRAY2BGR)
    marker25 = detect_marker25(canvas)
    assert marker25 is not None and abs(marker25[0]-144.5) < 2 and abs(marker25[1]-114.5)<2, marker25
    assert PRIMARY_TRACKING_ID == 23
    assert marker_quality([(9.2+i*0.1,(160.0,120.0)) for i in range(9)],10)[0]
    assert not marker_quality([(9.6,None)]*5,10)[0]
    assert not marker_quality([(9.2+i*0.1,(200.0,120.0)) for i in range(9)],10)[0]

    class FailNeutralDrone:
        def __init__(self): self.calls=[]
        def send_rc_control(self,*vector):
            self.calls.append(tuple(vector))
            if vector == (0,0,0,0): raise RuntimeError('neutral failure')
    fail = FailNeutralDrone()
    try:
        short_pulse(fail,(10,0,0,0),lambda:'safe',lambda *a:None,sleep=lambda _:None)
        raise AssertionError('Neutral failure must abort experiment')
    except RuntimeError as exc:
        assert 'neutral' in str(exc).lower() and fail.calls[-1]==(0,0,0,0)
    print('PASS: descent settling; Marker 23 primary reference and centrality gates; fail-closed neutral failure.')
    print('PASS: V6 SDK height-window validation, unsafe descent rejection, target 30-40 cm.')
    print('IMPORTANT: 40-cm hard takeoff ceiling is impossible to guarantee with auto-takeoff.')
    print('PASS: last RC interval always neutralizes before separate post-pulse review.')
    now=10.0
    good_tail=[(now-0.50 + i*0.10,(160.0,120.0)) for i in range(6)]
    assert reverse_stability(good_tail,now)[0]
    assert not reverse_stability(good_tail[:5],now)[0]
    assert not reverse_stability(good_tail[:3]+[(9.8,None)]+good_tail[4:],now)[0]
    assert not reverse_stability([(8.0,(160.0,120.0))]*6,now)[0]
    assert not reverse_stability(good_tail[:5]+[(9.95,(160.0,80.0))],now)[0]
    print('PASS: six-frame post-pulse reacquisition/stability gates reject missing, stale or off-centre samples.')
    print('PASS: camera RGB crop/180 rotation; RC signs; Marker23 centrality/freshness;')
    print('PASS: mocked positive/negative pulses; marker-loss stops further RC and attempts neutral.')
    print('SELF-TEST ONLY. NO DJITELLOPY IMPORT, NETWORK, TAKEOFF OR MOTOR COMMANDS.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--self-test',action='store_true')
    mode.add_argument('--fly',action='store_true')
    parser.add_argument('--axis',choices=('right','forward'))
    parser.add_argument('--enable-table-pulses', action='store_true', help='Explicit opt-in for short RC test after visual table-clearance verification; not a geofence')
    parser.add_argument('--permit-auto-takeoff-overshoot', action='store_true',
                        help='Explicitly acknowledge auto-takeoff may exceed 40cm; not a hard ceiling')
    args=parser.parse_args()
    if args.self_test:
        self_test()
    else:
        if not args.axis:
            parser.error('--fly requires --axis')
        run_live(args.axis, args.permit_auto_takeoff_overshoot, args.enable_table_pulses)


if __name__=='__main__':
    main()
