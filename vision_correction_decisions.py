"""Offline decisions from vision_drift_offline.py CSV (NO DRONE CONTROL).

Reads drift_offline_report.csv and computes small, bounded IMAGE-SPACE
correction requests only for confidently tracked frames in the fixed-height
capture phase. This is deliberately not an RC/drone command generator.

Usage:
    python vision_correction_decisions.py captures/takeoff40_20261116_143116
    python vision_correction_decisions.py --self-test
"""

import argparse
import csv
import math
from pathlib import Path
from tempfile import TemporaryDirectory

VALID_PHASE = 'TARGET_CAPTURE_START'
VALID_STATUS = 'TWO_MARKERS'
EXPECTED_IDS = {23, 25}
DEADBAND_PX = 5.0
GAIN = 0.5
MAX_REQUEST_PX = 4.0
MAX_STEP_PX = 20.0  # Reject sudden discontinuities between successive good frames.
REACQUIRE_FRAMES = 3


def clamp(x, limit=MAX_REQUEST_PX):
    return max(-limit, min(limit, x))


def rounded(x):
    return round(x, 2)


def score(error):
    """Bounded image-space request, NOT RC control."""
    return 0.0 if abs(error) <= DEADBAND_PX else rounded(clamp(-GAIN * error))


def parse_markers(value):
    try:
        return {int(s) for s in value.split(',') if s.strip()}
    except ValueError:
        return set()


def decide(rows):
    """Return (results, summary) with strict confidence gating.

    Base is the first trustworthy measurement in the target-height stage.
    Report dx/dy are already adjusted for observed marker scale by the
    upstream offline estimator; subtracting a target-height baseline avoids
    using a reference acquired during an entirely different flight phase.

    Loss of both trusted markers after locking baseline is latched to neutral.
    We intentionally do not resume corrections without a new deliberate run.
    """
    results = []
    baseline = None
    previous_error = None
    stopped = False
    recoverable = False
    reacquire_count, reacquire_prev = 0, None
    for row in rows:
        phase = row['phase']
        state = row['status']
        markers = parse_markers(row['markers'])
        good = (phase == VALID_PHASE and state == VALID_STATUS
                and EXPECTED_IDS.issubset(markers)
                and row['dx_px'].strip() != '' and row['dy_px'].strip() != '')
        result = {
            'time_s': row['time_s'], 'phase': phase, 'filename': row['filename'],
            'source_status': state, 'local_dx_px': '', 'local_dy_px': '',
            'request_image_x_px': 0, 'request_image_y_px': 0,
            'decision': ''
        }
        if phase != VALID_PHASE:
            result['decision'] = 'OUTSIDE_TARGET_STAGE_NEUTRAL'
        elif stopped:
            result['decision'] = 'TRACKING_LOST_LATCHED_NEUTRAL'
            if recoverable and good:
                try:
                    dx, dy = float(row['dx_px']), float(row['dy_px'])
                    if not (math.isfinite(dx) and math.isfinite(dy)):
                        raise ValueError('nonfinite')
                except ValueError:
                    reacquire_count, reacquire_prev = 0, None
                else:
                    if (reacquire_prev is None or
                            math.hypot(dx - reacquire_prev[0], dy - reacquire_prev[1]) > MAX_STEP_PX):
                        reacquire_count = 1
                    else:
                        reacquire_count += 1
                    reacquire_prev = (dx, dy)

                    if reacquire_count >= REACQUIRE_FRAMES:
                        baseline, previous_error = (dx, dy), (0.0, 0.0)
                        stopped, recoverable = False, False
                        reacquire_count, reacquire_prev = 0, None
                        result['local_dx_px'] = result['local_dy_px'] = 0.0
                        result['decision'] = 'TRACKING_RECOVERED_REFERENCE_SET_NEUTRAL'
                    else:
                        result['decision'] = 'TRACKING_REACQUIRING_NEUTRAL'
            elif recoverable:
                reacquire_count, reacquire_prev = 0, None
        elif not good:
            result['decision'] = 'UNTRUSTED_TRACKING_NEUTRAL'
            if baseline is not None:
                stopped = True
                recoverable = True
        else:
            try:
                dx, dy = float(row['dx_px']), float(row['dy_px'])
                if not math.isfinite(dx) or not math.isfinite(dy):
                    raise ValueError('nonfinite')
            except ValueError:
                result['decision'] = 'INVALID_MEASUREMENT_NEUTRAL'
                if baseline is not None:
                    stopped = True
            else:
                if baseline is None:
                    baseline = (dx, dy)
                    previous_error = (0.0, 0.0)
                    result['local_dx_px'] = result['local_dy_px'] = 0.0
                    result['decision'] = 'TARGET_REFERENCE_SET_NEUTRAL'
                else:
                    ex, ey = dx - baseline[0], dy - baseline[1]
                    if math.hypot(ex - previous_error[0], ey - previous_error[1]) > MAX_STEP_PX:
                        stopped = True
                        result['decision'] = 'SUDDEN_CHANGE_LATCHED_NEUTRAL'
                    else:
                        previous_error = (ex, ey)
                        xreq, yreq = score(ex), score(ey)
                        result.update(local_dx_px=rounded(ex), local_dy_px=rounded(ey),
                                      request_image_x_px=xreq, request_image_y_px=yreq,
                                      decision='IMAGE_REQUEST_ONLY' if (xreq or yreq)
                                               else 'DEADBAND_NEUTRAL')
        results.append(result)
    summary = {
        'rows': len(rows),
        'target_frames': sum(r['phase'] == VALID_PHASE for r in rows),
        'reference_set': baseline is not None,
        'requested_frames': sum(r['decision'] == 'IMAGE_REQUEST_ONLY' for r in results),
        'neutral_frames': sum(r['request_image_x_px'] == 0 and
                              r['request_image_y_px'] == 0 for r in results),
        'tracking_latched': stopped,
    }
    return results, summary


def process(folder):
    folder = Path(folder)
    source = folder / 'drift_offline_report.csv'
    if not source.is_file():
        raise SystemExit(f'Missing input: {source}')
    with source.open(newline='', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        required = {'time_s', 'phase', 'filename', 'markers', 'dx_px', 'dy_px', 'status'}
        if not required.issubset(reader.fieldnames or []):
            raise SystemExit(f'Missing fields: {sorted(required - set(reader.fieldnames or []))}')
        rows = list(reader)
    if not rows:
        raise SystemExit('Input CSV has no data rows')
    results, summary = decide(rows)
    target = folder / 'correction_offline_report.csv'
    with target.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=list(results[0]))
        writer.writeheader()
        writer.writerows(results)
    for key, value in summary.items():
        print(f'{key}: {value}')
    for r in [r for r in results if r['phase'] == VALID_PHASE][-4:]:
        print('LAST_TARGET:', r['time_s'], r['decision'],
              'error:', (r['local_dx_px'], r['local_dy_px']),
              'image_request:', (r['request_image_x_px'], r['request_image_y_px']))
    print('Saved:', target)
    print('IMAGE-SPACE DECISIONS ONLY. NO DRONE CONNECTION. NO FLIGHT.')


def self_test():
    def entry(t, phase, status, markers, dx, dy):
        return {'time_s': str(t), 'phase': phase, 'filename': f'f{t}.png',
                'status': status, 'markers': markers, 'dx_px': str(dx), 'dy_px': str(dy)}
    p = VALID_PHASE
    r = [entry(1, 'TAKEOFF_SENT', VALID_STATUS, '23,25', 30, 30),
         entry(11.4, p, VALID_STATUS, '23,25', -90, -20),
         entry(11.5, p, VALID_STATUS, '23,25', -93, -25),
         entry(11.6, p, VALID_STATUS, '23,25', -103, -24),
         entry(11.7, 'LAND_SENT', VALID_STATUS, '23,25', -105, -30)]
    a, s = decide(r)
    assert a[0]['decision'] == 'OUTSIDE_TARGET_STAGE_NEUTRAL'
    assert a[1]['decision'] == 'TARGET_REFERENCE_SET_NEUTRAL'
    assert a[2]['decision'] == 'DEADBAND_NEUTRAL'
    assert a[3]['decision'] == 'IMAGE_REQUEST_ONLY'
    assert (a[3]['request_image_x_px'], a[3]['request_image_y_px']) == (4.0, 0.0)
    assert a[4]['decision'] == 'OUTSIDE_TARGET_STAGE_NEUTRAL'
    assert s['requested_frames'] == 1
    r2 = r[:2] + [entry(11.5, p, 'NO_MARKERS_NEUTRAL', '', '', ''), r[3]]
    a2, s2 = decide(r2)
    assert a2[-1]['decision'] == 'TRACKING_REACQUIRING_NEUTRAL'
    assert s2['tracking_latched']
    r3 = r[:2] + [entry(11.5, p, VALID_STATUS, '23,25', 100, 100)]
    a3, _ = decide(r3)
    assert a3[-1]['decision'] == 'SUDDEN_CHANGE_LATCHED_NEUTRAL'
    assert score(1000) == -MAX_REQUEST_PX
    assert score(2) == 0
    recovery = r[:2] + [
        entry(11.45, p, 'SINGLE_MARKER_LOW_CONFIDENCE', '23', '', ''),
        entry(11.5, p, VALID_STATUS, '23,25', -100, -30),
        entry(11.6, p, VALID_STATUS, '23,25', -102, -30),
        entry(11.7, p, VALID_STATUS, '23,25', -103, -30),
        entry(11.8, p, VALID_STATUS, '23,25', -114, -30)]
    a4, s4 = decide(recovery)
    assert [v['decision'] for v in a4[-4:]] == [
        'TRACKING_REACQUIRING_NEUTRAL',
        'TRACKING_REACQUIRING_NEUTRAL',
        'TRACKING_RECOVERED_REFERENCE_SET_NEUTRAL',
        'IMAGE_REQUEST_ONLY']
    assert all(v['request_image_x_px'] == v['request_image_y_px'] == 0
               for v in a4[-4:-1])
    assert not s4['tracking_latched']
    print('PASS: neutral-on-loss, three-good-frame recovery, fresh reference, jump latch')
    print('NO DRONE CONNECTION. NO FLIGHT.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', nargs='?')
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test()
    elif args.folder:
        process(args.folder)
    else:
        parser.print_help()
