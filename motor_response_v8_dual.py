"""V7 â€” one-flight, short-pulse, evidence-first Tello motor-response calibration.

This is NOT an autonomous hover controller or a tabletop geofence. V7 performs
NO automatic altitude descent: previous V3â€“V6 recordings repeatedly lost markers
DURING descent. Instead it checks the existing post-takeoff height estimate,
uses Marker 23, records camera observations continuously, and attempts at most
ONE short +axis pulse and a conditional reverse pulse.

Self-test (no drone, no network): python -B motor_response_v7.py --self-test
Live flight (ONLY with independently verified safe recovery area):
  python -B motor_response_v7.py --fly --axis right --allow-takeoff-overshoot --verified-recovery-area

Do not use --fly over an unprotected 1 x 2 m tabletop or where loss of position
could cause injury or damage. Camera marker position is NOT a table-edge map.
DJI Tello automatic takeoff may exceed intended altitude; SDK height estimates
are not independent physical measurements. A motor-response result is NOT
promised: failing safety gates must mean NO PULSE.
"""
from __future__ import annotations

import argparse
import csv
from collections import deque
from datetime import datetime
from pathlib import Path
from statistics import median
import threading
import time
import math

HOST = '192.168.1.63'  # previously used lab IP; NOT guaranteed for another network
BATTERY_MIN = 40
FRAME_INTERVAL = 0.10
MAX_AIRBORNE = 22.0
SPEED = 10
RC_INTERVAL = 0.15
RC_SEND_COUNT = 3  # maximum ~0.45 s pulse; always send neutral afterward
HEIGHT_LOW, HEIGHT_HIGH = 50, 85  # SDK-estimated *post-takeoff* range, not physical altitude
HEIGHT_SPREAD = 6
FRAME_MAX_AGE = 0.4
TRACK_WINDOW = 0.60
TRACK_MIN_VALID = 4
TRACK_MAX_MISSING = 1
TRACK_MAX_SPREAD = 14.0
TRACK_CENTRE_X = 40.0
TRACK_CENTRE_Y = 38.0


def pair_anchor(m23,m25):
    if m23 is None or m25 is None: return None
    if not all(12 <= p[0] <= 308 and 12 <= p[1] <= 228 for p in (m23,m25)): return None
    if not 40 <= math.dist(m23,m25) <= 170: return None
    return ((m23[0]+m25[0])/2,(m23[1]+m25[1])/2)

def point_gate(observations, now):
    """Recent camera-space tracking only; NOT a physical boundary certificate."""
    recent = [(t, p) for t, p in observations if 0 <= now-t <= TRACK_WINDOW]
    if not recent or now - recent[-1][0] > FRAME_MAX_AGE:
        return False, 'stale/missing camera sample'
    if recent[-1][1] is None:
        return False, 'latest marker pair missing'
    valid = [p for _, p in recent if p is not None]
    if len(valid) < TRACK_MIN_VALID or len(recent)-len(valid)>TRACK_MAX_MISSING:
        return False, f'tracking insufficient: {len(valid)}/{len(recent)}'
    x = median(p[0] for p in valid)
    y = median(p[1] for p in valid)
    if abs(x-160)>TRACK_CENTRE_X or abs(y-120)>TRACK_CENTRE_Y:
        return False, f'marker pair midpoint off-centre ({x:.1f},{y:.1f})'
    if abs(valid[-1][0]-160)>TRACK_CENTRE_X or abs(valid[-1][1]-120)>TRACK_CENTRE_Y:
        return False, 'latest marker pair midpoint off-centre'
    spread = max(math.dist(p, (x,y)) for p in valid)
    if spread > TRACK_MAX_SPREAD:
        return False, f'marker spread {spread:.1f}px exceeds limit'
    return True, f'M23 ({x:.1f},{y:.1f}) valid {len(valid)}/{len(recent)} spread {spread:.1f}'


def height_gate(samples, now):
    """Fresh SDK height samples only; no proof of physical height."""
    tail = [(t,h) for t,h in samples if 0 <= now-t <= 0.85][-5:]
    if len(tail)<5 or now-tail[-1][0]>FRAME_MAX_AGE or tail[-1][0]-tail[0][0]<0.30:
        return False,'height telemetry too old/short'
    h=[v for _,v in tail]
    if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not 0<v<300 for v in h):
        return False,f'invalid SDK heights {h}'
    if max(h)-min(h)>HEIGHT_SPREAD:
        return False,f'height estimate unstable {h}'
    if any(v<HEIGHT_LOW or v>HEIGHT_HIGH for v in h):
        return False,f'SDK height outside {HEIGHT_LOW}-{HEIGHT_HIGH} cm: {h}'
    return True, f'SDK heights {h} (not physically measured)'


def motor_vector(axis, sign):
    if axis not in ('right','forward') or sign not in (-1,1):
        raise ValueError('bad motor-axis/sign')
    return (sign*SPEED,0,0,0) if axis=='right' else (0,sign*SPEED,0,0)


def send_short_pulse(drone, vec, verify, record, pause=time.sleep):
    """Gate BEFORE every nonzero RC send; neutral ALWAYS in finally."""
    sent=0
    record('PULSE_START', str(vec))
    try:
        for index in range(RC_SEND_COUNT):
            verify()
            drone.send_rc_control(*vec)
            sent+=1
            record('PULSE_COMMAND', f'{index+1}/{RC_SEND_COUNT} {vec}')
            pause(RC_INTERVAL)
    finally:
        try:
            drone.send_rc_control(0,0,0,0)
            record('PULSE_NEUTRAL',f'neutral ACK unavailable; sent={sent}')
        except Exception as exc:
            record('CRITICAL_NEUTRAL_FAILURE',repr(exc))
            raise RuntimeError('RC neutralization failed') from exc
    return sent


def detect_both(image, detector):
    import cv2
    import numpy as np
    corners, ids, _=detector.detectMarkers(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY))
    found={}
    if ids is None: return found
    for mid,c in zip(ids.flatten(),corners):
        if int(mid) in (23,25):
            xy=np.mean(c[0],axis=0)
            found[int(mid)]=(float(xy[0]),float(xy[1]))
    return found


def orient(rgb):
    import cv2
    import numpy as np
    if not isinstance(rgb,np.ndarray) or rgb.shape!=(720,320,3):
        raise RuntimeError(f'bad frame shape {getattr(rgb,"shape",None)}')
    return cv2.rotate(cv2.cvtColor(rgb[:240,:320].copy(),cv2.COLOR_RGB2BGR),cv2.ROTATE_180)


def run_live(axis):
    import cv2
    from djitellopy import Tello
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector=cv2.aruco.ArucoDetector(dictionary)
    folder=Path('captures')/f'motor_v8_dual_{axis}_{datetime.now():%Y%m%d_%H%M%S}'
    folder.mkdir(parents=True,exist_ok=False)
    stream_file=(folder/'observations.csv').open('w',encoding='utf-8',newline='')
    csvwriter=csv.writer(stream_file)
    csvwriter.writerow(['elapsed_s','phase','event','marker23_x','marker23_y','marker25_x','marker25_y','sdk_height_cm','image','detail'])
    origin=time.monotonic()
    lock=threading.RLock()
    observations=deque(maxlen=40)
    heights=deque(maxlen=40)
    stop=threading.Event()
    failures=[]
    state={'phase':'PREP','count':0}
    drone=Tello(host=HOST)
    connected=False; streaming=False; attempted=False; worker=None; airborne_at=None

    def setphase(p):
        with lock: state['phase']=p

    def log(name,detail=''):
        with lock:
            csvwriter.writerow([f'{time.monotonic()-origin:.3f}',state['phase'],name,'','','','','','',str(detail)])
            stream_file.flush()

    def record_images(reader):
        while not stop.is_set():
            stamp_start=time.monotonic()
            try:
                image=orient(reader.frame)
                detected=detect_both(image,detector)
                marker23=detected.get(23); marker25=detected.get(25)
                try: h=drone.get_height()
                except Exception: h=None
                recorded_at=time.monotonic()
                with lock:
                    idx=state['count']; state['count']+=1
                    phase=state['phase']; observations.append((recorded_at,pair_anchor(marker23,marker25)))
                    heights.append((recorded_at,h))
                    filename=f'{idx:04d}_{phase.lower()}.png'
                    csvwriter.writerow([f'{recorded_at-origin:.3f}',phase,'FRAME',
                                       '' if marker23 is None else marker23[0],
                                       '' if marker23 is None else marker23[1],
                                       '' if marker25 is None else marker25[0],
                                       '' if marker25 is None else marker25[1],
                                       '' if h is None else h,filename,'rotated_180'])
                    stream_file.flush()
                if not cv2.imwrite(str(folder/filename),image):
                    raise RuntimeError('frame image write failed')
                if idx%10==0:
                    print(f'FRAME {idx} {phase} M23={marker23} M25={marker25} H={h}',flush=True)
            except Exception as exc:
                failures.append(repr(exc)); stop.set()
                print('CRITICAL CAMERA:',exc,flush=True)
                return
            stop.wait(max(0.0,FRAME_INTERVAL-(time.monotonic()-stamp_start)))

    def check_time():
        if airborne_at is not None and time.monotonic()-airborne_at>MAX_AIRBORNE:
            raise RuntimeError('airborne time budget reached')

    def gate(require_marker=True):
        check_time()
        if failures or stop.is_set() or worker is None or not worker.is_alive():
            raise RuntimeError('camera recording failed')
        with lock: obs=list(observations); h=list(heights)
        now=time.monotonic()
        ok_h,why_h=height_gate(h,now)
        if not ok_h: raise RuntimeError('HEIGHT GATE: '+why_h)
        if require_marker:
            ok,why=point_gate(obs,now)
            if not ok: raise RuntimeError('MARKER GATE: '+why)
        return why_h

    def hold(seconds,phase,require_marker=True):
        setphase(phase)
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            gate(require_marker=require_marker)
            time.sleep(0.05)
        log('HOLD_OK',phase)

    try:
        print('V7 one-flight calibration: Marker 23 reference; NO DESCENT; NO autonomous correction',flush=True)
        drone.connect(); connected=True
        battery=drone.get_battery()
        log('BATTERY',battery)
        print('BATTERY',battery,'%',flush=True)
        if not isinstance(battery,(int,float)) or battery<BATTERY_MIN:
            raise RuntimeError('low/invalid battery; no flight')
        drone.set_video_direction(Tello.CAMERA_DOWNWARD)
        drone.streamon(); streaming=True
        reader=drone.get_frame_read()
        preview=None
        for _ in range(24):
            time.sleep(.25)
            try: image=orient(reader.frame)
            except RuntimeError:continue
            if image.std()>=10 and image.max()>=15:
                preview=image;break
        if preview is None:raise RuntimeError('no valid preview; no takeoff')
        if not cv2.imwrite(str(folder/'preflight.png'),preview):
            raise RuntimeError('failed to save preflight; no takeoff')
        print('PREVIEW',folder/'preflight.png',flush=True)
        print('IMPORTANT: marker centrality does NOT prove table-edge clearance.',flush=True)
        print('ONLY fly with independently verified safe recovery space and continuous human observation.',flush=True)
        consent=input('If area is independently safe for brief two-way motion, type SAFE-AREA-VERIFIED: ').strip()
        if consent!='SAFE-AREA-VERIFIED':
            log('CANCEL','No verified safe recovery area')
            print('CANCELLED; no takeoff',flush=True);return
        worker=threading.Thread(target=record_images,args=(reader,),daemon=True,name='v7-recorder')
        worker.start();time.sleep(.35)
        if failures or stop.is_set() or state['count']<2:
            raise RuntimeError('camera thread not ready')
        setphase('TAKEOFF')
        attempted=True; airborne_at=time.monotonic()
        log('TAKEOFF_SENT')
        drone.takeoff()
        log('TAKEOFF_ACK')
        print('TAKEOFF ACK; checking post-takeoff SDK height and Marker 23 WITHOUT descending',flush=True)
        # A short neutral period allows fresh SDK and camera samples; original
        # V3-V6 waited through a long descent and usually lost their markers.
        # Wait without motor movement to collect *fresh* samples. No reason
        # to fail on transient height readings before the first final gate.
        setphase('NEUTRAL_BEFORE')
        until_neutral=time.monotonic()+0.65
        while time.monotonic()<until_neutral:
            check_time()
            if stop.is_set() or failures or worker is None or not worker.is_alive():
                raise RuntimeError('camera recorder stopped during neutral baseline')
            time.sleep(0.05)
        log('NEUTRAL_RECORDED','0.65 sec; no horizontal commands')
        # Historical V5 data show occasional single-frame ArUco dropouts.
        # While all motors remain neutral, allow <=0.45 s to reacquire;
        # never ignore missing markers DURING an actual pulse.
        last_reason=''
        until=time.monotonic()+0.45
        while True:
            try:
                gate(require_marker=True)
                break
            except RuntimeError as exc:
                last_reason=str(exc)
                if not last_reason.startswith(('MARKER GATE:','HEIGHT GATE:')):
                    raise  # recorder/time faults cannot be retried
                if time.monotonic()>=until:
                    raise RuntimeError('PRE-PULSE GATES FAILED: '+last_reason)
                time.sleep(0.05)
        log('PRE_PULSE_GATE_PASS','reacquisition window <=0.45s')
        setphase('FIRST_PULSE')
        send_short_pulse(drone,motor_vector(axis,-1),lambda:gate(True),log)
        # Important: record post-response even if marker becomes missing; neutral
        # RC and land rather than blindly sending a new movement command.
        hold(0.65,'AFTER_FIRST',require_marker=False)
        try:
            gate(require_marker=True)
            with lock: tail=list(observations)[-6:]
            if len(tail)<6 or any(point is None for _,point in tail):
                raise RuntimeError('reverse needs 6 consecutive marker pair detections')
            log('REVERSE_GATE_PASS')
            setphase('REVERSE_PULSE')
            send_short_pulse(drone,motor_vector(axis,+1),lambda:gate(True),log)
            hold(0.65,'AFTER_REVERSE',require_marker=False)
        except RuntimeError as exc:
            log('REVERSE_SKIPPED',repr(exc))
            print('REVERSE SKIPPED:',exc,flush=True)
        log('FINISH','data collection done; landing')
    except KeyboardInterrupt:
        print('INTERRUPTED; neutral and land',flush=True)
        log('INTERRUPTED')
    except Exception as exc:
        print('STOP:',exc,flush=True)
        log('STOP',repr(exc))
    finally:
        if attempted:
            setphase('LANDING')
            try:
                drone.send_rc_control(0,0,0,0)
                log('FINAL_NEUTRAL_SENT')
            except BaseException as exc:
                log('FINAL_NEUTRAL_ERROR',repr(exc))
                print('CRITICAL neutral failed:',exc,flush=True)
            # A second Ctrl+C must not silently leave the recorder running.
            try:
                log('LANDING_SENT')
                drone.land()
                log('LANDING_ACK')
                print('LAND ACK: confirm propellers stopped and touchdown visually',flush=True)
            except KeyboardInterrupt:
                log('LANDING_INTERRUPTED','No landing ACK; physically verify landing')
                print('LANDING INTERRUPTED. Verify drone actually landed!',flush=True)
            except Exception as exc:
                log('LANDING_ERROR',repr(exc))
                print('LANDING FAILED: verify and use operator emergency procedure!',exc,flush=True)
        stop.set()
        if worker:
            worker.join(timeout=4)
            if worker.is_alive():print('WARNING: recorder thread did not stop',flush=True)
        if streaming:
            try:drone.streamoff()
            except Exception as exc:print('streamoff error',exc,flush=True)
        if connected:
            try:drone.end()
            except Exception as exc:print('drone.end error',exc,flush=True)
        stream_file.close()
        print('CSV:',folder/'observations.csv',flush=True)
        print('DATASET:',folder,flush=True)
        try: analyze_csv(folder)
        except Exception as exc: print('CSV ANALYSIS ERROR:',exc,flush=True)
        print('NOT AN AUTONOMOUS HOVER. NOT A PHYSICAL ALTITUDE OR TABLE SAFETY CERTIFICATION.',flush=True)


def self_test():
    now=100.0
    samples=[(99.5+i*.1,(160.,120.)) for i in range(6)]
    assert point_gate(samples,now)[0]
    assert not point_gate(samples[:-1]+[(now,None)],now)[0]
    assert not point_gate([(99.5+i*.1,(210.,120.)) for i in range(6)],now)[0]
    assert not point_gate([(now-.1,(160.,120.))],now)[0]
    assert height_gate([(99.5+i*.1,70) for i in range(6)],now)[0]
    assert not height_gate([(99.5+i*.1,30) for i in range(6)],now)[0]
    assert not height_gate([(99.5+i*.1,90) for i in range(6)],now)[0]
    assert motor_vector('right',1)==(10,0,0,0)
    assert motor_vector('right',-1)==(-10,0,0,0)
    assert motor_vector('forward',1)==(0,10,0,0)
    class Mock:
        def __init__(self):self.calls=[]
        def send_rc_control(self,*x):self.calls.append(x)
    for a in ('right','forward'):
        d=Mock();events=[]
        assert send_short_pulse(d,motor_vector(a,1),lambda:None,lambda *x:events.append(x),lambda _:None)==3
        assert d.calls==[motor_vector(a,1)]*3+[(0,0,0,0)]
        assert sum(x[0]=='PULSE_COMMAND' for x in events)==3
    d=Mock();k=[0]
    def guard():
        k[0]+=1
        if k[0]==2:raise RuntimeError('marker lost')
    try:send_short_pulse(d,motor_vector('right',1),guard,lambda *x:None,lambda _:None)
    except RuntimeError:pass
    else:raise AssertionError('lost marker did not stop pulse')
    assert d.calls==[(10,0,0,0),(0,0,0,0)],d.calls
    print('PASS: marker gates, missing-marker rejection, SDK-height gates, +/- axis vectors')
    print('PASS: mocked pulse and mid-pulse fail-closed neutralization; no drone/network import')
    print('REMINDER: Real flight and physical safety have NOT been validated.')


def analyze_csv(path):
    """Post-flight image-space response summary; NEVER asserts calibrated motor physics."""
    import csv
    from statistics import median
    file=Path(path)
    if file.is_dir(): file=file/'observations.csv'
    with file.open(encoding='utf-8-sig',newline='') as fh:
        rows=list(csv.DictReader(fh))
    commands=[r for r in rows if r.get('event')=='PULSE_COMMAND']
    print('--- V7 MOTOR EVIDENCE (OFFLINE CSV) ---')
    print('NONZERO RC COMMANDS LOGGED:',len(commands),
          'LAND ACK:',any(r.get('event')=='LANDING_ACK' for r in rows))
    phases=['NEUTRAL_BEFORE','FIRST_PULSE','AFTER_FIRST','REVERSE_PULSE','AFTER_REVERSE']
    summary={}
    for phase in phases:
        valid=[]; total=0
        for r in rows:
            if r.get('phase')!=phase or r.get('event')!='FRAME':continue
            total+=1
            try:
                valid.append((float(r['elapsed_s']),float(r['marker23_x']),float(r['marker23_y'])))
            except (TypeError,ValueError):pass
        if valid:
            result=(len(valid), total, round(median(x[0] for x in valid),3),
                    round(median(x[1] for x in valid),2),round(median(x[2] for x in valid),2))
            summary[phase]=result
            print(phase,'tracked/total, t, median_x, median_y:',result)
        elif total:
            print(phase,'tracked/total:',0,total)
    if not commands:
        print('RESULT: NO MOTOR RESPONSE MEASUREMENT (no logged pulse)')
    elif not all(k in summary and summary[k][0]>=3 for k in ('NEUTRAL_BEFORE','AFTER_FIRST')):
        print('RESULT: COMMAND(S) SENT, BUT MOTOR RESPONSE INCONCLUSIVE (insufficient before/after marker tracking)')
    else:
        a=summary['NEUTRAL_BEFORE']; b=summary['AFTER_FIRST']
        print('OBSERVED image-centre shift, before -> after LEFT -10:',
              'dx=',round(b[3]-a[3],2),'px, dy=',round(b[4]-a[4],2),'px',
              '(includes natural drift; NOT isolated motor effect)')
        if 'AFTER_REVERSE' in summary:
            c=summary['AFTER_REVERSE']
            print('OBSERVED shift after LEFT -10 -> after RIGHT +10:',
                  'dx=',round(c[3]-b[3],2),'px, dy=',round(c[4]-b[4],2),'px')
        print('RESULT: IMAGE-SPACE RESPONSE OBSERVED. REPEATED CONTROLLED FLIGHTS NEEDED TO VALIDATE CAUSAL MOTOR EFFECT.')
    return len(commands)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    m=p.add_mutually_exclusive_group(required=True)
    m.add_argument('--self-test',action='store_true')
    m.add_argument('--fly',action='store_true')
    m.add_argument('--analyze',metavar='DATASET_FOLDER_OR_CSV',help='offline motor response report')
    p.add_argument('--axis',choices=('right','forward'),default='right')
    p.add_argument('--allow-takeoff-overshoot',action='store_true')
    p.add_argument('--verified-recovery-area',action='store_true')
    args=p.parse_args()
    if args.self_test:
        self_test();return
    if args.analyze:
        analyze_csv(args.analyze);return
    if not args.allow_takeoff_overshoot or not args.verified_recovery_area:
        p.error('REFUSED BEFORE NETWORK: need --allow-takeoff-overshoot AND --verified-recovery-area; tabletop alone is NOT safe clearance')
    run_live(args.axis)


if __name__=='__main__':main()



