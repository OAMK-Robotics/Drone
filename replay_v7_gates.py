"""Retrospective camera/height gate probe; not a simulated drone flight.

  python -B replay_v7_gates.py --root captures

Reads V5 CSV snapshots. The actual V5 paths descend, but V7 would NOT:
this is a historical visibility-only check, not a forecast of V7 success.
"""
import argparse
import csv
from pathlib import Path
from motor_response_v7 import point_gate,height_gate

def run(root):
    paths=sorted(Path(root).glob('streaming_v5_marker25_right_*/observations.csv'))
    for p in paths:
        with p.open(encoding='utf-8-sig',newline='') as f:rows=list(csv.DictReader(f))
        ack=next((float(r['elapsed_s']) for r in rows if r['event']=='TAKEOFF_ACK'),None)
        if ack is None:continue
        samples=[r for r in rows if r['event']=='FRAME']
        found=None
        for k in range(10):
            lag=.65+k*.05
            now=ack+lag
            observed=[];heights=[]
            for r in samples:
                t=float(r['elapsed_s'])
                if t>now:break
                try:pt=(float(r['marker23_x']),float(r['marker23_y']))
                except (ValueError,TypeError):pt=None
                try:h=float(r['sdk_height_cm'])
                except (ValueError,TypeError):h=None
                observed.append((t,pt));heights.append((t,h))
            m,reason=point_gate(observed,now)
            h,hr=height_gate(heights,now)
            if m and h:
                found=(lag,reason,hr);break
        print(p.parent.name,':',
              ('HISTORICAL GATES PASS at +%.2fs; %s; %s' % found) if found else 'NO HISTORICAL GATE MATCH in 0.65-1.10s')
    print('NO DRONE COMMANDS. V7 omits the real V5 descent; this is NOT a flight success prediction.')
if __name__=='__main__':
    a=argparse.ArgumentParser(description=__doc__);a.add_argument('--root',default='captures')
    run(a.parse_args().root)
