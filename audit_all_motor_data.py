"""Offline-only V1-V6 CSV evidence audit, plus V7 result assessment.

  python -B audit_all_motor_data.py --root captures

Inspects existing CSV only. No drone/network imports; never sends any motor commands.
"""
import argparse
import csv
from pathlib import Path
from collections import Counter
from statistics import median
import math


def num(x):
    try:return float(x)
    except (ValueError,TypeError):return None


def describe_paired(rows):
    phases=('no_rc_1','no_rc_2','after_positive','after_negative')
    result={}
    for ph in phases:
        a=[r for r in rows if r.get('phase')==ph and num(r.get('marker_x_px')) is not None and num(r.get('marker_y_px')) is not None]
        if a:
            result[ph]=(len(a),round(median(num(r['seconds']) for r in a),3),
                        round(median(num(r['marker_x_px']) for r in a),2),
                        round(median(num(r['marker_y_px']) for r in a),2))
    return result


def streaming_phase_summary(rows):
    phases=('NEUTRAL_1','NEUTRAL_2','NEUTRAL_BEFORE','POSITIVE_PULSE',
            'AFTER_POSITIVE','NEGATIVE_PULSE','AFTER_NEGATIVE')
    out={}
    for ph in phases:
        items=[r for r in rows if r.get('phase')==ph and r.get('event')=='FRAME'
               and num(r.get('marker23_x')) is not None and num(r.get('marker23_y')) is not None]
        if items:
            out[ph]=(len(items),round(median(num(r['elapsed_s']) for r in items),3),
                     round(median(num(r['marker23_x']) for r in items),2),
                     round(median(num(r['marker23_y']) for r in items),2))
    return out


def inventory(csv_root):
    rows_out=[]
    for p in sorted(csv_root.rglob('*.csv')):
        try:
            with p.open(encoding='utf-8-sig',newline='') as f:
                r=list(csv.DictReader(f))
        except (UnicodeError,OSError,csv.Error):continue
        if not r:continue
        cols=r[0].keys()
        relevant=('marker23_x' in cols or 'marker_x_px' in cols or p.name=='measurements.csv')
        if not relevant:continue
        ev=Counter((x.get('event') or x.get('phase') or '') for x in r)
        marker23=sum(num(x.get('marker23_x',x.get('marker_x_px'))) is not None for x in r)
        marker25=sum(num(x.get('marker25_x')) is not None for x in r)
        pulse_commands=ev['PULSE_COMMAND']
        pulse_starts=ev['PULSE_START']
        pulse_acks=ev['PULSE_STOP']
        stops=[(x.get('event') or x.get('phase'),x.get('details') or x.get('event_detail') or x.get('detail') or '') for x in r if (x.get('event') or x.get('phase')) in ('STOP','REVERSE_SKIPPED','LANDING_ERROR')]
        rows_out.append((str(p.parent),len(r),marker23,marker25,pulse_commands,pulse_starts,pulse_acks,ev['LANDING_ACK'],stops,describe_paired(r) if p.name=='measurements.csv' else None))
    return rows_out


def main():
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument('--root',default='captures')
    args=a.parse_args()
    base=Path(args.root)
    if not base.is_dir():a.error(f'Cannot find {base}')
    data=inventory(base)
    if not data:a.error('No applicable capture CSV files found')
    print('=== MOTOR-RESPONSE EVIDENCE AUDIT (OFFLINE ONLY) ===')
    for folder,n,m23,m25,pulse_cmd,starts,stops,land,stop_reasons,pairs in data:
        print('\n',folder,f'rows={n} M23={m23} M25={m25} pulse_commands={pulse_cmd} pulse_starts={starts} pulse_stops={stops} landing_acks={land}')
        if stop_reasons:
            for event,reason in stop_reasons:print(' ',event,reason[:135])
        if not pairs and (pulse_cmd or starts):
            real_rows=list(csv.DictReader(open(Path(folder)/('measurements.csv' if 'paired_axis' in folder else 'observations.csv'),encoding='utf-8-sig')))
            for phase,rec in streaming_phase_summary(real_rows).items():
                print(' ',phase,'valid,time_s,median_x,median_y:',rec)
        if pairs:
            for phase,rec in pairs.items():print(' ',phase, 'valid,time_s,median_x,median_y:',rec)
            if 'no_rc_1' in pairs and 'no_rc_2' in pairs and 'after_positive' in pairs:
                _,t1,x1,y1=pairs['no_rc_1']; _,t2,x2,y2=pairs['no_rc_2'];_,t3,x3,y3=pairs['after_positive']
                if t2>t1:
                    pred=(x2+(x2-x1)*(t3-t2)/(t2-t1), y2+(y2-y1)*(t3-t2)/(t2-t1))
                    print('  +pulse apparent difference from projected early no-RC trend:',
                          (round(x3-pred[0],2),round(y3-pred[1],2)),
                          'PIXELS; unreliable if drift not constant')
    has_rc=sum(x[4] for x in data)
    paired=sum(x[5] for x in data if 'paired_axis' in x[0])
    print('\nCONCLUSION:',has_rc,'individually logged streaming nonzero RC send(s);',paired,
          'paired +/− pulse start(s) (each pair script intended 4 sends per pulse).')
    print('No CSV establishes physical motion ground truth or a validated motor-axis model.')
    print('If no image-space sign reversal reproducible across separately collected trials: INCONCLUSIVE.')
    print('No drone, network, or motor access.')

if __name__=='__main__':main()
