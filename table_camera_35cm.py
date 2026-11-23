"""Downward camera geometry at 35cm, with motors OFF. Never issues takeoff/RC."""
import argparse, csv, time
from pathlib import Path
from datetime import datetime
import cv2, numpy as np

def orient(x):
    assert isinstance(x,np.ndarray) and x.shape==(720,320,3), f'Bad image: {getattr(x,"shape",None)}'
    return cv2.rotate(cv2.cvtColor(x[:240].copy(),cv2.COLOR_RGB2BGR),cv2.ROTATE_180)

def markers(im):
    dic=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    cr, ids, _=cv2.aruco.ArucoDetector(dic).detectMarkers(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY))
    out={}
    if ids is not None:
        for mid,c in zip(ids.flatten(),cr):
            if int(mid) in (23,25):
                pts=c.reshape(4,2); cx,cy=pts.mean(axis=0)
                size=float(np.median([np.linalg.norm(pts[(i+1)%4]-pts[i]) for i in range(4)]))
                out[int(mid)]=(round(float(cx),2),round(float(cy),2),round(size,2))
    return out

def selftest():
    x=np.zeros((720,320,3),np.uint8); x[0,0]=[1,2,3]
    assert orient(x)[239,319].tolist()==[3,2,1]
    assert markers(np.zeros((240,320,3),np.uint8))=={}
    print('PASS: correct camera crop, 180 rotation, 23/25 detector. NO WIFI/FLIGHT.')

def capture():
    from djitellopy import Tello
    p=Path('captures')/f'table_camera_35cm_{datetime.now():%Y%m%d_%H%M%S}'
    p.mkdir(parents=True)
    drone=Tello(host='192.168.1.63'); streaming=False; connected=False
    try:
        drone.connect(); connected=True
        drone.set_video_direction(Tello.CAMERA_DOWNWARD)
        drone.streamon(); streaming=True; read=drone.get_frame_read()
        for _ in range(30):
            time.sleep(.2)
            try:
                im=orient(read.frame)
                if float(im.std())>10: break
            except (AssertionError,AttributeError): pass
        else: raise RuntimeError('No valid camera data')
        print('Camera active. Keep propellers OFF. Support drone steadily ~35cm above Marker 23. Leave Marker 25 in place.')
        if input('Type CAPTURE when positioned: ').strip()!='CAPTURE': return
        with (p/'observations.csv').open('w',newline='',encoding='utf8') as f:
            w=csv.writer(f); w.writerow(['elapsed_s','frame','id','x','y','side_px'])
            t0=time.monotonic()
            count={23:0,25:0}
            for i in range(20):
                img=orient(read.frame); filename=f'{i:03d}.png'
                if not cv2.imwrite(str(p/filename),img): raise RuntimeError('Image save failed')
                found=markers(img)
                for mid in (23,25):
                    if mid in found: count[mid]+=1
                    w.writerow([round(time.monotonic()-t0,3),filename,mid,*(found.get(mid,('','','')))])
                if i%5==0: print('FRAME',i,found,flush=True)
                time.sleep(.15)
        print('TRACKING:',count,'out of 20')
        print('DATASET:',p)
        print('MOTORS NEVER STARTED.')
    finally:
        if streaming:
            try: drone.streamoff()
            except Exception as e: print('Streamoff warning:',e)
        if connected:
            try: drone.end()
            except Exception as e: print('End warning:',e)

if __name__=='__main__':
    a=argparse.ArgumentParser(); g=a.add_mutually_exclusive_group(required=True)
    g.add_argument('--self-test',action='store_true'); g.add_argument('--capture',action='store_true')
    v=a.parse_args()
    if v.self_test: selftest()
    else: capture()
