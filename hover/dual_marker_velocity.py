import math
def estimate(samples):
    if len(samples) < 5: return None
    ts = [float(t) for t, _ in samples]
    ps = [tuple(map(float, p)) for _, p in samples]
    if any(not math.isfinite(z) for z in ts + [v for p in ps for v in p]): return None
    if any(b <= a or b-a > 0.30 for a,b in zip(ts,ts[1:])): return None
    if not 0.4 <= ts[-1]-ts[0] <= 2.0: return None
    mt = sum(ts)/len(ts)
    den = sum((t-mt)**2 for t in ts)
    avg = [sum(p[i] for p in ps)/len(ps) for i in (0,1)]
    v = [sum((t-mt)*(p[i]-avg[i]) for t,p in zip(ts,ps))/den for i in (0,1)]
    residual = max(math.hypot(*(p[i]-(avg[i]+v[i]*(t-mt)) for i in (0,1))) for t,p in zip(ts,ps))
    return tuple(round(x,5) for x in (*v,residual))
