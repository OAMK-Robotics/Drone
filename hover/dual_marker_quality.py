import math
from .dual_marker_velocity import estimate

def classify(samples, threshold=3.0):
    result = estimate(samples)
    if result is None: return "INSUFFICIENT", None
    dt = float(samples[-1][0]) - float(samples[0][0])
    signal = math.hypot(result[0], result[1]) * dt
    residual = result[2]
    ratio = signal / max(residual, 1e-8)
    return ("MOTION_CANDIDATE" if ratio >= threshold else "UNCERTAIN"), round(ratio, 2)
