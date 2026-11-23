def camera_relative_pose(found, image_center=(160.0, 120.0)):
    if 23 not in found or 25 not in found: return None
    ax, ay = found[23]["center"]
    bx, by = found[25]["center"]
    dx, dy = bx - ax, by - ay
    d2 = dx*dx + dy*dy
    if d2 < 100: return None
    ux = image_center[0] - (ax+bx)/2
    uy = image_center[1] - (ay+by)/2
    return ((ux*dx+uy*dy)/d2, (uy*dx-ux*dy)/d2)
