#!/usr/bin/env python3
"""Find mirror relationships between the four wheel meshes."""
import struct

M = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_description/meshes/"
names = ["wheel_lf_link", "wheel_lb_link", "wheel_rf_link", "wheel_rb_link"]


def verts(name):
    d = open(M + name + ".STL", "rb").read()
    n = struct.unpack("<I", d[80:84])[0]
    vs = set()
    off = 84
    for _ in range(n):
        v = struct.unpack("<12f", d[off:off + 48])
        off += 50
        for k in range(3):
            vs.add((round(v[3 + 3 * k], 4),
                    round(v[4 + 3 * k], 4),
                    round(v[5 + 3 * k], 4)))
    return vs


def transform(vs, sx, sy, sz):
    return {(round(sx * x, 4), round(sy * y, 4), round(sz * z, 4))
            for x, y, z in vs}


shapes = {n: verts(n) for n in names}
for n, s in shapes.items():
    xs = [v[0] for v in s]
    zs = [v[2] for v in s]
    print("%s verts=%d x[%.3f,%.3f] z[%.3f,%.3f]" % (
        n, len(s), min(xs), max(xs), min(zs), max(zs)))

print("\nrelationships (A == B under mirror):")
out = {}
for a in names:
    for b in names:
        if a == b:
            continue
        for sx, sz in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
            t = transform(shapes[a], sx, sz, 1)
            if t == shapes[b]:
                key = (a, b, sx, sz)
                print("  %s == %s mirrored x=%+d z=%+d" % (a, b, sx, sz))
                out.setdefault(a, []).append((b, sx, sz))
                break
