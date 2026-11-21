"""Rasterize the static map into a 2 m grid (run once, at image build).
Per cell: ground height, and the base/top of the solid above it.
Output: grid.bin (little-endian float32 arrays) + grid.json (header)."""
import json, struct, sys, array, os, time

STEP = 2.0
src = sys.argv[1] if len(sys.argv) > 1 else "arena"
out = sys.argv[2] if len(sys.argv) > 2 else "."
w = json.load(open(os.path.join(src, "v2/world.json")))
T = w["terrain"]
tnx, tny = T["nx"], T["ny"]
ter = array.array("h")
ter.frombytes(open(os.path.join(src, "v2/terrain.i16"), "rb").read())
if sys.byteorder != "little":
    ter.byteswap()

def ground(x, y):
    fx = (x - T["minX"]) / T["step"]; fy = (y - T["minY"]) / T["step"]
    i = min(max(int(fx), 0), tnx - 2); j = min(max(int(fy), 0), tny - 2)
    ax = min(max(fx - i, 0.0), 1.0); ay = min(max(fy - j, 0.0), 1.0)
    a = ter[j * tnx + i]; b = ter[j * tnx + i + 1]; c = ter[(j + 1) * tnx + i]; d = ter[(j + 1) * tnx + i + 1]
    return ((a * (1 - ax) + b * ax) * (1 - ay) + (c * (1 - ax) + d * ax) * ay) * 0.01

B = w["bounds"]
minx, miny = B["minX"], B["minY"]
nx = int((B["maxX"] - minx) / STEP) + 1
ny = int((B["maxY"] - miny) / STEP) + 1
N = nx * ny
G = array.array("f", [0.0]) * N
BASE = array.array("f", [1e9]) * N
TOP = array.array("f", [-1e9]) * N
for j in range(ny):
    y = miny + (j + 0.5) * STEP
    for i in range(nx):
        G[j * nx + i] = ground(minx + (i + 0.5) * STEP, y)

def pip(px, py, ring):
    inside = False
    n = len(ring) // 2
    jx, jy = ring[-2], ring[-1]
    for k in range(n):
        ix, iy = ring[2 * k], ring[2 * k + 1]
        if (iy > py) != (jy > py) and px < (jx - ix) * (py - iy) / (jy - iy) + ix:
            inside = not inside
        jx, jy = ix, iy
    return inside

t0 = time.time()
for p in w["prisms"]:
    ring = p["ring"]; holes = p.get("holes") or []
    if holes and isinstance(holes[0], (int, float)):
        holes = [holes]
    xs, ys = ring[0::2], ring[1::2]
    i0 = max(int((min(xs) - minx) / STEP), 0); i1 = min(int((max(xs) - minx) / STEP) + 1, nx - 1)
    j0 = max(int((min(ys) - miny) / STEP), 0); j1 = min(int((max(ys) - miny) / STEP) + 1, ny - 1)
    for j in range(j0, j1 + 1):
        y = miny + (j + 0.5) * STEP
        for i in range(i0, i1 + 1):
            x = minx + (i + 0.5) * STEP
            if pip(x, y, ring) and not any(pip(x, y, h) for h in holes if len(h) >= 6):
                k = j * nx + i
                # keep the solid that matters most for walking: the lowest-based one,
                # and the highest top for sight lines
                if p["base"] < BASE[k]:
                    BASE[k] = p["base"]
                if p["top"] > TOP[k]:
                    TOP[k] = p["top"]
print("prisms rasterized in %.1fs" % (time.time() - t0), file=sys.stderr)
with open(os.path.join(out, "grid.bin"), "wb") as f:
    for a in (G, BASE, TOP):
        if sys.byteorder != "little":
            a = array.array("f", a); a.byteswap()
        f.write(a.tobytes())
json.dump({"step": STEP, "minx": minx, "miny": miny, "nx": nx, "ny": ny}, open(os.path.join(out, "grid.json"), "w"))
print("grid", nx, ny, file=sys.stderr)
