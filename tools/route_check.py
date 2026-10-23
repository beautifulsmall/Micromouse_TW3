#!/usr/bin/env python3
"""Parse an 'M' map dump, flood to the goal, and report the route's turn
spacing against the R84 tangent constraint (2T = 197 mm)."""
import sys, re, math
from collections import deque

lines = open(sys.argv[1]).read().split("\n")
lines = [l for l in lines if l.strip()]
# Rows alternate: post row, cell row, post row, ...
N = (len(lines) - 1) // 2
# cell (x,y): x = column, y = row counted from the BOTTOM
# text row for cell y is lines[2*(N-1-y)+1]; post row above it is lines[2*(N-1-y)]

def cell_txt(y):
    return lines[2 * (N - 1 - y) + 1]

def post_txt_above(y):          # the wall row on the NORTH side of row y
    return lines[2 * (N - 1 - y)]

def wall_n(x, y):
    r = post_txt_above(y)
    i = 1 + 3 * x
    return r[i:i + 2] == "--"

def wall_e(x, y):
    r = cell_txt(y)
    i = 3 * x + 3
    return i < len(r) and r[i] == "|"

def wall_w(x, y):
    r = cell_txt(y)
    i = 3 * x
    return i < len(r) and r[i] == "|"

def wall_s(x, y):
    return wall_n(x, y - 1) if y > 0 else True

# find S and G
S = G = None
for y in range(N):
    r = cell_txt(y)
    for x in range(N):
        c = r[3 * x + 1:3 * x + 3].strip()
        if c == "S": S = (x, y)
        if c == "G": G = (x, y)
print(f"grid {N}x{N}  start {S}  goal {G}")

DIRS = {"N": (0, 1), "E": (1, 0), "S": (0, -1), "W": (-1, 0)}
def open_dir(x, y, d):
    return {"N": not wall_n(x, y), "E": not wall_e(x, y),
            "S": not wall_s(x, y), "W": not wall_w(x, y)}[d]

# BFS flood from goal
dist = {G: 0}
q = deque([G])
while q:
    cx, cy = q.popleft()
    for d, (dx, dy) in DIRS.items():
        if not open_dir(cx, cy, d): continue
        n = (cx + dx, cy + dy)
        if not (0 <= n[0] < N and 0 <= n[1] < N): continue
        if n in dist: continue
        dist[n] = dist[(cx, cy)] + 1
        q.append(n)

if S not in dist:
    print("NO ROUTE start -> goal"); sys.exit(1)
print(f"route length {dist[S]} cells")

# walk downhill from S
route = [S]
cur = S
while cur != G:
    best = None
    for d, (dx, dy) in DIRS.items():
        if not open_dir(cur[0], cur[1], d): continue
        n = (cur[0] + dx, cur[1] + dy)
        if n in dist and dist[n] == dist[cur] - 1:
            best = n; break
    route.append(best); cur = best

# headings along the route, and where they change
hdg = []
for i in range(len(route) - 1):
    dx = route[i + 1][0] - route[i][0]
    dy = route[i + 1][1] - route[i][1]
    hdg.append({(0, 1): "N", (1, 0): "E", (0, -1): "S", (-1, 0): "W"}[(dx, dy)])

CELL = 180.0
# ---- mirror of Route.h / Route.cpp -----------------------------------------
# Kept in step with the firmware ON PURPOSE: two independent implementations
# agreeing on the same maze is most of why the numbers are trusted. If these
# ever disagree with `route` on the mouse, one of them is lying about the
# geometry that will actually be flown.
RADII = [84, 73, 60, 45]        # ROUTE_RADII
MIN_STRAIGHT = 20.0             # ROUTE_MIN_STRAIGHT_MM
MIN_STRAIGHT_SAME = 0.0         # ROUTE_MIN_STRAIGHT_SAME_MM -- see Route.h
V, SLEW, A_LAT = 500.0, 6000.0, 4900.0
DEG = math.pi / 180.0

def clothoid_xy(L, t, w=1.0):
    w2, w3, t2 = w * w, w * w * w, t * t
    x = L * (w - t2 * w3 * w2 / 10.0 + t2 * t2 * w3 * w3 * w3 / 216.0)
    y = L * (t * w3 / 3.0 - t2 * t * w3 * w3 * w / 42.0 +
             t2 * t2 * t * w3 * w3 * w3 * w2 / 1320.0)
    return x, y

def tangent(R, v=V, slew=SLEW):
    """SpeedPath::tangent_for(R, clothoid_for_rail(R, v, slew), 90)."""
    L = v * v / (R * slew * DEG)
    t = L / (2.0 * R)
    xc, yc = clothoid_xy(L, t)
    m = yc - R * (1.0 - math.cos(t))
    x0 = xc - R * math.sin(t)
    return x0 + (R + m) / math.tan(math.pi / 4.0)

def rate(R, v=V):
    """Peak commanded turn rate, deg/s, at the speed the arc actually flies."""
    vv = min(v, math.sqrt(A_LAT * R))
    return vv / R / DEG

# ---- the U-turn, same closed form as Path.cpp ------------------------------
BODY_HALF, WALL_FACE, U_MARGIN = 35.0, 84.0, 12.0
# Overhang ahead of the tracked point. The setback is sized off the SWEPT
# ENVELOPE -- sqrt((R+half)^2 + front^2) -- not the centreline, which is the
# error that made runs 168-171 scrape the wall. Keep in step with Route.h.
BODY_HALF_LEN = 45.0   # measured: 90 mm long, axle central
U_LATE = 12.0          # the repeatable late start -- see Route.h

def uturn(offset=CELL, v=V, slew=SLEW):
    """Radius whose 180 offsets `offset` laterally, and how far it reaches."""
    def geom(R):
        L = v * v / (R * slew * DEG)
        t = L / (2.0 * R)
        x1, y1 = clothoid_xy(L, t)
        p = y1 - R * (1.0 - math.cos(t))
        return L, 2.0 * (R + p), x1 - R * math.sin(t) + R
    lo, hi = 20.0, 200.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if geom(mid)[1] < offset: lo = mid
        else: hi = mid
    R = 0.5 * (lo + hi)
    L, off, ext = geom(R)
    outer = math.hypot(R + BODY_HALF, BODY_HALF_LEN)
    return R, L, ext, outer - WALL_FACE + U_MARGIN + U_LATE

U_R, U_L, U_EXT, U_SET = uturn()

turns = []   # [cell index, from, to, sign, exit index, is_uturn]
for i in range(1, len(hdg)):
    if hdg[i] != hdg[i - 1]:
        left = {"N": "W", "W": "S", "S": "E", "E": "N"}[hdg[i - 1]] == hdg[i]
        turns.append([i, hdg[i - 1], hdg[i], +1 if left else -1, i, False])

# MERGE ADJACENT SAME-DIRECTION PAIRS. Left to right, non-overlapping -- a
# spiral of three merges the first pair only, which is right: a spiral is not
# two U-turns. Mirrors route_build_pass step 2b.
j = 0
while j + 1 < len(turns):
    if (turns[j][3] == turns[j + 1][3]
            and turns[j + 1][0] == turns[j][0] + 1):
        turns[j][4] = turns[j + 1][0]
        turns[j][5] = True
        del turns[j + 1]
    j += 1

print("\nroute:", " ".join(f"{c}" for c in route))
print("headings:", "".join(hdg))
print(f"\n{len(turns)} turns on the route")

# ---- the radius fit, same greedy demotion as Route.cpp ---------------------
rix = [0] * len(turns)
def T(j):
    # A U-turn reports its SETBACK, not a tangent -- it has no corner point.
    return U_SET if turns[j][5] else tangent(RADII[rix[j]])
def need_between(j):
    if turns[j][5] or turns[j - 1][5]:
        return MIN_STRAIGHT          # a U-turn is never "same direction"
    same = turns[j][3] == turns[j - 1][3]
    return MIN_STRAIGHT_SAME if same else MIN_STRAIGHT

ok = True
for _ in range(len(turns) * len(RADII) + 2):
    changed = False
    for j in range(len(turns)):
        if j == 0:
            room, need = turns[0][0] * CELL, T(0)
        else:
            room = (turns[j][0] - turns[j - 1][4]) * CELL
            need = T(j - 1) + T(j) + need_between(j)
        if need <= room:
            continue
        victim = j
        if j > 0 and T(j - 1) > T(j):
            victim = j - 1
        if turns[victim][5]:
            victim = j - 1 if (victim == j and j > 0) else j
        if turns[victim][5]:
            ok = False
            break
        if rix[victim] + 1 >= len(RADII):
            other = j - 1 if victim == j else j
            if other < 0 or rix[other] + 1 >= len(RADII):
                ok = False
                break
            victim = other
        rix[victim] += 1
        changed = True
        break
    if not changed or not ok:
        break
if len(turns):
    tail = (len(hdg) - turns[-1][4]) * CELL
    while ok and T(len(turns) - 1) > tail:
        if turns[-1][5] or rix[-1] + 1 >= len(RADII):
            ok = False
        else:
            rix[-1] += 1

if not ok:
    print("  NO RADIUS FITS -- the compiler would refuse this route")

print(f"  (U-turn geometry at v{V:.0f}: R {U_R:.1f}, clothoid {U_L:.1f}, "
      f"reaches {U_EXT:.1f} mm, entry setback {U_SET:.1f} mm)")
for j, (i, a, b, sgn, ex, isu) in enumerate(turns):
    R = U_R if isu else RADII[rix[j]]
    note = ""
    if j:
        gap = (i - turns[j - 1][4]) * CELL
        left = gap - T(j - 1) - T(j)
        note = f"   gap {gap:.0f} mm, straight {left:5.1f} mm"
    kind = "180 U-TURN" if isu else ("LEFT " if sgn > 0 else "RIGHT")
    cells = (f"{route[i]}->{route[ex]}" if isu else f"{route[i]}")
    print(f"  turn {j}: {cells}  {a}->{b}  R{R:<5.1f} {kind:<11}"
          f"{'  << dropped' if (not isu and rix[j]) else '':<14}{note}")

if turns:
    worst = max(rate(U_R if turns[j][5] else RADII[rix[j]])
                for j in range(len(turns)))
    print(f"\n  peak commanded turn rate on this route: {worst:.0f} deg/s")
    print(f"  ({sum(1 for j in range(len(turns)) if rix[j] and not turns[j][5])}"
          f" of {len(turns)} turns below R{RADII[0]}; "
          f"{sum(1 for t in turns if t[5])} U-turn(s))")
    capped = [j for j in range(len(turns))
              if math.sqrt(A_LAT * (U_R if turns[j][5] else RADII[rix[j]])) < V]
    if capped:
        print(f"  turns whose SPEED is capped by the lateral limit (the "
              f"deceleration has to fit in the approach): {capped}")
