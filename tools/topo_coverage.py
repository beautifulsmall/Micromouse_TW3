#!/usr/bin/env python3
"""Turn-topology coverage of a maze, from an 'M' map dump.

    python3 tools/topo_coverage.py 150_map.txt [--size 6]

WHY. bert, 2026-08-22: a test maze "is still just a subset of wall topologies
that can affect trajectory". This turns that from a worry into a checklist.

What decides a turn's sensor signature, and therefore whether the gate can
anchor it, is the LOCAL wall pattern, not the maze:

    track wall  is there a wall on the inside of the APPROACH cell? That is
                the wall the gate follows, and its end -- or the bare post
                where it would have ended -- is the landmark.
    outside     wall on the far side of the turn cell (the one you would hit
                going wide).
    ahead       wall straight on in the turn cell.

Two turn directions x three booleans = 16 topologies. This counts how many of
them a maze actually contains and names an example cell for each, so a
gathering session can be aimed at what is missing rather than repeating what
is already covered.
"""
import sys
from collections import Counter

DIRS = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}
LEFT = {'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}
RIGHT = {'N': 'E', 'E': 'S', 'S': 'W', 'W': 'N'}
BACK = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}


def parse(lines, n):
    """An 'M' dump is 2n+1 text rows; cell row y is 2*(n-y) from the top."""
    rows = [r for r in lines if r.strip()]
    top = len(rows) - (2 * n + 1)
    if top < 0:
        raise SystemExit(f"map too short for size {n}")
    rows = rows[top:]
    cell = {y: rows[2 * (n - y) - 1] for y in range(n)}
    hbar = {y: rows[2 * (n - y)] for y in range(n + 1)}   # hbar[y] is BELOW y
    W = {}
    for y in range(n):
        for x in range(n):
            W[(x, y)] = {
                'W': cell[y][3 * x] == '|',
                'E': cell[y][3 * (x + 1)] == '|',
                'S': hbar[y][3 * x + 1] == '-',
                'N': hbar[y + 1][3 * x + 1] == '-',
            }
    return W


def survey(W, n):
    topo, example = Counter(), {}
    for y in range(n):
        for x in range(n):
            for come in 'NESW':
                px, py = x - DIRS[come][0], y - DIRS[come][1]
                if not (0 <= px < n and 0 <= py < n):
                    continue
                if W[(px, py)][come]:
                    continue                       # cannot leave the approach cell
                for turn, side in ((LEFT[come], 'L'), (RIGHT[come], 'R')):
                    if W[(x, y)][turn]:
                        continue                   # cannot exit that way
                    key = (side, W[(px, py)][turn], W[(x, y)][BACK[turn]],
                           W[(x, y)][come])
                    topo[key] += 1
                    example.setdefault(key, (x, y, come))
    return topo, example


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    n = 6
    if '--size' in sys.argv:
        n = int(sys.argv[sys.argv.index('--size') + 1])
    if not args:
        raise SystemExit(__doc__)
    W = parse(open(args[0]).read().split('\n'), n)
    topo, example = survey(W, n)
    print(f"turn opportunities: {sum(topo.values())}")
    print(f"{'dir':>4}{'track':>11}{'outside':>9}{'ahead':>7}{'n':>5}   example")
    for k in sorted(topo, key=lambda k: (-topo[k], k)):
        s, ib, o, a = k
        print(f"{s:>4}{'wall' if ib else 'POST':>11}{'yes' if o else 'no':>9}"
              f"{'yes' if a else 'no':>7}{topo[k]:>5}   "
              f"cell {example[k][:2]} heading {example[k][2]}")
    missing = [k for s in 'LR' for ib in (True, False) for o in (True, False)
               for a in (True, False) if (k := (s, ib, o, a)) not in topo]
    print(f"\n{len(topo)} of 16 present; MISSING {len(missing)}:")
    for s, ib, o, a in missing:
        print(f"   {s}  track={'wall' if ib else 'POST':>4}  "
              f"outside={'yes' if o else 'no':>3}  ahead={'yes' if a else 'no':>3}")


if __name__ == '__main__':
    main()
