#!/usr/bin/env python3
"""Flag the points belonging to the person, so the viewer can pick them out.

    tag_person.py <viewer_dir>

Reads cloud.bin and frames.json written by bag_to_3d_t.py; writes flags.bin
(one uint8 per point, 1 = person) and person.json (the per-window track).

Two cheaper tests were tried on this recording and both failed, which is why
the shape test is the one here:

  by frequency   a sweep carries about 1320 points spread over 2800 occupied
                 cells, so even a wall is hit in a minority of sweeps. A
                 persistence threshold flagged noise the length of the
                 corridor.
  by time span   the person stood beside the robot for most of the 96 s while
                 the costmap captures were taken, so their cells are present
                 from end to end and read as structure.

What works is the test the robot ran live (amr_deploy/bin/detect_person.py):
in the 0.20-2.00 m band a person is a column 0.25-0.85 m across reaching
1.30-1.95 m, clear of the corridor walls. Clustering is connected components
on a 0.15 m grid rather than point to point, which keeps a million points
tractable.

The robot did not move during this recording, so the wall band is a fixed |y|.
A moving robot would need the background subtraction detect_person.py uses.
"""
import json
import sys

import cv2
import numpy as np

CELL = 0.15
Z_LO, Z_HI = 0.20, 2.00
WALL_Y = 0.70             # beyond this is the wall band for this corridor
MIN_W, MAX_W = 0.25, 0.85
MIN_TOP, MAX_TOP = 1.30, 1.95     # a standing adult seen from a 0.31 m sensor
MIN_EXT = 0.70
X_LO, X_HI = -4.5, 5.8    # the mapped stretch; outside it the walls splay and
                          # wall fragments were being read as columns
WIN = 8                   # sweeps per window, about 1 s -- as the live tracker
MIN_PTS = 10


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    v = sys.argv[1]

    A = np.fromfile(f"{v}/cloud.bin", np.float32).reshape(-1, 3)
    frames = json.load(open(f"{v}/frames.json"))["frames"]
    n = len(frames)
    offs = np.zeros(n + 1, np.int64)
    for i, f in enumerate(frames):
        offs[i + 1] = offs[i] + f["n"]

    flags = np.zeros(len(A), np.uint8)
    x0, y0 = A[:, 0].min() - 1, A[:, 1].min() - 1
    W = int((A[:, 0].max() - x0) / CELL) + 3
    H = int((A[:, 1].max() - y0) / CELL) + 3

    track = []
    for w0 in range(0, n, WIN):
        w1 = min(w0 + WIN, n)
        P = A[offs[w0]:offs[w1]]
        band = (P[:, 2] >= Z_LO) & (P[:, 2] <= Z_HI) & (np.abs(P[:, 1]) < WALL_Y)
        if band.sum() < MIN_PTS:
            continue
        Q = P[band]
        cx = ((Q[:, 0] - x0) / CELL).astype(np.int32)
        cy = ((Q[:, 1] - y0) / CELL).astype(np.int32)
        grid = np.zeros((H, W), np.uint8)
        grid[cy, cx] = 1
        ncc, lab = cv2.connectedComponents(grid, connectivity=8)
        pl = lab[cy, cx]
        keep = np.zeros(len(Q), bool)
        best = None
        for c in range(1, ncc):
            m = pl == c
            if m.sum() < MIN_PTS:
                continue
            C = Q[m]
            wdt = max(C[:, 0].ptp(), C[:, 1].ptp())
            top = C[:, 2].max()
            ext = top - C[:, 2].min()
            mx, my = C[:, 0].mean(), C[:, 1].mean()
            if (MIN_W <= wdt <= MAX_W and MIN_TOP <= top <= MAX_TOP
                    and ext >= MIN_EXT and X_LO <= mx <= X_HI and abs(my) < WALL_Y):
                keep |= m
                if best is None or m.sum() > best[0]:
                    best = (int(m.sum()), float(mx), float(my), float(top), float(wdt))
        flags[offs[w0] + np.where(band)[0][keep]] = 1
        if best:
            track.append((frames[w0]["t"], *best))

    windows = (n + WIN - 1) // WIN
    print(f"windows with a person-shaped column: {len(track)} of {windows}")
    print(f"tagged points: {flags.sum()} ({100 * flags.sum() / len(A):.2f}%)")
    print()
    print("  t(s)   pts       x       y    top   wide")
    for r in track[::4]:
        print(f"{r[0]:7.1f} {r[1]:5d} {r[2]:+7.2f} {r[3]:+7.2f} {r[4]:6.2f} {r[5]:6.2f}")

    flags.tofile(f"{v}/flags.bin")
    json.dump([{"t": r[0], "x": round(r[2], 3), "y": round(r[3], 3)} for r in track],
              open(f"{v}/person.json", "w"))
    print(f"\nwrote {v}/flags.bin and person.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
