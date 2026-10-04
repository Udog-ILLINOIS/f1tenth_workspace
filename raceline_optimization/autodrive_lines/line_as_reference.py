#!/usr/bin/env python3
"""Turn a bare x,y line into a TUM reference-track file, so TUM can give it a speed profile.

  python3 line_as_reference.py <line.csv: x_m,y_m> <track.csv> <out_track.csv> [margin_m]

For each line point, the wall distances come from the nearest track.csv centerline point:
with off = signed offset of the point to the left of the centerline,
w_tr_left = w_left - off and w_tr_right = w_right + off (the real walls, not the line's).
Then: make_tum_lines.py <out_track.csv> <name> centerline
gives the line TUM's spline smoothing and velocity profile with zero offset, which is the
same treatment the data-collection centerline got. Used for the IL (F1-NN) line.

margin_m (optional): first move any point closer than margin_m to a wall back to margin_m,
along the centerline normal. 0.25 matches the TUM lines (width_opt 0.50 = 2 x 0.25).
"""
import sys

import numpy as np

line_csv, track_csv, out = sys.argv[1:4]
margin = float(sys.argv[4]) if len(sys.argv) > 4 else None
L = np.loadtxt(line_csv, delimiter=',', comments='#')[:, :2]
T = np.loadtxt(track_csv, delimiter=',', comments='#')
C, wr, wl = T[:, :2], T[:, 2], T[:, 3]
tan = np.roll(C, -1, axis=0) - np.roll(C, 1, axis=0)
tan /= np.linalg.norm(tan, axis=1, keepdims=True)
left = np.column_stack([-tan[:, 1], tan[:, 0]])
rows = []
moved = 0
for p in L:
    j = int(np.argmin(np.hypot(*(C - p).T)))
    off = float((p - C[j]) @ left[j])
    if margin is not None:
        clamped = min(max(off, -(wr[j] - margin)), wl[j] - margin)
        if clamped != off:
            p = p + (clamped - off) * left[j]
            off = clamped
            moved += 1
    rows.append((p[0], p[1], wr[j] + off, wl[j] - off))
R = np.array(rows)
if (R[:, 2:] <= 0).any():
    sys.exit(f'{int((R[:, 2:] <= 0).any(1).sum())} line points lie outside the track')
with open(out, 'w') as f:
    f.write('# x_m,y_m,w_tr_right_m,w_tr_left_m\n')
    for r in R:
        f.write(f'{r[0]:.4f},{r[1]:.4f},{r[2]:.4f},{r[3]:.4f}\n')
if margin is not None:
    print(f'margin {margin} m: moved {moved} of {len(R)} points')
print(f'{len(R)} pts, wall distance right {R[:, 2].min():.2f}-{R[:, 2].max():.2f} m, '
      f'left {R[:, 3].min():.2f}-{R[:, 3].max():.2f} m -> {out}')
