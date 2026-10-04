#!/usr/bin/env python3
"""Diagnose how well a logged run tracked the active raceline.

  python3 analyze_run.py <run_dir> [--map MAP] [--scale S]

Map and speed scaling default to the run's metadata.json / summary.json (track,
speed_scaling), else to the folder name <map>_<line>_x<speed>_<YYYYmmdd_HHMMSS>.

Reports lateral error to the raceline, the actual vs. commanded (kinematic bicycle) yaw
rate, i.e. whether the tyres hold or the car slides/lags, and achieved vs demanded
lateral acceleration. Runs in the container (numpy).
"""
import argparse
import csv
import json
import os
import re

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('run')
ap.add_argument('--map', default=None, help='default: the run\'s track, else autodrive_roboracer')
ap.add_argument('--scale', type=float, default=None, help='speed scaling used (default: from the run)')
ap.add_argument('--wheelbase', type=float, default=0.33)
a = ap.parse_args()

info = {}
for fn in ('summary.json', 'metadata.json'):
    try:
        info.update(json.load(open(os.path.join(a.run, fn))))
    except (OSError, json.JSONDecodeError):
        pass
if a.map is None:
    a.map = info.get('track') or info.get('map') or 'autodrive_roboracer'

T = np.array([[float(v) for v in r] for r in list(csv.reader(open(os.path.join(a.run, 'telemetry.csv'))))[1:]])
t, lap, x, y, yaw, v, vc, dc, wall = T.T
wp = os.path.expanduser(f'~/ws/src/race_stack/stack_master/maps/{a.map}/global_waypoints.json')
W = json.load(open(wp))['global_traj_wpnts_iqp']['wpnts']
P = np.array([[w['x_m'], w['y_m']] for w in W])
scale = a.scale
if scale is None:
    m = re.search(r'_x([0-9.]+)(?:_\d{8}_\d{6})?$', a.run.rstrip('/'))
    scale = info.get('speed_scaling') or (float(m.group(1)) if m else 1.0)

err = np.array([np.min(np.hypot(P[:, 0] - px, P[:, 1] - py)) for px, py in zip(x, y)])
m = (lap >= 1) & (v > 0.5)
print(f'lateral error to raceline: mean {err[m].mean():.3f} m, p95 {np.percentile(err[m], 95):.3f}, '
      f'max {err[m].max():.3f}')

dt = np.diff(t)
dyaw = np.angle(np.exp(1j * np.diff(yaw)))
ok = (dt > 0.02) & (dt < 0.2) & (v[1:] > 1.0) & (np.abs(dyaw) > 1e-4)
r_act = dyaw[ok] / dt[ok]
r_kin = v[1:][ok] * np.tan(dc[1:][ok]) / a.wheelbase
vv = v[1:][ok]
k = np.linalg.lstsq(r_kin[:, None], r_act, rcond=None)[0][0]
print(f'actual / commanded yaw rate: {k:.2f}  (1.0 = tyres hold; < 1 = understeer, slip or lag)')
for lo, hi in ((1, 2), (2, 3), (3, 6)):
    sel = (vv >= lo) & (vv < hi)
    if sel.sum() > 10:
        kk = np.linalg.lstsq(r_kin[sel][:, None], r_act[sel], rcond=None)[0][0]
        print(f'  {lo}-{hi} m/s: {kk:.2f}  (n={sel.sum()})')
ay = np.abs(r_act * vv)
print(f'lateral accel achieved: p95 {np.percentile(ay, 95):.1f}, max {ay.max():.1f} m/s^2')
k_rl = np.array([w['kappa_radpm'] for w in W])
v_rl = np.array([w['vx_mps'] for w in W]) * scale
print(f'raceline demand at {scale}x: max lateral accel {np.max(np.abs(k_rl) * v_rl ** 2):.1f} m/s^2')
