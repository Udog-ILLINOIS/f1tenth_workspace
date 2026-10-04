#!/usr/bin/env python3
"""Swap custom global racelines into ForzaETH for an AutoDRIVE map.

ForzaETH drives whatever is in stack_master/maps/<map>/global_waypoints.json
(`global_traj_wpnts_iqp`). This tool keeps a library of lines per map in
racelines/<map>/<map>_<name>.json (full global_waypoints.json files; F1Tenth/racelines on
the Mac, ~/ws/racelines in the container) and copies the chosen one into place. Commands
take the short <name> (e.g. centerline); a <map>_ prefix is accepted too. ACTIVE holds the
short name.

  export <map>              write racelines/<map>/<map>_track.csv (x_m,y_m,w_tr_right_m,w_tr_left_m;
                            TUM centerline input, raw widths) to feed your own optimizer, and
                            save ForzaETH's own line as 'forza_iqp' (and 'forza_sp')
  import <map> <csv> <name> import a TUM-format raceline (traj_race_cl.csv:
                            s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2, psi 0 = north)
                            or a plain x,y[,vx] CSV (speed then needs --v-const)
  use <map> <name>          make <name> the line ForzaETH drives (restart base to apply)
  list <map>                show the library and which line is active

Coordinates must be in the map frame, i.e. AutoDRIVE's world frame. A line computed
from track.csv already is.
"""
import argparse
import copy
import json
import math
import os
import shutil
import sys

import numpy as np

WS = os.path.expanduser('~/ws')
MAPS = os.path.join(WS, 'src/race_stack/stack_master/maps')
LIB = os.path.join(WS, 'racelines')   # mounted from F1Tenth/racelines


def lib_dir(map_name):
    d = os.path.join(LIB, map_name)
    os.makedirs(d, exist_ok=True)
    return d


def short_name(map_name, name):
    """'<map>_centerline' or 'centerline' (or a .json/.csv file name) -> 'centerline'."""
    name = os.path.basename(name)
    for ext in ('.json', '.csv'):
        if name.endswith(ext):
            name = name[:-len(ext)]
    prefix = map_name + '_'
    return name[len(prefix):] if name.startswith(prefix) else name


def line_path(map_name, name, ext='.json'):
    """Library file for a line: racelines/<map>/<map>_<name><ext>."""
    return os.path.join(lib_dir(map_name), f'{map_name}_{short_name(map_name, name)}{ext}')


def active_path(map_name):
    return os.path.join(MAPS, map_name, 'global_waypoints.json')


def load(path):
    with open(path) as f:
        return json.load(f)


def save(d, path):
    with open(path, 'w') as f:
        json.dump(d, f)


def ensure_library(map_name):
    """First touch: keep the planner's original file as 'forza_iqp'."""
    base = line_path(map_name, 'forza_iqp')
    if not os.path.exists(base):
        shutil.copy(active_path(map_name), base)
        d = load(base)
        sp = copy.deepcopy(d)
        sp['global_traj_wpnts_iqp'] = d['global_traj_wpnts_sp']
        sp['global_traj_markers_iqp'] = d['global_traj_markers_sp']
        save(sp, line_path(map_name, 'forza_sp'))
    return base


def cmd_export(a):
    ensure_library(a.map)
    d = load(active_path(a.map))
    c = d['centerline_waypoints']['wpnts']
    out = line_path(a.map, 'track', '.csv')
    with open(out, 'w') as f:
        f.write('# x_m,y_m,w_tr_right_m,w_tr_left_m\n')
        for w in c:
            f.write(f"{w['x_m']:.4f},{w['y_m']:.4f},{w['d_right']:.4f},{w['d_left']:.4f}\n")
    widths = np.array([[w['d_right'], w['d_left']] for w in c])
    print(f'Wrote {out}: {len(c)} points, track width {widths.sum(1).min():.2f}-{widths.sum(1).max():.2f} m '
          '(raw, no safety margin; same direction as the map start pose)')
    print("Library now has 'forza_iqp' and 'forza_sp' (ForzaETH's own min-curvature and shortest-path lines).")


def read_line(path, v_const):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            rows.append([float(v) for v in line.replace(';', ',').split(',') if v.strip()])
    arr = np.array(rows)
    if arr.shape[1] >= 7:   # TUM traj_race_cl.csv
        return arr[:, 1], arr[:, 2], arr[:, 5], arr[:, 6]
    x, y = arr[:, 0], arr[:, 1]
    if arr.shape[1] >= 3:
        vx = arr[:, 2]
    elif v_const:
        vx = np.full(len(x), v_const)
    else:
        sys.exit('Plain x,y CSV has no speed column; pass --v-const <m/s>.')
    return x, y, vx, None


def cmd_import(a):
    ensure_library(a.map)
    a.name = short_name(a.map, a.name)
    base = load(active_path(a.map))
    x, y, vx, ax = read_line(a.csv, a.v_const)
    if np.hypot(x[0] - x[-1], y[0] - y[-1]) < 1e-3:   # closed duplicate endpoint
        x, y, vx = x[:-1], y[:-1], vx[:-1]
        ax = ax[:-1] if ax is not None else None
    n = len(x)
    xy = np.column_stack([x, y])
    seg = np.linalg.norm(np.roll(xy, -1, axis=0) - xy, axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)[:-1]])
    # heading (ROS: 0 = +x) and curvature by central differences on the closed loop
    dx = (np.roll(x, -1) - np.roll(x, 1)) / 2
    dy = (np.roll(y, -1) - np.roll(y, 1)) / 2
    psi = np.arctan2(dy, dx)
    dpsi = np.angle(np.exp(1j * (np.roll(psi, -1) - np.roll(psi, 1))))
    kappa = dpsi / (np.roll(seg, 1) + seg)
    if ax is None:
        ax = (np.roll(vx, -1) ** 2 - vx ** 2) / (2 * np.maximum(seg, 1e-3))

    # direction check against the map's centerline (ForzaETH's driving direction)
    cent = np.array([[w['x_m'], w['y_m']] for w in base['centerline_waypoints']['wpnts']])
    widths = np.array([[w['d_right'], w['d_left']] for w in base['centerline_waypoints']['wpnts']])
    cdir = np.roll(cent, -1, axis=0) - cent
    i0 = int(np.argmin(np.linalg.norm(cent - xy[0], axis=1)))
    if np.dot(cdir[i0], xy[1] - xy[0]) < 0:
        sys.exit('Line runs against the track direction; reverse it before importing.')

    # distances to the track edges via the nearest centerline point and its left normal
    d_right = np.empty(n)
    d_left = np.empty(n)
    for i, p in enumerate(xy):
        j = int(np.argmin(np.linalg.norm(cent - p, axis=1)))
        t = cdir[j] / max(np.linalg.norm(cdir[j]), 1e-9)
        off = float(np.dot(p - cent[j], np.array([-t[1], t[0]])))   # + = left of centerline
        d_left[i] = widths[j, 1] - off
        d_right[i] = widths[j, 0] + off
    if (d_left < 0).any() or (d_right < 0).any():
        print(f'WARNING: {int(((d_left < 0) | (d_right < 0)).sum())} points lie outside the mapped track.')

    wpnts = [{'id': i, 's_m': float(s[i]), 'd_m': 0.0, 'x_m': float(x[i]), 'y_m': float(y[i]),
              'd_right': float(d_right[i]), 'd_left': float(d_left[i]), 'psi_rad': float(psi[i]),
              'kappa_radpm': float(kappa[i]), 'vx_mps': float(vx[i]), 'ax_mps2': float(ax[i])}
             for i in range(n)]
    # ForzaETH's lines repeat the first point at s = lap length to close the loop
    close = dict(wpnts[0], id=n, s_m=float(s[-1] + seg[-1]))
    wpnts.append(close)
    vmax = float(vx.max())
    template = base['global_traj_markers_iqp']['markers'][0]
    markers = []
    for w in wpnts:
        m = copy.deepcopy(template)
        m['id'] = w['id']
        m['scale']['z'] = w['vx_mps'] / vmax
        m['pose']['position'] = {'x': w['x_m'], 'y': w['y_m'], 'z': w['vx_mps'] / vmax / 2}
        markers.append(m)

    out = copy.deepcopy(base)
    out['global_traj_wpnts_iqp'] = {'header': base['global_traj_wpnts_iqp']['header'], 'wpnts': wpnts}
    out['global_traj_markers_iqp'] = {'markers': markers}
    lap_t = float(np.sum(seg / np.maximum(vx, 0.1)))
    out['map_info_str'] = {'data': f'Imported raceline {a.name} from {os.path.basename(a.csv)}; '
                                   f'estimated lap time: {lap_t:.3f}s; maximum speed: {vmax:.3f}m/s; '}
    path = line_path(a.map, a.name)
    save(out, path)
    print(f"Imported '{a.name}': {n} points, {s[-1] + seg[-1]:.2f} m, vmax {vmax:.2f} m/s, "
          f'est. lap {lap_t:.2f} s, closest edge {min(d_left.min(), d_right.min()):.2f} m -> {path}')


def cmd_use(a):
    ensure_library(a.map)
    a.name = short_name(a.map, a.name)
    src = line_path(a.map, a.name)
    if not os.path.exists(src):
        sys.exit(f"No raceline '{a.name}'. Try: list {a.map}")
    shutil.copy(src, active_path(a.map))
    with open(os.path.join(lib_dir(a.map), 'ACTIVE'), 'w') as f:
        f.write(a.name + '\n')
    print(f"'{a.name}' is now active for {a.map}. Restart the base system to load it.")


def active_name(map_name):
    try:
        with open(os.path.join(lib_dir(map_name), 'ACTIVE')) as f:
            return f.read().strip()
    except FileNotFoundError:
        return 'forza_iqp'


def cmd_list(a):
    ensure_library(a.map)
    act = active_name(a.map)
    for fn in sorted(os.listdir(lib_dir(a.map))):
        if fn.startswith(a.map + '_') and fn.endswith('.json'):
            name = short_name(a.map, fn)
            w = load(os.path.join(lib_dir(a.map), fn))['global_traj_wpnts_iqp']['wpnts']
            print(f"{'*' if name == act else ' '} {name:24s} {len(w):5d} pts  "
                  f"vmax {max(p['vx_mps'] for p in w):5.2f} m/s")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('export'); p.add_argument('map'); p.set_defaults(f=cmd_export)
    p = sub.add_parser('import'); p.add_argument('map'); p.add_argument('csv'); p.add_argument('name')
    p.add_argument('--v-const', type=float, help='speed for x,y-only CSVs'); p.set_defaults(f=cmd_import)
    p = sub.add_parser('use'); p.add_argument('map'); p.add_argument('name'); p.set_defaults(f=cmd_use)
    p = sub.add_parser('list'); p.add_argument('map'); p.set_defaults(f=cmd_list)
    p = sub.add_parser('active'); p.add_argument('map')
    p.set_defaults(f=lambda a: print(active_name(a.map)))
    a = ap.parse_args()
    a.f(a)


if __name__ == '__main__':
    main()
