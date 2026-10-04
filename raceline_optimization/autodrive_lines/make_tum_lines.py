#!/usr/bin/env python3
"""Generate global racelines for an AutoDRIVE track with the TUM optimizer (../tum_optimizer).

  ../tum_optimizer/venv/bin/python make_tum_lines.py <track.csv> <track_name> [mode ...]

modes: shortest_path, mincurv, mincurv_iqp, centerline, mintime, mintime_from_<line>
       (default: shortest_path mincurv)

'mintime' is TUM's min-time optimization started on the reference line (stock IPOPT guess).
'mintime_from_<line>' is the same optimization with IPOPT's initial lateral offset taken from
<track_name>/<line>.csv (e.g. mintime_from_mincurv, mintime_from_f1nn), via the init_line
option added to main_globaltraj.py. Run the seed line's own mode first.

Runs tum_optimizer/main_globaltraj.py unmodified, overriding only its USER INPUT block:
vehicle file racecar_f1tenth_small_track.ini, this track, the opt_type, plots off, and the export path.
'centerline' has no TUM opt_type of its own. It is the mincurv run with the curvature QP
replaced by alpha = 0, i.e. TUM's spline-smoothed reference line with TUM's velocity profile.
The reference line is ForzaETH's centerline (track.csv), so this is the data-collection
centerline. Run on line_as_reference.py output, it gives any bare x,y line (e.g. the IL line)
the same treatment.

The track is rotated to start at the point nearest --start x,y (default: the AutoDRIVE spawn,
0.80,3.16), so the loop closes on the start straight instead of mid-corner. Closing in the
hairpin left a 0.25 rad kink (|kappa| 6.3 1/m) in the min-curvature line.

Writes <track_name>/<mode>.csv (TUM traj_race_cl format: s_m; x_m; y_m; psi_rad;
kappa_radpm; vx_mps; ax_mps2) and <track_name>/<mode>.log next to this script.
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TUM = os.path.normpath(os.path.join(HERE, '..', 'tum_optimizer'))
VEH = 'racecar_f1tenth_small_track.ini'   # racecar_f1tenth.ini dynamics, fine discretization

RUNNER = r'''
import os, re, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.show = lambda *a, **k: None
import trajectory_planning_helpers as tph
if os.environ["TUM_MODE"] == "centerline":
    def zero_offset(reftrack, **kw):
        return np.zeros(reftrack.shape[0]), 0.0
    tph.opt_min_curv.opt_min_curv = zero_offset
src = open("main_globaltraj.py").read()
for pat, rep in [
    (r'file_paths = \{"veh_params_file": "[^"]*"\}', 'file_paths = {"veh_params_file": "%s"}' % os.environ["TUM_VEH"]),
    (r'^file_paths\["track_name"\] = .*$', 'file_paths["track_name"] = "%s"' % os.environ["TUM_TRACK"]),
    (r"^opt_type = .*$", "opt_type = '%s'" % os.environ["TUM_OPT"]),
    (r'^file_paths\["traj_race_export"\] = .*$', 'file_paths["traj_race_export"] = "%s"' % os.environ["TUM_OUT"]),
]:
    src, n = re.subn(pat, rep, src, count=1, flags=re.M)
    assert n == 1, "config line not found: " + pat
if os.environ.get("TUM_INIT"):
    src, n = re.subn(r'"init_line": None', '"init_line": "%s"' % os.environ["TUM_INIT"], src, count=1)
    assert n == 1, "init_line option not found"
src = re.sub(r'("(?:raceline|raceline_curv|racetraj_vel|racetraj_vel_3d|imported_bounds|spline_normals|mincurv_curv_lin|mintime_plots)":\s*)True', r'\1False', src)
exec(compile(src, "main_globaltraj.py", "exec"), {"__name__": "__main__", "__file__": os.path.abspath("main_globaltraj.py")})
'''


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    args = sys.argv[1:]
    start = (0.80, 3.16)
    if '--start' in args:
        i = args.index('--start')
        start = tuple(float(v) for v in args[i + 1].split(','))
        del args[i:i + 2]
    track_csv, name = args[0], args[1]
    modes = args[2:] or ['shortest_path', 'mincurv']
    rows = [l for l in open(track_csv) if l.strip() and not l.startswith('#')]
    pts = [tuple(float(v) for v in l.split(',')[:2]) for l in rows]
    k = min(range(len(pts)), key=lambda j: (pts[j][0] - start[0]) ** 2 + (pts[j][1] - start[1]) ** 2)
    with open(os.path.join(TUM, 'inputs', 'tracks', name + '.csv'), 'w') as f:
        f.write('# x_m,y_m,w_tr_right_m,w_tr_left_m\n')
        f.writelines(rows[k:] + rows[:k])
    out_dir = os.path.join(HERE, name)
    os.makedirs(out_dir, exist_ok=True)
    for mode in modes:
        out = os.path.join(out_dir, mode + '.csv')
        opt, init = mode, ''
        if mode == 'centerline':
            opt = 'mincurv'
        elif mode.startswith('mintime_from_'):
            opt, init = 'mintime', os.path.join(out_dir, mode[len('mintime_from_'):] + '.csv')
            if not os.path.isfile(init):
                sys.exit('seed line missing: ' + init)
        env = dict(os.environ, TUM_MODE=mode, TUM_VEH=VEH, TUM_TRACK=name, TUM_OUT=out, TUM_OPT=opt,
                   TUM_INIT=init, MPLBACKEND='Agg')
        with open(os.path.join(out_dir, mode + '.log'), 'w') as log:
            r = subprocess.run([sys.executable, '-c', RUNNER], cwd=TUM, env=env,
                               stdout=log, stderr=subprocess.STDOUT)
        txt = open(os.path.join(out_dir, mode + '.log')).read()
        lap = re.findall(r'Estimated laptime: ([\d.]+)', txt)
        ipopt = ''
        if opt == 'mintime':
            it = re.findall(r'Number of Iterations\.+: (\d+)', txt)
            sec = re.findall(r'Total seconds in IPOPT\s+= ([\d.]+)', txt)
            ex = re.findall(r'EXIT: (.*)', txt)
            ipopt = f'  IPOPT {it[-1] if it else "?"} it, {sec[-1] if sec else "?"} s, {ex[-1] if ex else "?"}'
        print(f'{mode:22s} exit {r.returncode}  est. lap {lap[-1] if lap else "?"} s{ipopt}  -> {out}')
        if r.returncode:
            print(txt[-1500:])


if __name__ == '__main__':
    main()
