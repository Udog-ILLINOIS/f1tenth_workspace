#!/usr/bin/env python3
"""ForzaETH's centerline as a raceline, with a TUM speed profile.

  ../tum_optimizer/venv/bin/python forza_centerline_line.py <global_waypoints.json> <out.csv>

The path is ForzaETH's centerline (`centerline_waypoints` in the map's global_waypoints.json,
0.1 m spacing) unchanged. The TUM optimizer has no centerline mode, so the speed profile comes
from TUM's own functions (trajectory_planning_helpers) with the same vehicle file as the
TUM lines (tum_optimizer/params/racecar_f1tenth_small_track.ini: ggv, ax_max_machines, v_max, drag,
mass, numerical curvature preview/review). The lines then differ only in their path.
Output is TUM traj_race_cl format: s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2.
"""
import configparser
import json
import os
import sys

import numpy as np
import trajectory_planning_helpers as tph

TUM = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tum_optimizer'))

wp_json, out = sys.argv[1], sys.argv[2]
cent = json.load(open(wp_json))['centerline_waypoints']['wpnts']
xy = np.array([[w['x_m'], w['y_m']] for w in cent])
if np.hypot(*(xy[0] - xy[-1])) < 1e-3:
    xy = xy[:-1]

ini = configparser.ConfigParser()
ini.read(os.path.join(TUM, 'params', 'racecar_f1tenth_small_track.ini'))
g = lambda k: json.loads(ini.get('GENERAL_OPTIONS', k))
veh, vel, curv = g('veh_params'), g('vel_calc_opts'), g('curv_calc_opts')
ggv, ax_max = tph.import_veh_dyn_info.import_veh_dyn_info(
    ggv_import_path=os.path.join(TUM, 'inputs', 'veh_dyn_info', g('ggv_file')),
    ax_max_machines_import_path=os.path.join(TUM, 'inputs', 'veh_dyn_info', g('ax_max_machines_file')))

el = np.hypot(*np.diff(np.vstack([xy, xy[:1]]), axis=0).T)        # closed: n segments
psi, kappa = tph.calc_head_curv_num.calc_head_curv_num(
    path=xy, el_lengths=el, is_closed=True,
    stepsize_psi_preview=curv['d_preview_head'], stepsize_psi_review=curv['d_review_head'],
    stepsize_curv_preview=curv['d_preview_curv'], stepsize_curv_review=curv['d_review_curv'])
vx = tph.calc_vel_profile.calc_vel_profile(
    ggv=ggv, ax_max_machines=ax_max, v_max=veh['v_max'], kappa=kappa, el_lengths=el,
    closed=True, filt_window=vel['vel_profile_conv_filt_window'],
    dyn_model_exp=vel['dyn_model_exp'], drag_coeff=veh['dragcoeff'], m_veh=veh['mass'])
ax = tph.calc_ax_profile.calc_ax_profile(vx_profile=np.append(vx, vx[0]), el_lengths=el, eq_length_output=False)
t = tph.calc_t_profile.calc_t_profile(vx_profile=vx, ax_profile=ax, el_lengths=el)
s = np.concatenate([[0.0], np.cumsum(el)[:-1]])

with open(out, 'w') as f:
    f.write('# s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2\n')
    for i in range(len(xy)):
        f.write(f'{s[i]:.7f}; {xy[i, 0]:.7f}; {xy[i, 1]:.7f}; {psi[i]:.7f}; {kappa[i]:.7f}; {vx[i]:.7f}; {ax[i]:.7f}\n')
print(f'ForzaETH centerline: {len(xy)} pts, {el.sum():.2f} m, vmax {vx.max():.2f} m/s, '
      f'|kappa| max {np.abs(kappa).max():.2f} 1/m, est. lap {t[-1]:.3f} s -> {out}')
