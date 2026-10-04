#!/usr/bin/env python3
"""Compare logged runs side by side: one row per run, best first.

  python3 compare_runs.py [runs_dir] [--line SUBSTR] [--last N]

Reads <runs_dir>/*/summary.json written by run_logger, and one level deeper
(<runs_dir>/<experiment>/*/summary.json), so the default (F1Tenth/data) covers every
experiment. archive/ is skipped unless you pass it (or a folder inside it) as runs_dir.
Run folders are <track>_<line>_x<speed>_<YYYYmmdd_HHMMSS>. Stdlib only, so it runs on the Mac too
(in the container, pass ~/ws/runs: the default path is the Mac's F1Tenth/data).
"""
import argparse
import json
import os
import re

STAMP = re.compile(r'(\d{8}_\d{6})$')


def load_runs(top, sub='', depth=1):
    runs = []
    path = os.path.join(top, sub)
    for d in sorted(os.listdir(path)):
        p = os.path.join(path, d)
        if not os.path.isdir(p):
            continue
        f = os.path.join(p, 'summary.json')
        if os.path.isfile(f):
            try:
                s = json.load(open(f))
            except (json.JSONDecodeError, OSError):
                continue   # run killed mid-write
            s['run'] = os.path.join(sub, d)
            runs.append(s)
        elif depth > 0 and not (sub == '' and d == 'archive'):
            runs += load_runs(top, os.path.join(sub, d), depth - 1)
    return runs


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument('runs_dir', nargs='?', default=os.path.join(here, '..', '..', '..', '..', 'data'))
    ap.add_argument('--line', help='only runs whose raceline name contains this')
    ap.add_argument('--last', type=int, help='only the N most recent runs')
    a = ap.parse_args()

    runs = [s for s in load_runs(a.runs_dir) if not a.line or a.line in s['raceline']]
    if a.last:   # most recent by the timestamp at the end of the folder name
        runs.sort(key=lambda s: (STAMP.search(s['run']) or STAMP.search('0' * 8 + '_' + '0' * 6)).group(1))
        runs = runs[-a.last:]
    if not runs:
        print('No runs found.')
        return
    runs.sort(key=lambda s: (s['best_lap_s'] is None, s['best_lap_s'] or 0))

    def f(v, fmt):
        return format(v, fmt) if v is not None else format('-', '>' + fmt.split('.')[0])

    def short(s):   # line + speed without the track prefix (the track has its own column)
        if s.get('line'):
            sp = s.get('speed_scaling')
            return s['line'] + (f'_x{sp:g}' if sp is not None else '')
        t = s.get('track')
        r = s['raceline']
        return r[len(t) + 1:] if t and r.startswith(t + '_') else r
    hdr = f"{'track':20s} {'raceline':26s} {'laps':>4s} {'valid':>5s} {'crash':>5s} {'best s':>7s} {'mean s':>7s} " \
          f"{'std s':>6s} {'vmax':>5s} {'vmean':>5s} {'wall m':>6s}  run"
    print(hdr)
    print('-' * len(hdr))
    for s in runs:
        print(f"{(s.get('track') or '-')[:20]:20s} {short(s)[:26]:26s} {s['laps_completed']:4d} {s['valid_laps']:5d} "
              f"{s['crashes']:5d} {f(s['best_lap_s'], '7.3f')} {f(s['mean_lap_s'], '7.3f')} {f(s['std_lap_s'], '6.3f')} "
              f"{f(s['max_speed_mps'], '5.2f')} {f(s['mean_speed_mps'], '5.2f')} "
              f"{f(s['min_wall_dist_m'], '6.2f')}  {s['run']}")


if __name__ == '__main__':
    main()
