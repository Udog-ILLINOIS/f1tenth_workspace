#!/usr/bin/env python3
"""IL seed line: the authors' F1-NN raceline prediction for an AutoDRIVE track (input to mintime_from_f1nn).

  ../f1nn_vs_tum/.venv/bin/python make_f1nn_line.py <track.csv> <track_name>

Uses f1nn_vs_tum/common.py predict_f1nn: the track is scaled up by k = 11 m / mean track width (an F1-like width,
the k_width variant of f1nn_vs_tum), +1 m training margin added per side, the authors' recursive rollout with the
released split_01 checkpoint, and the line scaled back by 1/k. Raw network output, no smoothing (the paper seeds
the solver with the raw prediction); make_tum_lines.py clips it to the optimizer's corridor.

Writes <track_name>/f1nn.csv (x_m, y_m) next to this script.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, '..', 'f1nn_vs_tum')))
from common import load_tum_csv, predict_f1nn  # noqa: E402


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    track = load_tum_csv(sys.argv[1])
    k = 11.0 / np.mean(track[:, 2] + track[:, 3])
    line, secs = predict_f1nn(track, k)
    out_dir = os.path.join(HERE, sys.argv[2])
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, 'f1nn.csv')
    np.savetxt(out, line, fmt='%.6f', delimiter=',', header='x_m,y_m')
    print(f'k = {k:.2f}, {len(line)} points, inference {secs:.2f} s -> {out}')


if __name__ == '__main__':
    main()
