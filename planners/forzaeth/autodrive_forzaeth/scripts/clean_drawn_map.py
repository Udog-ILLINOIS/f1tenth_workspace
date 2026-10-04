#!/usr/bin/env python3
"""Turn a line-drawn track map into a ForzaETH map (track free, everything else occupied).

  python3 clean_drawn_map.py <drawn.png> <out.png> [--min-wall-px 60] [--wall-dilate 3]

A drawn map has thin black wall lines on white, so the track, the area outside it and the
islands inside it are all "free". ForzaETH's global planner skeletonizes the free space and
needs the track to be the only free region. This keeps only the free region that has holes
(the track ring), drops small black blobs first (grid dots, specks), thickens the 1-2 px wall
lines so the skeleton cannot leak diagonally between adjacent sections, and writes the result
as a 0/255 PNG. Resolution and origin are unchanged, so the map's YAML still applies.
"""
import argparse

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('src'); ap.add_argument('out')
    ap.add_argument('--min-wall-px', type=int, default=60, help='black blobs smaller than this are erased')
    ap.add_argument('--wall-dilate', type=int, default=3, help='square kernel (px) the walls are thickened by')
    a = ap.parse_args()

    img = cv2.imread(a.src, cv2.IMREAD_GRAYSCALE)
    wall = (img < 128).astype(np.uint8)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(wall, connectivity=8)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < a.min_wall_px:
            wall[lab == i] = 0
    if a.wall_dilate > 1:
        wall = cv2.dilate(wall, np.ones((a.wall_dilate, a.wall_dilate), np.uint8))

    free = (1 - wall).astype(np.uint8)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(free, connectivity=4)
    track = None
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if x == 0 or y == 0 or x + w == img.shape[1] or y + h == img.shape[0]:
            continue   # outside the track
        comp = (lab == i).astype(np.uint8)
        cnts, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        filled = np.zeros_like(comp)
        cv2.drawContours(filled, cnts, -1, 1, cv2.FILLED)
        holes = int(filled.sum()) - int(area)
        print(f'free region {i}: {area} px, holes {holes} px')
        if holes > 0 and (track is None or area > track[1]):
            track = (i, area)
    if track is None:
        raise SystemExit('No free region with a hole: the walls do not form a closed track.')
    out = np.where(lab == track[0], 255, 0).astype(np.uint8)
    cv2.imwrite(a.out, out)
    print(f'Track region {track[0]}: {track[1]} px -> {a.out}')


if __name__ == '__main__':
    main()
