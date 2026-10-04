# Min-time initialization comparison: AutoDRIVE RoboRacer track

Run 2026-10-04 on a MacBook (CPU), one run per line. Track: `racelines/autodrive_roboracer/autodrive_roboracer_track.csv`,
started at the spawn (0.80, 3.16). Vehicle file: `tum_optimizer/params/racecar_f1tenth_small_track.ini`.

**Lap times are TUM's estimates** (`Estimated laptime` in the logs: ggv velocity profile on the final line), not
measured laps in the sim or on the car. The min-time solver's own value (`INFO: Laptime`) was 6.320 s for all three
min-time runs.

## Min-time optimization (TUM `opt_mintime`, IPOPT) from three starting lines

| Starting line (IPOPT initial guess) | Seed offset \|n\| mean / max | IPOPT iterations | IPOPT time | Total runtime | Exit | Est. lap time |
|---|---|---|---|---|---|---|
| Centerline (stock, n = 0) | 0 / 0 m | 599 | 36.71 s | 39.02 s | Optimal Solution Found | 6.25 s |
| Min-curvature (`mincurv.csv`) | 0.459 / 0.885 m | 472 (−21%) | 14.79 s | 16.87 s | Optimal Solution Found | 6.25 s |
| IL / F1-NN (`f1nn.csv`) | 0.227 / 0.430 m | 555 (−7%) | 23.04 s | 25.11 s | Optimal Solution Found | 6.25 s |

- All three converge to the same line: 27.87 m long, v_max 7.28 m/s, 0.0 cm apart, min. distance to boundary 0.35 m.
- Only the lateral offset n is seeded. Other states and controls keep TUM's default guess.
- Times are wall-clock from one run each and will vary. The iteration counts are the reliable comparison.
- F1-NN seed: authors' released checkpoint `f1nn_split01_ep1500.pt`, track scaled up by k = 7.46 (11 m / mean width),
  raw recursive prediction (no smoothing), 116 points, inference 0.05 s. The track is far outside the model's training
  data (≈230 m of "F1 track", shorter than the network's input windows).

## Non-min-time lines on the same track (for reference)

| Line | Solver time | Total runtime | Est. lap time | Note |
|---|---|---|---|---|
| Centerline (TUM smoothed reference line) | none | 0.09 s | 6.59 s | |
| Shortest path | 0.004 s | 0.10 s | 7.78 s | curvature limit exceeded (3.064 rad/m), 0.10 m from boundary |
| Min-curvature | 0.002 s | 0.15 s | 6.35 s | |

## Reproduce

```
cd raceline_optimization/autodrive_lines
../f1nn_vs_tum/.venv/bin/python make_f1nn_line.py ../../racelines/autodrive_roboracer/autodrive_roboracer_track.csv autodrive_roboracer
../tum_optimizer/venv/bin/python make_tum_lines.py ../../racelines/autodrive_roboracer/autodrive_roboracer_track.csv autodrive_roboracer mincurv mintime mintime_from_mincurv mintime_from_f1nn
```
