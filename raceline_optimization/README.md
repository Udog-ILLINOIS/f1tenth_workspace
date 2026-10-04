# Raceline optimization

Global raceline generators (the code) and the track data they read.

```
raceline_optimization/
  tum_optimizer/          the optimizer: track in, raceline out
  f1nn_init_shehadeh2026/     authors' code for Shehadeh et al. 2026: F1-telemetry dataset (17 tracks) + raceline network
  archive/                    f1_telemetry_init_our_reimplementation/ (leftover of our deleted reimplementation of that paper)
  track_data/
    f1tenth_scale_tracks/ ~20 real circuits shrunk for F1Tenth: centerline + raceline CSVs + gym map PNG/YAML
    full_scale_tracks/    the same kind of circuits at real size: centerline + track widths
    occupancy_maps/       ROS occupancy-grid maps (TU Wien buildings + some circuits), plus a costmap path script
  setup.md                tum_optimizer setup, env fixes, F1Tenth vehicle profile, first results
```

## The two optimizers

| | `tum_optimizer/` | `f1nn_init_shehadeh2026/` |
|---|---|---|
| What | TUM's raceline tool (upstream: [TUMFTM/global_racetrajectory_optimization](https://github.com/TUMFTM/global_racetrajectory_optimization)) | Official code of Shehadeh et al. 2026 ([samir-shehadeh/f1-data-init-optimization](https://github.com/samir-shehadeh/f1-data-init-optimization)) |
| Input | Centerline + widths | Centerline + widths, plus F1 telemetry for training |
| Output | Raceline + speed profile | A *starting guess* for `tum_optimizer`'s min-time mode |
| Methods | shortest path, min curvature, min curvature IQP, min time | Neural network predicts an F1-driver-like offset from the centerline |
| Status | Working (see `setup.md`) | Dataset, network, one trained checkpoint (split_01). Not released: the min-time seed patch, RoboRacer tools |

`f1nn_init_shehadeh2026` doesn't replace `tum_optimizer`; it plugs into it. The min-time solver normally starts from the
centerline. This project swaps in a learned starting line so the solver converges in fewer iterations.

## The three track datasets

| Folder (upstream repo) | Scale | Contents | Used by |
|---|---|---|---|
| `track_data/f1tenth_scale_tracks/` ([f1tenth/f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks)) | 1:10-ish (length ≈1:12.6, width fixed 2.2 m) | `<Track>_centerline.csv`, `<Track>_raceline.csv`, `<Track>_map.png/.yaml` | `tum_optimizer` runs for the F1Tenth car; gym maps |
| `track_data/full_scale_tracks/` ([TUMFTM/racetrack-database](https://github.com/TUMFTM/racetrack-database)) | real size (tracks 9–14 m wide) | `tracks/<Track>.csv` (x, y, w_right, w_left), TUM racelines | `f1nn_init_shehadeh2026` uses 5 of these (mexico, montreal, sakhir, silverstone, spa) |
| `track_data/occupancy_maps/` ([CPS-TUWien/f1tenth_maps](https://github.com/CPS-TUWien/f1tenth_maps)) | F1Tenth | ROS map PNG/YAML for localization/sim; `costmaps/generate-costmap.py` | nothing yet |

`f1tenth_scale_tracks` and `full_scale_tracks` cover mostly the same circuits. One is shrunk for the car and the other is
real size. `occupancy_maps` is a different kind of data: grid images, not centerline CSVs.

## Notes
- Upstream git remotes are unchanged; only the local folder names differ from the repo names.
- `tum_optimizer/venv/` and `.venv/` had their absolute paths rewritten after the 2026-10-01 rename. If a venv acts up,
  rebuild it rather than moving it again.
