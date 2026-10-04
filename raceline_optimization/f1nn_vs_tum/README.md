# F1-NN vs tum_optimizer

Runs the authors' released model (`../f1nn_init_shehadeh2026`, checkpoint `f1nn_split01_ep1500.pt`) on the TUM
track datasets and compares its lines with `tum_optimizer`'s methods. The authors' clone is used read-only; their
code is followed as-is (not the paper text).

```
.venv/bin/python run_compare.py full        # ../track_data/full_scale_tracks, 25 real-size circuits
.venv/bin/python run_compare.py f1tenth     # ../track_data/f1tenth_scale_tracks, 23 F1Tenth-size circuits
.venv/bin/python run_compare.py f1tenth Spielberg Monza    # subset
```
Output: `results/<dataset>/compare.csv`, one overlay PNG per track, and every line as CSV in `results/<dataset>/lines/`.

## What gets compared (per track)
| line | how |
|---|---|
| TUM `shortest_path`, `mincurv`, `mincurv_iqp` | same calls as `tum_optimizer/main_globaltraj.py` (IQP with the tph 0.79 signature shim from `setup.md`) |
| raceline shipped with the dataset | `full_scale_tracks/racelines/` or `<Track>_raceline.csv` (made with other safety margins) |
| centerline | reference |
| F1-NN raw | the authors' recursive rollout, unchanged |
| F1-NN smoothed | ours: clip to the car's corridor, then a periodic Gaussian (σ = 2 nodes) to remove small wiggles |

Every line is scored with TUM's own lap-time evaluator (create_raceline -> curvature -> ggv velocity profile ->
lap time), with the same car for every method:
- full size: TUM's stock car, `params/racecar_fullsize.ini` (= `tum_optimizer/params/racecar.ini` at git HEAD; the
  file on disk there was edited to a 1:10 car), `ggv.csv`
- F1Tenth: `params/racecar_f1tenth.ini` (copy of `tum_optimizer/params/racecar_f1tenth.ini`), `ggv_f1tenth.csv`

## Conventions handled in `common.py`
- **Sign**: the authors' `d_m` is positive to the right; TUM's `n` is positive to the left.
- **Width margin**: the model was trained on widths +1 m per side (verified: +2.00 m total on all 5 shared TUM
  tracks). The margin is added for inference only; TUM's methods and the evaluator use the real widths, and F1-NN
  lines are clipped to the same corridor (`width_opt`) the TUM methods get (`clipped_frac` = share of nodes clipped).
- **Scale (F1Tenth only)**: the model works in F1 metres, so the track is scaled up by k, the margin added, the
  prediction made, and the line scaled back by 1/k. The F1Tenth downscale isn't uniform (length ≈ 1:12.6, width fixed
  at 2.2 m), so two k are tried: `k_width` = 11 m / track width (≈ 5, F1-like widths, tighter-than-F1 corners) and
  `k_length` = real circuit length / F1Tenth length (≈ 12.6, real corner shapes, ~28 m wide track).

## Model coverage
The released checkpoint is split_01: trained on cota, hungaroring, imola, interlagos, jeddah, melbourne, montreal,
monza, sakhir, shanghai, silverstone, singapore, spa, spielberg; never saw baku, barcelona, mexico.
`model_saw_track` in the CSV is `train` / `test` / `unseen` (circuits not in the authors' dataset at all).
