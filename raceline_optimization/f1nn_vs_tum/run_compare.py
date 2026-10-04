"""Authors' F1-NN model vs tum_optimizer's methods on the TUM track datasets.

  python run_compare.py full        # track_data/full_scale_tracks (real size, TUM stock full-size car)
  python run_compare.py f1tenth     # track_data/f1tenth_scale_tracks (F1Tenth car), F1-NN at two scale factors
  python run_compare.py f1tenth Spielberg Monza     # subset

Per track: TUM shortest_path / mincurv / mincurv_iqp, the raceline shipped with the dataset, the centerline, and the
F1-NN prediction (raw, and clip-then-smoothed). Every line is scored by TUM's own lap-time evaluator with the same
vehicle. Writes results/<dataset>/compare.csv, a per-track overlay PNG and every line as CSV.
"""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from common import HERE, RO, Prepared, evaluate, load_pars, load_tum_csv, predict_f1nn, seen_by_model, smooth_n, \
    tum_methods

FULL = RO / "track_data" / "full_scale_tracks"
F1T = RO / "track_data" / "f1tenth_scale_tracks"
F1_MEDIAN_WIDTH_M = 11.0      # median real width of the TUM-database F1 circuits (9-14 m)


def full_tracks():
    for p in sorted((FULL / "tracks").glob("*.csv")):
        ref = FULL / "racelines" / p.name
        yield p.stem, p, (np.loadtxt(ref, comments="#", delimiter=",")[:, :2] if ref.exists() else None), {"k=1": 1.0}


def f1tenth_tracks():
    for d in sorted(x for x in F1T.iterdir() if x.is_dir()):
        cl = next(d.glob("*_centerline.csv"), None)
        if cl is None:
            continue
        name = cl.name.replace("_centerline.csv", "")
        track = load_tum_csv(cl)
        rl = next(d.glob("*_raceline.csv"), None)
        ref = np.loadtxt(rl, comments="#", delimiter=";")[:, 1:3] if rl else None
        # Two ways to bring the track to F1 size (the F1Tenth downscale is not uniform, see README.md):
        ks = {"k_width": F1_MEDIAN_WIDTH_M / float(np.median(track[:, 2] + track[:, 3]))}
        full = FULL / "tracks" / f"{name}.csv"
        if full.exists():
            f = load_tum_csv(full)
            per = lambda a: np.linalg.norm(np.diff(np.vstack([a[:, :2], a[:1, :2]]), axis=0), axis=1).sum()  # noqa
            ks["k_length"] = per(f) / per(track)
        yield name, cl, ref, ks


def plot(prep, lines, title, path):
    fig, ax = plt.subplots(figsize=(9, 9))
    ref, nv = prep.reftrack, prep.normvec
    ax.plot(*np.vstack([ref[:, :2] + nv * ref[:, 2:3], ref[:1, :2] + nv[:1] * ref[:1, 2:3]]).T, "k", lw=0.7)
    ax.plot(*np.vstack([ref[:, :2] - nv * ref[:, 3:4], ref[:1, :2] - nv[:1] * ref[:1, 3:4]]).T, "k", lw=0.7)
    styles = {"TUM mincurv": ("tab:green", "-"), "TUM mincurv_iqp": ("tab:orange", "-"),
              "TUM shortest_path": ("tab:purple", ":")}
    for name, xy in lines.items():
        c, ls = styles.get(name, ("tab:red", "-") if "smoothed" in name else ("tab:pink", "--"))
        ax.plot(*np.vstack([xy, xy[:1]]).T, color=c, ls=ls, lw=0.9, label=name)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.legend(loc="best", fontsize=7)
    ax.set_title(title)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main():
    dataset = sys.argv[1] if len(sys.argv) > 1 else "full"
    subset = set(sys.argv[2:])
    pars = load_pars(HERE / "params" / ("racecar_fullsize.ini" if dataset == "full" else "racecar_f1tenth.ini"))
    w_opt = pars["optim_opts_mc"]["width_opt"]
    out = HERE / "results" / dataset
    (out / "lines").mkdir(parents=True, exist_ok=True)
    prev = out / "compare.csv"
    rows = pd.read_csv(prev).to_dict("records") if prev.exists() and not subset else []   # resume
    done = {r["track"] for r in rows}
    for name, csv, shipped, ks in (full_tracks() if dataset == "full" else f1tenth_tracks()):
        if (subset and name not in subset) or name in done:
            continue
        try:
            prep = Prepared(csv, pars)
        except OSError as e:          # TUM prep_track refuses some tracks (e.g. crossed normals in tight hairpins)
            print(f"{name}: TUM prep_track failed ({e}); skipped", flush=True)
            rows.append({"dataset": dataset, "track": name, "method": "TUM prep_track failed"})
            pd.DataFrame(rows).to_csv(out / "compare.csv", index=False)
            continue
        # candidate = (frame, n at the frame's nodes, compute seconds, scale k, fraction of nodes clipped)
        cands = {"centerline": (prep, np.zeros(len(prep.reftrack)), None, None, 0.0)}
        for m, (n, secs, frame) in tum_methods(prep).items():
            cands[m] = (frame, n, secs, None, 0.0)
        if shipped is not None:   # made with other safety margins: clip at the real edges only
            cands["raceline shipped with dataset"] = (prep, prep.n_from_global(shipped), None, None, 0.0)
        track = load_tum_csv(csv)
        lo, hi = -prep.reftrack[:, 2] + w_opt / 2, prep.reftrack[:, 3] - w_opt / 2
        for kname, k in ks.items():
            line, secs = predict_f1nn(track, k)
            n_free = prep.n_from_global(line, -1e9)            # no clipping, to measure how much is outside
            clipped = float(np.mean((n_free < lo) | (n_free > hi)))
            n = np.clip(n_free, lo, hi)                        # same corridor the TUM methods get
            tag = "" if dataset == "full" else f" [{kname}={k:.1f}]"
            cands[f"F1-NN raw{tag}"] = (prep, n, secs, k, clipped)
            cands[f"F1-NN smoothed{tag}"] = (prep, smooth_n(prep, n, w_opt), secs, k, clipped)
        drawn = {}
        for method, (frame, n, secs, k, clipped) in cands.items():
            m, race = evaluate(frame, n)
            rows.append({"dataset": dataset, "track": name, "model_saw_track": seen_by_model(name), "method": method,
                         "compute_s": secs, "scale_k": k, "clipped_frac": clipped, **m})
            safe = method.replace(" ", "_").replace("[", "").replace("]", "").replace("=", "")
            np.savetxt(out / "lines" / f"{name}__{safe}.csv", race, delimiter=",", fmt="%.4f", header="x_m,y_m",
                       comments="")
            if method != "centerline" and "raw" not in method and "shipped" not in method:
                drawn[method] = race
        plot(prep, drawn, f"{name} ({dataset}, model: {seen_by_model(name)})", out / f"{name}.png")
        df = pd.DataFrame(rows)
        df.to_csv(out / "compare.csv", index=False)
        print(df[df.track == name][["method", "compute_s", "laptime_s", "v_mean_mps", "max_abs_kappa", "clipped_frac"]]
              .to_string(index=False, float_format="%.3f"), flush=True)
    summarize(pd.DataFrame(rows))


def summarize(df):
    df = df[df.method != "TUM prep_track failed"]
    df = df.assign(method=df.method.str.replace(r"=[0-9.]+\]", "]", regex=True))   # group k_length over tracks
    base = df[df.method == "TUM mincurv"].set_index("track")["laptime_s"]
    df = df.assign(vs_mincurv_pct=100 * (df.laptime_s / df.track.map(base) - 1))
    for group, g in [("all tracks", df), ("tracks the model never saw", df[df.model_saw_track != "train"])]:
        agg = g.groupby("method", sort=False).agg(n=("track", "count"), compute_s=("compute_s", "mean"),
                                                  laptime_s=("laptime_s", "mean"),
                                                  vs_mincurv_pct=("vs_mincurv_pct", "mean"),
                                                  v_mean_mps=("v_mean_mps", "mean"))
        print(f"\nMean over {group} (vs_mincurv_pct = lap time relative to TUM mincurv on the same track)")
        print(agg.to_string(float_format="%.3f"))


if __name__ == "__main__":
    main()
