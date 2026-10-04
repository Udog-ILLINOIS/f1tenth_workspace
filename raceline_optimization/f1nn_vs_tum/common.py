"""Shared pieces: track loading, the authors' F1-NN model, TUM's methods and TUM's lap-time evaluator.

Conventions handled here (see README.md):
  * authors' d_m is positive to the RIGHT, TUM's n is positive to the LEFT  ->  n = -d
  * the authors' model was trained on widths with +1 m margin per side, so the margin is added for inference only;
    TUM's methods and the evaluator always see the real widths.
  * F1Tenth tracks are scaled up by k before inference (in F1 metres) and the prediction scaled back by 1/k.
"""
import configparser
import importlib.util
import json
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import pandas as pd
import torch
import trajectory_planning_helpers as tph
from scipy.interpolate import CubicSpline
from scipy.ndimage import gaussian_filter1d
from scipy.spatial import cKDTree

HERE = Path(__file__).resolve().parent
RO = HERE.parent                                            # raceline_optimization/
AUTHORS = RO / "f1nn_init_shehadeh2026"
sys.path.insert(0, str(AUTHORS))
sys.path.insert(0, str(RO / "tum_optimizer"))               # helper_funcs_glob (read-only use)

import helper_funcs_glob  # noqa: E402
from f1init.dataset import read_track, track_features  # noqa: E402
from f1init.geometry import tangents_normals  # noqa: E402
from f1init.model import load_model  # noqa: E402

_spec = importlib.util.spec_from_file_location("authors_predict", AUTHORS / "scripts" / "predict.py")
_pred = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_pred)
rollout = _pred.rollout                                     # the authors' recursive full-lap inference

CHECKPOINT = AUTHORS / "checkpoints" / "f1nn_split01_ep1500.pt"
MARGIN_M = 1.0                                              # authors' data/README.md: +1 m per side
AUTHORS_TRAIN = {"cota", "hungaroring", "imola", "interlagos", "jeddah", "melbourne", "montreal", "monza", "sakhir",
                 "shanghai", "silverstone", "singapore", "spa", "spielberg"}
# TUM database name -> authors' name (only where the circuit is in their dataset)
TUM_TO_AUTHORS = {"Austin": "cota", "Budapest": "hungaroring", "Catalunya": "barcelona", "Melbourne": "melbourne",
                  "MexicoCity": "mexico", "Montreal": "montreal", "Monza": "monza", "Sakhir": "sakhir",
                  "SaoPaulo": "interlagos", "Shanghai": "shanghai", "Silverstone": "silverstone", "Spa": "spa",
                  "Spielberg": "spielberg"}


def seen_by_model(tum_name: str) -> str:
    a = TUM_TO_AUTHORS.get(tum_name)
    if a is None:
        return "unseen"
    return "train" if a in AUTHORS_TRAIN else "test"


# ---------------------------------------------------------------------------------------------------------------- data
def load_tum_csv(path: Path) -> np.ndarray:
    """x_m, y_m, w_tr_right_m, w_tr_left_m (comma separated, '#' header)."""
    return np.loadtxt(path, comments="#", delimiter=",")


def load_pars(ini: Path) -> dict:
    parser = configparser.ConfigParser()
    if not parser.read(ini):
        raise ValueError(f"cannot read {ini}")
    g = lambda sec, key: json.loads(parser.get(sec, key))  # noqa: E731
    pars = {k: g("GENERAL_OPTIONS", k) for k in
            ["ggv_file", "ax_max_machines_file", "stepsize_opts", "reg_smooth_opts", "veh_params", "vel_calc_opts"]}
    pars["optim_opts_sp"] = g("OPTIMIZATION_OPTIONS", "optim_opts_shortest_path")
    pars["optim_opts_mc"] = g("OPTIMIZATION_OPTIONS", "optim_opts_mincurv")
    vdi = RO / "tum_optimizer" / "inputs" / "veh_dyn_info"
    pars["ggv"], pars["ax_mach"] = tph.import_veh_dyn_info.import_veh_dyn_info(
        ggv_import_path=str(vdi / pars["ggv_file"]), ax_max_machines_import_path=str(vdi / pars["ax_max_machines_file"]))
    return pars


# ---------------------------------------------------------------------------------------------------------- F1-NN model
_MODEL = None


def predict_f1nn(track: np.ndarray, k: float = 1.0):
    """Authors' model on a TUM-format track. Returns (global line xy in the track's own units, seconds).

    Scale up by k, add the +1 m training margin, run the authors' read_track (2 m resampling) + rollout, map the
    predicted offset (right-positive) to global xy, scale back by 1/k."""
    global _MODEL
    if _MODEL is None:
        _MODEL = load_model(str(CHECKPOINT))
        torch.set_num_threads(1)
    t0 = time.perf_counter()
    scaled = track * k
    scaled[:, 2:] += MARGIN_M
    with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False) as f:
        pd.DataFrame(scaled, columns=["x_m", "y_m", "w_tr_right_m", "w_tr_left_m"]).to_csv(f, index=False)
        tmp = f.name
    df = read_track(tmp)
    Path(tmp).unlink()
    geo = track_features(df)
    d = rollout(_MODEL, geo, None)
    xy = df[["x_m", "y_m"]].to_numpy()
    _, n_right = tangents_normals(xy)
    line = (xy + d[:, None] * n_right) / k
    return line, time.perf_counter() - t0


class Frame:
    """A reference line + right-pointing normals: what evaluate() needs."""

    def __init__(self, reftrack, normvec, pars):
        self.reftrack, self.normvec, self.pars = reftrack, normvec, pars


# -------------------------------------------------------------------------------------------------- TUM prepared track
class Prepared:
    """Track after TUM import_track + prep_track (the reference line all TUM methods work on)."""

    def __init__(self, track_csv: Path, pars: dict):
        imp = {"flip_imp_track": False, "set_new_start": False, "new_start": np.zeros(2), "min_track_width": None,
               "num_laps": 1}
        raw = helper_funcs_glob.src.import_track.import_track(imp_opts=imp, file_path=str(track_csv),
                                                              width_veh=pars["veh_params"]["width"])
        t0 = time.perf_counter()
        self.reftrack, self.normvec, self.a_interp, _, _ = helper_funcs_glob.src.prep_track.prep_track(
            reftrack_imp=raw, reg_smooth_opts=pars["reg_smooth_opts"], stepsize_opts=pars["stepsize_opts"],
            debug=False, min_width=None)
        self.prep_s = time.perf_counter() - t0
        self.pars = pars
        # periodic interpolating spline through the nodes, by chord length, for projecting other lines onto it
        xy = np.vstack([self.reftrack[:, :2], self.reftrack[:1, :2]])
        u = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))])
        self._sp, self.L, self.s_nodes = CubicSpline(u, xy, bc_type="periodic"), u[-1], u[:-1]
        self._sd = np.linspace(0, self.L, int(self.L / (0.05 * pars["stepsize_opts"]["stepsize_reg"])), endpoint=False)
        self._kd = cKDTree(self._sp(self._sd))

    def n_from_global(self, xy: np.ndarray, width_opt: float = 0.0) -> np.ndarray:
        """Any closed global line -> TUM n (left +) at every node, clipped to the width_opt corridor
        (width_opt = 0: clipped at the real track edges only)."""
        _, j = self._kd.query(xy)
        s = self._sd[j]
        for _ in range(3):
            d1 = self._sp(s, 1)
            t = d1 / np.linalg.norm(d1, axis=1, keepdims=True)
            s = np.mod(s + np.einsum("ij,ij->i", xy - self._sp(s), t), self.L)
        d1 = self._sp(s, 1)
        t = d1 / np.linalg.norm(d1, axis=1, keepdims=True)
        d = np.einsum("ij,ij->i", xy - self._sp(s), np.column_stack([-t[:, 1], t[:, 0]]))
        o = np.argsort(s)
        s, d = s[o], d[o]
        n = np.interp(self.s_nodes, np.concatenate([s - self.L, s, s + self.L]), np.tile(d, 3))
        return np.clip(n, -self.reftrack[:, 2] + width_opt / 2, self.reftrack[:, 3] - width_opt / 2)

    def global_from_n(self, n: np.ndarray) -> np.ndarray:
        return self.reftrack[:, :2] - self.normvec * n[:, None]          # TUM normvec points right


def smooth_n(prep: Prepared, n: np.ndarray, width_opt: float, sigma_nodes: float = 2.0) -> np.ndarray:
    """Clip-then-smooth (periodic Gaussian on n). The raw network output carries small wiggles that a velocity
    profile brakes for; this is our post-processing, not the authors'."""
    n = gaussian_filter1d(n, sigma_nodes, mode="wrap")
    return np.clip(n, -prep.reftrack[:, 2] + width_opt / 2, prep.reftrack[:, 3] - width_opt / 2)


# --------------------------------------------------------------------------------------------------- TUM's own methods
def tum_methods(prep: Prepared):
    """TUM shortest_path, mincurv and mincurv_iqp as in main_globaltraj.py.
    Returns {name: (n at the nodes, seconds, frame)}; IQP re-interpolates the reference line, so its frame differs."""
    p, ref, nv = prep.pars, prep.reftrack, prep.normvec
    out = {}
    t0 = time.perf_counter()
    a = tph.opt_shortest_path.opt_shortest_path(reftrack=ref, normvectors=nv, w_veh=p["optim_opts_sp"]["width_opt"])
    out["TUM shortest_path"] = (-a, time.perf_counter() - t0, prep)
    t0 = time.perf_counter()
    a = tph.opt_min_curv.opt_min_curv(reftrack=ref, normvectors=nv, A=prep.a_interp,
                                      kappa_bound=p["veh_params"]["curvlim"], w_veh=p["optim_opts_mc"]["width_opt"])[0]
    out["TUM mincurv"] = (-a, time.perf_counter() - t0, prep)
    # mincurv_iqp with the tph 0.79 signature (same shim as setup.md). Copies: iqp_handler edits reftrack in place.
    t0 = time.perf_counter()
    cx, cy, A, nv2 = tph.calc_splines.calc_splines(path=np.vstack([ref[:, :2], ref[:1, :2]]))
    sl = tph.calc_spline_lengths.calc_spline_lengths(coeffs_x=cx, coeffs_y=cy)
    psi, kap, dkap = tph.calc_head_curv_an.calc_head_curv_an(
        coeffs_x=cx, coeffs_y=cy, ind_spls=np.arange(len(ref)), t_spls=np.zeros(len(ref)), calc_dcurv=True)
    res = tph.iqp_handler.iqp_handler(
        reftrack=ref.copy(), normvectors=nv2.copy(), A=A, spline_len=sl, psi=psi, kappa=kap, dkappa=dkap,
        kappa_bound=p["veh_params"]["curvlim"], w_veh=p["optim_opts_mc"]["width_opt"], print_debug=False,
        plot_debug=False, stepsize_interp=p["stepsize_opts"]["stepsize_reg"],
        iters_min=p["optim_opts_mc"]["iqp_iters_min"], curv_error_allowed=p["optim_opts_mc"]["iqp_curverror_allowed"])
    secs = time.perf_counter() - t0
    out["TUM mincurv_iqp"] = (-res[0], secs, Frame(res[1], res[2], p))   # scored on IQP's own final reference line
    return out


# ------------------------------------------------------------------------------------------------ TUM lap-time scoring
def evaluate(prep, n: np.ndarray) -> dict:
    """TUM main_globaltraj.py post-processing on a line given as n at the nodes: create_raceline ->
    calc_head_curv_an -> calc_vel_profile (ggv) -> calc_t_profile."""
    p = prep.pars
    race, _, cx, cy, si, tv, _, _, el = tph.create_raceline.create_raceline(
        refline=prep.reftrack[:, :2], normvectors=prep.normvec, alpha=-n,
        stepsize_interp=p["stepsize_opts"]["stepsize_interp_after_opt"])
    _, kappa = tph.calc_head_curv_an.calc_head_curv_an(coeffs_x=cx, coeffs_y=cy, ind_spls=si, t_spls=tv)
    vx = tph.calc_vel_profile.calc_vel_profile(
        ggv=p["ggv"], ax_max_machines=p["ax_mach"], v_max=p["veh_params"]["v_max"], kappa=kappa, el_lengths=el,
        closed=True, filt_window=p["vel_calc_opts"]["vel_profile_conv_filt_window"],
        dyn_model_exp=p["vel_calc_opts"]["dyn_model_exp"], drag_coeff=p["veh_params"]["dragcoeff"],
        m_veh=p["veh_params"]["mass"])
    ax = tph.calc_ax_profile.calc_ax_profile(vx_profile=np.append(vx, vx[0]), el_lengths=el, eq_length_output=False)
    t = tph.calc_t_profile.calc_t_profile(vx_profile=vx, ax_profile=ax, el_lengths=el)
    return {"laptime_s": float(t[-1]), "v_mean_mps": float(np.sum(el) / t[-1]), "length_m": float(np.sum(el)),
            "max_abs_kappa": float(np.max(np.abs(kappa)))}, race
