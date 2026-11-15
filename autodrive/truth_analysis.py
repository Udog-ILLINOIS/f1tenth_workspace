"""Measured vs ground truth for one race (race_cars.sh runs it; or by hand):
  autodrive/track_builder/venv/bin/python autodrive/truth_analysis.py RACE_DIR TRACK
Reads RACE_DIR/truth_driver<k>.csv (truth_recorder.py) and the track's raceline (map frame) and writes
RACE_DIR/truth_report.txt, truth_driver<k>.png and truth_driver<k>_samples.csv.

Per car:
  localization error: the stack's pose (map frame) vs the sim's (world frame), after one least-squares fit
    of map -> world plus the body offset between base_link and the sim's reference point; split into
    along-track (+ = estimate ahead of the car) and lateral (+ = estimate left of the car), and heading
  wheel slip: wheel-odometry speed (/odom, what the localizer integrates) minus the true speed
  raceline offset: where the car really is vs the raceline (truth) and where the stack thinks it is
    (estimate): a crash with a small estimate offset but a large true offset is a localization crash,
    a large estimate offset is the controller (or the line) itself
  per crash (collision_count rises): those values over the 3 s before the respawn
"""
import csv
import math
import os
import sys

import numpy as np

WHEELBASE = 0.33     # m (control/config/vehicle.yaml)
WINDOW = 3.0          # s before a crash
TELEPORT = 0.3        # m between two truth samples = a respawn / reset
SETTLE = 1.5          # s after a respawn or start left out of the fit


def load(path):
    rows = {}
    with open(path) as f:
        for line in f:
            p = line.rstrip('\n').split(',')
            if p[0] == 'log':
                rows.setdefault('log', []).append((float(p[1]), p[2], ','.join(p[3:])))
                continue
            try:
                rows.setdefault(p[0], []).append([float(v) for v in p[1:]])
            except ValueError:
                pass
    return {k: (np.array(v) if k != 'log' else v) for k, v in rows.items()}


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def raceline(track):
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = os.path.join(root, 'planners', 'sigrobotics', 'ws', 'tracks', track, 'raceline.csv')
    r = np.loadtxt(path, delimiter=';', comments='#')
    xy = r[:, 1:3]
    # direction from the points (closed line): the file's psi is TUM's (0 = +y), not the x-axis heading
    dxy = np.roll(xy, -1, 0) - np.roll(xy, 1, 0)
    return xy, np.arctan2(dxy[:, 1], dxy[:, 0]), r[:, 0]


def offset(pts, line, psi):
    """Signed distance of pts from the raceline (+ = left of its direction) and the index of the nearest point."""
    d = ((pts[:, None, :] - line[None, :, :]) ** 2).sum(-1)
    i = d.argmin(1)
    n = np.stack([-np.sin(psi[i]), np.cos(psi[i])], 1)
    return ((pts - line[i]) * n).sum(1), i


def analyse(path, track, out_png):
    D = load(path)
    gt, est = D['gt'], D.get('est')
    if est is None or len(est) < 50:
        return None, 'no /localized_pose recorded'
    t0 = gt[0, 0]
    tg = gt[:, 0]
    # arrival times (wall clock): frame hitches show here even when the stamps are sim time
    tw = gt[:, 7] if gt.shape[1] > 7 else tg
    # respawns / resets: a truth jump
    step = np.hypot(np.diff(gt[:, 1]), np.diff(gt[:, 2]))
    jumps = tg[1:][step > TELEPORT]
    speed_gt = np.hypot(gt[:, 4], gt[:, 5])

    # truth at the estimate stamps
    te = est[:, 0]
    ok = (te >= tg[0]) & (te <= tg[-1])
    est, te = est[ok], te[ok]
    gx = np.interp(te, tg, gt[:, 1]); gy = np.interp(te, tg, gt[:, 2])
    gyaw = wrap(np.interp(te, tg, np.unwrap(gt[:, 3])))
    gv = np.interp(te, tg, speed_gt)
    # samples near a respawn are left out of the fit (the estimate is allowed to need time there)
    near = np.zeros(len(te), bool)
    for j in jumps:
        near |= (te > j - 0.2) & (te < j + SETTLE)
    near |= te < te[0] + SETTLE
    moving = gv > 0.3

    # fit world = R(th) est + t + d * heading: linear in (a = cos th, b = sin th, tx, ty, d), trimmed
    ex, ey, eyaw = est[:, 1], est[:, 2], est[:, 3]
    use = ~near & moving
    if use.sum() < 20:   # the car hardly drove (or crashed every few s): fit on whatever there is
        use = moving if moving.sum() >= 20 else np.ones(len(te), bool)
    base = use
    for _ in range(3):
        A = np.zeros((2 * use.sum(), 5)); B = np.zeros(2 * use.sum())
        A[0::2] = np.stack([ex[use], -ey[use], np.ones(use.sum()), np.zeros(use.sum()), np.cos(gyaw[use])], 1)
        A[1::2] = np.stack([ey[use], ex[use], np.zeros(use.sum()), np.ones(use.sum()), np.sin(gyaw[use])], 1)
        B[0::2] = gx[use]; B[1::2] = gy[use]
        a, b, tx, ty, d = np.linalg.lstsq(A, B, rcond=None)[0]
        th = math.atan2(b, a); c, s = math.cos(th), math.sin(th)
        wx = c * ex - s * ey + tx + d * np.cos(gyaw)
        wy = s * ex + c * ey + ty + d * np.sin(gyaw)
        err = np.hypot(wx - gx, wy - gy)
        cut = np.percentile(err[use], 90)
        trimmed = base & (err <= max(cut, 0.05))
        if trimmed.sum() < 20:
            break
        use = trimmed
    hx, hy = np.cos(gyaw), np.sin(gyaw)
    along = (wx - gx) * hx + (wy - gy) * hy
    lat = -(wx - gx) * hy + (wy - gy) * hx
    dyaw = wrap(eyaw + th - gyaw)

    # truth in the map frame, both against the raceline
    line, psi, _ = raceline(track)
    mx = c * (gx - tx - d * hx) + s * (gy - ty - d * hy)   # inverse of the fit (body offset removed)
    my = -s * (gx - tx - d * hx) + c * (gy - ty - d * hy)
    off_true, _ = offset(np.stack([mx, my], 1), line, psi)
    off_est, _ = offset(np.stack([ex, ey], 1), line, psi)

    # wheel slip: odometry speed vs true speed at the odometry stamps
    od = D.get('odom')
    slip_t = od[:, 0] if od is not None else np.array([])
    slip = (od[:, 1] - np.interp(slip_t, tg, speed_gt)) if od is not None else np.array([])
    dr = D.get('drive')
    cmd = D.get('cmd')

    # grip: true yaw rate vs what the commanded steering gives without sliding (v tan(steer) / wheelbase);
    # well below 1 = the front slides (understeer)
    grip_t = grip = np.array([])
    if dr is not None and len(dr) > 10:
        st_at = np.interp(tg, dr[:, 0], dr[:, 2])
        kin = speed_gt * np.tan(st_at) / WHEELBASE
        m = np.abs(kin) > 1.0
        grip_t, grip = tg[m], gt[m, 6] / kin[m]
    # grip-loss episodes anywhere in the race: ratio < 0.7 for 0.15 s or more
    episodes = []
    if len(grip):
        low = grip < 0.7
        i = 0
        while i < len(low):
            if low[i]:
                j = i
                while j + 1 < len(low) and low[j + 1] and grip_t[j + 1] - grip_t[j] < 0.1:
                    j += 1
                if grip_t[j] - grip_t[i] >= 0.15:
                    episodes.append((grip_t[i], grip_t[j], grip[i:j + 1].min()))
                i = j + 1
            else:
                i += 1

    # crashes: collision count rises; the respawn is the first truth jump at or after it (else the stamp)
    crashes = []
    coll = D.get('coll')
    if coll is not None:
        prev = None
        for t, n in coll:
            if prev is not None and n > prev and t > 0:
                j = jumps[(jumps >= t - 0.5)]
                crashes.append(j[0] if len(j) and j[0] - t < 1.0 else t)
            prev = n
    clean = ~near & moving
    for tc in crashes:
        clean &= ~((te > tc - WINDOW) & (te < tc + SETTLE))

    def stats(x):
        x = np.abs(x)
        if not len(x):
            return 'no samples'
        return f'median {np.median(x) * 100:.1f} cm, 95% {np.percentile(x, 95) * 100:.1f} cm, max {x.max() * 100:.1f} cm'

    rep = []
    # sim frame hitches: the truth stream is one message per sim frame (crash/respawn gaps included)
    dts = np.diff(tw) if len(tw) > 1 else np.zeros(1)
    long = (dts > 0.06).sum()
    rep.append(f'sim frames: {len(tg) / max(tg[-1] - tg[0], 1e-6):.0f}/s, gap median {np.median(dts) * 1000:.0f} ms, '
               f'p99 {np.percentile(dts, 99) * 1000:.0f} ms, {long} over 60 ms, longest {dts.max() * 1000:.0f} ms'
               + ('  WARNING: a hitchy sim (grip-loss crashes follow hitches; after a rebuild let macOS finish '
                  'scanning the app, close heavy apps)' if long > 5 else ''))
    rep.append(f'fit: map -> world rotation {math.degrees(th):.2f} deg, t ({tx:.3f}, {ty:.3f}) m, '
               f'base_link -> sim reference {d:.3f} m along the heading; {use.sum()} of {len(te)} samples')
    rep.append(f'clean driving ({clean.sum()} samples, crash windows and respawns out):')
    rep.append(f'  along-track {stats(along[clean])}   lateral {stats(lat[clean])}   '
               + (f'heading 95% {np.degrees(np.percentile(np.abs(dyaw[clean]), 95)):.2f} deg' if clean.any() else ''))
    rep.append(f'  raceline offset: true {stats(off_true[clean])}; estimated {stats(off_est[clean])}')
    mv = (np.interp(slip_t, tg, speed_gt) > 0.3) if len(slip) else np.array([], bool)
    if mv.any():
        rep.append(f'  wheel slip (odom - true speed): median {np.median(slip[mv]):+.3f}, '
                   f'5-95% {np.percentile(slip[mv], 5):+.2f}..{np.percentile(slip[mv], 95):+.2f} m/s, '
                   f'max {slip[mv].max():+.2f} m/s')
    def crash_near(a, b):
        return any(a - 0.5 < tc < b + 2.0 for tc in crashes)
    rep.append(f'grip-loss episodes (true yaw rate < 70% of the steering\'s for >= 0.15 s): {len(episodes)}, '
               f'{sum(crash_near(a, b) for a, b, _ in episodes)} followed by a crash within 2 s')
    for a, b, g in episodes:
        rep.append(f'   {a - t0:6.2f}-{b - t0:6.2f} s  min {g:.2f}  {"-> CRASH" if crash_near(a, b) else "recovered"}')
    rep.append(f'crashes: {len(crashes)}')
    logs = D.get('log', [])
    for k, tc in enumerate(crashes, 1):
        w = (te > tc - WINDOW) & (te < tc - 0.05)
        ws = (slip_t > tc - WINDOW) & (slip_t < tc - 0.05)
        if not w.any():
            rep.append(f' crash {k} at {tc - t0:.2f} s: no estimate in the {WINDOW:.0f} s before it'); continue
        i_last = np.where(w)[0][-1]
        e_now = math.hypot(along[i_last], lat[i_last])
        rep.append(f' crash {k} at race time {tc - t0:.2f} s, true pose ({gx[i_last]:.2f}, {gy[i_last]:.2f}) world, '
                   f'speed {gv[i_last]:.2f} m/s')
        rep.append(f'   localization error just before: {e_now * 100:.1f} cm (along {along[i_last] * 100:+.1f}, '
                   f'lateral {lat[i_last] * 100:+.1f}, heading {math.degrees(dyaw[i_last]):+.1f} deg); '
                   f'max over {WINDOW:.0f} s {np.hypot(along[w], lat[w]).max() * 100:.1f} cm')
        rep.append(f'   raceline offset just before: true {off_true[i_last] * 100:+.1f} cm, estimated '
                   f'{off_est[i_last] * 100:+.1f} cm (max |true| over {WINDOW:.0f} s '
                   f'{np.abs(off_true[w]).max() * 100:.1f} cm)')
        if ws.any():
            rep.append(f'   wheel slip over {WINDOW:.0f} s: max {slip[ws].max():+.2f} m/s, mean {slip[ws].mean():+.2f} m/s')
        wg = (grip_t > tc - WINDOW) & (grip_t < tc - 0.05)
        if wg.any():
            k0 = np.where(wg & (grip < 0.7))[0]
            onset = f', below 0.7 from {grip_t[k0[0]] - tc:+.2f} s' if len(k0) else ''
            rep.append(f'   grip (true / steering yaw rate): min {grip[wg].min():.2f}{onset}')
            if len(k0):
                to = grip_t[k0[0]]
                wo = (slip_t > to - 0.5) & (slip_t < to + 0.2)
                if wo.any():
                    rep.append(f'   wheel slip 0.5 s around the grip loss: {slip[wo].min():+.2f}..{slip[wo].max():+.2f} m/s')
                if cmd is not None:
                    wc = (cmd[:, 0] > to - 0.5) & (cmd[:, 0] < to + 0.2)
                    if wc.any():
                        rep.append(f'   throttle sent 0.5 s around it: {cmd[wc, 1].min():+.3f}..{cmd[wc, 1].max():+.3f}')
        if dr is not None:
            wd = (dr[:, 0] > tc - WINDOW) & (dr[:, 0] < tc)
            if wd.any():
                rep.append(f'   commanded: speed max {dr[wd, 1].max():.2f} m/s, steering '
                           f'{np.degrees(dr[wd, 2]).min():+.1f}..{np.degrees(dr[wd, 2]).max():+.1f} deg')
        before = [f'{lt - tc:+.1f}s {txt}' for lt, _, txt in logs if tc - 10 < lt < tc + 0.5]
        for b in before[-6:]:
            rep.append(f'   log {b}')
        # verdict from the numbers
        lat_err, true_off, est_off = abs(lat[i_last]), abs(off_true[i_last]), abs(off_est[i_last])
        slid = wg.any() and grip[wg].min() < 0.7
        # sim stall: the truth stream (sim frames) stops; the car drives on blind with its last commands
        wt = np.where((tg > tc - WINDOW) & (tg < tc - 0.05))[0]
        gaps = np.diff(tw[wt]) if len(wt) > 1 else np.array([])
        stall = gaps.max() if len(gaps) else 0.0
        if stall > 0.2:
            at = tg[wt][gaps.argmax()] - tc
            rep.append(f'   SIM STALL: no sim frame for {stall:.2f} s at {at:+.2f} s (every car of the sim at once '
                       f'if it shows in the other car too)')
        if stall > 0.2:
            v = 'SIM STALL: the sim froze, the car drove on blind with its last commands'
        elif e_now > 0.15 and est_off < true_off - 0.1:
            v = 'LOCALIZATION: the stack thought it was on its line, it was not'
        elif slid:
            v = 'GRIP LOSS: localization fine, the car stopped turning as steered (slid wide)'
        elif est_off > 0.15:
            v = 'CONTROL: the stack knew it was off its line (pure pursuit / speed / line)'
        else:
            v = 'UNCLEAR: localization and tracking both small just before (contact geometry / line margin?)'
        rep.append(f'   -> {v}')

    # plot
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(4, 1, figsize=(13, 11), sharex=True)
    rt = te - t0
    ax[0].plot(rt, along * 100, lw=.8, label='along-track (+ ahead)')
    ax[0].plot(rt, lat * 100, lw=.8, label='lateral (+ left)')
    ax[0].set_ylabel('loc. error [cm]'); ax[0].legend(loc='upper right')
    ax[1].plot(rt, off_true * 100, lw=.8, label='true')
    ax[1].plot(rt, off_est * 100, lw=.8, label='estimated')
    ax[1].set_ylabel('raceline offset [cm]'); ax[1].legend(loc='upper right')
    if len(slip):
        ax[2].plot(slip_t - t0, slip, lw=.6, color='C3')
    ax[2].set_ylabel('wheel slip [m/s]')
    ax[3].plot(tg - t0, speed_gt, lw=.8, label='true speed')
    if dr is not None:
        ax[3].plot(dr[:, 0] - t0, dr[:, 1], lw=.8, label='commanded')
    ax[3].set_ylabel('speed [m/s]'); ax[3].set_xlabel('race time [s]'); ax[3].legend(loc='upper right')
    for a in ax:
        a.grid(alpha=.3)
        for tc in crashes:
            a.axvline(tc - t0, color='r', lw=1.2)
            a.axvspan(tc - t0 - WINDOW, tc - t0, color='r', alpha=.07)
    fig.suptitle(os.path.basename(path) + ' (red: crash, shaded: 3 s before)')
    fig.tight_layout(); fig.savefig(out_png, dpi=90); plt.close(fig)

    samples = out_png.replace('.png', '_samples.csv')
    with open(samples, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'true_x', 'true_y', 'true_speed', 'err_along', 'err_lat', 'err_yaw_deg',
                    'off_true', 'off_est'])
        for i in range(len(te)):
            w.writerow([f'{te[i] - t0:.3f}', f'{gx[i]:.3f}', f'{gy[i]:.3f}', f'{gv[i]:.3f}', f'{along[i]:.3f}',
                        f'{lat[i]:.3f}', f'{math.degrees(dyaw[i]):.2f}', f'{off_true[i]:.3f}', f'{off_est[i]:.3f}'])
    return crashes, '\n'.join(rep)


def main():
    d, track = sys.argv[1], sys.argv[2]
    report = []
    for k in range(1, 9):
        p = os.path.join(d, f'truth_driver{k}.csv')
        if not os.path.exists(p):
            continue
        try:
            _, rep = analyse(p, track, os.path.join(d, f'truth_driver{k}.png'))
        except Exception as e:   # one car's bad recording never hides the others
            rep = f'analysis failed: {e!r}'
        report.append(f'== car {k}\n{rep}')
    text = '\n'.join(report)
    open(os.path.join(d, 'truth_report.txt'), 'w').write(text + '\n')
    print(text)


if __name__ == '__main__':
    main()
