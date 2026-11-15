"""Ghost replay: play recorded runs back in the sim, all at once, on top of each other.

  python3 autodrive/ghost_replay.py DIR [DIR ...] [--speed S]
    DIR: a race folder (data/head_to_head/...) with truth_driver<k>.csv (truth_recorder.py), e.g. the solo
    runs of ./race_cars.sh --solo. Every run in them is replayed together, each from its own release
    (first motion) at t = 0, by its own car: car k's run on car k (a clash, e.g. two runs of car 1, takes
    the next free car). --speed: playback rate (1 = real time, 0.5 = half speed).

The cars are parked (kinematic, their bridges ignored) and moved every frame to the recorded true pose
(ctl place, the sim's own ground-truth point), interpolated on the recorded sim-clock stamps. Crashes
replay as they happened (the sim's respawn shows as a jump to the start). At the end every replayed car
is put back on its start pose (car 1's own start, cars 2.. the grid: ctl grid) and unparked.
Needs a sim started with --cars N (N >= the runs); Ghost on so the cars pass through each other.
"""
import bisect
import csv
import math
import os
import socket
import sys
import threading
import time

CTL = ('127.0.0.1', 4580)


def ctl(line, timeout=5.0):
    with socket.create_connection(CTL, timeout=timeout) as s:
        s.sendall((line + '\n').encode())
        buf = b''
        while not buf.endswith(b'\n'):
            part = s.recv(65536)
            if not part:
                break
            buf += part
    return buf.decode().strip()


def load_run(path):
    """(t, x, y, yaw) of the true pose from release on, t from 0; lap events; lap times from laps.csv."""
    rows = []
    laps = []
    for r in csv.reader(open(path)):
        if not r:
            continue
        if r[0] == 'gt' and len(r) >= 8:
            t, x, y, yaw, vx, vy = map(float, r[1:7])
            rows.append((t, x, y, yaw, math.hypot(vx, vy)))
        elif r[0] == 'lap' and len(r) >= 3:
            laps.append((float(r[1]), int(r[2])))
    rows.sort()
    go = next((i for i, r in enumerate(rows) if r[4] > 0.05), None)
    if go is None:
        return None
    i0 = max(0, go - 1)
    t0 = rows[i0][0]
    ts = [r[0] - t0 for r in rows[i0:]]
    pose = [(r[1], r[2], r[3]) for r in rows[i0:]]
    # unwrap yaw so interpolation never spins the long way round
    out, prev = [], None
    for x, y, a in pose:
        if prev is not None:
            a = prev + math.remainder(a - prev, 2 * math.pi)
        out.append((x, y, a)); prev = a
    start = rows[0][1:4]   # where the car stood before its release (its start pose)
    return {'t': ts, 'pose': out, 'laps': [(t - t0, n) for t, n in laps if t >= t0], 'start': start}


def lap_times(d, k):
    p = os.path.join(d, 'laps.csv')
    if not os.path.exists(p):
        return []
    return [r['time_s'] for r in csv.DictReader(open(p)) if r['car'] == str(k)]


def pose_at(run, t):
    ts, P = run['t'], run['pose']
    if t <= ts[0]:
        return P[0]
    if t >= ts[-1]:
        return P[-1]
    i = bisect.bisect_right(ts, t)
    a, b = ts[i - 1], ts[i]
    w = 0.0 if b <= a else (t - a) / (b - a)
    if b - a > 0.3:   # a respawn (crash) or a long gap: jump, never slide through the walls
        w = 0.0 if w < 1.0 else 1.0
    return tuple(p + w * (q - p) for p, q in zip(P[i - 1], P[i]))


def main():
    args = sys.argv[1:]
    speed = 1.0
    if '--speed' in args:
        i = args.index('--speed'); speed = float(args[i + 1]); del args[i:i + 2]
    if not args or speed <= 0:
        sys.exit(__doc__)
    import json
    cars = json.loads(ctl('cars'))
    n_cars = len(cars.get('cars', []))
    runs, used = [], set()
    for d in args:
        for k in range(1, 9):
            p = os.path.join(d, f'truth_driver{k}.csv')
            if not os.path.exists(p):
                continue
            run = load_run(p)
            if run is None:
                print(f'{p}: the car never moved, skipped'); continue
            car = k if k not in used and k <= n_cars else next((c for c in range(1, n_cars + 1) if c not in used), None)
            if car is None:
                sys.exit(f'more runs than cars in the sim ({n_cars}): start it with --cars {len(runs) + 1}')
            used.add(car)
            run.update(car=car, src=f'{os.path.basename(os.path.normpath(d))} car {k}', laps_s=lap_times(d, k))
            runs.append(run)
    if not runs:
        sys.exit('no runs found (truth_driver<k>.csv)')
    if not cars.get('ghost'):
        print('note: Ghost is off, the replayed cars will hit each other: ./autodrive.sh ctl ghost on')
    for r in runs:
        print(f"car {r['car']}: {r['src']}, {r['t'][-1]:.1f} s, laps {' '.join(r['laps_s']) or '?'}")
        ctl(f"park {r['car']}")
    for r in runs:   # everyone on their start, then go together
        x, y, a = r['pose'][0]; ctl(f"place {r['car']} {x:.4f} {y:.4f} {a:.4f}")
    print('== replay in 2 s' + (f' at {speed:g}x' if speed != 1 else '')); time.sleep(2)

    t_start = time.monotonic()
    end = max(r['t'][-1] for r in runs)
    stop = threading.Event()

    def play(r):
        while not stop.is_set():
            t = (time.monotonic() - t_start) * speed
            x, y, a = pose_at(r, t)
            try:
                ctl(f"place {r['car']} {x:.4f} {y:.4f} {math.remainder(a, 2 * math.pi):.4f}", timeout=2.0)
            except OSError:
                time.sleep(0.05)
            if t > r['t'][-1]:
                return
            time.sleep(0.004)

    th = [threading.Thread(target=play, args=(r,), daemon=True) for r in runs]
    for x in th:
        x.start()
    shown = {r['car']: 0 for r in runs}
    try:
        while any(x.is_alive() for x in th):
            t = (time.monotonic() - t_start) * speed
            for r in runs:
                done = [n for tl, n in r['laps'] if tl <= t]
                if done and done[-1] > shown[r['car']]:
                    shown[r['car']] = done[-1]
                    print(f"  {t:6.2f} s  car {r['car']} lap {done[-1]}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print('\n== stopped')
    stop.set()
    for x in th:
        x.join(timeout=3)
    # results: total sim time per run from laps.csv
    print('== result (sim lap times)')
    tot = []
    for r in runs:
        ls = [float(v) for v in r['laps_s'] if v not in ('?', '')]
        if ls:
            tot.append((sum(ls), r['car']))
        print(f"car {r['car']}: laps {' '.join(r['laps_s']) or '?'}" + (f", total {sum(ls):.2f} s" if ls else ''))
    if len(tot) > 1:
        tot.sort()
        print(f"first: car {tot[0][1]}, by {tot[1][0] - tot[0][0]:.2f} s")
    # put the cars back: car 1 on its own start, cars 2.. on the grid, all reset and unparked
    for r in runs:
        if r['car'] == 1:
            x, y, a = r['start']; ctl(f'place 1 {x:.4f} {y:.4f} {a:.4f}')
    ctl('grid')
    for r in runs:
        ctl(f"resetcar {r['car']}")
    print('== cars back on their starts')


if __name__ == '__main__':
    main()
