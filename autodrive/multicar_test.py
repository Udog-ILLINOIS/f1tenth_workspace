#!/usr/bin/env python3
"""Bring-up checks for the multi-car sim (track_builder/track_select/MultiCar.cs), one PASS/FAIL line each.

Needs: the sim started with --cars 2 (./autodrive.sh sim --cars 2), driver 1's club stack container
(sigrobotics-ros-1, bridge on 4567) and driver 2's (sigrobotics_d2-ros-1, bridge on 4568, ROS_DOMAIN_ID 2),
both bridges running, and no race running (this drives the cars by publishing raw throttle/steering).
    python3 autodrive/multicar_test.py [--only NAME ...]
Checks: ghost_switch, connected, ports, truth, independent, lidar_sees_other, lidar_cars_off, ghost, contact, bridge_drop, rate.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONTAINER = {1: 'sigrobotics-ros-1', 2: 'sigrobotics_d2-ros-1'}
NS = '/autodrive/roboracer_1'
results = []


def ctl(cmd):
    out = subprocess.run([os.path.join(ROOT, 'autodrive.sh'), 'ctl'] + cmd.split(), capture_output=True, text=True, timeout=20).stdout
    return json.loads(out)


def cars():
    return {c['car']: c for c in ctl('cars')['cars']}


def ros(driver, cmd, timeout=30, background=False):
    full = f'source /opt/ros/humble/setup.bash; source /root/ws/install/setup.bash; {cmd}'
    args = ['docker', 'exec'] + (['-d'] if background else []) + [CONTAINER[driver], 'bash', '-c', full]
    return subprocess.run(args, capture_output=True, text=True, timeout=timeout).stdout


def drive(driver, throttle, steering, seconds):
    """Publish throttle/steering to that driver's bridge at 20 Hz for `seconds` (blocks)."""
    n = max(1, int(seconds * 20))
    ros(driver, f"ros2 topic pub -r 20 -t {n} {NS}/steering_command std_msgs/msg/Float32 '{{data: {steering}}}' >/dev/null &"
                f" ros2 topic pub -r 20 -t {n} {NS}/throttle_command std_msgs/msg/Float32 '{{data: {throttle}}}' >/dev/null; wait",
        timeout=seconds + 30)


def ips(driver):
    out = ros(driver, f'timeout 5 ros2 topic echo --once {NS}/ips')
    v = {k: float(x) for k, x in (l.split(': ') for l in out.splitlines() if ': ' in l)}
    return v.get('x'), v.get('y')


def scan_front(driver):
    """Smallest range within +-5 deg of straight ahead on that driver's /scan."""
    code = ("import rclpy,math;from rclpy.node import Node;from sensor_msgs.msg import LaserScan;"
            "rclpy.init();n=Node('probe');m=[];n.create_subscription(LaserScan,'/autodrive/roboracer_1/lidar',lambda s:m.append(s),10);"
            "[rclpy.spin_once(n,timeout_sec=0.2) for _ in range(40) if not m];s=m[0];"
            "r=[x for i,x in enumerate(s.ranges) if abs(s.angle_min+i*s.angle_increment)<math.radians(5) and math.isfinite(x)];"
            "print(min(r) if r else 'nan')")
    out = ros(driver, f'python3 -c "{code}"').strip().splitlines()
    return float(out[-1]) if out else float('nan')


def check(name, ok, detail):
    results.append((name, ok))
    print(f"{'PASS' if ok else 'FAIL'}  {name:18s} {detail}", flush=True)


def dist(a, b):
    return math.hypot(a['x'] - b['x'], a['y'] - b['y'])


def place_pair(gap=1.5):
    """Car 1 back at its start, car 2 `gap` m straight behind it, both still."""
    ctl('resetcar 2')
    c = cars()[1]
    ctl(f"place 2 {c['x'] - gap * math.cos(c['yaw']):.3f} {c['y'] - gap * math.sin(c['yaw']):.3f} {c['yaw']:.4f}")
    time.sleep(1.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', nargs='*')
    a = ap.parse_args()
    want = lambda n: not a.only or n in a.only

    ctl('hold off')   # a race_cars.sh run may have left cars held or parked
    for k in cars():
        ctl(f'unpark {k}')
    c = cars()
    if want('connected'):
        ok = len(c) >= 2 and ctl('status')['connected'] and all(c[k].get('connected', True) for k in c if k > 1)
        check('connected', ok, f"{len(c)} cars; car 2 port {c.get(2, {}).get('port')} connected {c.get(2, {}).get('connected')}")
    if want('ports'):
        ok = c.get(1, {}).get('port') == '4567' and c.get(2, {}).get('port') == '4568'
        check('ports', ok, f"car 1 on {c.get(1, {}).get('port')}, car 2 on {c.get(2, {}).get('port')}")

    place_pair()
    if want('truth'):   # each bridge reports its own car's position
        c = cars()
        p1, p2 = ips(1), ips(2)
        e1 = math.hypot(p1[0] - c[1]['x'], p1[1] - c[1]['y'])
        e2 = math.hypot(p2[0] - c[2]['x'], p2[1] - c[2]['y'])
        check('truth', e1 < 0.02 and e2 < 0.02, f"bridge 1 vs car 1 {e1 * 100:.1f} cm, bridge 2 vs car 2 {e2 * 100:.1f} cm (cars {dist(c[1], c[2]):.2f} m apart)")

    if want('independent'):
        b = cars()
        drive(2, 0.05, 0.0, 1.5)
        m = cars()
        moved2, moved1 = dist(b[2], m[2]), dist(b[1], m[1])
        place_pair()
        b = cars()
        drive(1, 0.05, 0.0, 1.5)
        m = cars()
        moved1b, moved2b = dist(b[1], m[1]), dist(b[2], m[2])
        ok = moved2 > 0.1 and moved1 < 0.005 and moved1b > 0.1 and moved2b < 0.005
        check('independent', ok, f"driver 2 cmd: car 2 {moved2:.2f} m, car 1 {moved1 * 1000:.1f} mm; "
                                  f"driver 1 cmd: car 1 {moved1b:.2f} m, car 2 {moved2b * 1000:.1f} mm")
        ros(1, f"timeout 8 ros2 topic pub -r 10 -t 3 /autodrive/reset_command std_msgs/msg/Bool '{{data: true}}' >/dev/null; true")
        time.sleep(1.5)

    for mode in ('ghost', 'contact'):
        if not want(mode) and not (mode == 'ghost' and (want('lidar_sees_other') or want('lidar_cars_off'))):
            continue
        ctl('contact ' + ('off' if mode == 'ghost' else 'on'))
        ctl('lidarcars on')   # a sim started with --ghost 1 has it off; lidar_cars_off switches it itself
        if want('lidar_sees_other'):
            # the range straight ahead must follow the gap one for one (the offset is the lidar's
            # mount plus car 1's rear overhang, the same at every gap)
            r = []
            for gap in (1.5, 2.0, 2.5):
                place_pair(gap)
                r.append(scan_front(2))
            ok = abs(r[1] - r[0] - 0.5) < 0.02 and abs(r[2] - r[1] - 0.5) < 0.02
            check(f'lidar_sees_other[{mode}]', ok, f"car 2 front range at gaps 1.5/2.0/2.5 m: " + "/".join(f"{x:.3f}" for x in r))
        if mode == 'ghost' and want('lidar_cars_off'):
            # at each gap, the range straight ahead with the lidars seeing cars (car 1's rear) and without
            # (past car 1, the track beyond): off must reach clearly farther at every gap. Moving car 2 back
            # adds the same to both, so comparing the gaps alone cannot tell car 1 from a wall.
            on, off = [], []
            for gap in (1.5, 2.0, 2.5):
                place_pair(gap)
                ctl('lidarcars on')
                time.sleep(0.5)
                on.append(scan_front(2))
                ctl('lidarcars off')
                time.sleep(0.5)
                off.append(scan_front(2))
            ctl('lidarcars on')
            ok = all(math.isfinite(a) and math.isfinite(b) and b - a > 0.3 for a, b in zip(on, off))
            check('lidar_cars_off', ok, "front range at gaps 1.5/2.0/2.5 m, lidar sees cars on: "
                  + "/".join(f"{x:.3f}" for x in on) + ", off: " + "/".join(f"{x:.3f}" for x in off))
        place_pair(1.5)
        if not want(mode):
            continue
        b = cars()
        drive(2, 0.1, 0.0, 2.5)   # straight into car 1
        m = cars()
        pushed = dist(b[1], m[1])
        ahead = ((m[2]['x'] - m[1]['x']) * math.cos(m[1]['yaw']) + (m[2]['y'] - m[1]['y']) * math.sin(m[1]['yaw']))
        if mode == 'ghost':
            check('ghost', pushed < 0.005 and ahead > 0, f"car 1 moved {pushed * 1000:.1f} mm; car 2 ended {ahead:+.2f} m along car 1's heading (passed through)")
        else:
            check('contact', pushed > 0.05 or ahead < 0, f"car 1 moved {pushed:.2f} m; car 2 at {ahead:+.2f} m along car 1's heading (blocked/pushed)")
        ros(1, f"timeout 8 ros2 topic pub -r 10 -t 3 /autodrive/reset_command std_msgs/msg/Bool '{{data: true}}' >/dev/null; true")
        time.sleep(1.5)

    if want('ghost_switch'):   # the Menu's Ghost switch changes contact, look and lidar together
        out = {}
        for g in ('on', 'off'):
            ctl('ghost ' + g)
            st = ctl('cars')
            decals = ctl('tree Roboracer 2 1')['tree']
            shown = 'Decals (inactive)' not in decals
            r = []
            for gap in (1.5, 2.0):
                place_pair(gap)
                r.append(scan_front(2))
            seen = abs(r[1] - r[0] - 0.5) < 0.02
            place_pair(1.5)
            b = cars()
            drive(2, 0.1, 0.0, 2.5)
            m = cars()
            pushed = dist(b[1], m[1]) > 0.05
            out[g] = (st['ghost'], not st['contact'], shown, seen, pushed)
            ros(1, "timeout 8 ros2 topic pub -r 10 -t 3 /autodrive/reset_command std_msgs/msg/Bool '{data: true}' >/dev/null; true")
            time.sleep(1.5)
        ok = out['on'] == (True, True, False, False, False) and out['off'] == (False, False, True, True, True)
        fmt = lambda t: f"ghost {t[0]}, no contact {t[1]}, decals {t[2]}, lidar sees {t[3]}, pushes {t[4]}"
        check('ghost_switch', ok, f"on: {fmt(out['on'])} | off: {fmt(out['off'])}")

    if want('rate'):
        time.sleep(2)
        c, s = cars(), ctl('status')
        check('rate', c[1]['frames_per_s'] >= 30 and c[2]['frames_per_s'] >= 30,
              f"render {s['fps']} fps; car 1 {c[1]['frames_per_s']} frames/s, car 2 {c[2]['frames_per_s']} frames/s")

    if want('bridge_drop'):
        ros(2, "pkill -INT -f '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch autodrive_bridge'; sleep 2; "
               "pkill -f '^/usr/bin/python3 /root/ws/install/autodrive_bridge/' ; true")
        time.sleep(3)
        c = cars()
        ok = not c[2]['connected'] and c[1]['connected'] and c[1]['last_frame_s'] < 0.5
        check('bridge_drop', ok, f"bridge 2 down: car 2 connected {c[2]['connected']}; car 1 connected {c[1]['connected']}, last frame {c[1]['last_frame_s']} s ago")
        ros(2, 'ros2 launch autodrive_bridge autodrive_launch.py > /tmp/bridge.log 2>&1', background=True)
        for _ in range(30):
            time.sleep(1)
            if cars()[2]['connected']:
                break
        check('reconnect', cars()[2]['connected'], 'car 2 reconnects when its bridge comes back')

    ctl('grid move')   # place_pair moved car 2's reset pose: back to its grid slot
    failed = [n for n, ok in results if not ok]
    print(f"\n{len(results) - len(failed)}/{len(results)} passed" + (f"; failed: {', '.join(failed)}" if failed else ''))
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
