#!/usr/bin/env python3
"""Show the saved map of the sim's current track in the AutoDRIVE app's Mapping tab.

    autodrive/map_feed.py             follow the sim's track (Ctrl-C stops)
    autodrive/map_feed.py --current   print the sim's current track name and exit

Polls the app's control port (127.0.0.1:4580) for the current track and publishes that track's
tracks/<track>/map.{pgm,yaml} on /map (latched, like map_server) in the club stack container.
Switching track in the sim republishes the new track's map (none saved: nothing is published).

It also runs autodrive/map_view_node.py, which sends that map, the car's pose and the overtaking lines
to the Mapping tab.

Both run in the container through one `docker compose exec` each, and get track names on their
stdin; they exit when this script does (stdin closes).

One feed at a time: a new feed stops any older one (pid in $TMPDIR/f1tenth_map_feed.pid), and a feed
whose parent is gone (getup.sh stopped, reparented to launchd) exits. Two feeds each keep a map_view
client on the app's port 4581, a newer client replaces the older there, and they then take turns
forever: 2026-11-12..14 two orphaned feeds made the app rebuild its map ~690 times, stalling the sim
(frames to the bridges 30-45/s, 150-200 gaps > 60 ms per race, crashes) until they were killed.
"""
import signal
import json
import os
import socket
import subprocess
import sys
import tempfile
import time

PIDFILE = os.path.join(tempfile.gettempdir(), 'f1tenth_map_feed.pid')
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COMPOSE = ['docker', 'compose', '-f', os.path.join(ROOT, 'planners', 'sigrobotics', 'docker-compose.yml')]
CTL = ('127.0.0.1', int(os.environ.get('AUTODRIVE_CONTROL_PORT', 4580)))

# Runs in the container: reads a track name per line, publishes its saved map on /map (latched).
PUBLISHER = r'''
import os, sys, threading
import numpy as np, rclpy, yaml
from PIL import Image
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

TRACKS = os.environ.get('TRACKS_DIR', '/root/ws/tracks')

def grid(track):
    d = os.path.join(TRACKS, track)
    if not os.path.isfile(os.path.join(d, 'map.yaml')):
        return None
    meta = yaml.safe_load(open(os.path.join(d, 'map.yaml')))
    pix = np.asarray(Image.open(os.path.join(d, meta['image'])).convert('L'), dtype=np.float32)
    p = pix / 255.0 if meta.get('negate', 0) else (255.0 - pix) / 255.0
    cells = np.full(pix.shape, -1, dtype=np.int8)
    cells[p > meta.get('occupied_thresh', 0.65)] = 100
    cells[p < meta.get('free_thresh', 0.25)] = 0
    m = OccupancyGrid()
    m.header.frame_id = 'map'
    m.info.resolution = float(meta['resolution'])
    m.info.height, m.info.width = cells.shape
    ox, oy, yaw = meta['origin']
    m.info.origin.position.x, m.info.origin.position.y = float(ox), float(oy)
    m.info.origin.orientation.z, m.info.origin.orientation.w = np.sin(yaw / 2), np.cos(yaw / 2)
    m.data = cells[::-1].ravel().tolist()   # row 0 at the origin
    return m

rclpy.init()
node = rclpy.create_node('sim_map_feed')
pub = node.create_publisher(OccupancyGrid, '/map', QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))

def read():
    for line in sys.stdin:
        track = line.strip()
        m = grid(track)
        if m is None:
            print(f'[map_feed] {track}: no saved map in {TRACKS}', flush=True)
            continue
        m.header.stamp = node.get_clock().now().to_msg()
        pub.publish(m)
        print(f'[map_feed] {track}: map {m.info.width}x{m.info.height} on /map', flush=True)
    rclpy.shutdown()

threading.Thread(target=read, daemon=True).start()
try:
    rclpy.spin(node)
except Exception:
    pass
'''


def ctl(cmd):
    """One control port command -> its JSON answer, or None when the app is not running."""
    try:
        with socket.create_connection(CTL, timeout=2) as s:
            s.sendall(cmd.encode() + b'\n')
            data = b''
            while not data.endswith(b'\n'):
                chunk = s.recv(65536)
                if not chunk:
                    break
                data += chunk
        return json.loads(data)
    except (OSError, ValueError):
        return None


def current_track():
    """The app's current track as a tracks/ folder name ('porto'), or None."""
    st = ctl('tracks')
    if not st or not st.get('current'):
        return None
    cur = st['current']
    for t in st.get('tracks', []):
        if cur in (t.get('name'), t.get('display')):
            return t['name']
    return cur.lower()


def start_publisher(code=PUBLISHER):
    cmd = 'source /opt/ros/humble/setup.bash; exec python3 -u -c "$0"'
    return subprocess.Popen(COMPOSE + ['exec', '-T', 'ros', 'bash', '-c', cmd, code],
                            stdin=subprocess.PIPE, text=True)


def start_view():
    with open(os.path.join(ROOT, 'autodrive', 'map_view_node.py')) as f:
        return start_publisher(f.read())


def is_feed(pid):
    """pid is a running map_feed.py (not a reused pid)."""
    try:
        out = subprocess.run(['ps', '-o', 'command=', '-p', str(pid)], capture_output=True, text=True).stdout
    except OSError:
        return False
    # a python running this script ("python3 .../autodrive/map_feed.py"), not a shell whose command line
    # merely mentions it, nor a one-shot --current
    w = out.split()
    return (len(w) >= 2 and 'ython' in os.path.basename(w[0]) and w[1].endswith('autodrive/map_feed.py')
            and '--current' not in w)


def take_over():
    """Stop every other feed (the pid file's, and any stray one), then record ours."""
    others = set()
    try:
        others.add(int(open(PIDFILE).read().split()[0]))
    except (OSError, ValueError, IndexError):
        pass
    try:   # strays from before the pid file existed
        out = subprocess.run(['pgrep', '-f', 'autodrive/map_feed.py'], capture_output=True, text=True).stdout
        others.update(int(x) for x in out.split())
    except (OSError, ValueError):
        pass
    others.discard(os.getpid())
    for pid in [p for p in others if is_feed(p)]:
        print(f'[map_feed] stopping the older map feed (pid {pid}): one feed at a time', flush=True)
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            continue
        for _ in range(30):
            if not is_feed(pid):
                break
            time.sleep(0.1)
        else:
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
    with open(PIDFILE, 'w') as f:
        f.write(f'{os.getpid()}\n')


def main():
    if sys.argv[1:] == ['--current']:
        track = current_track()
        if track is None:
            sys.exit('no answer on the sim control port (is the sim running?)')
        print(track)
        return
    take_over()
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))   # -> finally: the container side stops too
    procs = []
    try:
        feed(procs)
    finally:
        for p in procs:
            if p is not None and p.poll() is None:
                try:
                    p.stdin.close()   # the container side exits on EOF
                except OSError:
                    pass
                try:
                    p.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    p.kill()
        try:
            if open(PIDFILE).read().split()[0] == str(os.getpid()):
                os.remove(PIDFILE)
        except (OSError, IndexError):
            pass


def feed(procs):
    parent = os.getppid()
    pub, view, sent = None, None, None
    while True:
        if os.getppid() != parent:   # whoever started us (getup.sh) is gone: never live on as an orphan
            print('[map_feed] parent gone: exiting', flush=True)
            return
        if pub is None or pub.poll() is not None:
            pub, sent = start_publisher(), None
        if view is None or view.poll() is not None:
            view, sent = start_view(), None
        procs[:] = [pub, view]
        track = current_track()
        if track and track != sent:
            try:
                for p in (pub, view):
                    p.stdin.write(track + '\n')
                    p.stdin.flush()
                sent = track
            except OSError:
                pub = view = None
        time.sleep(1)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
