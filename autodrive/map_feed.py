#!/usr/bin/env python3
"""Show the saved map of the sim's current track in the AutoDRIVE app's Mapping tab.

    autodrive/map_feed.py             follow the sim's track (Ctrl-C stops)
    autodrive/map_feed.py --current   print the sim's current track name and exit

Polls the app's control port (127.0.0.1:4580) for the current track and publishes that track's
tracks/<track>/map.{pgm,yaml} on /map (latched, like map_server) in the club stack container. The
bridge's map_view node sends /map to the Mapping tab, so the bridge must be running (getup.sh starts
it). Switching track in the sim republishes the new track's map (none saved: nothing is published).

The publisher runs in the container through one `docker compose exec`, and gets track names on its
stdin; it exits when this script does (stdin closes).
"""
import json
import os
import socket
import subprocess
import sys
import time

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


def start_publisher():
    cmd = 'source /opt/ros/humble/setup.bash; exec python3 -u -c "$0"'
    return subprocess.Popen(COMPOSE + ['exec', '-T', 'ros', 'bash', '-c', cmd, PUBLISHER],
                            stdin=subprocess.PIPE, text=True)


def main():
    if sys.argv[1:] == ['--current']:
        track = current_track()
        if track is None:
            sys.exit('no answer on the sim control port (is the sim running?)')
        print(track)
        return
    pub, sent = None, None
    while True:
        if pub is None or pub.poll() is not None:
            pub, sent = start_publisher(), None
        track = current_track()
        if track and track != sent:
            try:
                pub.stdin.write(track + '\n')
                pub.stdin.flush()
                sent = track
            except OSError:
                pub = None
        time.sleep(1)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
