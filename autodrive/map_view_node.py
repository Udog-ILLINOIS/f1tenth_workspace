#!/usr/bin/env python3
"""The stack's map, pose and overtaking lines, streamed to the AutoDRIVE app's Mapping tab (sim only).

Runs in the club stack container, started by autodrive/map_feed.py, which writes the sim's track name to its
stdin. Connects to the app on host.docker.internal:4581 (MapView.cs) and reconnects when it restarts.

Lines drawn, on top of each other in this order:
  grey    tracks/<track>/raceline.csv
  cyan    tracks/<track>/segments/[ABC]_*.csv, plan_segment's viable mintime segments (offline, re-read every 5 s)
  red     a spliner candidate refused (/local_path/candidate_*, /local_path/candidate_status)
  yellow  a spliner candidate it could drive, orange if only in a tight gap
  green   /local_path where it leaves the raceline: the line the car is steering on
From segment_planner_node (/segments/lines): cyan viable, red solved but refused, green the one chosen
"""
import glob
import json
import math
import os
import socket
import struct
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped  # noqa: F401
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

TRACKS = os.environ.get('TRACKS_DIR', '/root/ws/tracks')
GREY, CYAN, RED = (170, 170, 170, 170), (40, 200, 240, 230), (235, 60, 60, 255)
YELLOW, ORANGE, GREEN = (240, 220, 50, 255), (250, 150, 30, 255), (60, 230, 90, 255)
MAX_AGE = 0.5   # s: an older /local_path or candidate is not drawn


def frame(kind, payload=b''):
    return kind + struct.pack('<I', len(payload)) + payload


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def line(color, xy):
    xy = np.asarray(xy, dtype=np.float32).reshape(-1, 2)
    return struct.pack('<BBBBI', *color, len(xy)) + xy.tobytes()


def load_line(path):
    try:
        return np.loadtxt(path, comments='#', delimiter=';', ndmin=2)[:, 1:3]
    except (OSError, ValueError, IndexError):
        return None


def path_xy(msg):
    return np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses], dtype=np.float32).reshape(-1, 2)


class MapView(Node):
    def __init__(self):
        super().__init__('map_view')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.sock, self.next_connect, self.last_heartbeat = None, 0.0, 0.0
        self.map_msg, self.map_dirty = None, False
        self.track, self.raceline, self.segments, self.loaded = None, None, [], 0.0
        self.local, self.local_t, self.valid = None, 0.0, True
        self.cand = {'left': (None, 0.0), 'right': (None, 0.0)}
        self.status = {}
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, latched)
        self.create_subscription(Path, '/local_path', self.local_cb, 10)
        self.create_subscription(Bool, '/local_path/valid', self.valid_cb, 10)
        self.create_subscription(Path, '/local_path/candidate_left', lambda m: self.cand_cb('left', m), 10)
        self.create_subscription(Path, '/local_path/candidate_right', lambda m: self.cand_cb('right', m), 10)
        self.create_subscription(String, '/local_path/candidate_status', self.status_cb, 10)
        self.seg_lines, self.seg_t = [], 0.0
        self.pose_stamp = (None, 0.0)   # (TF map -> base_link stamp [ns], monotonic time it was first seen)
        self.create_subscription(String, '/segments/lines', self.seg_cb, 10)
        self.create_timer(0.02, self.tick)
        self.create_timer(0.1, self.send_lines)

    def map_cb(self, msg):
        self.map_msg, self.map_dirty = msg, True

    def local_cb(self, msg):
        self.local, self.local_t = path_xy(msg), time.monotonic()

    def valid_cb(self, msg):
        self.valid = msg.data

    def cand_cb(self, side, msg):
        self.cand[side] = (path_xy(msg), time.monotonic())

    def status_cb(self, msg):
        self.status = dict(kv.split('=') for kv in msg.data.split())

    def seg_cb(self, msg):
        self.seg_lines, self.seg_t = json.loads(msg.data)['lines'], time.monotonic()

    def set_track(self, track):
        self.track, self.loaded = track, 0.0

    def load(self):
        if self.track is None or time.monotonic() - self.loaded < 5.0:
            return
        self.loaded = time.monotonic()
        d = os.path.join(TRACKS, self.track)
        self.raceline = load_line(os.path.join(d, 'raceline.csv'))
        found = (load_line(f) for f in sorted(glob.glob(os.path.join(d, 'segments', '[ABC]_*.csv'))))
        self.segments = [s for s in found if s is not None]

    def connect(self):
        now = time.monotonic()
        if now < self.next_connect:
            return
        self.next_connect = now + 1.0
        try:
            s = socket.create_connection(('host.docker.internal', 4581), timeout=0.2)
        except OSError:
            return
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.settimeout(2.0)
        self.sock = s
        self.map_dirty = self.map_msg is not None
        print('[map_view] connected to the Mapping tab', flush=True)

    def send(self, data):
        if self.sock is None:
            return False
        try:
            self.sock.sendall(data)
            return True
        except OSError:
            self.sock.close()
            self.sock = None
            return False

    def send_lines(self):
        self.load()
        out = []
        if self.raceline is not None:
            out.append(line(GREY, self.raceline))
        out += [line(CYAN, s) for s in self.segments]
        now = time.monotonic()
        if now - self.seg_t < 2.0:
            order = {'refused': 0, 'viable': 1, 'chosen': 2}
            state_color = {'refused': RED, 'viable': CYAN, 'chosen': GREEN}
            for ln in sorted(self.seg_lines, key=lambda x: order[x['state']]):
                out.append(line(state_color[ln['state']], ln['xy']))
        colors = {'ok': YELLOW, 'tight': ORANGE, 'rejected': RED}
        for side, (xy, t) in self.cand.items():
            if xy is not None and len(xy) > 1 and now - t < MAX_AGE:
                out.append(line(colors.get(self.status.get(side), RED), xy))
        if self.valid and self.local is not None and now - self.local_t < MAX_AGE and self.raceline is not None:
            rl = self.raceline[:len(self.local)]
            k = min(len(rl), len(self.local))
            moved = np.hypot(*(self.local[:k] - rl[:k]).T) > 0.01
            if moved.any():
                keep = moved | np.roll(moved, 1) | np.roll(moved, -1)
                out.append(line(GREEN, self.local[:k][keep]))
        self.send(frame(b'L', struct.pack('<I', len(out)) + b''.join(out)))

    def tick(self):
        if self.sock is None:
            self.connect()
            if self.sock is None:
                return
        if self.map_dirty:
            self.map_dirty = False
            m = self.map_msg
            cells = np.asarray(m.data, dtype=np.int8)
            head = struct.pack('<IIffff', m.info.width, m.info.height, m.info.resolution, m.info.origin.position.x,
                               m.info.origin.position.y, yaw_of(m.info.origin.orientation))
            if not self.send(frame(b'M', head + cells.tobytes())):
                return
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time(), timeout=Duration(seconds=0.0))
            # age: wall time since the pose's stamp last changed, not this node's clock minus the stamp: the stack
            # may run on the sim's /clock (use_sim_time) while this node runs on the wall clock
            stamp = Time.from_msg(t.header.stamp).nanoseconds
            if stamp != self.pose_stamp[0]:
                self.pose_stamp = (stamp, time.monotonic())
            age = time.monotonic() - self.pose_stamp[1]
            if age <= 0.5:
                tr = t.transform
                if not self.send(frame(b'P', struct.pack('<ffff', tr.translation.x, tr.translation.y,
                                                         yaw_of(tr.rotation), max(age, 0.0)))):
                    return
        except TransformException:
            pass
        now = time.monotonic()
        if now - self.last_heartbeat >= 1.0:
            self.last_heartbeat = now
            self.send(frame(b'H'))


rclpy.init()
node = MapView()


def read():
    for text in sys.stdin:
        node.set_track(text.strip())
    rclpy.try_shutdown()


threading.Thread(target=read, daemon=True).start()
try:
    rclpy.spin(node)
except Exception:
    pass
