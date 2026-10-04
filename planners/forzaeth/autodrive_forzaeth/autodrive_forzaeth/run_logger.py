#!/usr/bin/env python3
"""Per-run lap / speed / crash logger for comparing global racelines.

Writes <runs_dir>/<label>_<YYYYmmdd_HHMMSS>/ (forzaeth.sh passes label <map>_<line>_x<speed>;
runs_dir is ~/ws/runs[/<experiment>] = F1Tenth/data[/<experiment>] on the Mac):
  laps.csv       one row per lap: time, valid, start type, speeds, closest wall, the sim's own lap time
  events.csv     every crash: when, where, how fast, what detected it
  telemetry.csv  20 Hz trace: pose, speed, commanded speed/steer, closest wall
  summary.json   rolled-up numbers, rewritten (atomically) after every lap/crash and at exit

Laps: the finish line is the map's initial_pose (the sim's spawn), perpendicular to its
heading. forzaeth.sh resets the car to the spawn before each race, so lap 1 is a standing
start (timed from the moment the car moves) and the laps after it are flying laps. Best and
mean lap time in the summary use flying laps only, when there are any. max_laps counts
flying and aborted laps, not standing ones: max_laps 10 = a standing lap + 10 timed laps.

Crashes: the sim's own collision counter (/autodrive/roboracer_1/collision_count) going
up, or `stuck` (commanded > stuck_cmd m/s but moving < stuck_speed m/s for stuck_time s).
With reset_on_crash (default) every crash resets the car to the spawn over the API. The
interrupted lap is logged as aborted (valid = 0), and timing restarts from the standing
start at the line, so the next lap has start = standing (slower than a flying lap).
"""
import csv
import json
import math
import os
from datetime import datetime

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int32, String


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RunLogger(Node):
    def __init__(self):
        super().__init__('run_logger')
        p = self.declare_parameter
        self.raceline = p('raceline', 'default').value
        self.map_yaml = p('map_yaml', '').value
        runs_dir = p('runs_dir', os.path.expanduser('~/ws/runs')).value
        self.max_laps = p('max_laps', 0).value
        ns = p('vehicle_ns', '/autodrive/roboracer_1').value
        self.reset_on_crash = p('reset_on_crash', True).value
        self.laser_x = p('laser_x', 0.2733).value
        # body box in base_link (rear axle) frame, metres, for the closest-wall metric
        self.body = (p('body_x_min', -0.08).value, p('body_x_max', 0.42).value,
                     p('body_half_width', 0.15).value)
        self.stuck_cmd = p('stuck_cmd', 0.5).value
        self.stuck_speed = p('stuck_speed', 0.15).value
        self.stuck_time = p('stuck_time', 1.5).value
        self.settle_time = p('reset_settle_time', 1.0).value   # ignore detections right after a reset
        self.line_half_len = p('finish_line_half_length', 3.0).value

        with open(self.map_yaml) as f:
            x0, y0, th0 = yaml.safe_load(f)['initial_pose']
        self.line_p = np.array([x0, y0])
        self.line_t = np.array([math.cos(th0), math.sin(th0)])   # direction of travel

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.dir = os.path.join(runs_dir, f'{self.raceline}_{stamp}')
        os.makedirs(self.dir, exist_ok=True)
        self.laps_f = open(os.path.join(self.dir, 'laps.csv'), 'w', newline='')
        self.laps_w = csv.writer(self.laps_f)
        self.lap_cols = ['lap', 'lap_time_s', 'valid', 'start', 'crashes', 'max_speed_mps',
                         'mean_speed_mps', 'min_wall_dist_m', 'distance_m', 'sim_lap_time_s']
        self.laps_w.writerow(self.lap_cols)
        self.events_f = open(os.path.join(self.dir, 'events.csv'), 'w', newline='')
        self.events_w = csv.writer(self.events_f)
        self.events_w.writerow(['t_s', 'lap', 'type', 'x', 'y', 'speed_mps', 'min_wall_dist_m', 'reset'])
        self.tele_f = open(os.path.join(self.dir, 'telemetry.csv'), 'w', newline='')
        self.tele_w = csv.writer(self.tele_f)
        self.tele_w.writerow(['t_s', 'lap', 'x', 'y', 'yaw', 'speed_mps', 'cmd_speed_mps',
                              'cmd_steer_rad', 'min_wall_dist_m'])

        self.t0 = None
        self.pose = None
        self.speed = 0.0
        self.cmd_speed = 0.0
        self.cmd_steer = 0.0
        self.wall = float('inf')
        self.prev_along = None
        self.lap = 0                  # 0 = out-lap, not timed
        self.lap_start = None
        self.lap_kind = 'flying'
        self.lap_stats = self._fresh_lap()
        self.laps = []
        self.crashes = []
        self.coll_count = None
        self.sim_last_lap = None
        self.settle_until = -1e9
        self.stuck_since = None
        # forzaeth.sh resets the car onto the line before a race, so lap 1 is a standing start
        self.arm_standing_start = p('standing_start', True).value
        self.done = False

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Odometry, '/car_state/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/car_state/scan', self.scan_cb, 10)
        self.create_subscription(AckermannDriveStamped, '/drive', self.drive_cb, 10)
        self.create_subscription(Int32, ns + '/collision_count', self.collision_cb, qos)
        self.create_subscription(Float32, ns + '/last_lap_time', self.sim_lap_cb, qos)
        self.reset_pub = self.create_publisher(Bool, '/autodrive/reset_command', qos)
        self.status_pub = self.create_publisher(String, '/run_logger/status', 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f"Logging '{self.raceline}' to {self.dir}"
                               + (' (crash -> reset to start)' if self.reset_on_crash else ''))

    def _fresh_lap(self):
        return {'crashes': 0, 'v_max': 0.0, 'v_sum': 0.0, 'n': 0,
                'wall': float('inf'), 'dist': 0.0, 'last_xy': None}

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------------- inputs ----------------
    def drive_cb(self, msg):
        self.cmd_speed = msg.drive.speed
        self.cmd_steer = msg.drive.steering_angle

    def sim_lap_cb(self, msg):
        self.sim_last_lap = float(msg.data)

    def collision_cb(self, msg):
        if self.coll_count is not None and msg.data > self.coll_count:
            self.crash('collision', self.now())
        self.coll_count = msg.data

    def scan_cb(self, msg):
        r = np.asarray(msg.ranges, dtype=np.float32)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ok = np.isfinite(r) & (r > msg.range_min) & (r > 0.02)
        if not ok.any():
            return
        px = self.laser_x + r[ok] * np.cos(a[ok])
        py = r[ok] * np.sin(a[ok])
        xmin, xmax, hw = self.body
        dx = np.maximum(np.maximum(xmin - px, px - xmax), 0.0)
        dy = np.maximum(np.abs(py) - hw, 0.0)
        self.wall = float(np.min(np.hypot(dx, dy)))

    def odom_cb(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        t = self.now()
        if self.t0 is None:
            self.t0 = t
        xy = np.array(self.pose[:2])

        rel = xy - self.line_p
        along = float(rel @ self.line_t)
        lateral = abs(float(rel[0] * -self.line_t[1] + rel[1] * self.line_t[0]))
        if (self.prev_along is not None and self.prev_along < 0.0 <= along
                and lateral < self.line_half_len and not self.done and t > self.settle_until):
            self.cross_line(t)
        self.prev_along = along

        s = self.lap_stats
        s['v_max'] = max(s['v_max'], self.speed)
        s['v_sum'] += self.speed
        s['n'] += 1
        s['wall'] = min(s['wall'], self.wall)
        if s['last_xy'] is not None and np.linalg.norm(xy - s['last_xy']) < 1.0:   # skip reset jumps
            s['dist'] += float(np.linalg.norm(xy - s['last_xy']))
        s['last_xy'] = xy

    # ---------------- logic ----------------
    def lap_row(self, t, valid):
        s = self.lap_stats
        return {'lap': self.lap,
                'lap_time_s': round(t - self.lap_start, 3),
                'valid': int(valid),
                'start': self.lap_kind,
                'crashes': s['crashes'],
                'max_speed_mps': round(s['v_max'], 3),
                'mean_speed_mps': round(s['v_sum'] / max(s['n'], 1), 3),
                'min_wall_dist_m': round(s['wall'], 3),
                'distance_m': round(s['dist'], 2),
                'sim_lap_time_s': None}

    def record_lap(self, row):
        self.laps.append(row)
        self.laps_w.writerow([row[c] for c in self.lap_cols])
        self.laps_f.flush()
        self.write_summary()

    def counted_laps(self):
        return sum(1 for l in self.laps if l['start'] != 'standing')

    def start_lap(self, t, kind):
        self.lap += 1
        self.lap_start = t
        self.lap_kind = kind
        self.lap_stats = self._fresh_lap()

    def cross_line(self, t):
        if self.arm_standing_start:
            return   # just reset onto the line; tick() starts the standing lap
        if self.lap > 0 and self.lap_start is not None:
            row = self.lap_row(t, valid=self.lap_stats['crashes'] == 0)
            row['sim_lap_time_s'] = self.sim_last_lap
            self.record_lap(row)
            self.get_logger().info(
                f"Lap {self.lap} ({row['start']}): {row['lap_time_s']:.3f} s  vmax {row['max_speed_mps']:.2f}  "
                f"vmean {row['mean_speed_mps']:.2f} m/s  closest wall {row['min_wall_dist_m']:.2f} m")
            if self.max_laps and self.counted_laps() >= self.max_laps:
                self.done = True
                self.get_logger().info(f'{self.max_laps} timed laps logged; run complete.')
                return
        elif self.lap == 0:
            self.get_logger().info('Crossed the line, timing starts (lap 1).')
        self.start_lap(t, 'flying')

    def crash(self, kind, t):
        if self.pose is None or t < self.settle_until or self.done:
            return
        self.lap_stats['crashes'] += 1
        ev = {'t_s': round(t - (self.t0 or t), 2), 'lap': self.lap, 'type': kind,
              'x': round(self.pose[0], 2), 'y': round(self.pose[1], 2),
              'speed_mps': round(self.speed, 2), 'min_wall_dist_m': round(self.wall, 3),
              'reset': int(self.reset_on_crash)}
        self.crashes.append(ev)
        self.events_w.writerow(list(ev.values()))
        self.events_f.flush()
        self.get_logger().warn(f"CRASH ({kind}) lap {self.lap} at x={ev['x']} y={ev['y']}, {ev['speed_mps']} m/s"
                               + (' -> reset' if self.reset_on_crash else ''))
        if self.reset_on_crash:
            if self.lap > 0 and self.lap_start is not None:
                row = self.lap_row(t, valid=False)
                row['start'] = 'aborted'
                self.record_lap(row)
                if self.max_laps and self.counted_laps() >= self.max_laps:
                    self.done = True
            for _ in range(3):
                self.reset_pub.publish(Bool(data=True))
            # back at the spawn, on the line: next forward move starts a standing-start lap
            self.settle_until = t + self.settle_time
            self.lap_start = None
            self.prev_along = None
            self.stuck_since = None
            self.arm_standing_start = True
        self.write_summary()

    def tick(self):
        if self.pose is None or self.t0 is None:
            return
        t = self.now()
        if self.arm_standing_start and t > self.settle_until and not self.done:
            # the spawn sits on the finish line: start timing as soon as the car moves off it
            if self.speed > 0.2:
                self.arm_standing_start = False
                self.start_lap(t, 'standing')
                self.get_logger().info(f'Standing start, lap {self.lap}')
        if t > self.settle_until and self.cmd_speed > self.stuck_cmd and self.speed < self.stuck_speed:
            self.stuck_since = self.stuck_since or t
            if t - self.stuck_since > self.stuck_time:
                self.stuck_since = None
                self.crash('stuck', t)
        else:
            self.stuck_since = None

        self.tele_w.writerow([round(t - self.t0, 3), self.lap, round(self.pose[0], 3),
                              round(self.pose[1], 3), round(self.pose[2], 4), round(self.speed, 3),
                              round(self.cmd_speed, 3), round(self.cmd_steer, 4), round(self.wall, 3)])
        self.status_pub.publish(String(data=json.dumps(
            {'raceline': self.raceline, 'lap': self.lap, 'speed': round(self.speed, 2),
             'crashes': len(self.crashes), 'laps_done': len(self.laps)})))

    def write_summary(self):
        valid = [l for l in self.laps if l['valid']]
        flying = [l['lap_time_s'] for l in valid if l['start'] == 'flying']
        times = [l['lap_time_s'] for l in valid]
        summary = {
            'raceline': self.raceline,
            'laps_completed': len([l for l in self.laps if l['start'] != 'aborted']),
            'valid_laps': len(valid),
            'crashes': len(self.crashes),
            'best_lap_s': min(flying or times) if times else None,
            'best_lap_is_flying': bool(flying),
            'mean_lap_s': round(float(np.mean(flying or times)), 3) if times else None,
            'std_lap_s': round(float(np.std(flying or times)), 3) if len(flying or times) > 1 else None,
            'max_speed_mps': max((l['max_speed_mps'] for l in self.laps), default=None),
            'mean_speed_mps': round(float(np.mean([l['mean_speed_mps'] for l in valid])), 3) if valid else None,
            'min_wall_dist_m': min((l['min_wall_dist_m'] for l in valid), default=None),
            'laps': self.laps,
            'crash_events': self.crashes,
        }
        tmp = os.path.join(self.dir, 'summary.json.tmp')
        with open(tmp, 'w') as f:
            json.dump(summary, f, indent=2)
        os.replace(tmp, os.path.join(self.dir, 'summary.json'))

    def close(self):
        self.write_summary()
        for f in (self.laps_f, self.events_f, self.tele_f):
            f.close()


def main():
    rclpy.init()
    node = RunLogger()
    try:
        while rclpy.ok() and not node.done:   # race_launch shuts down when this exits
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.get_logger().info(f'Run saved to {node.dir}')
        node.destroy_node()
        rclpy.try_shutdown()
