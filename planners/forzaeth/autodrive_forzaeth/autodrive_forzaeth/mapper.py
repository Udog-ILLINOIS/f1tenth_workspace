#!/usr/bin/env python3
"""Ground-truth mapper for AutoDRIVE.

Replaces ForzaETH's cartographer + mapping_node for the sim: scans are ray-traced into
a grid at AutoDRIVE's exact pose, so the map lives in AutoDRIVE's world frame and no
localization is needed later (the adapter's odometry is already in that frame).

Writes stack_master/maps/<map_name>/<map_name>.{png,yaml} (+ pf_map copies) in the same
format as global_planner's mapping_node: white = known free track, black = everything
else, with `initial_pose` in the yaml. Saves automatically after one lap, or on Ctrl-C.

With `autopilot:=true` it also drives the car slowly with a disparity extender, so the
lap needs no human.
"""
import math
import os

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Mapper(Node):
    def __init__(self):
        super().__init__('autodrive_mapper')
        p = self.declare_parameter
        self.map_name = p('map_name', 'autodrive_roboracer').value
        self.maps_dir = p('maps_dir', os.path.expanduser('~/ws/src/race_stack/stack_master/maps')).value
        self.res = p('resolution', 0.05).value
        self.size_m = p('size_m', 120.0).value
        self.laser_x = p('laser_x', 0.2733).value
        self.max_range = p('max_range', 9.5).value
        self.kernel = p('filter_kernel_size', 5).value
        self.min_lap_m = p('min_lap_length', 15.0).value
        self.close_radius = p('lap_close_radius', 0.6).value
        self.autopilot = p('autopilot', False).value
        self.ap_speed = p('autopilot_speed', 1.5).value
        self.car_half_width = p('car_half_width', 0.2).value

        n = int(self.size_m / self.res)
        self.n = n
        self.free = np.zeros((n, n), np.uint16)
        self.occ = np.zeros((n, n), np.uint16)
        self.origin = None          # world xy of grid cell (0, 0)
        self.pose = None            # x, y, yaw
        self.start = None
        self.travelled = 0.0
        self.last_xy = None
        self.path = []
        self.saved = False
        self.scan_count = 0

        self.create_subscription(Odometry, '/car_state/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/car_state/scan', self.scan_cb, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.get_logger().info(
            f"Mapping '{self.map_name}'. Drive one full lap"
            + (' (autopilot on)' if self.autopilot else ' (drive manually in the sim)')
            + '; the map saves itself when you get back to the start.')

    def odom_cb(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
        if self.origin is None:
            half = self.size_m / 2.0
            self.origin = np.array([p.position.x - half, p.position.y - half])
            self.start = self.pose
            self.last_xy = self.pose[:2]
            self.get_logger().info(f'Start pose: x={self.pose[0]:.2f} y={self.pose[1]:.2f} yaw={self.pose[2]:.2f}')
            return
        d = math.hypot(self.pose[0] - self.last_xy[0], self.pose[1] - self.last_xy[1])
        if d > 0.05:
            self.travelled += d
            self.last_xy = self.pose[:2]
            self.path.append(self.pose[:2])
        back = math.hypot(self.pose[0] - self.start[0], self.pose[1] - self.start[1])
        if not self.saved and self.travelled > self.min_lap_m and back < self.close_radius:
            self.get_logger().info(f'Lap closed after {self.travelled:.1f} m, saving map.')
            self.save()
            if self.autopilot:
                self.stop_car()

    def scan_cb(self, msg):
        if self.pose is None or self.origin is None:
            return
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        if self.autopilot and not self.saved:
            self.drive(ranges, angles)
        self.scan_count += 1
        if self.saved or self.scan_count % 2:
            return   # half rate is plenty for mapping

        x, y, yaw = self.pose
        lx = x + self.laser_x * math.cos(yaw)
        ly = y + self.laser_x * math.sin(yaw)
        valid = np.isfinite(ranges) & (ranges > 0.05)
        hit = valid & (ranges < self.max_range)
        r = np.where(valid, np.minimum(ranges, self.max_range), 0.0)
        th = yaw + angles

        # free cells: sample every half cell along each ray, stop just short of the hit
        step = self.res * 0.5
        k = np.arange(0.0, self.max_range, step)
        rr = k[None, :]
        mask = rr < (r[:, None] - self.res)
        px = lx + rr * np.cos(th)[:, None]
        py = ly + rr * np.sin(th)[:, None]
        self._accumulate(self.free, px[mask], py[mask])
        # occupied cells: beam endpoints that actually hit something
        self._accumulate(self.occ, lx + r[hit] * np.cos(th[hit]), ly + r[hit] * np.sin(th[hit]))

    def _accumulate(self, grid, xs, ys):
        ix = ((xs - self.origin[0]) / self.res).astype(np.int64)
        iy = ((ys - self.origin[1]) / self.res).astype(np.int64)
        ok = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
        idx = np.unique(iy[ok] * self.n + ix[ok])   # one vote per cell per scan
        flat = grid.reshape(-1)
        flat[idx] = np.minimum(flat[idx].astype(np.uint32) + 1, 65000).astype(np.uint16)

    # ---------------- disparity-extender autopilot ----------------
    def drive(self, ranges, angles):
        r = np.nan_to_num(ranges, nan=0.0, posinf=self.max_range).copy()
        r = np.clip(r, 0.0, self.max_range)
        inc = abs(angles[1] - angles[0])
        # extend each disparity by the car's half width on the far side
        for i in np.where(np.abs(np.diff(r)) > 0.3)[0]:
            near = min(r[i], r[i + 1])
            n_ext = int(math.ceil(math.atan2(self.car_half_width + 0.1, max(near, 0.1)) / inc))
            if r[i] < r[i + 1]:
                r[i + 1:i + 1 + n_ext] = np.minimum(r[i + 1:i + 1 + n_ext], near)
            else:
                lo = max(0, i - n_ext + 1)
                r[lo:i + 1] = np.minimum(r[lo:i + 1], near)
        fov = np.abs(angles) < math.radians(90)
        cand = np.where(fov, r, -1.0)
        best = int(np.argmax(cand))
        target = angles[best]
        steer = max(-0.4, min(0.4, 0.8 * target))
        front = r[np.abs(angles) < math.radians(10)]
        speed = self.ap_speed * (0.6 if abs(steer) > 0.2 or (front.size and front.min() < 1.5) else 1.0)
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.drive.speed = float(speed)
        cmd.drive.steering_angle = float(steer)
        self.drive_pub.publish(cmd)

    def stop_car(self):
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.drive_pub.publish(cmd)

    # ---------------- output ----------------
    def save(self):
        if self.origin is None:
            self.get_logger().error('No data yet, nothing to save.')
            return
        free = (self.free >= 2) & (self.free > 3 * self.occ)
        bw = np.where(free, 255, 0).astype(np.uint8)
        kernel = np.ones((self.kernel, self.kernel), np.uint8)
        bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN, kernel, iterations=2)

        # keep only the free region the car actually drove through
        n_lab, labels = cv2.connectedComponents(bw)
        if self.path and n_lab > 2:
            votes = np.zeros(n_lab, np.int64)
            for x, y in self.path:
                ix = int((x - self.origin[0]) / self.res)
                iy = int((y - self.origin[1]) / self.res)
                if 0 <= ix < self.n and 0 <= iy < self.n:
                    votes[labels[iy, ix]] += 1
            votes[0] = 0
            bw = np.where(labels == int(np.argmax(votes)), 255, 0).astype(np.uint8)
        # fill holes inside the track ribbon smaller than ~0.5 m^2 (single-scan dropouts)
        inv = cv2.bitwise_not(bw)
        n_h, lab_h, stats, _ = cv2.connectedComponentsWithStats(inv)
        for i in range(1, n_h):
            if stats[i, cv2.CC_STAT_AREA] * self.res ** 2 < 0.5:
                bw[lab_h == i] = 255

        # crop to the track plus a margin, adjusting the origin
        ys, xs = np.nonzero(bw)
        if xs.size == 0:
            self.get_logger().error('Map is empty; was the sim streaming scans?')
            return
        m = int(1.0 / self.res)
        x0, x1 = max(xs.min() - m, 0), min(xs.max() + m, self.n - 1)
        y0, y1 = max(ys.min() - m, 0), min(ys.max() + m, self.n - 1)
        bw = bw[y0:y1 + 1, x0:x1 + 1]
        origin = [float(self.origin[0] + x0 * self.res), float(self.origin[1] + y0 * self.res), 0.0]

        out = os.path.join(self.maps_dir, self.map_name)
        os.makedirs(out, exist_ok=True)
        for name in (self.map_name, 'pf_map'):
            cv2.imwrite(os.path.join(out, name + '.png'), cv2.flip(bw, 0))
            with open(os.path.join(out, name + '.yaml'), 'w') as f:
                yaml.dump({'image': name + '.png',
                           'resolution': float(self.res),
                           'origin': origin,
                           'negate': 0,
                           'occupied_thresh': 0.65,
                           'free_thresh': 0.196,
                           'initial_pose': [float(v) for v in self.start]},
                          f, default_flow_style=False)
        self.saved = True
        self.get_logger().info(f'Saved {out}/{self.map_name}.png ({bw.shape[1]}x{bw.shape[0]} px). '
                               'Next: ./forzaeth.sh plan')


def main():
    rclpy.init()
    node = Mapper()
    try:
        while rclpy.ok() and not node.saved:   # mapping_launch shuts down when this exits
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.saved:
            node.save()
        if node.autopilot:
            node.stop_car()
        node.destroy_node()
        rclpy.try_shutdown()
