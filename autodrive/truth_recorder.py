"""Truth recorder: runs inside a driver's container (race_cars.sh starts one per driver) and prints CSV
lines to stdout, all stamped with sim time (the bridge's frame stamps):
  est,t,x,y,yaw                    /localized_pose: the stack's pose estimate (map frame, base_link)
  gt,t,x,y,yaw,vx,vy,wz,rx         /ground_truth/odom: the sim's true pose and velocity (world frame);
                                   rx = when it arrived here (wall clock): with sim-time stamps
                                   (fast_bridge SimClock) a frame hitch shows only in rx
  odom,t,vx,wz                     /odom: wheel odometry as the stack sees it (slip shows here)
  drive,t,speed,steer              /drive: what the controller commanded (t = newest gt stamp, as cmd:
                                   one clock with gt; /drive's own stamp is pure pursuit's wall clock)
  cmd,t,throttle,steering          what the sim was told (/autodrive/roboracer_1/*_command, t = newest gt stamp)
  coll,t,n                         collision_count changes (t = newest gt stamp)
  lap,t,n                          lap_count changes
  log,t,level,text                 the localizer's own log lines (re-anchors, resets, jumps)
Read-only: it only subscribes, the stack is untouched. truth_analysis.py turns it into errors per crash.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from std_msgs.msg import Float32, Int32


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def st(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class Recorder(Node):
    def __init__(self):
        super().__init__('truth_recorder')
        self.t = 0.0   # newest sim stamp seen
        self.last = {}
        q = qos_profile_sensor_data
        self.create_subscription(PoseStamped, '/localized_pose', self.est, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.gt, q)
        self.create_subscription(Odometry, '/odom', self.odom, q)
        self.create_subscription(AckermannDriveStamped, '/drive', self.drive, q)
        self.create_subscription(Int32, '/autodrive/roboracer_1/collision_count', lambda m: self.change('coll', m), q)
        self.create_subscription(Int32, '/autodrive/roboracer_1/lap_count', lambda m: self.change('lap', m), q)
        self.create_subscription(Log, '/rosout', self.log, 100)
        self.thr = self.steer = float('nan')
        self.create_subscription(Float32, '/autodrive/roboracer_1/throttle_command', self.thr_cb, q)
        self.create_subscription(Float32, '/autodrive/roboracer_1/steering_command', self.steer_cb, q)

    def thr_cb(self, m):
        self.thr = float(m.data); self.out('cmd', self.t, self.thr, self.steer)

    def steer_cb(self, m):
        self.steer = float(m.data)

    def out(self, *f):
        sys.stdout.write(','.join(f'{v:.4f}' if isinstance(v, float) else str(v) for v in f) + '\n')

    def est(self, m):
        p = m.pose
        self.out('est', st(m.header), p.position.x, p.position.y, yaw(p.orientation))

    def gt(self, m):
        self.t = st(m.header)
        p, v = m.pose.pose, m.twist.twist
        self.out('gt', self.t, p.position.x, p.position.y, yaw(p.orientation), v.linear.x, v.linear.y, v.angular.z,
                 time.time())

    def odom(self, m):
        self.out('odom', st(m.header), m.twist.twist.linear.x, m.twist.twist.angular.z)

    def drive(self, m):
        self.out('drive', self.t, float(m.drive.speed), float(m.drive.steering_angle))

    def change(self, kind, m):
        if self.last.get(kind) != m.data:
            self.last[kind] = m.data
            self.out(kind, self.t, m.data)

    def log(self, m):
        if 'mcl' in m.name or 'locali' in m.name or 'pure_pursuit' in m.name:
            text = m.msg.replace('\n', ' ').replace(',', ';')
            self.out('log', self.t, m.level, f'{m.name}: {text}')


def main():
    rclpy.init()
    n = Recorder()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        sys.stdout.flush()


if __name__ == '__main__':
    main()
