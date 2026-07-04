#!/usr/bin/env python3
"""
stuck_monitor_node.py  ---  Phat hien KET BANH / TRUOT (9-DOF + pose)
=====================================================================
Giai quyet van de so 1: xe ket khe gach, banh quay tai cho, khong tien.

Tin hieu phat hien:
  1) POSE KHONG TIEN (chinh): trong mot cua so thoi gian, dang lenh chay
     (v_cmd > nguong) nhung pose (/agv/odom) gan nhu khong dich chuyen.
     Day la dau hieu ket dang tin nhat vi luc ket camera van thay marker
     nen pose dung im.
  2) TRUOT LECH (phu, dung IMU): dang lenh di THANG (w_cmd ~ 0) nhung
     gyro.z that lai lon -> mot banh truot lam xe xoay ngoai y muon.
  Ngoai ra log them rung dong (phuong sai gia toc) de tien tune sau.

Vao : /cmd_vel, /agv/odom, /imu/data
Ra  : /agv/stuck (std_msgs/Bool)  -> True = dang ket
"""

import math
import statistics
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class StuckMonitorNode(Node):

    def __init__(self):
        super().__init__('stuck_monitor_node')

        # ---------- Tham so (tune tai day) ----------
        self.window        = 1.5     # s, cua so xet tien do
        self.v_cmd_min     = 0.03    # m/s, coi nhu "dang lenh chay"
        self.min_expected  = 0.04    # m, quang duong toi thieu ky vong moi xet
        self.progress_ratio= 0.3     # actual < ratio*expected -> ket
        self.slip_gyro     = 0.5     # rad/s, gyro.z lon khi di thang -> truot
        self.rate_hz       = 10.0

        # ---------- ROS I/O ----------
        self.stuck_pub = self.create_publisher(Bool, '/agv/stuck', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(Odometry, '/agv/odom', self.odom_cb, 20)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 50)

        # ---------- Trang thai ----------
        self.x = self.y = 0.0
        self.have_pose = False
        self.v_cmd = self.w_cmd = 0.0
        self.gyro_z = 0.0
        self.accel_buf = deque(maxlen=20)
        self.buf = deque()              # (t, x, y, v_cmd)
        self.was_stuck = False

        self.create_timer(1.0 / self.rate_hz, self.check)
        self.get_logger().info('Stuck monitor khoi tao.')

    # =================================================================
    def cmd_cb(self, msg):
        self.v_cmd = float(msg.linear.x)
        self.w_cmd = float(msg.angular.z)

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.have_pose = True

    def imu_cb(self, msg):
        self.gyro_z = float(msg.angular_velocity.z)
        a = msg.linear_acceleration
        self.accel_buf.append(math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # =================================================================
    def check(self):
        if not self.have_pose:
            return

        t = self.now()
        self.buf.append((t, self.x, self.y, self.v_cmd))
        while self.buf and (t - self.buf[0][0]) > self.window:
            self.buf.popleft()

        stuck = False
        reason = ''

        if len(self.buf) >= 5:
            t0, x0, y0, _ = self.buf[0]
            elapsed = t - t0
            if elapsed >= self.window * 0.8:
                mean_v = sum(b[3] for b in self.buf) / len(self.buf)
                expected = mean_v * elapsed
                actual = math.hypot(self.x - x0, self.y - y0)
                if (mean_v > self.v_cmd_min and expected > self.min_expected
                        and actual < self.progress_ratio * expected):
                    stuck = True
                    reason = f'POSE khong tien (mong {expected:.2f}m, thuc {actual:.2f}m)'

        # Truot lech (IMU): di thang nhung dang xoay
        if (self.v_cmd > self.v_cmd_min and abs(self.w_cmd) < 0.1
                and abs(self.gyro_z) > self.slip_gyro):
            stuck = True
            reason = f'TRUOT (gyro.z={self.gyro_z:.2f} rad/s khi di thang)'

        # Rung dong (chi de log/tune)
        vib = statistics.pstdev(self.accel_buf) if len(self.accel_buf) >= 5 else 0.0

        self.stuck_pub.publish(Bool(data=stuck))

        if stuck and not self.was_stuck:
            self.get_logger().warn(f'KET: {reason} | vib={vib:.2f}')
        elif (not stuck) and self.was_stuck:
            self.get_logger().info('Da thoat ket.')
        self.was_stuck = stuck


def main(args=None):
    rclpy.init(args=args)
    node = StuckMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()