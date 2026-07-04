#!/usr/bin/env python3
"""
wheel_odom_node.py  ---  Wheel odometry tu DEM XUNG buoc ESP32 (open-loop)
==========================================================================
Phase 2 cua "wheel odometry". Nen tang cho SLAM sau nay.

Nguyen ly:
  ESP32 dem xung step moi banh (co dau theo chieu) va publish tich luy len
  /wheel_ticks (geometry_msgs/Vector3: x=pulseL, y=pulseR, z=ms_esp32).
  Node nay lay DELTA xung giua 2 ban tin -> quang duong moi banh ->
  mo hinh vi sai (differential drive) -> tich luy pose (x, y, theta).

  Quang duong 1 banh = (delta_pulse / ppr) * (2*pi*r)
  d_center = (dL + dR) / 2
  d_theta  = (dL - dR) / L      (theta duong = quay TRAI, khop IMU/cmd_vel)

  Tich phan diem giua (midpoint) cho chinh xac hon khi vua di vua quay:
      theta_mid = theta + d_theta/2
      x += d_center * cos(theta_mid)
      y += d_center * sin(theta_mid)
      theta += d_theta

LUU Y (gioi han da biet, ghi trong claude.md):
  - OPEN-LOOP: neu banh truot/ket, xung van dem nhung xe khong di -> odom bao
    DU. Khong bat truot. Viec bat truot van dua vao stuck_monitor + snap ArUco.
  - KHONG broadcast TF (agv_pose_node da phat map->base_link). Node nay chi
    publish /wheel_odom de verify + lam nen SLAM, CHUA thay the /agv/odom.

Vao : /wheel_ticks (geometry_msgs/Vector3)  -- x=pulseL, y=pulseR (tich luy)
Ra  : /wheel_odom  (nav_msgs/Odometry)      -- frame odom -> base_link

Tham so (dat sau khi flash firmware, lat dau neu sai - khoi sua code):
  ppr            (6400.0)   xung/vong banh thuc te
  wheel_radius   (0.0725)   m
  wheel_base     (0.65)     m, khoang cach 2 banh
  invert_left    (False)    lat dau xung banh trai neu tien ma dL am
  invert_right   (False)    lat dau xung banh phai
  swap_lr        (False)    doi cho L<->R neu noi nham day
  ticks_topic    (/wheel_ticks)
  odom_topic     (/wheel_odom)
  reset_jump     (5000)     |delta| lon hon -> coi ESP32 vua reset, bo qua buoc
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


def quat_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class WheelOdomNode(Node):

    def __init__(self):
        super().__init__('wheel_odom_node')

        # ---------- Tham so ----------
        self.declare_parameter('ppr', 6400.0)
        self.declare_parameter('wheel_radius', 0.0725)
        self.declare_parameter('wheel_base', 0.65)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('swap_lr', False)
        self.declare_parameter('ticks_topic', '/wheel_ticks')
        self.declare_parameter('odom_topic', '/wheel_odom')
        self.declare_parameter('reset_jump', 5000)   # xung

        self.ppr     = float(self.get_parameter('ppr').value)
        self.r       = float(self.get_parameter('wheel_radius').value)
        self.L       = float(self.get_parameter('wheel_base').value)
        self.inv_l   = bool(self.get_parameter('invert_left').value)
        self.inv_r   = bool(self.get_parameter('invert_right').value)
        self.swap    = bool(self.get_parameter('swap_lr').value)
        ticks_topic  = self.get_parameter('ticks_topic').value
        odom_topic   = self.get_parameter('odom_topic').value
        self.reset_jump = int(self.get_parameter('reset_jump').value)

        # Quang duong moi 1 xung (m). 2*pi*r = chu vi banh.
        self.m_per_tick = (2.0 * math.pi * self.r) / self.ppr

        # ---------- Trang thai pose ----------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.dist_total = 0.0        # quang duong tinh tien tich luy (co dau)
        self.have_prev = False
        self.prev_pl = 0
        self.prev_pr = 0
        self.prev_t = self.now()

        # ---------- ROS I/O ----------
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 20)
        # /wheel_dist: quang duong tinh tien tich luy -> agv_pose lay delta de
        # dead-reckon (thay cho cmd_vel*dt). Vo huong, co dau (lui = giam).
        self.dist_pub = self.create_publisher(Float32, '/wheel_dist', 20)
        self.create_subscription(Vector3, ticks_topic, self.ticks_cb, 20)

        self.get_logger().info(
            f'Wheel odom khoi tao. ppr={self.ppr:.0f} r={self.r:.4f} L={self.L:.3f} '
            f'm/tick={self.m_per_tick*1000:.4f}mm | sub {ticks_topic} -> pub {odom_topic}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # =================================================================
    def ticks_cb(self, msg):
        # Doc xung tich luy (co the lat dau / doi cho theo tham so)
        pl = int(round(msg.x))
        pr = int(round(msg.y))
        if self.swap:
            pl, pr = pr, pl
        if self.inv_l:
            pl = -pl
        if self.inv_r:
            pr = -pr

        if not self.have_prev:
            self.prev_pl = pl
            self.prev_pr = pr
            self.prev_t = self.now()
            self.have_prev = True
            return

        d_pl = pl - self.prev_pl
        d_pr = pr - self.prev_pr

        # Phat hien ESP32 reset (xung nhay ve 0 -> delta khong lo) -> bo buoc nay
        if abs(d_pl) > self.reset_jump or abs(d_pr) > self.reset_jump:
            self.get_logger().warn(
                f'Bo buoc: delta xung bat thuong (dL={d_pl}, dR={d_pr}) '
                f'- co the ESP32 vua reset. Re-baseline.')
            self.prev_pl = pl
            self.prev_pr = pr
            self.prev_t = self.now()
            return

        t = self.now()
        dt = t - self.prev_t
        self.prev_pl = pl
        self.prev_pr = pr
        self.prev_t = t

        # Quang duong moi banh (m)
        dL = d_pl * self.m_per_tick
        dR = d_pr * self.m_per_tick

        d_center = 0.5 * (dL + dR)
        d_theta  = (dL - dR) / self.L      # duong = quay trai

        # Quang duong tinh tien tich luy (co dau) -> /wheel_dist
        self.dist_total += d_center
        self.dist_pub.publish(Float32(data=float(self.dist_total)))

        # Tich phan diem giua
        theta_mid = self.theta + 0.5 * d_theta
        self.x += d_center * math.cos(theta_mid)
        self.y += d_center * math.sin(theta_mid)
        self.theta = math.atan2(math.sin(self.theta + d_theta),
                                math.cos(self.theta + d_theta))

        # Van toc (cho twist)
        v = d_center / dt if dt > 1e-6 else 0.0
        w = d_theta / dt if dt > 1e-6 else 0.0

        self.publish_odom(v, w)

        self.get_logger().info(
            f'odom x={self.x:.3f} y={self.y:.3f} th={math.degrees(self.theta):.1f} '
            f'| dL={dL*1000:.1f}mm dR={dR*1000:.1f}mm v={v:.3f} w={w:.3f}',
            throttle_duration_sec=0.5)

    def publish_odom(self, v, w):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = quat_from_yaw(self.theta)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        # Covariance tho: open-loop, kha tin cay ngan han, troi dan dai han
        odom.pose.covariance[0]  = 0.02   # x
        odom.pose.covariance[7]  = 0.02   # y
        odom.pose.covariance[35] = 0.05   # yaw (wheel-only, kem IMU)
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomNode()
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