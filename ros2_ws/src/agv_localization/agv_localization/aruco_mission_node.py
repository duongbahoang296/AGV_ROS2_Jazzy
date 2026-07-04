#!/usr/bin/env python3
"""
aruco_mission_node.py  (v2 - line following)
============================================
Dieu huong AGV bam marker ArUco tren san, KHONG dung encoder.

Khac ban v1 (pure pursuit -> toi diem nhung toi o goc xien, troi dan):
  v2 dung LINE-FOLLOWING, ket hop du 3 dai luong de robot luon nam
  tren duong thang noi cac marker va TOI MARKER TRONG TU THE CAN THANG:

    1) Goc huong (yaw IMU/BNO055): giu robot song song hanh lang.
       Moc "huong hanh lang" giu xuyen suot, chi lat 180 do o buoc TURN.
    2) y (lech ngang cua marker, tu ArucoInfo.y): keo robot ve lai
       dung duong thang khi bi truot banh.
    3) x / distance: biet con cach marker bao xa -> giam toc & dung.

  Luat lai khi thay marker:
       angular.z = k_yaw*(corridor_heading - yaw) + k_lat*y
  Khi KHONG thay marker (doan mu): chi giu huong bang IMU.

  An toan: moi chang co TIMEOUT. Neu qua lau khong thay marker muc tieu
  -> dung & bao LOST (tranh xe chay hoang vuot qua marker cuoi).

Giao tiep giu nguyen:
  Sub /aruco_info (agv_msgs/ArucoInfo) | Sub /imu/data (sensor_msgs/Imu)
  Pub /cmd_vel (geometry_msgs/Twist)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from agv_msgs.msg import ArucoInfo


# =====================================================================
# CAU HINH NHIEM VU  ---  marker 0..4 thang hang; quay dau tai 4 de ve 0
# =====================================================================
MISSION = [
    {"type": "goto", "id": 1},
    {"type": "goto", "id": 2},
    {"type": "goto", "id": 3, "action": "pickup",  "dwell": 2.0},
    {"type": "goto", "id": 4, "action": "dropoff", "dwell": 2.0},
    {"type": "turn", "deg": 180.0},
    {"type": "goto", "id": 3},
    {"type": "goto", "id": 2},
    {"type": "goto", "id": 1},
    {"type": "goto", "id": 0, "action": "home",    "dwell": 0.0},
]


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class ArucoMissionNode(Node):

    def __init__(self):
        super().__init__('aruco_mission_node')

        # ---------- Tham so dieu khien (tune tai day) ----------
        self.cruise_speed = 0.12     # m/s
        self.min_speed    = 0.05     # m/s khi toi gan marker
        self.max_angular  = 0.6      # rad/s gioi han xoay
        self.turn_speed   = 0.5      # rad/s xoay tai cho

        self.k_yaw  = 1.2            # P giu huong theo IMU
        self.k_ct   = 1.0            # P keo ve duong theo cross-track (rad/s / m)
        self.k_dist = 0.5            # P giam toc theo khoang cach

        self.arrival_dist = 0.45     # m, distance <= nguong -> toi
        self.commit_dist  = 0.55     # m, da cam ket toi (doan mu cuoi)
        self.heading_tol  = math.radians(8.0)   # can thang huong du tot
        self.lateral_tol  = 0.04     # m, lech ngang du nho
        self.turn_tol     = math.radians(3.0)

        self.detect_timeout = 0.30   # s, marker coi nhu mat neu cu hon
        self.leg_timeout    = 20.0   # s, qua lau khong thay marker -> LOST

        # ---------- ROS I/O ----------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(ArucoInfo, '/aruco_info', self.aruco_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 50)

        # ---------- Trang thai cam bien ----------
        self.yaw = None
        self.det_x = None
        self.det_y = None
        self.det_dist = None
        self.det_bearing = None
        self.det_time = None

        # ---------- May trang thai ----------
        self.state = 'INIT'
        self.step_idx = 0
        self.current_target_id = None
        self.corridor_heading = None   # moc huong hanh lang (rad, he IMU)
        self.committed = False
        self.dwell_until = None
        self.turn_target_yaw = None
        self.leg_start_time = None
        self.last_log = 0.0

        self.create_timer(0.05, self.control_loop)   # 20 Hz
        self.get_logger().info('Mission node v2 khoi tao. Cho IMU...')

    # =================================================================
    def imu_cb(self, msg):
        q = msg.orientation
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

    def aruco_cb(self, msg):
        if self.current_target_id is None or int(msg.id) != self.current_target_id:
            return
        self.det_x = float(msg.x)
        self.det_y = float(msg.y)
        self.det_dist = float(msg.distance)
        self.det_bearing = math.radians(float(msg.angle))
        self.det_time = self.now()

    # =================================================================
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def target_seen(self):
        return self.det_time is not None and \
            (self.now() - self.det_time) <= self.detect_timeout

    def publish_cmd(self, lin, ang):
        ang = max(-self.max_angular, min(self.max_angular, ang))
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self.cmd_pub.publish(t)

    def stop(self):
        self.publish_cmd(0.0, 0.0)

    def log_throttle(self, text, period=0.5):
        if self.now() - self.last_log >= period:
            self.last_log = self.now()
            self.get_logger().info(text)

    # =================================================================
    def begin_step(self):
        if self.corridor_heading is None:
            # Gia dinh robot xuat phat da can thang theo hanh lang
            self.corridor_heading = self.yaw

        if self.step_idx >= len(MISSION):
            self.state = 'DONE'
            self.get_logger().info('=== HOAN THANH NHIEM VU ===')
            return

        step = MISSION[self.step_idx]
        self.leg_start_time = self.now()

        if step['type'] == 'goto':
            self.current_target_id = int(step['id'])
            self.committed = False
            self.det_time = None
            self.state = 'DRIVE'
            self.get_logger().info(
                f'-> GOTO marker {self.current_target_id} '
                f'(huong hanh lang = {math.degrees(self.corridor_heading):.1f} do)')

        elif step['type'] == 'turn':
            deg = float(step['deg'])
            # Xoay tuong doi so voi HUONG HANH LANG (moc sach), khong phai yaw xien
            self.turn_target_yaw = wrap_pi(self.corridor_heading + math.radians(deg))
            self.current_target_id = None
            self.state = 'TURN'
            self.get_logger().info(f'-> TURN {deg:.0f} do')

        else:
            self.step_idx += 1
            self.begin_step()

    def finish_step(self):
        self.step_idx += 1
        self.begin_step()

    # =================================================================
    def control_loop(self):
        if self.yaw is None:
            self.stop()
            return

        if self.state == 'INIT':
            self.begin_step()
        elif self.state == 'DRIVE':
            self.do_drive()
        elif self.state == 'ARRIVED':
            self.do_arrived()
        elif self.state == 'TURN':
            self.do_turn()
        elif self.state in ('DONE', 'LOST'):
            self.stop()

    # ---------------- DRIVE ----------------
    def do_drive(self):
        e_yaw = wrap_pi(self.corridor_heading - self.yaw)

        if self.target_seen():
            x, y = self.det_x, self.det_y
            dist, bearing = self.det_dist, self.det_bearing

            if dist < self.commit_dist:
                self.committed = True

            # Cross-track THAT cua robot so voi duong noi marker, dung IMU
            # de tach bach phan lech HUONG khoi phan lech NGANG.
            # d_ct = 0 khi robot nam dung tren duong (ke ca dang xien goc),
            # va BAT BIEN khi robot xoay tai cho -> khong bi triet tieu.
            d_ct = y * math.cos(e_yaw) - x * math.sin(e_yaw)

            aligned = abs(e_yaw) <= self.heading_tol
            centered = abs(d_ct) <= self.lateral_tol

            # Toi noi: du gan, da can thang huong va nam tren duong
            if dist <= self.arrival_dist and aligned and centered:
                self.arrive()
                return

            # Lai = giu huong hanh lang + keo ve duong bang cross-track
            ang = self.k_yaw * e_yaw + self.k_ct * d_ct
            lin = self.k_dist * (dist - self.arrival_dist)
            lin = max(self.min_speed, min(self.cruise_speed, lin))
            self.publish_cmd(lin, ang)

            self.log_throttle(
                f'[{self.current_target_id}] x={x:.2f} y={y:.2f} d_ct={d_ct:.3f} '
                f'd={dist:.2f} e_yaw={math.degrees(e_yaw):.1f} '
                f'-> v={lin:.2f} w={ang:.2f}')

        else:
            if self.committed:
                # marker da chui xuong duoi camera o doan cuoi -> coi nhu toi
                self.arrive()
                return

            # Doan mu: di thang, chi giu huong bang IMU
            ang = self.k_yaw * e_yaw
            self.publish_cmd(self.cruise_speed, ang)
            self.log_throttle(
                f'[mu->{self.current_target_id}] e_yaw={math.degrees(e_yaw):.1f} '
                f'-> w={ang:.2f}')

            # An toan: qua lau khong thay marker muc tieu
            if self.now() - self.leg_start_time > self.leg_timeout:
                self.stop()
                self.state = 'LOST'
                self.get_logger().error(
                    f'LOST: khong thay marker {self.current_target_id} '
                    f'sau {self.leg_timeout:.0f}s. Dung xe.')

    def arrive(self):
        self.stop()
        step = MISSION[self.step_idx]
        action = step.get('action')
        msg = f'TOI marker {self.current_target_id}'
        if action:
            msg += f' (action={action})'
            # TODO: tich hop nang-ha xy lanh tai day
        self.get_logger().info(msg)
        self.dwell_until = self.now() + float(step.get('dwell', 0.0))
        self.state = 'ARRIVED'

    # ---------------- ARRIVED ----------------
    def do_arrived(self):
        self.stop()
        if self.now() >= self.dwell_until:
            self.finish_step()

    # ---------------- TURN ----------------
    def do_turn(self):
        err = wrap_pi(self.turn_target_yaw - self.yaw)
        if abs(err) <= self.turn_tol:
            self.stop()
            # Cap nhat moc huong hanh lang theo huong moi sau khi xoay
            self.corridor_heading = self.turn_target_yaw
            self.finish_step()
            return
        ang = self.turn_speed if err > 0 else -self.turn_speed
        if abs(err) < math.radians(20.0):
            ang *= 0.4
        self.publish_cmd(0.0, ang)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoMissionNode()
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