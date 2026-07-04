#!/usr/bin/env python3
"""
path_follower_node.py  (v5 - khu hoi, moc theo KHOANG CACH TOI MARKER)
======================================================================
Khac v4: diem DUNG/QUAY duoc kich hoat khi camera DO DUOC marker muc tieu
o khoang cach <= D (so do truc tiep tu /aruco_info, KHONG tich luy sai so
dead-reckon). Nho vay luot di va luot ve nhat quan, khong phu thuoc anh sang
hay thoi diem marker "vua hien ra".

Nhiem vu gom cac buoc:
  - goto : di theo huong (dir = +1 tien theo +x, -1 theo -x), giu tim hanh
           lang (y=0), DUNG khi marker `stop_marker` do duoc <= `stop_dist`
           (m, tinh tu TAM robot toi marker). stop=True -> dung & dwell.
  - turn : quay tai cho toi huong tuyet doi (heading_deg), dung yaw IMU.

Giu nguyen: lai dju (vung chet + loc muot), thoat ket nhip ngan + dung ngay
khi mat marker, covariance bao do tuoi.

LUU Y khoang cach: marker chi nhin ro khi TAM robot cach marker >~0.45m
(camera o dau xe, cach tam 0.25m). Nen stop_dist nen >= ~0.45, mac dinh 0.50.

Vao : /agv/odom (Odometry), /agv/stuck (Bool), /aruco_info (agv_msgs/ArucoInfo)
Ra  : /cmd_vel (Twist)
LUU Y: TAT aruco_mission_node truoc khi chay.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_msgs.msg import Int8, Int32, Float32, String
from agv_msgs.msg import ArucoInfo

import yaml
from agv_localization.grid_planner import GridPlanner


# =====================================================================
# PATH FOLLOWER 2D (luoi marker)
#   Nhan marker dich qua /agv/goto (Int32) -> planner sinh chuoi buoc
#   (turn + goto) Manhattan -> di tien/lui/trai/phai, re 90 do o cho doi truc.
#
#   goto: stop_marker = ID can canh, stop_dist (m), heading_deg = huong di.
#         Lai bam duong thang qua marker dich theo truc cua heading.
#   turn: heading_deg = goc tuyet doi (0=+x, 90=-y phai, 180=-x, -90=+y trai)
# =====================================================================


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class PathFollowerNode(Node):

    def __init__(self):
        super().__init__('path_follower_node')

        # ---------- Bam tuyen ----------
        self.cruise_speed = 0.12
        self.min_speed    = 0.06
        self.max_angular  = 0.6
        self.max_linear   = 0.30
        self.lookahead    = 0.55
        self.kp_ang       = 0.9
        self.kp_ct        = 1.0      # keo ve duong thang (cross-track) tren luoi 2D
        self.ang_deadband = math.radians(3.0)
        self.ang_smooth   = 0.5
        self.brake_dist   = 0.25     # giam toc khi con cach diem dung < day
        self.turn_thresh  = 0.5      # lech goc lon -> di cham
        self.leg_timeout  = 40.0
        self.det_fresh    = 0.3      # s, do marker con "moi"

        # ---------- Quay tai cho ----------
        self.kp_turn      = 1.2
        self.turn_speed   = 0.35     # rad/s toi da
        self.turn_min_w   = 0.12     # toc toi thieu (de quay cham luc gan dich)
        self.turn_tol     = math.radians(2.0)   # siet: phai quay sat 90 do moi xong
        self.turn_slow    = math.radians(20.0)  # con <20 do thi quay cham lai
        self.turn_timeout = 25.0
        self.settle_dur   = 0.8
        self.stuck_grace  = 1.5      # s, an han khong xet stuck dau moi doan di

        # ---------- Thoat ket ----------
        self.recover_speed   = 0.15
        self.recover_pulse   = 0.40
        self.recover_pause   = 0.50
        self.recover_ang_max = 0.30
        self.recover_progress= 0.06
        self.max_recover     = 5
        self.loc_cov_max     = 0.25

        # ---------- Nang-ha lung (cylinder) ----------
        # Cho status thuoc {3 TARGET_REACHED, 4 MIN_LIMIT, 5 MAX_LIMIT} -> coi xong.
        self.lift_done_set = (3, 4, 5)
        self.lift_timeout  = 15.0    # s, qua lau -> FAULT (cylinder ket/sensor loi)
        self.lift_up_h     = 20.0    # cm, nang pallet
        self.lift_down_h   = 7.0     # cm, ha pallet (cang ha)
        self.back_dist     = 0.50    # m, lui ra khoi gam pallet sau khi ha
        self.back_speed    = 0.18    # m/s, toc do lui (du luc thang ma sat tinh)

        # ---------- Ban do luoi + planner ----------
        self.declare_parameter('marker_map_file', '')
        self.declare_parameter('goal_marker', -1)    # -1 = cho lenh qua /agv/goto
        map_file = self.get_parameter('marker_map_file').value
        self.markers = {}
        self.grid_rows = 4
        self.grid_cols = 5
        self.grid_spacing = 0.4
        self.home_marker = 0
        self.load_map(map_file)
        self.planner = GridPlanner(self.grid_rows, self.grid_cols,
                                   self.grid_spacing, stop_dist=0.50)
        # Marker xe dang dung gan nhat (cap nhat khi canh duoc marker)
        self.current_marker = self.home_marker

        # ---------- ROS I/O ----------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cyl_pub = self.create_publisher(Float32, '/cylinder_cmd', 10)
        self.create_subscription(Odometry, '/agv/odom', self.odom_cb, 20)
        self.create_subscription(Bool, '/agv/stuck', self.stuck_cb, 10)
        self.create_subscription(ArucoInfo, '/aruco_info', self.aruco_cb, 10)
        self.create_subscription(Int8, '/cylinder_status', self.cyl_status_cb, 10)
        self.create_subscription(Int32, '/agv/goto', self.goto_cb, 10)
        self.create_subscription(String, '/agv/task', self.task_cb, 10)
        self.create_subscription(String, '/agv/tasks', self.tasks_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 50)
        self.create_subscription(Float32, '/wheel_dist', self.wheel_dist_cb, 20)

        # ---------- Trang thai ----------
        self.have_pose = False
        self.x = self.y = self.yaw = 0.0
        self.loc_cov = 0.0
        self.stuck = False
        self.aruco = {}              # id -> (x_forward, y_lat, t)
        self.mission = []            # chuoi buoc DONG do planner sinh
        self.state = 'IDLE'          # IDLE = cho lenh /agv/goto
        self.step_idx = 0
        self.leg_time = 0.0
        self.dwell_until = 0.0
        self.settle_until = 0.0
        self.target_yaw = 0.0
        self.turn_start = 0.0
        self.recover_anchor = (0.0, 0.0)
        self.recover_attempts = 0
        self.recover_phase = 'pulse'
        self.recover_phase_until = 0.0
        self.last_log = 0.0
        self.ang_cmd = 0.0
        self.cyl_status = 0          # status cylinder moi nhat
        self.lift_start = 0.0        # thoi diem bat dau cho nang-ha
        self.lift_moving = False     # da thay cylinder BAT DAU chuyen dong chua
        self.yaw_imu = None          # yaw IMU tho (rad) - de quay thuan IMU
        self.turn_yaw_off = 0.0      # offset: yaw_world = yaw_imu + off (chot luc quay)
        # ---------- Hang doi nhiem vu (X->Y nang-ha) ----------
        self.task_queue = []         # danh sach (src, dst) cho xu ly
        self.cur_task = None         # nhiem vu dang chay
        self.task_phase = None       # 'to_src' | 'to_dst' | 'to_home'
        self.back_until_pose = None  # moc pose de lui ra du back_dist
        self.back_start = (0.0, 0.0)
        self.wheel_dist = None       # quang duong tinh tien tich luy (m, co dau)
        self.back_wheel_start = None # wheel_dist luc bat dau lui
        self.back_t0 = 0.0
        self.back_yaw_target = None   # huong giu thang khi lui
        self.back_in_heading = 0.0    # heading luc di vao marker dich (tinh marker lui ve)
        self.just_backed = False     # vua lui ra -> chang ke bo pha tien toi giao diem

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            f'Path follower 2D khoi tao. {len(self.markers)} marker. '
            f'Cho lenh /agv/goto (Int32 = marker dich).')

        # Neu co goal_marker tham so -> chay luon (tien test)
        g = int(self.get_parameter('goal_marker').value)
        if g >= 0:
            self.get_logger().info(f'goal_marker tham so = {g}, se di khi co pose.')
            self._pending_goal = g
        else:
            self._pending_goal = None

    # =================================================================
    def load_map(self, path):
        if not path:
            self.get_logger().warn('Chua dat marker_map_file - dung mac dinh 4x5.')
            return
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            self.grid_rows = int(data.get('grid_rows', 4))
            self.grid_cols = int(data.get('grid_cols', 5))
            self.grid_spacing = float(data.get('grid_spacing', 0.4))
            self.home_marker = int(data.get('home_marker', 0))
            for k, v in data.get('markers', {}).items():
                self.markers[int(k)] = {'x': float(v['x']), 'y': float(v['y'])}
        except Exception as e:
            self.get_logger().error(f'Loi doc ban do: {e}')

    def goto_cb(self, msg):
        self.start_mission(int(msg.data))

    def imu_cb(self, msg):
        q = msg.orientation
        self.yaw_imu = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def wheel_dist_cb(self, msg):
        self.wheel_dist = float(msg.data)

    def snap_90(self, yaw):
        k = round(yaw / (math.pi / 2.0))
        return wrap_pi(k * (math.pi / 2.0))

    def task_cb(self, msg):
        # Lenh "src,dst" vd "15,10": nang pallet tu marker 15 toi marker 10.
        if self._enqueue_task(msg.data):
            if self.state == 'IDLE':
                self.next_task()

    def tasks_cb(self, msg):
        # Nhieu nhiem vu mot lan, phan tach bang ';'
        # vd: "3,11; 4,3; 8,2"  -> xep 3 nhiem vu vao hang doi.
        added = 0
        for chunk in msg.data.split(';'):
            chunk = chunk.strip()
            if not chunk:
                continue
            if self._enqueue_task(chunk):
                added += 1
        self.get_logger().info(f'=== Da nhan {added} nhiem vu. Tong hang doi: '
                               f'{len(self.task_queue)} viec. ===')
        if added > 0 and self.state == 'IDLE':
            self.next_task()

    def _enqueue_task(self, text):
        # Phan tich "src,dst" -> them vao task_queue. Tra True neu thanh cong.
        try:
            parts = text.replace(' ', '').split(',')
            src, dst = int(parts[0]), int(parts[1])
        except Exception:
            self.get_logger().error(f'Lenh task sai dinh dang: "{text}" (can "src,dst").')
            return False
        if src not in self.markers or dst not in self.markers:
            self.get_logger().error(f'Marker {src} hoac {dst} khong co trong ban do.')
            return False
        self.task_queue.append((src, dst))
        self.get_logger().info(
            f'+ Them nhiem vu: nang {src} -> {dst}. Hang doi: {len(self.task_queue)} viec.')
        return True

    # Lay nhiem vu ke trong hang doi. Het viec -> ve home (cang ha).
    def next_task(self):
        if self.task_queue:
            src, dst = self.task_queue.pop(0)
            self.cur_task = (src, dst)
            self.task_phase = 'to_src'
            self.get_logger().info(f'=== Nhiem vu: nang {src} -> {dst} ===')
            self.build_leg(self.current_marker, src,
                           lift_h=self.lift_up_h, back_after=False, action='pickup')
        elif self.current_marker != self.home_marker:
            self.cur_task = None
            self.task_phase = 'to_home'
            self.get_logger().info('=== Het nhiem vu -> ve home ===')
            self.build_leg(self.current_marker, self.home_marker,
                            back_after=False, action='home',
                           end_heading=0.0)   # ve home xong quay ve huong +x ban dau
        else:
            self.cur_task = None
            self.task_phase = None
            self.get_logger().info('=== Da o home, het viec. Cho lenh moi. ===')
            self.state = 'IDLE'

    # Dung chuoi buoc cho mot chang va bat dau chay.
    # end_heading: neu khac None, them cu QUAY cuoi ve huong nay (do) sau khi toi
    #   dich + nang/ha. Dung de ve home dung huong ban dau.
    def build_leg(self, start_id, goal_id, lift_h=None, back_after=False,
                  action=None, end_heading=None):
        cur_heading = math.degrees(self.snap_90(self.yaw))
        skip = self.just_backed       # vua lui ra -> khong tien lai giao diem cu
        self.just_backed = False
        steps = self.planner.plan_leg(start_id, goal_id, cur_heading,
                                      lift_h=lift_h, back_after=back_after,
                                      action=action, skip_approach=skip)
        if not steps:
            steps = [{'type': 'goto', 'stop_marker': goal_id, 'stop_dist': 0.5,
                      'heading_deg': cur_heading, 'reach': 'to_node',
                      'lift_h': lift_h, 'action': action, 'back_after': back_after}]
        # Them cu quay cuoi ve huong mong muon (vd ve home nhin +x nhu ban dau).
        if end_heading is not None:
            steps.append({'type': 'turn', 'heading_deg': float(end_heading)})
        self.mission = steps
        self.goal_id = goal_id
        self.step_idx = 0
        for i, s in enumerate(steps):
            self.get_logger().info(f'  [{i}] {s}')
        self.begin_step()

    # =================================================================
    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.loc_cov = msg.pose.covariance[0]
        self.have_pose = True

    def stuck_cb(self, msg):
        self.stuck = bool(msg.data)

    def cyl_status_cb(self, msg):
        self.cyl_status = int(msg.data)

    def aruco_cb(self, msg):
        self.aruco[int(msg.id)] = (float(msg.x), float(msg.y), self.now())

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def loc_fresh(self):
        return self.loc_cov <= self.loc_cov_max

    def marker_dist(self, mid):
        """Khoang cach forward tam->marker neu vua do (con moi), else None."""
        det = self.aruco.get(mid)
        if det and (self.now() - det[2]) < self.det_fresh:
            return det[0]
        return None

    def publish_cmd(self, lin, ang, allow_reverse=False):
        if not allow_reverse:
            lin = max(0.0, lin)               # mac dinh chi tien (an toan)
        else:
            lin = max(-self.max_linear, min(self.max_linear, lin))
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
    def start_mission(self, goal_id):
        if goal_id not in self.markers:
            self.get_logger().error(f'Marker dich {goal_id} khong co trong ban do.')
            return
        if self.state not in ('IDLE', 'DONE', 'FAULT', 'LOST'):
            self.get_logger().warn(
                f'Dang ban (state={self.state}), bo qua lenh di {goal_id}.')
            return
        start_id = self.current_marker
        # Huong xe dang quay (do) -> planner biet co can tien toi giao diem truoc
        cur_heading = math.degrees(self.snap_90(self.yaw))
        steps = self.planner.plan(start_id, goal_id, start_heading_deg=cur_heading)
        if not steps:
            self.get_logger().info(f'Da o marker {goal_id} roi, khong can di.')
            return
        self.mission = steps
        self.goal_id = goal_id
        self.step_idx = 0
        self.get_logger().info(
            f'=== Nhiem vu: {start_id} -> {goal_id} ({len(steps)} buoc) ===')
        for i, s in enumerate(steps):
            self.get_logger().info(f'  [{i}] {s}')
        self.begin_step()

    def advance_task(self):
        # Goi khi vua hoan thanh mot chang (da toi dich + nang/ha xong).
        if self.task_phase == 'to_src':
            # Da nang pallet o nguon -> di toi dich, ha pallet, roi LUI ra.
            src, dst = self.cur_task
            self.task_phase = 'to_dst'
            self.get_logger().info(f'  Da nang o {src}. Di toi {dst} de ha.')
            self.build_leg(self.current_marker, dst,
                           lift_h=self.lift_down_h, back_after=True, action='dropoff')
        elif self.task_phase == 'to_dst':
            # Da ha pallet o dich va lui ra -> lam nhiem vu ke / ve home.
            self.get_logger().info('  Da ha pallet va lui ra. Xu ly viec ke tiep.')
            self.next_task()
        elif self.task_phase == 'to_home':
            # Da ve home va ha cang.
            self.task_phase = None
            self.get_logger().info('=== Da ve home. Cho lenh moi. ===')
            self.state = 'IDLE'
        else:
            self.state = 'IDLE'

    def begin_step(self):
        if self.step_idx >= len(self.mission):
            self.stop()
            self.current_marker = self.goal_id   # da toi dich chang nay
            # Neu dang trong mot nhiem vu (task) -> chuyen phase ke tiep.
            if self.task_phase is not None:
                self.advance_task()
            else:
                self.get_logger().info(
                    f'=== TOI NOI marker {self.goal_id}. San sang lenh moi. ===')
                self.state = 'IDLE'
            return
        step = self.mission[self.step_idx]
        if step['type'] == 'turn':
            self.target_yaw = math.radians(step['heading_deg'])
            self.turn_start = self.now()
            # Chot offset: yaw_world = yaw_imu + turn_yaw_off. Lay tai thoi diem
            # bat dau quay, dua tren yaw_world hien tai (do marker da nan dung).
            # Trong luc quay CHI dung IMU (khong cho marker ghi de) -> khong loan.
            if self.yaw_imu is not None:
                self.turn_yaw_off = wrap_pi(self.yaw - self.yaw_imu)
            self.state = 'TURN'
            self.get_logger().info(
                f'[buoc {self.step_idx}] QUAY toi {step["heading_deg"]:.0f} do '
                f'(thuan IMU, off={math.degrees(self.turn_yaw_off):.1f})')
        else:  # goto
            self.leg_time = self.now()
            self.ang_cmd = 0.0
            self._committed = False
            self.state = 'DRIVE'
            self.get_logger().info(
                f'[buoc {self.step_idx}] -> heading {step["heading_deg"]:.0f}, '
                f'dung khi marker {step["stop_marker"]} cach <= {step["stop_dist"]:.2f}m')

    # Lai bam DUONG THANG theo huong di (heading) tren luoi 2D.
    # QUAN HE DAU (theo log thuc te): w > 0 -> yaw TANG; w < 0 -> yaw GIAM.
    #   -> giu heading: w = +kp_ang * e_yaw (e_yaw>0 nghia yaw<heading, can tang).
    #   -> cross-track: dau da kiem dung khi chay truc Y (giu nguyen).
    def steer_heading(self, heading_rad, line_coord):
        """
        heading_rad: huong di mong muon (rad, world).
        line_coord: toa do co dinh cua duong (y neu di truc X, x neu di truc Y).
        Tra ve (ang_cmd, yaw_err).
        """
        e_yaw = wrap_pi(heading_rad - self.yaw)

        h = abs(wrap_pi(heading_rad))
        if h < math.radians(45) or h > math.radians(135):
            # di truc X -> duong y = line. (Da verify dung khi di +x.)
            lateral = self.y - line_coord
            fwd_sign = 1.0 if abs(wrap_pi(heading_rad)) < math.radians(90) else -1.0
            w_ct = -self.kp_ct * fwd_sign * lateral
        else:
            # di truc Y -> duong x = line.
            # Dau da kiem bang vat ly cho ca +90 (ve home) lan -90 (toi marker):
            #   +y lech +x -> quay trai (w>0); -y lech +x -> quay phai (w<0).
            lateral = self.x - line_coord
            fwd_sign = 1.0 if heading_rad > 0 else -1.0
            w_ct = +self.kp_ct * fwd_sign * lateral

        # Giu heading + keo ve duong (cross-track).
        w = self.kp_ang * e_yaw + w_ct
        return w, e_yaw

    def line_coord_for(self, step):
        """Toa do co dinh cua duong di cho buoc goto (theo marker dich)."""
        mid = step['stop_marker']
        heading = math.radians(step['heading_deg'])
        h = abs(wrap_pi(heading))
        if h < math.radians(45) or h > math.radians(135):
            return self.markers[mid]['y']   # di truc X -> giu y cua marker
        else:
            return self.markers[mid]['x']   # di truc Y -> giu x cua marker

    # =================================================================
    def control_loop(self):
        if not self.have_pose:
            self.stop()
            return

        # Co goal cho san tu tham so -> chay khi vua co pose
        if self._pending_goal is not None and self.state == 'IDLE':
            g = self._pending_goal
            self._pending_goal = None
            self.start_mission(g)
            return

        if self.state == 'IDLE':
            self.stop()
        elif self.state == 'DRIVE':
            self.do_drive()
        elif self.state == 'TURN':
            self.do_turn()
        elif self.state == 'RECOVER':
            self.do_recover()
        elif self.state == 'DWELL':
            self.do_dwell()
        elif self.state == 'BACK':
            self.do_back()
        elif self.state == 'LIFTING':
            self.do_lifting()
        elif self.state == 'SETTLE':
            self.do_settle()
        elif self.state in ('DONE', 'LOST', 'FAULT'):
            self.stop()

    # ---------------- DRIVE (goto, moc theo khoang cach marker) ----------------
    def do_drive(self):
        # KHONG con thoat ket bang nhip ngan (recover lam mat marker tren luoi
        # -> FAULT). Khi stuck chi canh bao nhe, van tiep tuc di binh thuong;
        # neu ket that su lau thi leg_timeout se bat (dung han).
        if self.stuck:
            self.log_throttle('Co tin hieu ket nhung van tiep tuc di (da bo recover).')

        step = self.mission[self.step_idx]
        heading = math.radians(step['heading_deg'])
        line = self.line_coord_for(step)
        ang, e = self.steer_heading(heading, line)

        reach = step.get('reach', 'stop_at')
        mid = step['stop_marker']

        # --- Dieu kien dung tuy kieu reach ---
        if reach == 'stop_at':
            # Dung CACH marker stop_dist (do truc tiep tu camera) - cho diem dich.
            d = self.marker_dist(mid)
            if d is not None:
                self._last_md = d            # ghi khoang cach moi nhat con thay
                if d <= step['stop_dist']:
                    self.arrive()
                    return
                # Da rat gan (trong nguong + bien) -> "cam ket": neu marker mat
                # ngay sau day (chui xuong gam) cung coi nhu da toi.
                self._committed = d <= (step['stop_dist'] + 0.15)
            else:
                # Mat marker. Neu vua truoc do da cam ket (rat gan) -> dung luon.
                if getattr(self, '_committed', False):
                    self.get_logger().info(
                        'Marker chui xuong gam khi da rat gan -> coi nhu toi.')
                    self._committed = False
                    self.arrive()
                    return
        else:  # to_node: TIEN toi dung giao diem (toa do marker) bang pose 2D.
            # Dung khi tam xe da toi toa do marker theo truc dang di.
            mx = self.markers[mid]['x']
            my = self.markers[mid]['y']
            h = abs(wrap_pi(heading))
            if h < math.radians(45) or h > math.radians(135):
                along = self.x          # di truc X -> so sanh x
                target = mx
                # huong +x: toi khi x >= mx; huong -x: khi x <= mx
                reached = (self.x >= mx - 0.03) if abs(wrap_pi(heading)) < math.radians(90) \
                          else (self.x <= mx + 0.03)
            else:
                along = self.y          # di truc Y -> so sanh y
                target = my
                # heading -90 = di -y (y giam) -> toi khi y <= my.
                # heading +90 = di +y (y tang) -> toi khi y >= my.
                reached = (self.y <= my + 0.03) if heading < 0 \
                          else (self.y >= my - 0.03)
            if reached:
                self.arrive()
                return
            d = None  # to_node khong dung khoang cach marker de giam toc

        # Lai dju: deadband chi khu rung khi CA goc nho VA lenh lai nho.
        # KHONG ep ang=0 chi vi e_yaw nho -> neu khong cross-track (keo ve duong
        # khi lech ngang song song) bi xoa, xe di lech ma khong nan lai.
        if abs(e) < self.ang_deadband and abs(ang) < 0.05:
            ang = 0.0
        self.ang_cmd = (1.0 - self.ang_smooth) * self.ang_cmd + self.ang_smooth * ang

        # Toc dai: cham khi lech goc lon, giam toc khi gan diem dung
        if abs(e) > self.turn_thresh:
            lin = self.min_speed
        elif reach == 'stop_at' and d is not None:
            remain = d - step['stop_dist']
            ratio = min(1.0, max(0.0, remain) / self.brake_dist)
            lin = max(self.min_speed, self.cruise_speed * ratio)
        else:
            lin = self.cruise_speed     # to_node hoac chua thay marker -> chay deu

        self.publish_cmd(lin, self.ang_cmd)
        dstr = f'{d:.2f}' if d is not None else '--'
        self.log_throttle(
            f'[wp{self.step_idx} {reach}] pose=({self.x:.2f},{self.y:.2f}) '
            f'm{mid}={dstr} e={math.degrees(e):.1f} '
            f'-> v={lin:.2f} w={self.ang_cmd:.2f}')

        if self.now() - self.leg_time > self.leg_timeout:
            self.stop()
            self.state = 'LOST'
            self.get_logger().error(
                f'LOST: chua canh duoc marker {step["stop_marker"]} sau '
                f'{self.leg_timeout:.0f}s. Dung xe.')

    # ---------------- TURN (dung IMU thuan - khong bi marker snap som) ----------
    def do_turn(self):
        # Luc quay tai cho, neu dung self.yaw (co marker snap_90) thi khi xe quay
        # toi ~70-80 do, marker snap pose ve -90 som -> do_turn tuong da toi ->
        # dung som. Dung IMU thuan: do goc quay THAT, lien tuc, khong snap.
        # Dau da thong nhat (marker cung chieu IMU) nen IMU va he world khop.
        if self.yaw_imu is not None:
            yaw_now = wrap_pi(self.yaw_imu + self.turn_yaw_off)
        else:
            yaw_now = self.yaw

        yaw_err = wrap_pi(self.target_yaw - yaw_now)
        if abs(yaw_err) < self.turn_tol:
            self.stop()
            self.settle_until = self.now() + self.settle_dur
            self.state = 'SETTLE'
            self.get_logger().info(f'Quay xong (yaw={math.degrees(yaw_now):.1f} do).')
            return

        mag = min(self.turn_speed, max(self.turn_min_w, self.kp_turn * abs(yaw_err)))
        w = math.copysign(mag, yaw_err)
        self.publish_cmd(0.0, w)
        self.log_throttle(
            f'[turn] yaw={math.degrees(yaw_now):.1f} -> '
            f'{math.degrees(self.target_yaw):.0f}, err={math.degrees(yaw_err):.1f}, w={w:.2f}')

        if self.now() - self.turn_start > self.turn_timeout:
            self.stop()
            self.state = 'FAULT'
            self.get_logger().error(
                f'FAULT: quay khong xong sau {self.turn_timeout:.0f}s '
                f'(co the ket banh khi xoay). Dung xe.')

    def do_settle(self):
        self.stop()
        if self.now() >= self.settle_until:
            self.step_idx += 1
            self.begin_step()

    # ---------------- RECOVER ----------------
    def do_recover(self):
        if not self.loc_fresh():
            self.stop()
            self.state = 'FAULT'
            self.get_logger().error(
                'FAULT: mat marker khi thoat ket (cov={:.2f}). '
                'Dung de tranh troi. Can dat lai xe vao tam nhin marker.'
                .format(self.loc_cov))
            return

        moved = math.hypot(self.x - self.recover_anchor[0],
                           self.y - self.recover_anchor[1])
        if moved > self.recover_progress:
            self.get_logger().info(f'Da nhich ra {moved:.2f}m -> tiep tuc.')
            self.state = 'DRIVE'
            return

        if self.recover_attempts > self.max_recover:
            self.stop()
            self.state = 'FAULT'
            self.get_logger().error(
                f'FAULT: ket o ({self.x:.2f},{self.y:.2f}), thu '
                f'{self.max_recover} nhip khong thoat. Can ho tro (van de co khi).')
            return

        now = self.now()
        if self.recover_phase == 'pulse':
            step = self.mission[self.step_idx]
            heading = math.radians(step['heading_deg'])
            line = self.line_coord_for(step)
            ang, _ = self.steer_heading(heading, line)
            ang = max(-self.recover_ang_max, min(self.recover_ang_max, ang))
            self.publish_cmd(self.recover_speed, ang)
            if now >= self.recover_phase_until:
                self.recover_phase = 'pause'
                self.recover_phase_until = now + self.recover_pause
                self.recover_attempts += 1
        else:
            self.stop()
            if now >= self.recover_phase_until:
                self.recover_phase = 'pulse'
                self.recover_phase_until = now + self.recover_pulse

    # ---------------- arrive / lifting / dwell ----------------
    def arrive(self):
        self.stop()
        step = self.mission[self.step_idx]
        self.current_marker = step['stop_marker']   # da canh duoc marker nay
        action = step.get('action')
        msg = f'TOI buoc {self.step_idx} (canh marker {step["stop_marker"]})'
        if action:
            msg += f' (action={action})'
        self.get_logger().info(msg)

        # Co nang-ha? -> gui lenh cylinder, sang state LIFTING (xe dung yen).
        lift_h = step.get('lift_h')
        if lift_h is not None:
            self.cyl_status = 0                  # reset truoc khi cho
            self.lift_moving = False             # chua thay chuyen dong
            self.cyl_pub.publish(Float32(data=float(lift_h)))
            self.lift_start = self.now()
            self.state = 'LIFTING'
            self.get_logger().info(
                f'  -> nang-ha cylinder toi {lift_h:.0f}cm, cho hoan tat...')
            return

        # Khong nang-ha: di tiep ngay (chi dung han o marker DICH cuoi mission)
        if step.get('dwell', 0.0) > 0:
            self.dwell_until = self.now() + float(step.get('dwell', 0.0))
            self.state = 'DWELL'
        else:
            self.step_idx += 1
            self.begin_step()

    def do_lifting(self):
        # Xe DUNG YEN tuyet doi khi nang-ha (tranh lech khoi ke).
        self.stop()

        # Phai thay cylinder BAT DAU chuyen dong (1 MOVING_UP / 2 MOVING_DOWN)
        # truoc da. Tranh nhan nham status "3 TARGET_REACHED" CON DONG LAI tu
        # lan nang-ha truoc (ESP32 giu status cu, publish 10Hz) -> neu khong,
        # buoc dropoff se "xong" ngay lap tuc roi quay 180 do khi cylinder con
        # dang ha.
        if not self.lift_moving:
            if self.cyl_status in (1, 2):
                self.lift_moving = True
            # Chua chuyen dong -> chi cho + check timeout, KHONG xet "xong".
            if self.now() - self.lift_start > self.lift_timeout:
                self.state = 'FAULT'
                self.get_logger().error(
                    f'FAULT: cylinder khong bat dau chuyen dong sau '
                    f'{self.lift_timeout:.0f}s (status={self.cyl_status}). '
                    f'Kiem tra cylinder/sensor/day.')
            return

        # Da chuyen dong roi -> gio moi chap nhan da dung (3/4/5)
        if self.cyl_status in self.lift_done_set:
            self.get_logger().info(
                f'  Nang-ha xong (status={self.cyl_status}).')
            step = self.mission[self.step_idx]
            self.dwell_until = self.now() + float(step.get('dwell', 0.0))
            self.state = 'DWELL'
            return

        # An toan: cylinder ket / sensor loi -> FAULT, khong treo vo han.
        if self.now() - self.lift_start > self.lift_timeout:
            self.state = 'FAULT'
            self.get_logger().error(
                f'FAULT: nang-ha khong xong sau {self.lift_timeout:.0f}s '
                f'(status={self.cyl_status}). Kiem tra cylinder/sensor.')

    def do_dwell(self):
        self.stop()
        if self.now() >= self.dwell_until:
            step = self.mission[self.step_idx]
            # Sau khi HA pallet, neu can lui ra khoi gam pallet -> vao BACK.
            if step.get('back_after', False):
                self.back_wheel_start = self.wheel_dist
                self.back_start = (self.x, self.y)
                self.back_t0 = self.now()
                # Heading luc di vao marker dich (de tinh marker xe lui ve).
                self.back_in_heading = float(step.get('heading_deg', 0.0))
                # Chot huong de giu thang khi lui (theo IMU thuan).
                if self.yaw_imu is not None:
                    self.turn_yaw_off = wrap_pi(self.yaw - self.yaw_imu)
                    self.back_yaw_target = wrap_pi(self.yaw_imu + self.turn_yaw_off)
                else:
                    self.back_yaw_target = None
                self.state = 'BACK'
                self.get_logger().info(f'  Lui ra khoi gam pallet {self.back_dist:.2f}m...')
                return
            self.step_idx += 1
            self.begin_step()

    # ---------------- BACK (lui thang ra khoi gam pallet) ----------------
    def do_back(self):
        # Do quang lui bang wheel_dist (chinh xac, khong bi marker snap lam treo).
        moved = None
        if self.wheel_dist is not None and self.back_wheel_start is not None:
            moved = abs(self.wheel_dist - self.back_wheel_start)

        # Fallback: neu khong co wheel_dist, uoc luong theo thoi gian * toc do.
        if moved is None:
            elapsed = self.now() - self.back_t0
            moved = self.back_speed * elapsed

        if moved >= self.back_dist:
            self.stop()
            self.get_logger().info(f'  Da lui ra {moved:.2f}m.')
            self._finish_back()
            return

        # An toan: gioi han thoi gian lui (tranh treo neu odom loi)
        if self.now() - self.back_t0 > 12.0:
            self.stop()
            self.get_logger().warn('  Lui qua 12s -> dung lui, di tiep.')
            self._finish_back()
            return

        # Lui thang, giu huong bang IMU (chong lech trai/phai khi lui).
        w = 0.0
        if self.yaw_imu is not None and self.back_yaw_target is not None:
            yaw_now = wrap_pi(self.yaw_imu + self.turn_yaw_off)
            e = wrap_pi(self.back_yaw_target - yaw_now)
            w = self.kp_ang * e          # giu yaw = huong luc bat dau lui
        self.publish_cmd(-self.back_speed, w, allow_reverse=True)

    def _finish_back(self):
        # Sau khi lui ra, xe da ve marker LIEN KE (nguoc huong vua di vao).
        # Cap nhat current_marker de planner tinh duong tu DUNG vi tri thuc.
        old = self.current_marker
        self.current_marker = self.planner.neighbor_back(
            self.current_marker, self.back_in_heading)
        if self.current_marker != old:
            self.get_logger().info(
                f'  Sau lui: xe ve marker {self.current_marker} '
                f'(tu {old}, lui nguoc huong {self.back_in_heading:.0f}).')
        self.just_backed = True
        # Goi THANG advance_task, KHONG qua begin_step: begin_step (nhanh het
        # mission) se gan current_marker = goal_id (=marker dich cu, vd 11) ->
        # de len gia tri 7 vua dat -> planner tinh sai. advance_task chuyen phase
        # nhiem vu (to_dst -> next_task) dung voi current_marker = 7.
        self.advance_task()


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()