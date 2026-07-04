#!/usr/bin/env python3
"""
agv_pose_node.py  ---  Localization: tu ID marker -> POSE the gioi
==================================================================
Bien he thong tu "song bang ID" sang "song bang pose (x, y, yaw)".

Nguyen ly:
  - yaw robot lay tu IMU/BNO055 (moc the gioi can chinh 1 lan luc khoi dong).
  - Khi THAY marker co trong ban do: tinh truc tiep pose robot trong he map
        robot_world = marker_world - R(yaw) . [x_robot, y_robot]
    -> "snap" vi tri ve gia tri dung (khong can yaw cua marker).
  - GIUA cac marker: dead-reckon tho bang cmd_vel + yaw (se troi; chinh la
    ly do can dat marker gan nhau de luon thay marker).

Xuat:
  - /agv/odom (nav_msgs/Odometry, frame map -> base_link)
  - TF map -> base_link  (de xem tren RViz)

Vao:
  - /aruco_info (agv_msgs/ArucoInfo)
  - /imu/data   (sensor_msgs/Imu)
  - /cmd_vel    (geometry_msgs/Twist)  -> dead-reckon giua marker

Tham so:
  - marker_map_file : duong dan markers.yaml
"""

import math
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from agv_msgs.msg import ArucoInfo
from tf2_ros import TransformBroadcaster


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class AgvPoseNode(Node):

    def __init__(self):
        super().__init__('agv_pose_node')

        # ---------- Tham so ----------
        self.declare_parameter('marker_map_file', '')
        self.declare_parameter('pos_blend', 0.7)   # do tin vao do marker khi snap
        self.declare_parameter('max_jump', 0.5)    # m, snap nhay hon nay -> bo (nhieu)
        map_file = self.get_parameter('marker_map_file').value
        self.pos_blend = float(self.get_parameter('pos_blend').value)
        self.max_jump = float(self.get_parameter('max_jump').value)

        self.markers = {}
        self.frame_id = 'map'
        self.start_marker = 0
        self.initial_yaw = 0.0
        self.load_map(map_file)

        # ---------- Trang thai pose ----------
        self.x = self.markers.get(self.start_marker, {'x': 0.0})['x']
        self.y = self.markers.get(self.start_marker, {'y': 0.0})['y']
        self.yaw = self.initial_yaw          # yaw the gioi
        self.yaw_offset = None               # yaw_world = yaw_imu - offset
        self.last_yaw_imu = None             # yaw IMU tho gan nhat (de hieu chinh)
        self.last_cmd_v = 0.0
        self.last_cmd_time = 0.0
        self.last_time = self.now()
        self.last_marker_id = None
        self.last_marker_time = 0.0
        self.initialized = False             # da co fix marker dau tien chua

        # Dead-reckon bang quang duong THAT tu wheel odom (/wheel_dist).
        # Fallback ve cmd_vel*dt neu mat odom (vd quen chay wheel_odom_node).
        self.wheel_dist = None               # gia tri tich luy moi nhat nhan duoc
        self.wheel_dist_prev = None          # gia tri tai lan update truoc
        self.wheel_dist_time = 0.0

        # ---------- ROS I/O ----------
        self.odom_pub = self.create_publisher(Odometry, '/agv/odom', 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 50)
        self.create_subscription(ArucoInfo, '/aruco_info', self.aruco_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(Float32, '/wheel_dist', self.wheel_dist_cb, 20)

        self.create_timer(0.03, self.update)   # ~33 Hz
        self.get_logger().info(
            f'Pose node khoi tao. {len(self.markers)} marker, '
            f'bat dau tai marker {self.start_marker}.')

    # =================================================================
    def load_map(self, path):
        if not path:
            self.get_logger().error('Chua dat tham so marker_map_file!')
            return
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            self.frame_id = data.get('frame_id', 'map')
            self.start_marker = int(data.get('start_marker', 0))
            self.initial_yaw = float(data.get('initial_yaw', 0.0))
            for k, v in data.get('markers', {}).items():
                self.markers[int(k)] = {
                    'x': float(v['x']), 'y': float(v['y']),
                    'yaw': float(v.get('yaw', 0.0))}
        except Exception as e:
            self.get_logger().error(f'Loi doc ban do: {e}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # =================================================================
    def imu_cb(self, msg):
        q = msg.orientation
        yaw_imu = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.last_yaw_imu = yaw_imu
        if self.yaw_offset is None:
            # Can chinh mot lan: gan yaw the gioi = initial_yaw luc khoi dong
            self.yaw_offset = wrap_pi(yaw_imu - self.initial_yaw)
            self.get_logger().info(
                f'Da can chinh huong. yaw_offset={math.degrees(self.yaw_offset):.1f} do')
        self.yaw = wrap_pi(yaw_imu - self.yaw_offset)

    def cmd_cb(self, msg):
        self.last_cmd_v = float(msg.linear.x)
        self.last_cmd_time = self.now()

    def wheel_dist_cb(self, msg):
        # Quang duong tinh tien tich luy (m, co dau) tu wheel odom.
        self.wheel_dist = float(msg.data)
        self.wheel_dist_time = self.now()

    def aruco_cb(self, msg):
        mid = int(msg.id)
        if mid not in self.markers or self.yaw_offset is None:
            return
        x_r, y_r = float(msg.x), float(msg.y)
        yaw_marker = float(msg.yaw)   # huong xe so voi map (do aruco do)

        # Loc quan sat: chi nhan marker DU GAN va DU THANG. Marker o xa (x_r lon)
        # hoac lech ngang nhieu (|y_r| lon) -> goc yaw do rat nhieu -> snap sai ->
        # pose nhay loan. Bo qua, dung dead-reckon toi khi marker du tin cay.
        if not (0.2 <= x_r <= 0.9):
            return
        if abs(y_r) > 0.25:
            return

        # --- Huong xe trong map: snap yaw marker ve boi so 90 do ---
        yaw_snapped = self.snap_90(yaw_marker)
        # Neu yaw marker do LECH XA boi so 90 (>30 do) -> dang nhieu (marker
        # nghieng/xa) -> KHONG tin huong nay, bo qua ca quan sat.
        if abs(wrap_pi(yaw_marker - yaw_snapped)) > math.radians(30):
            return

        # Cap nhat yaw the gioi theo marker (tuyet doi, khong troi) va hieu
        # chinh lai yaw_offset cua IMU cho khop -> IMU het troi dan.
        self.yaw = yaw_snapped
        if self.last_yaw_imu is not None:
            self.yaw_offset = wrap_pi(self.last_yaw_imu - yaw_snapped)

        # --- Vi tri robot trong map 2D ---
        mx = self.markers[mid]['x']
        my = self.markers[mid]['y']
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        # robot_map = marker_map - R(yaw).[x_r, y_r]
        rx = mx - (x_r * c - y_r * s)
        ry = my - (x_r * s + y_r * c)

        if not self.initialized:
            self.x, self.y = rx, ry
            self.initialized = True
        elif mid != self.last_marker_id:
            self.x, self.y = rx, ry           # doi marker -> snap (binh thuong)
        else:
            jump = math.hypot(rx - self.x, ry - self.y)
            if jump > self.max_jump:
                self.get_logger().warn(
                    f'Bo doc nhieu marker {mid}: nhay {jump:.2f}m')
                return
            a = self.pos_blend
            self.x = (1.0 - a) * self.x + a * rx
            self.y = (1.0 - a) * self.y + a * ry

        self.last_marker_id = mid
        self.last_marker_time = self.now()

    def snap_90(self, yaw):
        """Snap goc ve boi so 90 do gan nhat (rad)."""
        k = round(yaw / (math.pi / 2.0))
        return wrap_pi(k * (math.pi / 2.0))

    # =================================================================
    def update(self):
        t = self.now()
        dt = t - self.last_time
        self.last_time = t
        if self.yaw_offset is None:
            return

        # Dead-reckon vi tri giua cac marker, dung yaw IMU.
        # Uu tien quang duong THAT tu wheel odom (/wheel_dist); neu mat odom
        # (>0.5s khong nhan) thi fallback ve cmd_vel*dt nhu cu.
        if self.wheel_dist is not None and (t - self.wheel_dist_time) < 0.5:
            if self.wheel_dist_prev is None:
                self.wheel_dist_prev = self.wheel_dist   # baseline, delta=0 lan dau
            d = self.wheel_dist - self.wheel_dist_prev
            self.wheel_dist_prev = self.wheel_dist
            self.x += d * math.cos(self.yaw)
            self.y += d * math.sin(self.yaw)
        else:
            # Mat odom -> re-baseline khi co lai (tranh cong kep doan fallback)
            self.wheel_dist_prev = None
            v = self.last_cmd_v if (t - self.last_cmd_time) < 0.3 else 0.0
            self.x += v * math.cos(self.yaw) * dt
            self.y += v * math.sin(self.yaw) * dt

        self.publish_pose(t)

    def publish_pose(self, t):
        odom = Odometry()
        stamp = self.get_clock().now().to_msg()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = quat_from_yaw(self.yaw)
        odom.twist.twist.linear.x = self.last_cmd_v

        # Do "tuoi" cua dinh vi -> dat vao covariance (nho = vua thay marker,
        # lon dan = lau khong thay marker -> dang dead-reckon, kem tin cay).
        age = (t - self.last_marker_time) if self.last_marker_id is not None else 999.0
        cov = min(0.0025 + 0.5 * age, 10.0)
        odom.pose.covariance[0] = cov     # x
        odom.pose.covariance[7] = cov     # y
        odom.pose.covariance[35] = 0.01   # yaw (tu IMU, tin cay)
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.frame_id
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation = quat_from_yaw(self.yaw)
        self.tf_broadcaster.sendTransform(tf)

        # Tuoi do marker gan nhat (de biet pose dang "tuoi" hay dang troi)
        age = t - self.last_marker_time if self.last_marker_id is not None else -1
        self.get_logger().info(
            f'pose x={self.x:.2f} y={self.y:.2f} yaw={math.degrees(self.yaw):.1f} '
            f'| marker={self.last_marker_id} age={age:.1f}s',
            throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = AgvPoseNode()
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