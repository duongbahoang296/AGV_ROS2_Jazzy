#!/usr/bin/env python3
"""
ArUco localization node cho AGV.

Sửa lỗi chính so với bản cũ:
  - Camera chĩa NGHIÊNG xuống sàn, nên z_cam KHÔNG phải khoảng cách mặt sàn.
    Phải xoay tvec theo góc pitch + độ cao camera rồi mới chiếu xuống sàn.
  - Offset camera trừ sai dấu/sai trục.
  - estimatePoseSingleMarkers + DetectorParameters_create() đã bị xóa từ
    OpenCV 4.7+, nên thêm fallback dùng solvePnP.

Cách hiệu chỉnh nhanh (xem mục CALIBRATE bên dưới): chỉnh camera_pitch_deg
và camera_height cho tới khi z_robot (chiều cao marker so với sàn) ~ 0,
vì marker đang nằm trên sàn.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from agv_msgs.msg import ArucoInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import math


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_localization_node')

        # ---------- Publisher & Subscriber ----------
        self.publisher_ = self.create_publisher(ArucoInfo, '/aruco_info', 10)
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        # ---------- Tham số marker ----------
        self.marker_length = 0.04   # cạnh đen ngoài cùng của marker = 4cm

        # ---------- Lắp đặt camera ----------
        # Cách mới KHÔNG cần góc pitch. Chỉ cần 2 số đo thực tế:
        # Độ cao tâm quang camera so với mặt sàn (mét).
        self.camera_height = 0.27
        # Camera lắp phía TRƯỚC tâm robot bao nhiêu mét.
        # (Tâm robot ở phía sau camera => xa marker hơn => CỘNG vào forward.)
        self.camera_offset_forward = 0.23

        # Bật True để in tvec thô + z_robot ra log, phục vụ hiệu chỉnh.
        self.debug = True

        # ---------- Setup ArUco ----------
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        # Tương thích cả OpenCV cũ lẫn mới
        try:
            self.parameters = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.parameters = cv2.aruco.DetectorParameters()

        # Tinh chỉnh góc ở mức subpixel -> định vị góc chính xác hơn nhiều,
        # giảm đáng kể sai số khoảng cách với marker nhỏ.
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.parameters.cornerRefinementWinSize = 5
        self.parameters.cornerRefinementMaxIterations = 30
        self.parameters.cornerRefinementMinAccuracy = 0.01

        # Detector dạng object (OpenCV >= 4.7), nếu không có thì dùng hàm cũ.
        self.detector = None
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        # ---------- Thông số nội (intrinsic) camera ----------
        self.camera_matrix = np.array([[730.8, 0, 352.0],
                                       [0, 734.8, 249.1],
                                       [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.array([0.14, -0.69, -0.001, 0.009, 0], dtype=np.float32)

        self.window_name = "ArUco Detection View"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.get_logger().info("Node da khoi tao. He thong san sang.")

    # ---------------------------------------------------------------
    # Phát hiện marker (tương thích nhiều phiên bản OpenCV)
    # ---------------------------------------------------------------
    def detect_markers(self, gray):
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.dictionary, parameters=self.parameters)
        return corners, ids

    # ---------------------------------------------------------------
    # Ước lượng pose 1 marker (tương thích nhiều phiên bản OpenCV)
    # Trả về tvec dạng (3,)
    # ---------------------------------------------------------------
    def estimate_tvec(self, marker_corners):
        if hasattr(cv2.aruco, 'estimatePoseSingleMarkers'):
            _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [marker_corners], self.marker_length,
                self.camera_matrix, self.dist_coeffs)
            return tvecs[0][0]

    # ---------------------------------------------------------------
    # Uoc luong pose 1 marker (tuong thich nhieu phien ban OpenCV)
    # Tra ve (tvec, rvec) - rvec de tinh yaw marker (huong xe so voi map).
    # ---------------------------------------------------------------
    def estimate_pose(self, marker_corners):
        if hasattr(cv2.aruco, 'estimatePoseSingleMarkers'):
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [marker_corners], self.marker_length,
                self.camera_matrix, self.dist_coeffs)
            return tvecs[0][0], rvecs[0][0]

        # Fallback cho OpenCV >= 4.7 dùng solvePnP
        half = self.marker_length / 2.0
        obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float32)
        img_points = marker_corners.reshape(-1, 2).astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(
            obj_points, img_points, self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None, None
        return tvec.reshape(-1), rvec.reshape(-1)

    # ---------------------------------------------------------------
    # Tinh yaw cua marker trong khung anh = huong XE so voi map.
    #
    # Marker dan canh tren ve +x map (co dinh). Khi xe quay, marker xuat
    # hien xoay trong anh. Goc xoay quanh truc vuong goc san (truc z camera
    # nhin xuong) chinh la huong xe so voi map.
    #
    # rvec -> ma tran xoay R. Truc x cua marker (cot dau cua R) chieu len
    # mat phang anh (x,y) cho biet marker dang xoay bao nhieu.
    #   yaw_marker = atan2(R[1,0], R[0,0])  (xoay quanh truc quang camera)
    # Quy uoc dau khop he robot (trai duong) xu ly o agv_pose.
    # ---------------------------------------------------------------
    def marker_yaw(self, rvec):
        R, _ = cv2.Rodrigues(rvec)
        # Goc xoay marker quanh truc nhin camera. KHONG dao dau -> marker CUNG
        # chieu IMU: quay trai = yaw tang, quay phai = yaw giam. Toan he (IMU,
        # marker, do_turn, planner) dung chung quy uoc nay -> het mau thuan dau.
        yaw = math.atan2(R[1, 0], R[0, 0])
        return yaw

    # ---------------------------------------------------------------
    # Biến đổi tvec (hệ camera) -> hệ robot (mặt sàn) - KHÔNG cần pitch
    #
    # Ý tưởng: marker nằm trên sàn, camera ở độ cao h cố định, nên thành
    # phần thẳng đứng của vector camera->marker LUÔN = h (bất kể nghiêng).
    #   d3d   = |tvec|                          (khoảng cách 3D, rất tin cậy)
    #   horiz = sqrt(d3d^2 - h^2)               (khoảng cách ngang trên sàn)
    #   x_c   = thành phần trái/phải (không bị pitch ảnh hưởng nếu không roll)
    #   forward_cam = sqrt(horiz^2 - x_c^2)     (thành phần tiến)
    # Sau đó cộng offset để quy về tâm robot.
    # ---------------------------------------------------------------
    def cam_to_robot(self, tvec):
        x_c, y_c, z_c = float(tvec[0]), float(tvec[1]), float(tvec[2])

        d3d = math.sqrt(x_c ** 2 + y_c ** 2 + z_c ** 2)
        h = self.camera_height

        horiz2 = d3d ** 2 - h ** 2
        horiz = math.sqrt(horiz2) if horiz2 > 0 else 0.0

        fwd2 = horiz ** 2 - x_c ** 2
        forward_cam = math.sqrt(fwd2) if fwd2 > 0 else 0.0

        x_robot = forward_cam + self.camera_offset_forward  # tiến (về tâm robot)
        y_robot = -x_c                                      # trái dương
        z_robot = 0.0                                       # giả định trên sàn
        return x_robot, y_robot, z_robot

    # ---------------------------------------------------------------
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids = self.detect_markers(gray)

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

                for i in range(len(ids)):
                    tvec, rvec = self.estimate_pose(corners[i])
                    if tvec is None:
                        continue

                    # Biến đổi sang hệ robot trên mặt sàn
                    x_robot, y_robot, z_robot = self.cam_to_robot(tvec)

                    # Yaw marker = huong xe so voi map (marker dan canh tren ve +x)
                    yaw_marker = self.marker_yaw(rvec)

                    # Khoảng cách trên mặt sàn từ tâm robot tới marker
                    dist = math.sqrt(x_robot ** 2 + y_robot ** 2)
                    # Góc lệch: dương = marker lệch sang TRÁI của robot
                    angle = math.degrees(math.atan2(y_robot, x_robot))

                    if self.debug:
                        # d3d = khoảng cách 3D camera->marker (kiểm tra pose).
                        d3d = float(np.linalg.norm(tvec))
                        # Nếu dist vẫn lệch: kiểm tra d3d có hợp lý không
                        # (vd ~0.35 cho trường hợp của bạn), và xem dấu offset.
                        self.get_logger().info(
                            f"tvec=({tvec[0]:.3f},{tvec[1]:.3f},{tvec[2]:.3f}) "
                            f"d3d={d3d:.3f} | robot x={x_robot:.3f} "
                            f"y={y_robot:.3f} dist={dist:.3f}")

                    # Publish
                    info_msg = ArucoInfo()
                    info_msg.id = int(ids[i][0])
                    info_msg.distance = float(dist)
                    info_msg.angle = float(angle)
                    info_msg.x = float(x_robot)
                    info_msg.y = float(y_robot)
                    info_msg.yaw = float(yaw_marker)   # huong xe so voi map (rad)
                    self.publisher_.publish(info_msg)

                    # Hiển thị
                    cv2.putText(cv_image, f"ID:{info_msg.id} | D:{dist:.2f}m",
                                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(cv_image, f"Angle: {angle:.1f} deg",
                                (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    cv2.putText(cv_image, f"Pos X:{x_robot:.2f} Y:{y_robot:.2f}",
                                (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    cv2.putText(cv_image, f"Yaw: {math.degrees(yaw_marker):.1f} deg",
                                (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

            cv2.imshow(self.window_name, cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Loi: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()