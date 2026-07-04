#!/usr/bin/env python3
"""
mqtt_ros2_bridge.py
--------------------
Cầu nối giữa GUI web (gửi lệnh qua MQTT/HiveMQ) và AGV chạy ROS2.

Node này subscribe một topic MQTT (mặc định: agv/tasks/request), nhận payload
dạng JSON {"data": "3,11; 4,3; 8,2"} do GUI gửi lên, rồi publish lại thành
std_msgs/msg/String trên topic ROS2 /agv/tasks — tương đương với việc chạy:

    ros2 topic pub --once /agv/tasks std_msgs/msg/String "{data: '3,11; 4,3; 8,2'}"

nhưng tự động, không cần gõ tay mỗi lần.

Cấu hình được khai báo dưới dạng ROS2 parameters, nên node này có thể:
  1) Chạy độc lập để test:
        python3 mqtt_ros2_bridge.py --ros-args \
            -p broker_host:=xxxxxxxx.s1.eu.hivemq.cloud \
            -p broker_port:=8883 \
            -p username:=<hivemq_username> \
            -p password:=<hivemq_password> \
            -p mqtt_topic:=agv/tasks/request

  2) Hoặc được khai báo trong launch file cùng các node khác của AGV
     (xem file mẫu agv_bringup.launch.py đi kèm).

Cài đặt phụ thuộc:
    pip install paho-mqtt
"""

import json
import ssl
import sys
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import paho.mqtt.client as mqtt


DEFAULT_ROS_TOPIC = "/agv/tasks"


class MqttRos2Bridge(Node):
    def __init__(self):
        super().__init__("mqtt_ros2_bridge")

        # ---- Khai báo ROS2 parameters (đọc được từ launch file hoặc --ros-args -p) ----
        self.declare_parameter("broker_host", os.environ.get("AGV_MQTT_HOST", "broker.hivemq.com"))
        self.declare_parameter("broker_port", int(os.environ.get("AGV_MQTT_PORT", "8883")))
        self.declare_parameter("username", os.environ.get("AGV_MQTT_USER", ""))
        self.declare_parameter("password", os.environ.get("AGV_MQTT_PASS", ""))
        self.declare_parameter("mqtt_topic", os.environ.get("AGV_MQTT_TOPIC", "agv/tasks/request"))
        self.declare_parameter("use_tls", os.environ.get("AGV_MQTT_USE_TLS", "true").lower() != "false")
        self.declare_parameter("ros_topic", DEFAULT_ROS_TOPIC)

        self.broker_host = self.get_parameter("broker_host").value
        self.broker_port = self.get_parameter("broker_port").value
        self.username = self.get_parameter("username").value
        self.password = self.get_parameter("password").value
        self.mqtt_topic = self.get_parameter("mqtt_topic").value
        self.use_tls = self.get_parameter("use_tls").value
        self.ros_topic = self.get_parameter("ros_topic").value

        # ---- Publisher ROS2 -- tương đương lệnh ros2 topic pub thủ công ----
        self.publisher_ = self.create_publisher(String, self.ros_topic, 10)
        self.get_logger().info(f"Đã tạo publisher ROS2 trên topic '{self.ros_topic}'")

        # ---- Thiết lập MQTT client ----
        # LƯU Ý: paho-mqtt >= 2.0 bắt buộc phải chỉ định callback_api_version,
        # nếu không sẽ báo lỗi ngay khi khởi tạo Client().
        self.mqtt_client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"agv-ros2-bridge-{os.getpid()}",
            protocol=mqtt.MQTTv5,
        )

        if self.username:
            self.mqtt_client.username_pw_set(self.username, self.password)

        if self.use_tls:
            self.mqtt_client.tls_set(cert_reqs=ssl.CERT_REQUIRED)

        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect

        self.get_logger().info(
            f"Đang kết nối tới MQTT broker {self.broker_host}:{self.broker_port} ..."
        )

        # connect_async + loop_start(): không chặn (block) node nếu broker tạm
        # thời không tới được lúc khởi động (ví dụ mạng chưa sẵn sàng khi AGV
        # vừa bật nguồn). paho sẽ tự động thử kết nối lại trong nền, và node
        # ROS2 vẫn khởi động bình thường -- an toàn khi dùng trong launch file.
        try:
            self.mqtt_client.connect_async(self.broker_host, self.broker_port, keepalive=60)
            self.mqtt_client.loop_start()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Không thể khởi tạo kết nối MQTT: {e}")

    # ---------------- MQTT callbacks ----------------

    def on_mqtt_connect(self, client, userdata, flags, reason_code, properties=None):
        if reason_code == 0:
            self.get_logger().info("Kết nối MQTT thành công.")
            client.subscribe(self.mqtt_topic, qos=1)
            self.get_logger().info(f"Đã subscribe topic MQTT: '{self.mqtt_topic}'")
        else:
            self.get_logger().error(f"Kết nối MQTT thất bại, mã lỗi: {reason_code}")

    def on_mqtt_disconnect(self, client, userdata, disconnect_flags, reason_code, properties=None):
        self.get_logger().warn(f"Mất kết nối MQTT (mã: {reason_code}). Sẽ tự thử kết nối lại.")

    def on_mqtt_message(self, client, userdata, msg):
        raw = msg.payload.decode("utf-8", errors="replace")
        self.get_logger().info(f"Nhận từ MQTT [{msg.topic}]: {raw}")

        task_string = self._extract_task_string(raw)
        if task_string is None:
            self.get_logger().error(f"Payload không hợp lệ, bỏ qua: {raw}")
            return

        self.publish_task(task_string)

    @staticmethod
    def _extract_task_string(raw: str):
        """Chấp nhận payload JSON {"data": "..."} hoặc chuỗi thô "3,11; 4,3"."""
        raw = raw.strip()
        try:
            parsed = json.loads(raw)
            if isinstance(parsed, dict) and "data" in parsed:
                return str(parsed["data"])
        except json.JSONDecodeError:
            pass
        # fallback: coi cả payload là chuỗi nhiệm vụ thô
        if raw:
            return raw
        return None

    # ---------------- ROS2 publish ----------------

    def publish_task(self, task_string: str):
        msg = String()
        msg.data = task_string
        self.publisher_.publish(msg)
        self.get_logger().info(f"Đã publish lên '{self.ros_topic}': '{task_string}'")

    def destroy_node(self):
        try:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = MqttRos2Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())