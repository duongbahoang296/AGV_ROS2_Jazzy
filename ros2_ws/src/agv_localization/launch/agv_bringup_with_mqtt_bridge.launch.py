"""
agv_bringup_with_mqtt_bridge.launch.py
----------------------------------------
Ví dụ tích hợp mqtt_ros2_bridge vào launch file chung của hệ thống AGV.

Cách dùng thực tế: copy đoạn Node(...) bên dưới vào launch file bringup
hiện tại của bạn (file khởi động cùng lúc driver AGV, navigation, v.v.),
đặt cạnh các node khác. File này chỉ là ví dụ độc lập để bạn xem cấu trúc.
"""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    mqtt_bridge_node = Node(
        package="agv_localization",          # đổi thành package thực tế chứa file mqtt_ros2_bridge.py
        executable="mqtt_ros2_bridge",        # tên entry point khai báo trong setup.py (xem ghi chú bên dưới)
        name="mqtt_ros2_bridge",
        output="screen",
        parameters=[{
            "broker_host": "20495f1f8e264ece8e2a1cc781d62182.s1.eu.hivemq.cloud",
            "broker_port": 8883,
            "username": "agv",
            "password": "agv12345678",
            "mqtt_topic": "agv/tasks/request",
            "use_tls": True,
            "ros_topic": "/agv/tasks",
        }],
        # respawn=True để ROS2 tự khởi động lại node này nếu nó bị crash
        # (ví dụ mất kết nối mạng nghiêm trọng), không ảnh hưởng các node khác.
        respawn=True,
        respawn_delay=3.0,
    )

    # -----------------------------------------------------------------
    # Ở đây bạn thêm các node hiện có của hệ thống AGV, ví dụ:
    #
    # agv_driver_node = Node(package="agv_driver", executable="agv_driver_node", ...)
    # navigation_node = Node(package="agv_nav", executable="nav_node", ...)
    #
    # rồi add tất cả vào cùng LaunchDescription bên dưới.
    # -----------------------------------------------------------------

    return LaunchDescription([
        mqtt_bridge_node,
        # agv_driver_node,
        # navigation_node,
    ])