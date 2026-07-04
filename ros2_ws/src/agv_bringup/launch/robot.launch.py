from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    loc_pkg = get_package_share_directory('agv_localization')
    pkg_path = get_package_share_directory('agv_bringup')
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot_base.urdf')
    # robot_description_content = Command(['xacro ', urdf_file])

    # Ban do marker (cai qua data_files -> nam o share/agv_localization/config)
    marker_map = os.path.join(loc_pkg, 'config', 'markers.yaml')

    # Co bat/tat path_follower. Mac dinh BAT (chay chung cho tien).
    #   ros2 launch agv_bringup robot.launch.py                      -> co path_follower
    #   ros2 launch agv_bringup robot.launch.py run_follower:=false  -> KHONG (de chay tay)
    run_follower = LaunchConfiguration('run_follower')

    return LaunchDescription([

        DeclareLaunchArgument(
            'run_follower', default_value='true',
            description='Chay path_follower_node chung trong launch (false de chay tay)'),

        # 1. Micro-ROS Agent
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyACM0'],
            output='screen'
        ),

        # 2. Driver Motor
        Node(
            package='agv_motor_driver',
            executable='motor_driver',
            output='screen'
        ),

        # 3. Camera
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            output='screen'
        ),

        # 4. TF tu base_link -> camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_broadcaster',
            arguments=[
                '--x', '0.25',
                '--y', '0.0',
                '--z', '0.0',
                '--yaw', '0',
                '--pitch', '0',
                '--roll', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link'
            ]
        ),

        # 5. Aruco Localization (detector -> /aruco_info)
        Node(
            package='agv_localization',
            executable='aruco_localization_node',
            output='screen'
        ),

        # ============================================================
        # CAC NODE AN TOAN (chi doc & publish, KHONG dieu khien dong co)
        # -> gop vao launch cho tien. Tat ca van la node da verify.
        # ============================================================

        # 6. Localization: ID marker -> pose the gioi (/agv/odom + TF)
        Node(
            package='agv_localization',
            executable='agv_pose_node',
            output='screen',
            parameters=[{'marker_map_file': marker_map}]
        ),

        # 7. Stuck monitor: phat hien ket/truot -> /agv/stuck
        Node(
            package='agv_localization',
            executable='stuck_monitor_node',
            output='screen'
        ),

        # ------------------------------------------------------------
        # 8. Wheel odom: CHUA VERIFY -> de comment, mo sau khi do xong
        #    dau x/y va ppr that. Day la node an toan (chi publish odom).
        # ------------------------------------------------------------
        Node(
            package='agv_localization',
            executable='wheel_odom_node',
            output='screen',
            parameters=[{
                'invert_left': False,
                'invert_right': False,
                'swap_lr': False,
                'ppr': 6400.0,
            }]
        ),

        # ------------------------------------------------------------
        # 9. Path follower: NODE DUY NHAT RA LENH DONG CO (/cmd_vel).
        #    Gop vao launch cho tien (mac dinh BAT). Tat de chay tay:
        #        ros2 launch agv_bringup robot.launch.py run_follower:=false
        #        ros2 run agv_localization path_follower_node \
        #            --ros-args -p marker_map_file:=<duong_dan markers.yaml>
        # ------------------------------------------------------------
        Node(
            package='agv_localization',
            executable='path_follower_node',
            output='screen',
            parameters=[{'marker_map_file': marker_map}],
            condition=IfCondition(run_follower)
        ),

        # 10. Aruco mission (cu - tranh /cmd_vel voi path follower, BO)
        # Node(
        #     package='agv_localization',
        #     executable='aruco_mission_node',
        #     output='screen'
        # ),

    ])