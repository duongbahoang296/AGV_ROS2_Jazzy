from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'agv_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hoang',
    maintainer_email='duongbahoang296@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_localization_node = agv_localization.aruco_localization_node:main',
            'aruco_mission_node = agv_localization.aruco_mission_node:main',
            'agv_pose_node = agv_localization.agv_pose_node:main',
            'path_follower_node = agv_localization.path_follower_node:main',
            'stuck_monitor_node = agv_localization.stuck_monitor_node:main',
            'wheel_odom_node = agv_localization.wheel_odom_node:main',
            'mqtt_ros2_bridge = agv_localization.mqtt_ros2_bridge:main',

        ],
    },
)