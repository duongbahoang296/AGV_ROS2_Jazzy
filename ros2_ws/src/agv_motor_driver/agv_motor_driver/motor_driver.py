#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


class MotorDriver(Node):

    def __init__(self):

        super().__init__('motor_driver')

        self.r = 0.0725
        self.L = 0.65
        self.ppr = 6400.0

        self.pub = self.create_publisher(
            Vector3,
            '/wheel_speed',
            10
        )

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.get_logger().info(
            'Motor driver started'
        )

    def cmd_callback(self, msg):

        v = msg.linear.x
        w = msg.angular.z

        omega_left = (v - (self.L / 2.0) * (-w)) / self.r # Đảo dấu w
        omega_right = (v + (self.L / 2.0) * (-w)) / self.r # Đảo dấu w

        left_steps = (
            omega_left *
            self.ppr /
            (2.0 * math.pi)
        )

        right_steps = (
            omega_right *
            self.ppr /
            (2.0 * math.pi)
        )

        wheel_msg = Vector3()

        wheel_msg.x = left_steps
        wheel_msg.y = right_steps
        wheel_msg.z = 0.0

        self.pub.publish(wheel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
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