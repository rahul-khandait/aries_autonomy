#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

import math
from tf_transformations import euler_from_quaternion


class IMUOdomValidator(Node):

    def __init__(self):

        super().__init__('imu_odom_validator')

        self.imu_yaw = None
        self.odom_yaw = None

        self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

    def imu_callback(self, msg):

        q = msg.orientation

        orientation_list = [q.x, q.y, q.z, q.w]

        _, _, yaw = euler_from_quaternion(orientation_list)

        self.imu_yaw = yaw

        self.compare()

    def odom_callback(self, msg):

        q = msg.pose.pose.orientation

        orientation_list = [q.x, q.y, q.z, q.w]

        _, _, yaw = euler_from_quaternion(orientation_list)

        self.odom_yaw = yaw

        self.compare()

    def compare(self):

        if self.imu_yaw is None or self.odom_yaw is None:
            return

        difference = abs(self.imu_yaw - self.odom_yaw)

        self.get_logger().info(
            f"IMU yaw: {self.imu_yaw:.3f} | ODOM yaw: {self.odom_yaw:.3f} | Difference: {difference:.3f}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = IMUOdomValidator()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
