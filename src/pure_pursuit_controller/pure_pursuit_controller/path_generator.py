#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class PathGenerator(Node):

    def __init__(self):

        super().__init__('path_generator')

        self.publisher = self.create_publisher(Path, '/global_path', 10)

        self.timer = self.create_timer(1.0, self.publish_path)

        # Example waypoint list
        self.waypoints = [
            (0.0,0.0),
            (1.0,0.0),
            (2.0,0.5),
            (3.0,1.2),
            (4.0,2.0),
            (5.0,3.0),
            (6.0,4.0),
            (7.0,5.0),
            (7.5,6.0),
            (7.5,6.5),
            (7.0,7.0),
            (6.0,7.0),
            (5.0,6.0),
            (3.5,4.5),
            (2.5,4.0),
            (2.0,4.0),
            (1.5,4.5),
            (1.0,5.0),
            (0.5,5.5),
            (0.0,6.0)
        ]


    def publish_path(self):

        path_msg = Path()

        path_msg.header.frame_id = "map"

        for wp in self.waypoints:

            pose = PoseStamped()

            pose.header.frame_id = "map"

            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]

            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.publisher.publish(path_msg)

        self.get_logger().info("Publishing global path")


def main(args=None):

    rclpy.init(args=args)

    node = PathGenerator()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
