#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
import tf_transformations


class PurePursuit(Node):

    def __init__(self):

        super().__init__('pure_pursuit_controller')

        self.lookahead_distance = 0.5
        self.speed = 0.2

        self.goal_reached = False

        self.current_waypoint_index = 0

        self.reached_waypoints = set()
        self.waypoint_tolerance = 2.0

        self.current_pose = None
        self.path = []

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        self.create_subscription(
            Path,
            '/global_path',
            self.path_callback,
            10)

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        self.timer = self.create_timer(
            0.1,
            self.control_loop)

    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        orientation_list = [q.x, q.y, q.z, q.w]

        (_, _, yaw) = tf_transformations.euler_from_quaternion(orientation_list)

        self.current_pose = (x, y, yaw)

    def path_callback(self, msg):

        self.path = []

        for pose in msg.poses:

            px = pose.pose.position.x
            py = pose.pose.position.y

            self.path.append((px, py))

    def check_waypoint_reached(self):

        if self.current_pose is None:
            return

        if len(self.path) == 0:
            return

        x, y, _ = self.current_pose

        for i, (wx, wy) in enumerate(self.path):

            distance = math.sqrt((wx - x)**2 + (wy - y)**2)

            if distance <= self.waypoint_tolerance and i not in self.reached_waypoints:

                self.reached_waypoints.add(i)

                self.get_logger().info(
                    f"Reached waypoint {i} at ({wx:.2f}, {wy:.2f})"
                )


    def get_lookahead_point(self):

        if self.current_pose is None:
            return None

        if len(self.path) == 0:
            return None

        x, y, _ = self.current_pose

        # move forward in the waypoint list
        while self.current_waypoint_index < len(self.path):

            px, py = self.path[self.current_waypoint_index]

            dist = math.sqrt((px-x)**2 + (py-y)**2)

            if dist > self.lookahead_distance:
                return px, py

            # waypoint passed → go to next
            self.current_waypoint_index += 1

        return None

    def control_loop(self):

        if self.goal_reached:
            return

        if self.current_pose is None:
            return

        if len(self.path) == 0:
            return

        self.check_waypoint_reached()
        
        lookahead = self.get_lookahead_point()

        if lookahead is None:
            return

        lx, ly = lookahead

        x, y, yaw = self.current_pose

        dx = lx - x
        dy = ly - y

        local_x = math.cos(-yaw)*dx - math.sin(-yaw)*dy
        local_y = math.sin(-yaw)*dx + math.cos(-yaw)*dy

        if local_x == 0:
            return

        # STOP ROBOT AT FINAL WAYPOINT
        last_x, last_y = self.path[-1]

        x, y, _ = self.current_pose

        dist_to_goal = math.sqrt((last_x - x)**2 + (last_y - y)**2)

        if dist_to_goal < 1.0:

            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)

            self.get_logger().info("Final waypoint reached. Rover stopped.")

            self.goal_reached = True
            
            return


        curvature = (2*local_y)/(self.lookahead_distance**2)

        cmd = Twist()

        cmd.linear.x = self.speed
        cmd.angular.z = curvature * self.speed

        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = PurePursuit()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
