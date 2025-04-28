#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point32, Polygon #geometry_msgs not in CMake file

import math

class RacetrackController(Node):
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        super().__init__("parking_controller")

        self.declare_parameter("drive_topic")
        DRIVE_TOPIC = self.get_parameter("drive_topic").value # set in launch file; different for simulator vs racecar

        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)

        self.create_subscription(Point32, "/lookahead_point", 
            self.lookahead_point_callback, 1)

        self.parking_distance = 0.05 # meters; try playing with this number!
        self.acceptable_angle = 10 * math.pi / 180
        self.relative_x = 0
        self.relative_y = 0

        self.L = 0.1
        self.L_ah = 0.33
        self.speed = 1.0
        self.backwards = False

        self.get_logger().info("Parking Controller Initialized")

    def lookahead_point_callback(self, msg):
        self.relative_x = msg.x
        self.relative_y = msg.y
        self.get_logger().info(f"relative x {self.relative_x}")
        self.get_logger().info(f"relative y {self.relative_y}")
        eta = math.atan(self.relative_y / self.relative_x)
        delta = math.atan(2 * self.L * math.sin(eta) / self.L_ah)

        current_time = self.get_clock().now()
        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.frame_id = "base_link"
        drive_cmd.header.stamp = current_time.to_msg()

        drive_cmd.drive.speed = 1 * self.speed
        drive_cmd.drive.steering_angle = delta

        self.drive_pub.publish(drive_cmd)


def main(args=None):
    rclpy.init(args=args)
    pc = RacetrackController()
    rclpy.spin(pc)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
