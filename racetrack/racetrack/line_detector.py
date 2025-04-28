#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32, Polygon #geometry_msgs not in CMake file

# import your color segmentation algorithm; call this function in ros_image_callback!
from computer_vision.line_segmentation import line_segmentation


class LineDetector(Node):
    def __init__(self):
        super().__init__("line_detector")
        # Subscribe to ZED camera RGB frames
        self.line_pub  = self.create_publisher(Polygon, "/lines_px", 10)
        self.debug_pub = self.create_publisher(Image, "/line_debug_img", 10)
        self.image_sub = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.image_callback, 5)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

        self.get_logger().info("Line Detector Initialized")

    def image_callback(self, image_msg):
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        # lines = [[1, 12, 34, 56], [156, 561, 23 ,55]] # [[x1 y1 x2 y2]]
        lines = line_segmentation(image)


        
        line_pixel_msg = Polygon()
        for l in lines:
            line = l[0]
            first_point = Point32()
            first_point.x = float(line[0])
            first_point.y = float(line[1])
            first_point.z = 0.0

            second_point = Point32()
            second_point.x = float(line[2])
            second_point.y = float(line[3])
            second_point.z = 0.0

            line_pixel_msg.points.append(first_point)
            line_pixel_msg.points.append(second_point)

            # Draw a line from (0, 0) to (511, 511) with blue color and thickness of 5
            cv2.line(image, (line[0], line[1]), (line[2], line[3]), (255, 0, 0), 5)
        
        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.debug_pub.publish(debug_msg)
        self.line_pub.publish(line_pixel_msg)

def main(args=None):
    rclpy.init(args=args)
    line_detector = LineDetector()
    rclpy.spin(line_detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
