#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32, Polygon #geometry_msgs not in CMake file

# import your color segmentation algorithm; call this function in ros_image_callback!
<<<<<<< HEAD
from racetrack.racetrack.computer_vision.line_segmentation import cd_color_segmentation
=======
from computer_vision.color_segmentation import line_segmentation
>>>>>>> 386787a (fixed ros nodes)


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
        lines = [[1, 12, 34, 56], [156, 561, 23 ,55]] # [[x1 y1 x2 y2]]
        # lines = line_segmentation(image, None)

        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.debug_pub.publish(debug_msg)
        line_pixel_msg = Polygon()
        for l in lines:
            first_point = Point32()
            first_point.x = l[0]
            first_point.y = l[1]
            first_point.z = 0

            second_point = Point32()
            second_point.x = l[2]
            second_point.y = l[3]
            second_point.z = 0

            line_pixel_msg.points.append(first_point)
            line_pixel_msg.points.append(second_point)
            
        self.line_pub.publish(line_pixel_msg)

def main(args=None):
    rclpy.init(args=args)
    line_detector = LineDetector()
    rclpy.spin(line_detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
