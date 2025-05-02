#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32, Polygon #geometry_msgs not in CMake file

# import your color segmentation algorithm; call this function in ros_image_callback!
from computer_vision.line_segmentation import line_segmentation, transformUvToXy
import math

class LineDetector(Node):
    def __init__(self):
        super().__init__("line_detector")
        # Subscribe to ZED camera RGB frames
        self.line_pub  = self.create_publisher(Polygon, "/lines_px", 10)
        self.debug_pub = self.create_publisher(Image, "/line_debug_img", 10)
        self.lookahead_pub = self.create_publisher(Point32, "/lookahead_point", 10)
        
        
        self.image_sub = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.image_callback, 5)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

        self.get_logger().info("Line Detector Initialized")

    def image_callback(self, image_msg):
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        # left_line/right_line = [[1, 12, 34, 56], [156, 561, 23 ,55]] # [[x1 y1 x2 y2]]
        # transformed = [((x1, y1), (x2, y2))]
        right_line, left_line, right_transformed, left_transformed = line_segmentation(image)
        #self.get_logger().info(f'{left_line=}, {right_line=}, {left_transformed=}, {right_transformed=}')
        lines = [left_line, right_line]


        line_pixel_msg = Polygon()
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
        color = 0
        if lines is not None:
            for l in lines:
                # publish lines for rosbag
                if l is not None:
                    line = l
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
                    cv2.line(image, (line[0], line[1]), (line[2], line[3]), colors[color % len(colors)], 5)
                    color = color + 1


            # Publish relative xy position of object in real world
            if left_transformed is None or right_transformed is None:
                '''
                # NEW STUFF
                if left_transformed is not None:
                    x_px, y_px = self.perp_bisector_endpoint(left_line, 100, direction="increasing")
                    cv2.circle(image, (int(x_px), int(y_px)), 10, (255,0,255), -1)
                    lookahead_x, lookahead_y = transformUvToXy(x_px, y_px)

                elif right_transformed is not None:
                    x_px, y_px = self.perp_bisector_endpoint(right_line, 100, direction="decreasing")
                    cv2.circle(image, (int(x_px), int(y_px)), 10, (255,0,255), -1)
                    lookahead_x, lookahead_y = transformUvToXy(x_px, y_px)
                else:
                    lookahead_x, lookahead_y = (1, 0)

                lookahead_point_msg = Point32()
                lookahead_point_msg.x = float(lookahead_x)
                lookahead_point_msg.y = float(lookahead_y)
                lookahead_point_msg.z = 0.0
                self.lookahead_pub.publish(lookahead_point_msg)
                '''
                
                # OLD STUFF
                lookahead_point_msg = Point32()
                lookahead_point_msg.x = 0.0
                lookahead_point_msg.y = 0.0
                lookahead_point_msg.z = 0.0
                
            else:
            	# OLD ALGORITHM
                # lookahead_x = float((left_transformed[1][0] + right_transformed[1][0])/2.0)
                # lookahead_y = float((left_transformed[1][1] + right_transformed[1][1])/2.0)
                #self.get_logger().info(f'{lookahead_x=}, {lookahead_y=}')
                

                # STRATEGY 1
                # left_mid = ((left_transformed[0][0] + left_transformed[1][0])/2, (left_transformed[0][1] + left_transformed[1][1])/2)
                # right_mid = ((right_transformed[0][0] + right_transformed[1][0])/2, (right_transformed[0][1] + right_transformed[1][1])/2)
                # lookahead_x = float((left_mid[0] + right_mid[0])/2.0)
                # lookahead_y = float((left_mid[1] + right_mid[1])/2.0)

                # STRATEGY 2
                # x_mid_px = (left_line[0] + left_line[2]) / 4.0 + (right_line[0] + right_line[2]) / 4.0 
                # y_mid_px = (left_line[1] + left_line[3]) / 4.0 + (right_line[1] + right_line[3]) / 4.0 
                # cv2.circle(image, (int(x_mid_px), int(y_mid_px)), 10, (0,255,255), -1)
                # lookahead_x, lookahead_y = transformUvToXy(x_mid_px, y_mid_px)

                # STRATEGY 3
                I = self.intersection(left_line, right_line)
                B = self.bisector_endpoint_int(I, (right_line[0], right_line[1]), (left_line[0], left_line[1]), 50)
                # cv2.circle(image, (int(I[0]), int(I[1])), 10, (0,0,255), -1)
                # cv2.line(image, (int(I[0]),int(I[1])), (int(B[0]),int(B[1])), (0,0,255), 2)
                cv2.circle(image, (int(B[0]), int(B[1])), 10, (0,0,255), -1)
                lookahead_x, lookahead_y = transformUvToXy(B[0], B[1])
                # MIDPOINT
                # lookahead_x, lookahead_y = transformUvToXy(I[0], I[1])
                
                lookahead_point_msg = Point32()
                lookahead_point_msg.x = float(lookahead_x)
                lookahead_point_msg.y = float(lookahead_y)
                lookahead_point_msg.z = 0.0
                self.lookahead_pub.publish(lookahead_point_msg)
            
            debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.debug_pub.publish(debug_msg)
            self.line_pub.publish(line_pixel_msg)
                
    def intersection(self, seg1, seg2):
        """
        seg1, seg2 : [x1, y1, x2, y2]
        Returns
        -------
        I : (xi, yi)  intersection of the two infinite lines
        B : (xb, yb)  endpoint of the internal-angle bisector,
                    clipped so xb ∈ [min_x, max_x] of the four endpoints.
        """
        x1, y1, x2, y2 = seg1
        x3, y3, x4, y4 = seg2

        # 1) compute intersection I
        a1, b1, c1 =  y2-y1,  x1-x2,  x2*y1 - x1*y2
        a2, b2, c2 =  y4-y3,  x3-x4,  x4*y3 - x3*y4
        denom = a1*b2 - a2*b1
        if abs(denom) < 1e-8:
            raise ValueError("Lines are parallel or coincident – no unique intersection.")
        xi = (b1*c2 - b2*c1) / denom
        yi = (c1*a2 - c2*a1) / denom
        # I = np.array([xi, yi], dtype=float)
        return (xi, yi)
    
    def bisector_endpoint_int(self, P, A, B, L):
        """
        P, A, B: array-like coordinates [x, y]
        L: desired length of the bisector
        Returns: tuple of ints (x, y) for endpoint C
        """
        P, A, B = map(np.asarray, (P, A, B))
        # unit vectors along PA and PB
        v1 = (A - P) / np.linalg.norm(A - P)
        v2 = (B - P) / np.linalg.norm(B - P)
        # bisector direction
        w = v1 + v2
        w_hat = w / np.linalg.norm(w)
        # endpoint (float)
        C_float = P + L * w_hat
        # round and convert to ints
        C_int = tuple(np.round(C_float).astype(int))
        return C_int
    
    def perp_bisector_endpoint(self, line, L, direction="increasing"):
        x1, y1, x2, y2 = line
        mx = (.25*x1 + .75*x2)
        my = (.25*y1 + .75*y2)

        dx = x2 - x1
        dy = y2 - y1

        perp1 = (-dy, dx)
        perp2 = ( dy, -dx)

        if direction == "increasing":
            vx, vy = perp1 if perp1[0] >= 0 else perp2
        elif direction == "decreasing":
            vx, vy = perp1 if perp1[0] <= 0 else perp2
        else:
            raise ValueError("direction must be 'increasing' or 'decreasing'")

        norm = math.hypot(vx, vy)
        if norm == 0:
            raise ValueError("Zero-length segment → perpendicular undefined")
        ux, uy = vx / norm, vy / norm

        xe = mx + ux * L
        ye = my + uy * L

        return xe, ye
       
def main(args=None):
    rclpy.init(args=args)
    line_detector = LineDetector()
    rclpy.spin(line_detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
