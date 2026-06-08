#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point
from cv_bridge import CvBridge


class LineGateConeFollower:

    def __init__(self):

        rospy.init_node("line_gate_cone_follower")

        self.bridge = CvBridge()

        # ========== PUBLISHERS ==========
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # cone outputs
        self.image_pub = rospy.Publisher("/cone_detection/image", Image, queue_size=1)
        self.center_pub = rospy.Publisher("/cone_detection/center_point", Point, queue_size=1)
        self.mask_pub = rospy.Publisher("/cone_detection/mask", Image, queue_size=1)

        # ========== SUBSCRIBER ==========
        self.image_sub = rospy.Subscriber("/main_camera/image_raw", Image, self.image_callback)

        self.twist = Twist()
        self.frame = None

        # gains
        self.kp_line = 0.004
        self.kp_gate = 0.003
        self.kp_cone = 0.003

        self.rate = rospy.Rate(20)

        rospy.loginfo("Line + Gate + Cone follower started")

        self.main_loop()

    def image_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("CV Bridge Error: %s", e)

    # ================= PROCESS FRAME =================
    def process_frame(self):

        if self.frame is None:
            return

        frame = self.frame.copy()
        h, w, _ = frame.shape

        roi = frame[int(h/2):h, 0:w]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # ==================================================
        # ================= LINE FOLLOW ====================
        # ==================================================
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 60])

        mask_line = cv2.inRange(hsv, lower_black, upper_black)
        M = cv2.moments(mask_line)

        line_detected = False

        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            error = cx - (w // 2)

            self.twist.linear.x = 0.6
            self.twist.angular.z = -error * self.kp_line
            line_detected = True

        # ==================================================
        # ================= GATE DETECTION =================
        # ==================================================
        lower_magenta = np.array([140, 100, 100])
        upper_magenta = np.array([170, 255, 255])

        mask_gate = cv2.inRange(hsv, lower_magenta, upper_magenta)
        contours_gate, _ = cv2.findContours(mask_gate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        gate_detected = False

        for c in contours_gate:
            area = cv2.contourArea(c)

            if area > 2000:
                x, y, w2, h2 = cv2.boundingRect(c)
                gate_cx = x + w2 // 2
                error = gate_cx - (w // 2)

                self.twist.linear.x = 0.8
                self.twist.angular.z = -error * self.kp_gate

                gate_detected = True
                break

        # ==================================================
        # ================= CONE DETECTION =================
        # ==================================================
        frame_blur = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv_full = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)

        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 120])

        mask_cone = cv2.inRange(hsv_full, lower_black, upper_black)

        kernel = np.ones((5, 5), np.uint8)
        mask_cone = cv2.morphologyEx(mask_cone, cv2.MORPH_CLOSE, kernel)

        h2, w2 = mask_cone.shape
        mask_cone[0:int(h2 * 0.6), :] = 0

        contours, _ = cv2.findContours(mask_cone, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            hull = cv2.convexHull(largest)
            area = cv2.contourArea(hull)

            if area > 300:
                x, y, cw, ch = cv2.boundingRect(hull)

                center_x = x + cw // 2
                center_y = y + ch // 2

                # publish cone center
                p = Point()
                p.x = center_x
                p.y = center_y
                p.z = area
                self.center_pub.publish(p)

                # override behavior slightly if cone detected
                error = center_x - (w // 2)
                self.twist.angular.z = -error * self.kp_cone
                self.twist.linear.x = 0.4

        # ==================================================
        # fallback behavior
        # ==================================================
        if not line_detected and not gate_detected and len(contours) == 0:
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.4

        # publish cone mask + debug
        mask_msg = self.bridge.cv2_to_imgmsg(mask_cone, "mono8")
        self.mask_pub.publish(mask_msg)

    # ================= MAIN LOOP =================
    def main_loop(self):

        while not rospy.is_shutdown():

            self.process_frame()
            self.cmd_pub.publish(self.twist)

            self.rate.sleep()


if __name__ == "__main__":
    try:
        LineGateConeFollower()
    except rospy.ROSInterruptException:
        pass