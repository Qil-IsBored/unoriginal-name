#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class LineGateFollower:

    def __init__(self):

        rospy.init_node("line_gate_follower")

        self.bridge = CvBridge()

        # Camera topic (change if needed: /main_camera/image_raw etc.)
        self.image_sub = rospy.Subscriber("/main_camera/image_raw", Image, self.image_callback)

        # Velocity publisher
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.twist = Twist()

        # Control gains
        self.kp_line = 0.004
        self.kp_gate = 0.003

        rospy.loginfo("Line + Gate follower started")

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w, _ = frame.shape

        # =========================
        # REGION OF INTEREST (bottom half for line)
        # =========================
        roi = frame[int(h/2):h, 0:w]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # =========================
        # BLACK LINE DETECTION
        # =========================
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 60])

        mask_line = cv2.inRange(hsv, lower_black, upper_black)

        M = cv2.moments(mask_line)

        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            error = cx - (w // 2)

            # Move forward while correcting left/right
            self.twist.linear.x = 0.5
            self.twist.angular.z = -error * self.kp_line

        else:
            # No line found → slow scan
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.5

        # =========================
        # MAGENTA HOOP DETECTION
        # =========================
        lower_magenta = np.array([140, 100, 100])
        upper_magenta = np.array([170, 255, 255])

        mask_gate = cv2.inRange(hsv, lower_magenta, upper_magenta)

        contours, _ = cv2.findContours(mask_gate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            area = cv2.contourArea(c)

            if area > 2000:  # filter noise
                x, y, gw, gh = cv2.boundingRect(c)
                gate_cx = x + gw // 2
                gate_error = gate_cx - (w // 2)

                # prioritize gate correction
                self.twist.linear.x = 0.7
                self.twist.angular.z = -gate_error * self.kp_gate

                break  # only use biggest gate

        # =========================
        # PUBLISH COMMAND
        # =========================
        self.cmd_pub.publish(self.twist)


if __name__ == "__main__":
    try:
        node = LineGateFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass