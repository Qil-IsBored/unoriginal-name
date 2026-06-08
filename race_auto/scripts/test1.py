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

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.image_sub = rospy.Subscriber("/main_camera/image_raw", Image, self.image_callback)

        self.twist = Twist()

        self.frame = None

        # gains
        self.kp_line = 0.004
        self.kp_gate = 0.003

        self.rate = rospy.Rate(20)  # IMPORTANT: constant control loop

        rospy.loginfo("AUTO Line + Gate follower started")

        self.main_loop()

    def image_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def process_frame(self):

        if self.frame is None:
            return

        frame = self.frame.copy()
        h, w, _ = frame.shape

        roi = frame[int(h/2):h, 0:w]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # ================= LINE =================
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

        # ================= GATE =================
        lower_magenta = np.array([140, 100, 100])
        upper_magenta = np.array([170, 255, 255])

        mask_gate = cv2.inRange(hsv, lower_magenta, upper_magenta)

        contours, _ = cv2.findContours(mask_gate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            area = cv2.contourArea(c)

            if area > 2000:
                x, y, w2, h2 = cv2.boundingRect(c)
                gate_cx = x + w2 // 2
                error = gate_cx - (w // 2)

                # OVERRIDE line control when gate detected
                self.twist.linear.x = 0.8
                self.twist.angular.z = -error * self.kp_gate

                break

        # fallback if nothing detected
        if not line_detected and len(contours) == 0:
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.4

    def main_loop(self):

        while not rospy.is_shutdown():

            self.process_frame()

            # publish continuously (VERY IMPORTANT for Clover)
            self.cmd_pub.publish(self.twist)

            self.rate.sleep()


if __name__ == "__main__":
    try:
        LineGateFollower()
    except rospy.ROSInterruptException:
        pass