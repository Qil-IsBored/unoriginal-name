#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

class TrackerNode:

    def __init__(self):

        rospy.loginfo("Starting tracker node...")

        self.bridge = CvBridge()

        # FRAME CENTER
        self.frame_width = 320
        self.frame_height = 240

        self.frame_center_x = self.frame_width // 2
        self.frame_center_y = self.frame_height // 2

        # STORE LATEST CONE POSITION
        self.cone_x = None
        self.cone_y = None

        self.alpha = 0.3  # smoothing factor

        self.smoothed_x = None
        self.smoothed_y = None

        rospy.loginfo("Frame center X: {}".format(self.frame_center_x))
        rospy.loginfo("Frame center Y: {}".format(self.frame_center_y))

        # SUBSCRIBE
        self.center_sub = rospy.Subscriber("/cone_detection/center_point", Point, self.center_callback)

        self.image_sub = rospy.Subscriber("/cone_detection/image", Image, self.image_callback)

        # ERROR PUBLISHER
        self.error_pub = rospy.Publisher( "/cone_tracking/error", Point, queue_size=10)

        # PUBLISHER (final visualization)
        self.image_pub = rospy.Publisher( "/cone_tracking/image", Image, queue_size=1)

        rospy.loginfo("Tracker node started.")


    def center_callback(self, msg):

        # cone position from detector
        self.cone_x = int(msg.x)
        self.cone_y = int(msg.y)

        # EMA 
        if self.smoothed_x is None:
            self.smoothed_x = self.cone_x
            self.smoothed_y = self.cone_y

        else:
            # EMA smoothing
            self.smoothed_x = int(self.alpha * self.cone_x + (1 - self.alpha) * self.smoothed_x)
            self.smoothed_y = int(self.alpha * self.cone_y + (1 - self.alpha) * self.smoothed_y)

        self.cone_x = self.smoothed_x
        self.cone_y = self.smoothed_y

        # CALCULATE ERROR
        error_x = self.cone_x - self.frame_center_x
        error_y = self.cone_y - self.frame_center_y

        # CREATE ERROR MESSAGE
        error_msg = Point()
        error_msg.x = error_x
        error_msg.y = error_y
        error_msg.z = 0

        # PUBLISH ERROR
        self.error_pub.publish(error_msg)

        # PRINT DEBUG
        # rospy.loginfo("Cone X: {} | Error X: {}".format( cone_x, error_x))
        # rospy.loginfo("Cone Y: {} | Error Y: {}".format( cone_y, error_y))

        rospy.loginfo("Error X: {} | Error Y: {}".format(error_x, error_y))

    def image_callback(self, msg):

        # convert image
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        except Exception as e:
            rospy.logerr("CV Bridge Error: {}".format(e))
            return

        # draw frame center (blue)
        cv2.circle( frame, (self.frame_center_x, self.frame_center_y), 6, (255, 0, 0), -1)

        # draw cone + line only if detected
        if self.cone_x is not None and self.cone_y is not None:

            # # cone center (red)
            # cv2.circle(frame, (self.cone_x, self.cone_y), 6, (0, 0, 255), -1)

            # line between frame center and cone
            cv2.line( frame, (self.frame_center_x, self.frame_center_y), (self.cone_x, self.cone_y), (0, 255, 255), 2)

        # publish final image
        out_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.image_pub.publish(out_msg)


if __name__ == "__main__":

    rospy.init_node("tracker_node")

    node = TrackerNode()

    rospy.spin()