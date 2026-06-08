#!/usr/bin/env python3

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class VisionNode:

    def __init__(self):

        rospy.loginfo("Starting vision node...")

        # Bridge between ROS Image <-> OpenCV image
        self.bridge = CvBridge()

        # Subscribe to FRONT CAMERA RAW IMAGE
        self.image_sub = rospy.Subscriber("/front_camera/image_raw", Image, self.image_callback)

        # Publish LIGHTWEIGHT / RESIZED IMAGE
        self.image_pub = rospy.Publisher("/front_camera/image_throttled", Image, queue_size=1)

        self.last_process_time = rospy.Time.now()

        self.process_interval = rospy.Duration(0.2)  # 5 FPS

        rospy.loginfo("Vision node ready.")

    def image_callback(self, msg):

        now = rospy.Time.now()

        # FPS throttling
        if now - self.last_process_time < self.process_interval:
            return

        self.last_process_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            height, width, channels = frame.shape

            rospy.loginfo(f"Raw image size: {width}x{height}")

        except Exception as e:
            rospy.logerr("CV Bridge Error: %s", e)
            return

        # Resize frame
        frame = cv2.resize(frame, (320, 240))

        throttled_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")

        self.image_pub.publish(throttled_msg)

        rospy.loginfo_throttle(2, "Publishing throttled front camera")


if __name__ == "__main__":

    rospy.init_node("vision_node")

    node = VisionNode()

    rospy.spin()