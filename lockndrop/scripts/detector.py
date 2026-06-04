#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge


class ConeDetector:

    def __init__(self):

        rospy.init_node("cone_detector")

        self.bridge = CvBridge()

        # SUBSCRIBE to throttled camera
        # self.image_sub = rospy.Subscriber("/front_camera/iamge_throttled", Image, self.image_callback)
        self.image_sub = rospy.Subscriber("/front_camera/image_raw", Image, self.image_callback)

        # PUBLISH debug image
        self.image_pub = rospy.Publisher("/cone_detection/image", Image, queue_size=1)

        # PUBLISH cone center
        self.center_pub = rospy.Publisher("/cone_detection/center_point", Point, queue_size=1)

        # PUBLISH mask
        self.mask_pub = rospy.Publisher("/cone_detection/mask", Image, queue_size=1)

        # Publish debug visualization
        # self.debug_pub = rospy.Publisher("/cone/debug", Image, queue_size=1)

        rospy.loginfo("Cone detector started")


    def image_callback(self, msg):

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        except Exception as e:
            rospy.logerr("CV Bridge Error: %s", e)
            return
        
        frame = cv2.GaussianBlur(frame, (11, 11), 0)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ORANGE COLOR RANGE
        # lower_orange = np.array([5, 100, 100])
        # upper_orange = np.array([20, 255, 255])

        # Better orange range
        lower_orange = np.array([3, 80, 60])
        upper_orange = np.array([25, 255, 255])

        # Create mask
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Remove noise
        kernel = np.ones((5,5), np.uint8)
        # mask = cv2.erode(mask, kernel, iterations=1)
        # mask = cv2.dilate(mask, kernel, iterations=2)

        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # If cone found
        if contours:

            # Largest contour
            largest_contour = max(contours, key=cv2.contourArea)

            # Convex Hull
            hull = cv2.convexHull(largest_contour)

            # countour by largest area
            # area = cv2.contourArea(largest_contour)

            # countour by Convex hull area
            area = cv2.contourArea(hull)

            # Ignore tiny noise
            if area > 300:

                # x, y, w, h = cv2.boundingRect(largest_contour)
                x, y, w, h = cv2.boundingRect(hull)

                # Center point
                center_x = x + w // 2
                center_y = y + h // 2

                # Draw rectangle
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Draw convex hull shape
                # cv2.drawContours(frame, [hull], -1, (255, 0, 0), 2)

                # Draw center point
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

                # Publish center
                point = Point()
                point.x = center_x
                point.y = center_y
                point.z = area

                self.center_pub.publish(point)

                rospy.loginfo(f"Cone detected at X:{center_x} Y:{center_y}")

        # Publish mask
        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")

        self.mask_pub.publish(mask_msg)
        
        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")

        self.image_pub.publish(debug_msg)



if __name__ == "__main__":

    detector = ConeDetector()

    rospy.spin()