#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point

from clover import srv

class LineFollower:

    def __init__(self):

        rospy.init_node("line_follower")

        rospy.wait_for_service('set_velocity')

        self.set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)

        self.center_x = None

        rospy.Subscriber(
            "/cone_detection/center_point",
            Point,
            self.point_callback
        )

        rospy.loginfo("Line follower started (NO AUTO TAKEOFF)")

        self.control_loop()

    def point_callback(self, msg):
        self.center_x = msg.x

    def control_loop(self):

        rate = rospy.Rate(20)

        image_center = 160
        Kp = 0.003

        while not rospy.is_shutdown():

            if self.center_x is not None:

                error = self.center_x - image_center

                vx = 0.4
                vy = -Kp * error

                self.set_velocity(
                    vx=vx,
                    vy=vy,
                    vz=0.0,
                    yaw=float('nan'),
                    frame_id='body'
                )

            rate.sleep()


if __name__ == "__main__":
    LineFollower()