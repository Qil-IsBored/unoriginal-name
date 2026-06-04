#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Point
from clover import srv


class AltitudeController:

    def __init__(self):

        rospy.init_node("altitude_controller")

        self.error_y = 0

        self.deadzone = 25

        rospy.Subscriber(
            "/cone_tracking/error",
            Point,
            self.error_callback
        )

        self.set_velocity = rospy.ServiceProxy(
            "set_velocity",
            srv.SetVelocity
        )

        rospy.on_shutdown(self.stop_drone)

        rospy.loginfo("Altitude controller started")

    def error_callback(self, msg):

        self.error_y = msg.y

        vz = -self.error_y * 0.0025

        vz = max(min(vz, 0.25), -0.25)

        if abs(self.error_y) < self.deadzone:
            vz = 0

        try:

            self.set_velocity(
                vx=0,
                vy=0,
                vz=vz,
                frame_id='body'
            )

            rospy.loginfo(
                "EY: %.1f | VZ: %.3f",
                self.error_y,
                vz
            )

        except rospy.ServiceException as e:

            rospy.logerr(e)

    def stop_drone(self):

        try:

            self.set_velocity(
                vx=0,
                vy=0,
                vz=0,
                frame_id='body'
            )

        except:
            pass


if __name__ == "__main__":

    AltitudeController()

    rospy.spin()