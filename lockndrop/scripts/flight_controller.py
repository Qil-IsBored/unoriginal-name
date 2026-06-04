#!/usr/bin/env python3

import math
import rospy

from std_srvs.srv import Trigger
from std_msgs.msg import Float32
from clover import srv


class FlightController:

    def __init__(self):

        rospy.init_node("flight_controller")

        rospy.loginfo("FLIGHT CONTROLLER STARTED")

        # --------------------------------
        # COMMAND VALUES
        # --------------------------------

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0

        # --------------------------------
        # LAST UPDATE TIMES
        # --------------------------------

        now = rospy.Time.now()

        self.last_vx = now
        self.last_vy = now
        self.last_vz = now
        self.last_yaw = now

        # command timeout (seconds)
        self.timeout = 0.2

        # --------------------------------
        # CLOVER SERVICES
        # --------------------------------

        rospy.loginfo("Waiting for Clover services...")

        rospy.wait_for_service("set_velocity")
        rospy.wait_for_service("set_yaw_rate")

        self.set_velocity = rospy.ServiceProxy(
            "set_velocity",
            srv.SetVelocity
        )

        self.set_yaw_rate = rospy.ServiceProxy(
            "set_yaw_rate",
            srv.SetYawRate
        )

        # --------------------------------
        # SUBSCRIBERS
        # --------------------------------

        rospy.Subscriber(
            "/lockndrop/cmd_vx",
            Float32,
            self.vx_callback,
            queue_size=1
        )

        rospy.Subscriber(
            "/lockndrop/cmd_vy",
            Float32,
            self.vy_callback,
            queue_size=1
        )

        rospy.Subscriber(
            "/lockndrop/cmd_vz",
            Float32,
            self.vz_callback,
            queue_size=1
        )

        rospy.Subscriber(
            "/lockndrop/cmd_yaw_rate",
            Float32,
            self.yaw_callback,
            queue_size=1
        )

        rospy.on_shutdown(self.stop_drone)

        rospy.loginfo("Flight controller ready")

    # ====================================
    # CALLBACKS
    # ====================================

    def vx_callback(self, msg):

        self.vx = msg.data
        self.last_vx = rospy.Time.now()

    def vy_callback(self, msg):

        self.vy = msg.data
        self.last_vy = rospy.Time.now()

    def vz_callback(self, msg):

        self.vz = msg.data
        self.last_vz = rospy.Time.now()

    def yaw_callback(self, msg):

        self.yaw_rate = msg.data
        self.last_yaw = rospy.Time.now()

    # ====================================
    # TIMEOUT CHECK
    # ====================================

    def apply_timeout(self):

        now = rospy.Time.now()

        if (now - self.last_vx).to_sec() > self.timeout:
            self.vx = 0.0

        if (now - self.last_vy).to_sec() > self.timeout:
            self.vy = 0.0

        if (now - self.last_vz).to_sec() > self.timeout:
            self.vz = 0.0

        if (now - self.last_yaw).to_sec() > self.timeout:
            self.yaw_rate = 0.0

    # ====================================
    # MAIN LOOP
    # ====================================

    def run(self):

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():

            self.apply_timeout()

            try:

                self.set_yaw_rate(
                    self.yaw_rate
                )

                self.set_velocity(
                    vx=self.vx,
                    vy=self.vy,
                    vz=self.vz,
                    frame_id="body"
                )

            except rospy.ServiceException as e:

                rospy.logerr_throttle(
                    1.0,
                    str(e)
                )

            rate.sleep()

    # ====================================
    # SHUTDOWN
    # ====================================

    def stop_drone(self):

        rospy.loginfo("Stopping drone")

        try:

            self.set_yaw_rate(0)

            self.set_velocity(
                vx=0,
                vy=0,
                vz=0,
                frame_id="body"
            )

        except:
            pass


if __name__ == "__main__":

    controller = FlightController()

    controller.run()