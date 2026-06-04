#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from clover import srv


class Homer:

    def __init__(self):

        rospy.init_node("homer")

        rospy.loginfo("HOMING NODE STARTED")

        self.homing_enabled = False

        # same gain used by student
        self.kp_yaw = 0.004

        self.error_x = None
        self.last_seen = rospy.Time.now()

        rospy.Subscriber(
            "/mission/target_locked",
            Bool,
            self.lock_callback
        )

        rospy.Subscriber(
            "/cone_tracking/error",
            Point,
            self.error_callback
        )

        self.set_yaw_rate = rospy.ServiceProxy(
            "set_yaw_rate",
            srv.SetYawRate
        )

        self.set_velocity = rospy.ServiceProxy(
            "set_velocity",
            srv.SetVelocity
        )

        rospy.on_shutdown(self.stop_drone)

        rospy.sleep(1)

        # self.run()

    # -------------------------------------

    def error_callback(self, msg):

        self.error_x = msg.x
        self.last_seen = rospy.Time.now()

    # -------------------------------------

    def lock_callback(self, msg):

        if msg.data:

            rospy.loginfo("LOCK RECEIVED")

            self.homing_enabled = True

    def stop_drone(self):

        try:

            self.set_yaw_rate(0)

            self.set_velocity(
                vx=0,
                vy=0,
                vz=0,
                frame_id='body'
            )

        except rospy.ServiceException:
            pass

    # -------------------------------------

    def run(self):

        rate = rospy.Rate(30)

        rospy.loginfo("Waiting for target lock...")

        while not rospy.is_shutdown():

            if self.homing_enabled:
                break

            rate.sleep()

        rospy.loginfo("RAM START")

        lost_start = None

        while not rospy.is_shutdown():

            # keep steering during attack
            if self.error_x is not None:

                yaw_rate = self.kp_yaw * self.error_x

                yaw_rate = max(
                    min(yaw_rate, 2.0),
                    -2.0
                )

                self.set_yaw_rate(-yaw_rate)

            # attack forward
            self.set_velocity(
                vx=1.6,
                vy=0,
                vz=0,
                frame_id='body'
            )

            # -------------------------
            # target lost?
            # -------------------------

            time_since_seen = (
                rospy.Time.now() - self.last_seen
            ).to_sec()

            if time_since_seen > 0.08:

                if lost_start is None:
                    lost_start = rospy.Time.now()

                lost_time = (
                    rospy.Time.now() - lost_start
                ).to_sec()

                if lost_time > 0.10:

                    rospy.logwarn(
                        "TARGET LOST -> BRAKING"
                    )

                    brake_start = rospy.Time.now()

                    while (
                        rospy.Time.now() - brake_start
                    ).to_sec() < 0.25:

                        self.set_velocity(
                            vx=-0.8,
                            vy=0,
                            vz=0,
                            frame_id='body'
                        )

                        self.set_yaw_rate(0)

                        rate.sleep()

                    self.set_velocity(
                        vx=0,
                        vy=0,
                        vz=0,
                        frame_id='body'
                    )

                    self.set_yaw_rate(0)

                    rospy.loginfo(
                        "HOMING COMPLETE"
                    )

                    return

            else:

                lost_start = None

            rate.sleep()


if __name__ == "__main__":

    Homer()

    rospy.spin()