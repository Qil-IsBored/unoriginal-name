#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from clover import srv


class AltitudeHoming:

    def __init__(self):

        rospy.init_node("altitude_homing")

        rospy.loginfo("ALTITUDE + HOMING NODE STARTED")

        # SERVICES
        self.set_velocity = rospy.ServiceProxy(
            'set_velocity',
            srv.SetVelocity
        )

        # TRACKING DATA
        self.error_y = None
        self.last_seen = rospy.Time.now()

        rospy.Subscriber(
            "/cone_tracking/error",
            Point,
            self.error_callback
        )

        rospy.on_shutdown(self.stop_drone)

        rospy.sleep(1.0)

        self.run()

    # --------------------------------

    def error_callback(self, msg):

        self.error_y = msg.y
        self.last_seen = rospy.Time.now()

    # --------------------------------

    def update_tracking(self):

        if self.error_y is None:
            return False

        # Student's original Z controller
        vz = -self.error_y * 0.0025

        vz = max(min(vz, 0.25), -0.25)

        if abs(self.error_y) < 25:
            vz = 0

        try:

            self.set_velocity(
                vx=0,
                vy=0,
                vz=vz,
                frame_id='body'
            )

        except rospy.ServiceException as e:

            rospy.logerr(e)

        return abs(self.error_y)

    # --------------------------------

    def stop_drone(self):

        try:

            self.set_velocity(
                vx=0,
                vy=0,
                vz=0,
                frame_id='body'
            )

        except rospy.ServiceException:
            pass

    # --------------------------------

    def run(self):

        rate = rospy.Rate(30)

        rospy.loginfo("ACTIVE LOCK PHASE")

        confirm_start = None

        # =====================================
        # LOCK PHASE
        # =====================================

        while not rospy.is_shutdown():

            error_y_abs = self.update_tracking()

            if error_y_abs is False:
                rate.sleep()
                continue

            # target lost
            if rospy.Time.now() - self.last_seen > rospy.Duration(0.5):

                confirm_start = None

                rate.sleep()
                continue

            centered = (error_y_abs < 35)

            if centered:

                if confirm_start is None:

                    confirm_start = rospy.Time.now()

                elapsed = (
                    rospy.Time.now() - confirm_start
                ).to_sec()

                rospy.loginfo(
                    "Locking target... %.1f / 0.8",
                    elapsed
                )

                if elapsed > 0.8:

                    rospy.loginfo(
                        "TARGET LOCKED"
                    )

                    break

            else:

                confirm_start = None

            rate.sleep()

        # =====================================
        # STABILIZATION PHASE
        # =====================================

        rospy.loginfo(
            "FINAL STABILIZATION"
        )

        pause_start = rospy.Time.now()

        while not rospy.is_shutdown():

            error_y_abs = self.update_tracking()

            if error_y_abs > 45:

                rospy.logwarn(
                    "Aim drifted -> re-locking"
                )

                return self.run()

            if (
                rospy.Time.now() - pause_start
            ).to_sec() > 0.3:

                break

            rate.sleep()

        # =====================================
        # RAM PHASE
        # =====================================

        rospy.loginfo(
            "RAM START"
        )

        lost_start = None

        while not rospy.is_shutdown():

            self.set_velocity(
                vx=1.6,
                vy=0,
                vz=0,
                frame_id='body'
            )

            time_since_seen = (
                rospy.Time.now() -
                self.last_seen
            ).to_sec()

            if time_since_seen > 0.08:

                if lost_start is None:

                    lost_start = rospy.Time.now()

                lost_time = (
                    rospy.Time.now() -
                    lost_start
                ).to_sec()

                if lost_time > 0.10:

                    rospy.logwarn(
                        "TARGET LOST -> BRAKING"
                    )

                    brake_start = rospy.Time.now()

                    while (
                        rospy.Time.now() -
                        brake_start
                    ).to_sec() < 0.25:

                        self.set_velocity(
                            vx=-0.8,
                            vy=0,
                            vz=0,
                            frame_id='body'
                        )

                        rate.sleep()

                    self.set_velocity(
                        vx=0,
                        vy=0,
                        vz=0,
                        frame_id='body'
                    )

                    rospy.loginfo(
                        "MISSION COMPLETE"
                    )

                    return

            else:

                lost_start = None

            rate.sleep()


if __name__ == "__main__":

    AltitudeHoming()

    rospy.spin()