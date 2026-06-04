#!/usr/bin/env python3
#!zaquan's code

import rospy
from geometry_msgs.msg import Point
from clover import srv


class KillerNode:

    def __init__(self):

        rospy.init_node("killer_node")

        # -------------------------
        # YAW PID
        # -------------------------
        self.kp = 0.004
        self.ki = 0.0001
        self.kd = 0.00005

        self.prev_error = 0
        self.integral = 0
        self.dt = 0.1
        self.deadzone = 3

        # -------------------------
        # SERVICES
        # -------------------------
        self.set_yaw_rate = rospy.ServiceProxy('set_yaw_rate', srv.SetYawRate)
        self.set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)

        # -------------------------
        # TRACKING DATA
        # -------------------------
        self.error_x = None
        self.error_y = 0
        self.last_seen = rospy.Time.now()

        rospy.Subscriber("/cone_tracking/error", Point, self.error_callback)

        rospy.on_shutdown(self.stop_drone)

        rospy.loginfo("KILLER NODE STARTED 🚀")

        rospy.sleep(1.0)
        self.run()

    # --------------------------------
    def error_callback(self, msg):
        self.error_x = msg.x
        self.error_y = msg.y
        self.last_seen = rospy.Time.now()

    # --------------------------------
    def update_tracking(self):

        if self.error_x is None:
            return False, False

        error = self.error_x

        derivative = (error - self.prev_error) / self.dt

        self.integral += error * self.dt
        self.integral = max(min(self.integral, 1000), -1000)

        yaw_rate = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        if abs(error) < self.deadzone:
            yaw_rate = 0
            self.integral = 0

        yaw_rate = max(min(yaw_rate, 2.0), -2.0)
        self.prev_error = error

        # vertical correction
        vz = -self.error_y * 0.0025
        vz = max(min(vz, 0.25), -0.25)

        if abs(self.error_y) < 25:
            vz = 0

        try:
            self.set_yaw_rate(-yaw_rate)

            self.set_velocity(
                vx=0,
                vy=0,
                vz=vz,
                frame_id='body'
            )

        except rospy.ServiceException as e:
            rospy.logerr(e)

        return abs(error), abs(self.error_y)

    # --------------------------------
    def stop_drone(self):

        try:
            self.set_yaw_rate(0)
            self.set_velocity(vx=0, vy=0, vz=0, frame_id='body')
        except rospy.ServiceException as e:
            rospy.logerr(e)

    # --------------------------------
    def run(self):

        rate = rospy.Rate(30)
        rospy.loginfo("ACTIVE LOCK PHASE")

        confirm_start = None

        # ---------------- LOCK ----------------
        while not rospy.is_shutdown():

            error_x_abs, error_y_abs = self.update_tracking()

            if error_x_abs is False:
                rate.sleep()
                continue

            if rospy.Time.now() - self.last_seen > rospy.Duration(0.5):
                confirm_start = None
                rate.sleep()
                continue

            centered = (error_x_abs < 35 and error_y_abs < 35)

            if centered:

                if confirm_start is None:
                    confirm_start = rospy.Time.now()

                elapsed = (rospy.Time.now() - confirm_start).to_sec()

                rospy.loginfo("Locking target... %.1f / 0.8", elapsed)

                if elapsed > 0.8:
                    rospy.loginfo("TARGET LOCKED ✔")
                    break

            else:
                confirm_start = None

            rate.sleep()

        # ---------------- STABILIZE ----------------
        rospy.loginfo("FINAL STABILIZATION")

        pause_start = rospy.Time.now()

        while not rospy.is_shutdown():

            error_x_abs, error_y_abs = self.update_tracking()

            if error_x_abs > 45 or error_y_abs > 45:
                rospy.logwarn("Aim drifted → re-locking")
                return self.run()

            if (rospy.Time.now() - pause_start).to_sec() > 0.3:
                break

            rate.sleep()

        # ---------------- RAM + BRAKE ----------------
        rospy.loginfo("RAM START 💥")

        lost_start = None

        while not rospy.is_shutdown():

            # yaw tracking
            if self.error_x is not None:
                yaw_rate = max(min(self.kp * self.error_x, 2.0), -2.0)
                self.set_yaw_rate(-yaw_rate)

            # forward push
            self.set_velocity(vx=1.6, vy=0, vz=0, frame_id='body')

            # ---------------- FAST LOSS DETECTION ----------------
            time_since_seen = (rospy.Time.now() - self.last_seen).to_sec()

            if time_since_seen > 0.08:

                if lost_start is None:
                    lost_start = rospy.Time.now()

                lost_time = (rospy.Time.now() - lost_start).to_sec()

                # ---------------- BRAKE BY REVERSE THRUST ----------------
                if lost_time > 0.10:

                    rospy.logwarn("Cone LOST → ACTIVE BRAKE")

                    # 🚨 reverse thrust braking phase
                    brake_start = rospy.Time.now()

                    while (rospy.Time.now() - brake_start).to_sec() < 0.25:

                        # opposite thrust to cancel momentum
                        self.set_velocity(
                            vx=-0.8,
                            vy=0,
                            vz=0,
                            frame_id='body'
                        )
                        self.set_yaw_rate(0)

                        rate.sleep()

                    # final hover settle
                    self.set_velocity(vx=0, vy=0, vz=0, frame_id='body')
                    self.set_yaw_rate(0)

                    rospy.sleep(0.3)

                    rospy.loginfo("MISSION COMPLETE ✔")
                    return

            else:
                lost_start = None

            rate.sleep()


if __name__ == "__main__":

    KillerNode()
    rospy.spin()