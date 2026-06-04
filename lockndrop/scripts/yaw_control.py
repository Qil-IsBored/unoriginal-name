#!/usr/bin/env python3

import rospy
from clover import srv
from geometry_msgs.msg import Point
from std_msgs.msg import Float32


class YawController:

    def __init__(self):

        rospy.loginfo("Starting yaw controller...")

        # YAW PID
        self.kp = 0.004
        self.ki = 0.0001
        self.kd = 0.00005

        self.prev_error = 0
        self.integral = 0

        self.dt = 0.1

        self.deadzone = 3

        # SUBSCRIBER
        self.error_sub = rospy.Subscriber("/cone_tracking/error", Point, self.error_callback)

        # Publisher
        # self.yaw_pub = rospy.Publisher("/lockndrop/cmd_yaw_rate", Float32, queue_size=1)

        # CLOVER SERVICE
        self.set_yaw_rate = rospy.ServiceProxy( 'set_yaw_rate', srv.SetYawRate)

        rospy.on_shutdown(self.stop_yaw)

        rospy.loginfo("Yaw controller started.")


    def error_callback(self, msg):

        error_x = msg.x

        # DERIVATIVE
        derivative = (error_x - self.prev_error) / self.dt

        # INTEGRAL
        self.integral += error_x * self.dt

        # CLAMP
        self.integral = max(min(self.integral, 1000), -1000)

        # PID
        yaw_rate = (self.kp * error_x + self.ki * self.integral + self.kd * derivative)

        # LIMIT
        yaw_rate = max(min(yaw_rate, 3.0), -3.0)

        # DEADZONE
        if abs(error_x) < self.deadzone:

            yaw_rate = 0
            self.integral = 0

        # STORE
        self.prev_error = error_x

        yaw_rate_cmd = -yaw_rate

        try:

            self.set_yaw_rate(-yaw_rate)

            rospy.loginfo("EX: {:.1f} | YAW: {:.3f}".format(error_x, yaw_rate))

        except rospy.ServiceException as e:

            rospy.logerr(e)


    def stop_yaw(self):

        rospy.loginfo("Stopping yaw...")

        try:

            self.set_yaw_rate(0.0)

        except rospy.ServiceException as e:

            rospy.logerr(e)


if __name__ == "__main__":

    rospy.init_node("yaw_controller")

    controller = YawController()

    rospy.spin()