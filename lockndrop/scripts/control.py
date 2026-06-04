#!/usr/bin/env python3

import rospy
from clover import srv
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger

class YawController:

    def __init__(self):

        rospy.loginfo("Starting controller node...")

        # YAW_PID
        self.kp_yaw = 0.0015
        self.ki_yaw = 0.0001
        self.kd_yaw = 0.00005

        self.prev_error_yaw = 0
        self.integral_yaw = 0

        # Z / Linear Z / Altitude PID
        self.kp_z = 0.0004
        self.ki_z = 0.0000
        self.kd_z = 0.00001

        self.prev_error_z = 0
        self.integral_z = 0

        self.dt = 0.2  # assume ~10Hz loop

        # Dead Zone
        self.deadzone_x = 3
        self.deadzone_y = 30

        # integral clamp
        self.integral_limit = 100

        # SUBSCRIBER
        self.error_sub = rospy.Subscriber("/cone_tracking/error", Point, self.error_callback)

        #Clover service
        self.set_yaw_rate = rospy.ServiceProxy('set_yaw_rate', srv.SetYawRate)
        self.set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)

        rospy.on_shutdown(self.stop_drone)

        rospy.loginfo("Controller node started.")

    def error_callback(self, msg):

        # GET error 
        error_x = msg.x
        error_y = msg.y

       # YAW PID
        derivative_yaw = (error_x - self.prev_error_yaw) / self.dt
        self.integral_yaw += error_x * self.dt
        # INTEGRAL Clamp
        self.integral_yaw = max(min(self.integral_yaw, 1000), -1000)
        # self.integral_yaw = max(min(self.integral_yaw, self.integral_limit), -self.integral_limit)
        
        # PID output
        yaw_rate = ( self.kp_yaw * error_x + self.ki_yaw * self.integral_yaw + self.kd_yaw * derivative_yaw)


        # if abs(error_y) < self.deadzone_y:
        #     error_y = 0

        # YAW Limit Rate
        yaw_rate = max(min(yaw_rate, 0.5), -0.5)
        # DEADZONE (zero small corrections)
        if abs(error_x) < self.deadzone_x:
            yaw_rate = 0
            self.integral_yaw = 0  # optional but recommended
        # store previous error
        self.prev_error_yaw = error_x

        if abs(error_y) < self.deadzone_y:
            # LOCK ALTITUDE
            vz = 0

            # reset PID memory
            self.integral_z = 0

            # send stop command
            #self.set_velocity(vx=0, vy=0, vz=0, frame_id='body')

            #return

        else:
            # Z PID
            derivative_z = (error_y - self.prev_error_z) / self.dt
            self.integral_z += error_y * self.dt

            # INTEGRAL CLAMP
            self.integral_z = max(min(self.integral_z, 100), -100)

            # PID OUTPUT
            vz = -( self.kp_z * error_y + self.ki_z * self.integral_z + self.kd_z * derivative_z)
        # LIMIT SPEED
        vz = max(min(vz, 1.0), -1.0)

        # STORE PREVIOUS ERROR
        self.prev_error_z = error_y


        try:

            # YAW
            self.set_yaw_rate(-yaw_rate)

            # ALTITUDE
            # self.set_velocity(vx=0, vy=0, vz=vz, frame_id='body')

            rospy.loginfo("EX: {:.1f} | EY: {:.1f} | YAW: {:.3f} | VZ: {:.3f}".format(error_x, error_y, yaw_rate, vz))

        except rospy.ServiceException as e:

            rospy.logerr(e)

    def stop_drone(self):

        rospy.loginfo("Stopping drone...")

        try:

            # stop yaw
            self.set_yaw_rate(0)

            # stop movement
            self.set_velocity(
                vx=0,
                vy=0,
                vz=0,
                frame_id='body'
            )

        except rospy.ServiceException as e:

            rospy.logerr(e)


if __name__ == "__main__":

    rospy.init_node("yaw_controller")

    controller = YawController()

    rospy.spin()