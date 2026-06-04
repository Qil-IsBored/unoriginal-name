#!/usr/bin/env python3

import math
import rospy
import subprocess
import os

from clover import srv
from std_srvs.srv import Trigger


class MissionManager:

    def __init__(self):

        rospy.init_node("mission_manager")

        rospy.loginfo("=== LOCKNDROP MISSION MANAGER ===")

        # TAKEOFF HEIGHT
        self.takeoff_height = 1.2
        # Clover services
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.land = rospy.ServiceProxy('land', Trigger)
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

        # Store process handles
        self.detector_process = None
        self.tracker_process = None

        rospy.loginfo("Mission manager ready.")

    def start_detector(self):

        rospy.loginfo("Starting detector.py ...")

        self.detector_process = subprocess.Popen(["rosrun","lockndrop", "detector.py"])

    def start_tracker(self):

        rospy.loginfo("Starting tracker.py ...")

        self.tracker_process = subprocess.Popen([ "rosrun", "lockndrop", "tracker.py"])

    def start_yaw_controller(self):

        rospy.loginfo("Starting yaw_control.py ...")

        self.yaw_process = subprocess.Popen(["rosrun", "lockndrop", "yaw_control.py"])

    def stop_all_nodes(self):

        rospy.loginfo("Stopping all mission nodes...")

        if self.detector_process:
            self.detector_process.terminate()

        if self.tracker_process:
            self.tracker_process.terminate()

    def takeoff_wait(self):

        self.navigate(x=0, y=0, z=self.takeoff_height, yaw=float('nan'), speed=0.5, frame_id='body', auto_arm=True)

        while not rospy.is_shutdown():

            telem = self.get_telemetry(frame_id='navigate_target')

            distance = math.sqrt( telem.x**2 + telem.y**2 + telem.z**2)

            if distance < 0.15:
                break

            rospy.sleep(0.1)

        rospy.loginfo("Takeoff complete")

    def run(self):

        # START DETECTOR
        self.start_detector()

        rospy.sleep(0.2)

        # START TRACKER
        self.start_tracker()

        rospy.sleep(0.2)

        self.start_yaw_controller()

        rospy.sleep(0.2)

        rospy.loginfo("Detection system ONLINE.")

        rospy.loginfo("Mission manager running...")

        input("\nPress ENTER to ARM and TAKEOFF...\n")

        # TAKEOFF
        self.takeoff_wait()

        rospy.spin()

    def shutdown(self):

        rospy.loginfo("Mission manager shutdown.")

        self.stop_all_nodes()

        try:
            self.land()

        except:
            pass


if __name__ == "__main__":

    manager = MissionManager()

    rospy.on_shutdown(manager.shutdown)

    manager.run()
