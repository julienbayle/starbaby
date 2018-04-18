#!/usr/bin/env python

import rospy
import math
import actionlib
from std_msgs.msg           import Float64
from starbaby_calibrate.msg import CalibrateFeedback
from starbaby_calibrate.msg import CalibrateAction
from starbaby_calibrate.msg import CalibrateResult
from sensor_msgs.msg        import Imu

# File name is calibrate.py instead of starbaby_calibrate.py as ROS
# is unable to import package msg if name is the same
# Reference : https://answers.ros.org/question/173507/cannot-import-custom-service/
# Took me one hour to understand this !
class starbabyCalibrate:

    _feedback = CalibrateFeedback()
    _result = CalibrateResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, CalibrateAction, execute_cb = self.execute_cb, auto_start = False)
        self._as.start()

        rospy.loginfo("Calibration action %s ready " % self._action_name)

    def execute_cb(self, goal):
        # imu
        self.last_imu = []
        self.average = 0
        
        # turn counter
        self.start_angle = -1
        self.turn_count = -1
        self.angle_position = True

        rospy.loginfo("Calibration waiting for inputs messages...")

        imu_sub = rospy.Subscriber('imu', Imu, self.execute_imu_cb)
        laser_angle_sub = rospy.Subscriber('scan_min_distance_angle', Float64, self.execute_min_distance_angle_cb)
        laser_distance_sub = rospy.Subscriber('scan_min_distance', Float64, self.execute_min_distance_cb)
        
        r = rospy.Rate(1)

        rospy.loginfo("Starting calibration...")
        while not rospy.is_shutdown():
           if self._as.is_preempt_requested():
             rospy.loginfo('%s: Preempted' % self._action_name)
             self._as.set_preempted()
             break
           rospy.loginfo("Calibration processing...")
           r.sleep()

        rospy.loginfo("Calibration stopped, cleaning...")

        imu_sub.unregister()
        laser_angle_sub.unregister()
        laser_distance_sub.unregister()
        
        rospy.loginfo("Calibration done")

    def execute_imu_cb(self, data):
        now = rospy.Time.now()
        if self.average == 0 and len(last_imu < 100):
          self.last_imu.append(data.angular_velocity.z)
          rospy.loginfo("IMU gz : %f" % data.angular_velocity.z-self.average)
	else:
          self.average = sum(last_imu)/float(len(last_imu))

    def execute_min_distance_cb(self, data):
        rospy.loginfo("Distance : %f" % data.data)

    def execute_min_distance_angle_cb(self, data):
        if self.start_angle < 0:
           selft.start_angle = data.data
           rospy.loginfo("Start angle : %f" % data.data)

        if 
if __name__ == '__main__':
    rospy.init_node('starbaby_calibrate')
    starbaby_calibrate = starbabyCalibrate(rospy.get_name())
    rospy.spin()
