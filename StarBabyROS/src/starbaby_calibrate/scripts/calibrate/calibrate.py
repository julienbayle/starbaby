#!/usr/bin/env python

import rospy
import math
import actionlib
from std_msgs.msg           import Float64
from starbaby_calibrate.msg import CalibrateFeedback
from starbaby_calibrate.msg import CalibrateAction
from starbaby_calibrate.msg import CalibrateResult
from sensor_msgs.msg        import Imu

# The robot must be at 30 cm from a wall with nothing around (1 meter minimum)
#
# Phases
#
# IMU biais 
#  - compute IMU biais when the robot is not moving
#
# IMU coef
# - compute IMU coeficient to convert speed in degree per second
# - use your pad to make the robot turn clockwise
#
# End :
#  - action server returns the result message and log the calibration result 


# File name is calibrate.py instead of starbaby_calibrate.py as ROS
# is unable to import message from the package if the name is the same
# Reference : https://answers.ros.org/question/173507/cannot-import-custom-service/
class starbabyCalibrate:

    _feedback = CalibrateFeedback()
    _result = CalibrateResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, CalibrateAction, execute_cb = self.execute_cb, auto_start = False)
        self._as.start()

        rospy.loginfo("Calibration action server started %s", self._action_name)

    def execute_cb(self, goal):

        # current phase name and completeness (from 0 to 100)
        self.phase = "IMU biais"
        self.phase_progress = 0

        # imu measure memory to compute biais
        self.imu_biais_data = []
        self.imu_biais_max_iteration = 200

        # imu computed biais on gz
        self.imu_biais_gz = 0
        
	# imu rotation computed as an integral of gz over time
	self.rotation = 0.0

	# imu rotation memory every turn
        self.imu_calib = []
 
        # turn counter
        self.previous_angle = 0
        self.turn_count = 0
        self.max_turn = 10
        
        # imu average angle error for a 360 degree turn
        self.imu_angle_error = 0 

        # start distance for odometry calibration
        self.start_distance = -1
     
        rospy.loginfo("Waiting for inputs messages...")

        imu_sub = rospy.Subscriber('imu', Imu, self.execute_imu_cb)
        laser_angle_sub = rospy.Subscriber('scan_min_distance_angle', Float64, self.execute_min_distance_angle_cb)
        laser_distance_sub = rospy.Subscriber('scan_min_distance', Float64, self.execute_min_distance_cb)
        
        r = rospy.Rate(1)

        rospy.loginfo("Starting calibration...")
        while not (rospy.is_shutdown() or self.phase == "End"):
           if self._as.is_preempt_requested():
             rospy.loginfo('%s: Preempted' % self._action_name)
             self._as.set_preempted()
             break
           rospy.loginfo("Calibration phase : %s (%f %%)", self.phase, 100*self.phase_progress)
           r.sleep()

        rospy.loginfo("Add to the IMU biais: %f" % self.imu_biais_gz)
        rospy.loginfo("Average rotation error %f",self.imu_angle_error)
        rospy.loginfo("Multiply the to_degree_per_second parameter %f",(360/(360+self.imu_angle_error)))
 
        rospy.loginfo("Calibration stopped or finished, cleaning...")

        imu_sub.unregister()
        laser_angle_sub.unregister()
        laser_distance_sub.unregister()
        
        rospy.loginfo("Calibration process done")

    def execute_imu_cb(self, data):

        now = rospy.get_time()
        if self.phase == "IMU biais" and len(self.imu_biais_data) < self.imu_biais_max_iteration:
          self.imu_biais_data.append(data.angular_velocity.z)
          self.phase_progress = len(self.imu_biais_data) / float(self.imu_biais_max_iteration)
	elif self.phase == "IMU biais":
          self.imu_biais_gz = sum(self.imu_biais_data)/float(len(self.imu_biais_data))
          self.phase = "IMU coef"
          self.phase_progress = 0
        elif self.phase == "IMU coef" and self.turn_count > 0 :
          self.rotation += (now - self.lasttime)*(data.angular_velocity.z-self.imu_biais_gz)
        self.lasttime = now
          
    def execute_min_distance_cb(self, data):
        if self.start_distance < 0:
           self.start_distance = data.data
           rospy.loginfo("Calibration / Start distance : %f" % data.data)

    def execute_min_distance_angle_cb(self, data):
        angle = data.data

        # Detect turn when angle pass over 0 degree ( turns such as LIDAR angle increase   
        if self.previous_angle > angle and angle > 180:
          rospy.logwarn("Robot must turn clockwise, please restart calibration process")

        if self.phase == "IMU coef" and self.previous_angle > 180 and angle < 100:
          self.turn_count += 1
          self.phase_progress = self.turn_count / float(self.max_turn)   
          self.imu_calib.append((self.turn_count, angle, self.rotation))
 
        if self.phase == "IMU coef" and self.turn_count >= self.max_turn:
          last_imu_angle = last_lidar_angle = 0
          delta_rots = []
          for turn, lidar_angle, imu_angle in self.imu_calib:
            lidar_rot = -360 -lidar_angle + last_lidar_angle # ROS see positive angle when turning counter-clockwise
            imu_rot = imu_angle - last_imu_angle 
            delta_rot = lidar_rot - imu_rot 
            rospy.loginfo("%f - Rotation seen by the LIDAR %f degrees and by the IMU %f degrees (Delta %f)", \
              turn, lidar_rot, imu_rot, delta_rot)
            last_imu_angle = imu_angle
            last_lidar_angle = lidar_angle
            if turn > 2: # First two turns are ignored
              delta_rots.append(delta_rot)
          self.imu_angle_error = sum(delta_rots)/float(len(delta_rots))
          self.phase = "End"
         
        self.previous_angle = angle

if __name__ == '__main__':
    rospy.init_node('starbaby_calibrate')
    starbaby_calibrate = starbabyCalibrate(rospy.get_name())
    rospy.spin()
