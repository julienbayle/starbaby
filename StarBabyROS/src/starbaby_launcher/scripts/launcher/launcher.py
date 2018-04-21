#!/usr/bin/env python

import rospy
import math
import actionlib
from std_msgs.msg          import Float64
from std_msgs.msg          import Int16
from std_msgs.msg          import UInt8
from starbaby_launcher.msg import LauncherFeedback
from starbaby_launcher.msg import LauncherAction
from starbaby_launcher.msg import LauncherResult

# File name is launcher.py instead of starbaby_launcher.py as ROS
# is unable to import message from the package if the name is the same
# Reference : https://answers.ros.org/question/173507/cannot-import-custom-service/
class starbabyLauncher:

    feedback = LauncherFeedback()
    result = LauncherResult()
    
    waiting_ball = False
    # Average and standart deviation speed

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, LauncherAction, execute_cb = self.execute_cb, auto_start = False)
        self._as.start()

        rospy.loginfo("Waiting for inputs messages...")
	self.launcher_speed_sub = rospy.Subscriber('/launcher/counter', Int16, self.execute_speed_cb)
        self.launcher_pub = rospy.Publisher('/launcher/pwm', Float64, queue_size=10)
        self.servo_launcher_pub = rospy.Publisher('/servo/launcher', UInt8, queue_size=10)
        
        rospy.loginfo("Launcher action server started %s", self._action_name)

    def execute_cb(self, goal):

        self.nb_balls = 0
        self.avg_speed = 0
        self.std_dev_speed = 0
        self.nb_speed = 0
        
        rospy.loginfo("Starting launcher...")
        
        last_speed = 0
        for speed in range (80, goal.speed, 2):
          self.launcher_pub.publish(speed)
          last_speed = speed
          rospy.sleep(0.01)

        while self.nb_balls < goal.nb_balls:
          rospy.loginfo("Loading a ball in the launcher")
          self.waiting_ball = True

          self.servo_launcher_pub.publish(170)
          rospy.sleep(0.5)
          self.servo_launcher_pub.publish(50)
          rospy.sleep(0.5)
        
          #if self.waiting_ball:
          #  rospy.loginfo("No ball thrown")
          #  break
          #else
          #  rospy
          self.waiting_ball = False
        
        rospy.sleep(1)
        rospy.loginfo("Stopping launcher...")
        
        for speed in range (last_speed, 0, -2):
          self.launcher_pub.publish(speed)
          rospy.sleep(0.01)

        rospy.loginfo("Launcher stopped")

        rospy.loginfo("%d ball(s) thrown", self.nb_balls)
        self.result.nb_balls = self.nb_balls
        self._as.set_succeeded(self.result)

    def execute_speed_cb(self, data):
        if self.waiting_ball:
          # Update speed average and standard deviation
          if self.avg_speed == 0:
            self.avg_speed = data.data
          else:
            prev_avg_speed = self.avg_speed
            self.avg_speed += (data.data-self.avg_speed)/float(self.nb_speed)            
            self.std_dev_speed += (data.data-prev_avg_speed)*(data.data-self.avg_speed) 
          self.nb_speed += 1
          if self.nb_speed > 2: 
            std_dev = math.sqrt(self.std_dev_speed/(self.nb_speed - 1))
            rospy.loginfo("Launcher speed : %f (Average: %f, standard deviation : %f, d %f - %f)", data.data, self.avg_speed, std_dev, math.fabs(data.data - self.avg_speed), 2*std_dev)
            if math.fabs(data.data - self.avg_speed) > 2*std_dev:
              self.nb_balls += 1
              self.waiting_ball = False
          
if __name__ == '__main__':
    rospy.init_node('starbaby_launcher')
    starbaby_launcher = starbabyLauncher(rospy.get_name())
    rospy.spin()
