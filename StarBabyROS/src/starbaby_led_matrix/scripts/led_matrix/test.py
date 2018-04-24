#!/usr/bin/env python

import math
import roslib
import rospy
import actionlib
from starbaby_launcher.msg import LauncherFeedback
from starbaby_launcher.msg import LauncherAction
from starbaby_launcher.msg import LauncherResult
from starbaby_launcher.msg import LauncherGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

def Position(odom_data):
    global pose
    pose = odom_data.pose.pose #  the x,y,z pose and quaternion orientation

def sendBalls(_nb_balls):
    rospy.loginfo("Waiting clint...")
    client = actionlib.SimpleActionClient('/starbaby_launcher', LauncherAction)
    client.wait_for_server()
    goal = LauncherGoal(nb_balls=_nb_balls, speed=230)
    rospy.loginfo("Ready...")
    client.send_goal(goal)
    client.wait_for_result()
    launcherResult = client.get_result()
    rospy.loginfo("Nb balles envoyees %d", launcherResult.nb_balls)


def avance(distance):
	global pose
	x_start = pose.position.x
        y_start = pose.position.y

	twist = Twist()
	v_max = 0.12
	if distance > 0:
		twist_inc = 0.03
	else:
		twist_inc = -0.03
	
	twist.linear.x = 0 * twist_inc / abs(twist_inc)
	rospy.loginfo("About to be moving forward!")
	while math.sqrt(math.pow(pose.position.x - x_start, 2) + math.pow(pose.position.y - y_start,2)) < abs(distance):
		if abs(twist.linear.x) < v_max:
			twist.linear.x = twist.linear.x + twist_inc
		p.publish(twist)
  		rospy.sleep(0.1)
  		rospy.loginfo("Distance : %f, speed : %f", pose.position.x - x_start, twist.linear.x )

  	while abs(twist.linear.x) > 0:
		twist.linear.x = twist.linear.x - twist_inc
		if abs(twist.linear.x) < abs(0.1 * twist_inc / abs(twist_inc)):
			twist.linear.x = 0
		p.publish(twist)
  		rospy.loginfo("Distance : %f, speed : %f", pose.position.x - x_start, twist.linear.x )
  		rospy.sleep(0.1)

  	twist = Twist()
  	rospy.loginfo("Stopping!")
  	p.publish(twist)

def rotate(angle):
	global pose
        angleRad = angle/180
	_start = pose.orientation.z

	twist = Twist()
	v_max = 1.5
	if angle > 0:
		twist_inc = 0.03
	else:
		twist_inc = -0.03
	
	twist.angular.z = 0
	rospy.loginfo("About to rotate!")
	while abs(pose.orientation.z - _start) < abs(angleRad):
		if abs(twist.angular.z) < v_max:
			twist.angular.z += twist_inc
		p.publish(twist)
  		rospy.sleep(0.1)
  		rospy.loginfo("Angle : %f, speed : %f", 180*(pose.orientation.z - _start)/3.14, twist.angular.z )

  	while abs(twist.angular.z) > 0:
		twist.angular.z -= twist_inc
		if abs(twist.angular.z) < abs(0.1 * twist_inc / abs(twist_inc)):
			twist.angular.z = 0
		p.publish(twist)
  		rospy.loginfo("Angle : %f, speed : %f", 180*(pose.orientation.z - _start)/3.14, twist.angular.z )
  		rospy.sleep(0.1)

  	twist = Twist()
  	rospy.loginfo("Stopping!")
  	p.publish(twist)
if __name__=="__main__":
    try:
        global pose
	rospy.init_node('move')
	rospy.Subscriber('/starbaby/odom',Odometry,Position)
	rospy.sleep(0.1)
	p = rospy.Publisher('/starbaby/cmd_vel', Twist, queue_size=10)
	rospy.sleep(0.1)

	avance(0.5)
        rotate(90)
        rotate(-90)
        avance(-0.5)
        #sendBalls(1)
#	avance(-0.5)
#	avance(0.5)
#	avance(-0.5)
    except KeyboardInterrupt:
        pass
