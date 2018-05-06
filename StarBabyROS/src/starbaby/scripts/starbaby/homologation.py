#!/usr/bin/env python

import math
import roslib
import rospy
import actionlib
import tf
from math import pi
from starbaby_launcher.msg import LauncherFeedback
from starbaby_launcher.msg import LauncherAction
from starbaby_launcher.msg import LauncherResult
from starbaby_launcher.msg import LauncherGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

# Speed control refresh period (100Hz)
SPEED_DT = 1.0/100.0 
SLOPE_IT = 40.0

def pose_cb(odom_data):
    global position, yaw, ready
    ready += 1
    position = odom_data.pose.pose.position #  the x,y,z pose and quaternion orientation
    quaternion = (
        odom_data.pose.pose.orientation.x,
        odom_data.pose.pose.orientation.y,
        odom_data.pose.pose.orientation.z,
        odom_data.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]

def gap_cb(gap_data):
    global gap, ready
    ready += 1
    gap = gap_data.range

def sendBalls(_nb_balls):
    rospy.loginfo("Waiting launcher...")
    client = actionlib.SimpleActionClient('/starbaby_launcher', LauncherAction)
    client.wait_for_server()
    goal = LauncherGoal(nb_balls=_nb_balls, speed=230)
    rospy.loginfo("Ready...")
    client.send_goal(goal)
    client.wait_for_result()
    launcherResult = client.get_result()
    rospy.loginfo("Nb balles envoyees %d", launcherResult.nb_balls)

def suiviBordure(distance, v_max, goal_gap, max_gap_error=0.01):
	global position, gap
	x_start = position.x
        y_start = position.y

	twist = Twist()
	
        rospy.loginfo("Starting to move")
	
        while math.sqrt((position.x - x_start)**2 + (position.y - y_start)**2) < abs(distance):
    	    gap_error = goal_gap-gap
            if gap_error > max_gap_error:
                rospy.loginfo("Gap too large %f", gap_error)
                break 
            twist.angular.z = 0.6*twist.angular.z + 0.4*gap_error*10

	    if abs(twist.linear.x) < v_max:
    		twist.linear.x += v_max/SLOPE_IT * distance/abs(distance)
    	    
            twist.linear.x *= 0.2 + 0.8*(1-abs(gap_error)/max_gap_error)

            p.publish(twist)
            rospy.loginfo("Distance : %.2f, gap: %.2f, speed : %.2f,%.2f", 100*math.sqrt((position.x - x_start)**2 + (position.y - y_start)**2), 100*gap_error, 100*twist.linear.x, 180*twist.angular.z/pi )
	    rospy.sleep(SPEED_DT)
        
        stop()
	rospy.sleep(0.1)
  	rospy.loginfo("Distance : %f, speed : %f", math.sqrt((position.x - x_start)**2 + (position.y - y_start)**2), twist.linear.x )


def avance(distance, v_max):
	global position
	x_start = position.x
        y_start = position.y

	twist = Twist()
	
        rospy.loginfo("Starting to move")
	
        while math.sqrt((position.x - x_start)**2 + (position.y - y_start)**2) < abs(distance):
	    if abs(twist.linear.x) < v_max:
    		twist.linear.x += v_max/SLOPE_IT * distance/abs(distance)
    	    p.publish(twist)
  	    rospy.loginfo("Distance : %f, speed : %f", math.sqrt((position.x - x_start)**2 + (position.y - y_start)**2), twist.linear.x )
	    rospy.sleep(SPEED_DT)
        
        stop()
	rospy.sleep(0.1)
  	rospy.loginfo("Distance : %f, speed : %f", math.sqrt((position.x - x_start)**2 + (position.y - y_start)**2), twist.linear.x )

def stop():
    twist = Twist()
    print twist
    p.publish(twist)
    rospy.loginfo("Robot is not moving anymore")

def add_angle(a,b):
    return (a + b + pi)%(2*pi) -pi

def rotate(angle, v_max):
	global yaw
        _objective = add_angle(yaw, pi*angle/180)

	twist = Twist()
	twist_inc = v_max/SLOPE_IT * angle/abs(angle)
	
	rospy.loginfo("Starting rotation")
	
        while abs(add_angle(yaw, -1 * _objective)) > 2*v_max*SPEED_DT:
	    if abs(twist.angular.z) < v_max:
    	        twist.angular.z += v_max/SLOPE_IT * angle/abs(angle)
                p.publish(twist)
  	    rospy.loginfo("Angle : %f, speed : %f", 180*abs(add_angle(yaw, -1 * _objective))/pi , twist.angular.z )
        
  	    rospy.sleep(SPEED_DT)

        stop()
	rospy.sleep(0.1)
  	rospy.loginfo("Angle : %f, speed : %f", 180*abs(add_angle(yaw, -1 * _objective))/pi , twist.angular.z )

if __name__=="__main__":
    try:
        global gap, position, yaw, ready
        ready = 0
	rospy.init_node('move')
	rospy.Subscriber('/range/side',Range,gap_cb)
	rospy.Subscriber('/starbaby/odom',Odometry,pose_cb)
	p = rospy.Publisher('/starbaby/cmd_vel', Twist, queue_size=10)
        while ready < 2:
            rospy.loginfo("waiting...")
            rospy.sleep(0.2)
        

        visual_delay = 1
        
        rgap = 0;
        for i in range(50):
            rgap += gap
            rospy.sleep(0.02)
        rgap /= 50

        suiviBordure(0.21, 0.5,rgap, 0.015)
	rospy.sleep(visual_delay)
        suiviBordure(0.22, 0.3,rgap, 0.015)
	rospy.sleep(visual_delay)
        for i in range(2):
            suiviBordure(-0.15,0.3,rgap, 0.015)
	    rospy.sleep(visual_delay)
            suiviBordure(0.15, 0.3,rgap, 0.015)
	    rospy.sleep(visual_delay)
        suiviBordure(0.04, 0.5, rgap, 0.015)        
        avance(0.3, 1) 
	rospy.sleep(visual_delay)
        rotate(90, 1.2)
	rospy.sleep(visual_delay)
        avance(0.5, 1)
	rospy.sleep(visual_delay)
        rotate(110, 1.2)
	rospy.sleep(visual_delay)
        avance(0.3,1)
	rospy.sleep(visual_delay)
        sendBalls(1)
#	rospy.sleep(1)
#        rotate(-90, 1.2)
#        avance(-0.5, 0.12)
#	avance(-0.5)
#	avance(0.5)
#	avance(-0.5)
    except KeyboardInterrupt:
        pass

