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
from std_msgs.msg import Bool
from std_msgs.msg import UInt8
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

def ils_cb(ils_data):
    global ils, ready
    ils = ils_data.data
    ready += 1

def gap_cb(gap_data):
    global gap, ready
    gap = gap_data.range
    ready += 1

def sonar_cb(sonar_data):
    global d_sonar, ready
    d_sonar = sonar_data.range
    ready += 1

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

def reduce_speed_sonar():
    global d_sonar
    
    # If sonar is below d_max, stop the robot, else, robot can go on
    reduce_speed_sonar = 0
    if d_sonar < 0.1:
        reduce_speed_sonar = 1
    return reduce_speed_sonar

def openArm():
    p_servo.publish(90)

def closeArm():
    p_servo.publish(174)

def suiviBordure(distance, v_max, goal_gap, max_gap_error=0.01, sonar_stop=False):
	global position, gap, d_sonar
	x_start = position.x
        y_start = position.y

	twist = Twist()
	
        rospy.loginfo("Starting to move")
	
        while math.sqrt((position.x - x_start)**2 + (position.y - y_start)**2) < abs(distance):
    	    gap_error = goal_gap-gap
            if gap_error > max_gap_error:
                rospy.loginfo("Gap too large %f", gap_error)
                break

            if d_sonar <= 0.1 and sonar_stop:
                rospy.loginfo("Sonar stop %f", d_sonar)
                break

            twist.angular.z = 0.6*twist.angular.z + 0.4*gap_error*10

	    if abs(twist.linear.x) < v_max:
    		twist.linear.x += v_max/SLOPE_IT * distance/abs(distance)
    	    
            # If gap is OK, do not reduce speed. If gap error is near maximum, reduce speed at 80%
            reduce_speed_gap_error = 0.8*abs(gap_error)/max_gap_error
            
            twist.linear.x *= 1-max(reduce_speed_sonar(), reduce_speed_gap_error)

            p.publish(twist)
            rospy.loginfo("Distance : %.2f, gap: %.2f, speed : %.2f,%.2f, sonar: %.2f", 100*math.sqrt((position.x - x_start)**2 + (position.y - y_start)**2), 100*gap_error, 100*twist.linear.x, 180*twist.angular.z/pi, d_sonar*100)
	    rospy.sleep(SPEED_DT)
        
        stop()
	rospy.sleep(0.1)
  	rospy.loginfo("Distance : %f, speed : %f", math.sqrt((position.x - x_start)**2 + (position.y - y_start)**2), twist.linear.x )


def avance(distance, v_max, sonar_check=True):
	global position
	x_start = position.x
        y_start = position.y

	twist = Twist()
	
        rospy.loginfo("Starting to move")
	
        while math.sqrt((position.x - x_start)**2 + (position.y - y_start)**2) < abs(distance):
	    if abs(twist.linear.x) < v_max:
    		twist.linear.x += v_max/SLOPE_IT * distance/abs(distance)

            if sonar_check:
                twist.linear.x *= 1-reduce_speed_sonar()

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
        global gap, position, yaw, ready, ils, d_sonar
        ready = 0
	rospy.init_node('move')
	rospy.Subscriber('/range/side',Range,gap_cb)
	rospy.Subscriber('/sonar/center',Range,sonar_cb)
	rospy.Subscriber('/starbaby/odom',Odometry,pose_cb)
	rospy.Subscriber('/ils',Bool,ils_cb)
	p = rospy.Publisher('/starbaby/cmd_vel', Twist, queue_size=10)
	p_servo = rospy.Publisher('/servo/front', UInt8, queue_size=10)
        while ready < 4:
            rospy.loginfo("waiting...")
            rospy.sleep(0.2)
        

        visual_delay = 3
        
        rospy.loginfo("Attente tirette")
        ils_count =0
        while ils_count < 5:
            if ils:
                ils_count += 1
            rospy.sleep(0.2)

        rospy.loginfo("Tirette OK")
        openArm()
        rgap = 0;
        ils_loop_count = 0
        ils_count =0
        while ils_count < 5:
            if not ils:
                ils_count += 1
            rgap += gap
            ils_loop_count += 1
            rospy.sleep(0.02)
        closeArm()
        rgap /= ils_loop_count

        suiviBordure(0.21, 0.2,rgap, 0.025)
	rospy.sleep(visual_delay)
        suiviBordure(0.22, 0.08,rgap, 0.015)
	rospy.sleep(visual_delay)
        for i in range(0):
            suiviBordure(-0.15,0.08,rgap, 0.015)
	    rospy.sleep(visual_delay)
            suiviBordure(0.15, 0.08,rgap, 0.015)
	    rospy.sleep(visual_delay)
        suiviBordure(0.10, 0.08, rgap, 0.035)        
	rospy.sleep(visual_delay)
        suiviBordure(2.0, 0.19, rgap, 0.035, sonar_stop=True)        
	openArm()
        rospy.sleep(1)
        avance(0.13, 0.08, sonar_check=False) 
	rospy.sleep(0.5)
        avance(-0.20, 0.08, sonar_check=False) 
        closeArm()
        rospy.sleep(1)
        rospy.sleep(visual_delay)
        rotate(165, 1)
	rospy.sleep(visual_delay)
        avance(0.9, 0.19)
	rospy.sleep(visual_delay)
        rotate(30, 1)
	rospy.sleep(visual_delay)
        sendBalls(8)
    except KeyboardInterrupt:
        pass

