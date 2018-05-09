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
from starbaby_led_matrix.msg import Eye
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist


class Homologation:

    # Speed control refresh period (100Hz)
    SPEED_DT = 1.0/100.0 
    SLOPE_IT = 40.0

    def __init__(self, visual_delay):
        self.ready = 0
        self.visual_delay = visual_delay
        self.d_sonar_side = {}
        rospy.init_node('Homologation')
	rospy.Subscriber('/range/side', Range, self.gap_cb)
	rospy.Subscriber('/sonar/center', Range, self.sonar_center_cb)
	rospy.Subscriber('/sonar/left', Range, self.sonar_left_cb)
	rospy.Subscriber('/sonar/right', Range, self.sonar_right_cb)
	rospy.Subscriber('/starbaby/odom', Odometry, self.pose_cb)
	rospy.Subscriber('/ils', Bool, self.ils_cb)
	self.cmd_pub = rospy.Publisher('/starbaby/cmd_vel', Twist, queue_size=10)
	self.servo_pub = rospy.Publisher('/servo/front', UInt8, queue_size=10)
        self.side_pub = rospy.Publisher('/is_orange', Bool, queue_size=10)
        self.eye_pub = rospy.Publisher('/starbaby/eye', Eye, queue_size=10)

    def pose_cb(self,odom_data):
        self.position = odom_data.pose.pose.position
        quaternion = (
            odom_data.pose.pose.orientation.x,
            odom_data.pose.pose.orientation.y,
            odom_data.pose.pose.orientation.z,
            odom_data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]
        self.ready += 1
            
    def ils_cb(self, ils_data):
        self.ils = ils_data.data
        self.ready += 1

    def gap_cb(self, gap_data):
        self.gap = gap_data.range
        self.ready += 1

    def sonar_right_cb(self, sonar_data):
        self.sonar_cb(sonar_data, 'right')

    def sonar_left_cb(self, sonar_data):
        self.sonar_cb(sonar_data, 'left')

    def sonar_center_cb(self, sonar_data):
        self.sonar_cb(sonar_data, 'center')

    def sonar_cb(self, sonar_data, side):
        self.d_sonar_side[side] = sonar_data.range
        self.d_sonar = min(self.d_sonar_side.values())
        self.ready += 1

    def is_ready(self):
        return self.ready > 5

    def send_balls(self, nb):
        rospy.loginfo("Waiting launcher service...")
        client = actionlib.SimpleActionClient('/starbaby_launcher', LauncherAction)
        client.wait_for_server()
        goal = LauncherGoal(nb_balls=nb, speed=230)
        rospy.loginfo("Launcher ready")
        client.send_goal(goal)
        client.wait_for_result()
        launcherResult = client.get_result()
        rospy.loginfo("Nb balles envoyees %d", launcherResult.nb_balls)

    def open_arm(self):
        self.servo_pub.publish(90)

    def close_arm(self):
        self.servo_pub.publish(174)

    def move(self, 
            d_x = 0,        # Distance
            v_x_max=0.1,
            d_yaw=0,        # Rotation
            v_yaw_max=1, 
            gap_goal=-1,    # Border tracking
            gap_max_error=0.025,
            sonar_stop=0.08,
            sonar_exit=-1):
        """ Function retour true on success and false on error (sonar_exit or border tracking failure """
        x_start = self.position.x
        y_start = self.position.y
        yaw_goal = self.add_angle(self.yaw, pi*d_yaw/180)

        twist = Twist()
          
        rospy.loginfo("Starting to move")
           
        while (d_x and math.sqrt((self.position.x - x_start)**2 + (self.position.y - y_start)**2) < abs(d_x)) \
            or (d_yaw and abs(self.add_angle(self.yaw, -1 * yaw_goal)) > 2*v_yaw_max*self.SPEED_DT):
           
            # If pure rotation, no sonar detection
            if d_yaw and not d_x:
                sonar_stop = -1

            # Border tracking
            gap_error = gap_goal - self.gap
            if gap_max_error > 0 and gap_goal > 0 and gap_error > gap_max_error:
                rospy.loginfo("Gap too large %f", gap_error)
                self.stop()
                return False

            # Sonar stop
            if sonar_exit > 0 and self.d_sonar < sonar_exit:
                rospy.loginfo("Sonar below exit threshold (%f) %f", sonar_exit, self.d_sonar)
                self.stop()
                return False
            
            # Speed control
            if d_yaw and abs(twist.angular.z) < v_yaw_max:
                twist.angular.z += v_yaw_max/self.SLOPE_IT * d_yaw/abs(d_yaw)
            if gap_goal > 0:
                twist.angular.z = 0.6*twist.angular.z + 0.4*gap_error*10
            
            if d_x and abs(twist.linear.x) < v_x_max:
                twist.linear.x += v_x_max/self.SLOPE_IT * d_x/abs(d_x)
            if sonar_stop > 0 and sonar_stop > self.d_sonar:
                twist.linear.x = 0
                rospy.loginfo("Blocage du au sonar")
            if gap_goal > 0:
                reduce_speed_gap_error = 0.8*abs(gap_error)/gap_max_error
                twist.linear.x *= 1-min(1, abs(gap_error)/0.02)

            self.cmd_pub.publish(twist)
            rospy.loginfo("dx: %.1f cm (%.1f), dyaw: %.1f deg (%.1f), gap: %.1f cm, cmd_vel: %.1f cm/s,%.1f deg/s, sonar: %.1f com", 
                    100*math.sqrt((self.position.x - x_start)**2 + (self.position.y - y_start)**2), 
                    100*d_x,
                    180*abs(self.add_angle(self.yaw, -1 * yaw_goal))/pi,
                    d_yaw,
                    100*gap_error, 
                    100*twist.linear.x, 
                    180*twist.angular.z/pi, 
                    self.d_sonar*100)
            rospy.sleep(self.SPEED_DT)
            
        self.stop()

    def stop(self):
        twist = Twist()
        self.cmd_pub.publish(twist)
	rospy.sleep(self.visual_delay)
        rospy.loginfo("Robot stopped")

    def add_angle(self,a,b):
        return (a + b + pi)%(2*pi) -pi

    def wait_ils_off(self):
        rgap = 0;
        ils_loop_count = 0
        ils_count = 0
        while ils_count < 5:
            rgap += self.gap
            ils_loop_count += 1
            if not self.ils:
                ils_count += 1
            self.eye_pub.publish(Eye(text="-O-", fps=50))
            rospy.sleep(0.1)
        return rgap / ils_loop_count
    
    def wait_ils_on(self):
        ils_count = 0
        while ils_count < 5:
            if self.ils:
                ils_count += 1
            rospy.sleep(0.1)

    def set_side(self, name):
        self.color = name
        self.side_pub.publish(self.color == "orange")

    def change_side(self):
        if self.color == "vert":
            self.set_side("orange")
        else:
            self.set_side("vert")

if __name__=="__main__":
    try:
	robot = Homologation(4)
        
        while not robot.is_ready() :
            rospy.loginfo("Waiting for all subscribers")
            rospy.sleep(0.2)
        rospy.loginfo("All subscribers ready")

        robot.set_side("vert")
        rospy.loginfo("Attente tirette")
        robot.wait_ils_on()
        robot.close_arm() 
        rospy.loginfo("Tirette OK")
        
        side_choosen = False
        while not side_choosen:
            rospy.loginfo("Calibration distance suivi bordure en attendant retrait de la tirette")
            rgap = robot.wait_ils_off()
            rospy.loginfo("Tirette retiree, attente 2 secondes pour changement de camp")  
            rospy.sleep(2)
            if robot.ils:
                rospy.loginfo("Changement sens")
                robot.change_side()
            else:
                side_choosen = True

        if robot.color == "vert":
            
            # On tangeante la tirette
            robot.move(d_x=0.18, v_x_max=0.2, gap_goal = rgap)
                
            # On charge les balles
            robot.move(d_x=0.155, v_x_max=0.08, gap_goal = rgap)
            for i in range(3):
                robot.move(d_x=-0.09, v_x_max=0.08, gap_goal = rgap)
                robot.move(d_x=0.09, v_x_max=0.08, gap_goal = rgap)
            
            # On sort de dessous le recuperateur
            robot.move(d_x=-0.2, v_x_max=0.08, gap_goal = rgap, gap_max_error=0.04)
            robot.move(d_x=0.2, v_x_max=0.08, gap_goal = rgap, gap_max_error=0.04)
            
            # On avance jusqu'a l'abeille et on la pousse
            robot.move(d_x=2, v_x_max=0.19, gap_goal = rgap, gap_max_error=0.04, sonar_exit=0.10)
            robot.open_arm()
            rospy.sleep(1)
            robot.move(d_x=0.12, v_x_max=0.08, sonar_stop=-1)
            rospy.sleep(0.5)
            robot.move(d_x=-0.12, v_x_max=0.08, sonar_stop=-1)
            robot.close_arm()
            rospy.sleep(1)
           
            # On va tirer les balles
            robot.move(d_yaw=140, sonar_stop=-1)
            robot.move(d_x=1, v_x_max=0.18)
            robot.move(d_yaw=30, sonar_stop=-1)
            robot.send_balls(8)
        else:
            robot.move(d_yaw=-90)
            robot.move(d_x=0.37, v_x_max=0.15)
            robot.move(d_yaw=90)
            robot.move(d_x=1.0, v_x_max=0.15)
            robot.move(d_yaw=90)
            robot.move(d_x=2, v_x_max=0.19, sonar_exit=0.10, sonar_stop=-1)
            robot.move(d_x=0.07, v_x_max=0.15, sonar_stop=-1)
            robot.move(d_yaw=-90)
            robot.open_arm()
            rospy.sleep(1)
            robot.move(d_x=0.5, sonar_exit=0.10, sonar_stop=-1)
            robot.move(d_x=0.12, sonar_stop=-1)
            rospy.sleep(0.5)
            robot.move(d_x=-0.15, v_x_max=0.08, sonar_stop=-1)
            robot.close_arm()
            rospy.sleep(0.5)
            robot.move(d_yaw=90)
            robot.move(d_x=0.12, sonar_stop=-1)
            robot.move(d_x=-0.03, sonar_stop=-1)
            robot.move(d_yaw=90)
            robot.move(d_x=2, v_x_max=0.2, gap_goal = rgap)
            # On charge les balles
           # robot.move(d_x=-0.15, v_x_max=0.08, gap_goal = rgap)
           # for i in range(3):
           #     robot.move(d_x=0.09, v_x_max=0.08, gap_goal = rgap)
           #     robot.move(d_x=-0.09, v_x_max=0.08, gap_goal = rgap)
            
            # On sort de dessous le recuperateur
           # robot.move(d_x=-0.2, v_x_max=0.08, gap_goal = rgap, gap_max_error=0.04)
           # robot.move(d_yaw=180, sonar_stop=-1)
            
           # rospy.sleep(1)
           
            # On va tirer les balles
            #robot.move(d_yaw=140, sonar_stop=-1)
            #robot.move(d_x=1, v_x_max=0.18)
            #robot.move(d_yaw=30, sonar_stop=-1)
            #robot.send_balls(8)
            
    except KeyboardInterrupt:
        pass
