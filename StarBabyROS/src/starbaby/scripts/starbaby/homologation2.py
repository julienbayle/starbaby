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
    
    # Slope for speed control (40 means 0,4 sec at 100Hz)
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
	self.cmd_pub = rospy.Publisher('/starbaby/auto_cmd_vel', Twist, queue_size=10)
	self.servo_pub = rospy.Publisher('/servo/front', UInt8, queue_size=10)
        self.side_pub = rospy.Publisher('/is_orange', Bool, queue_size=10)
        self.eye_pub = rospy.Publisher('/starbaby/eye', Eye, queue_size=10)
        self.mode_pub = rospy.Publisher('/starbaby/mode_auto', Bool, queue_size=10)
        self.mode_pub.publish(True)

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
        # ILS = break
        if self.cancel_requested():
            rospy.loginfo("Skip send ball action as cancel is requested")
            return
        
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
        # ILS = break
        if self.cancel_requested():
            rospy.loginfo("Skip open arm action as cancel is requested")
            return
        
        self.servo_pub.publish(90)

    def close_arm(self):
        # ILS = break
        if self.cancel_requested():
            rospy.loginfo("Skip close arm action as cancel is requested")
            return
        
        self.servo_pub.publish(174)

    def move(self, 
            d_x = 0,        # Distance
            v_x_max=0.15,
            d_yaw=0,        # Rotation
            v_yaw_max=1.2, 
            gap_goal=-1,    # Border tracking
            gap_max_error=0.025,
            sonar_stop=0.08,
            sonar_exit=-1,
            sonar_min_exit=False):
        """ Function retour true on success and false on error (sonar_exit or border tracking failure """
        x_start = self.position.x
        y_start = self.position.y
        yaw_start = self.yaw
        d_yaw_rad = self.principal_angle(math.radians(d_yaw))
        d_sonar_min_mem = d_sonar_min = 1000
        

        twist = Twist()
          
        rospy.loginfo("Starting to move")
           
        while (d_x and math.sqrt((self.position.x - x_start)**2 + (self.position.y - y_start)**2) < abs(d_x)) \
            or (d_yaw and abs(self.principal_angle(self.yaw - yaw_start)) < abs(d_yaw_rad)):
           
            # ILS = break
            if self.cancel_requested():
                rospy.loginfo("Skip move action as cancel is requested")
                self.stop()
                return False

            # If pure rotation, no sonar detection
            if d_yaw and not d_x:
                sonar_stop = -1
            
            # Sonar min detection
            d_sonar_min = 0.5 * d_sonar_min + 0.5*self.d_sonar_side['center']
            if sonar_min_exit and d_sonar_min > d_sonar_min_mem:
                rospy.loginfo("Sonar min trourve")
                self.stop()
                return True
            else:
                d_sonar_min_mem = d_sonar_min

            # Border tracking
            gap_error = gap_goal - self.gap
            if gap_max_error > 0 and gap_goal > 0 and gap_error > gap_max_error:
                rospy.loginfo("Gap too large %f", gap_error)
                self.stop()
                return False

            # Sonar exit
            if sonar_exit > 0 and self.d_sonar < sonar_exit:
                start = rospy.Time.now()
                time_to_wait = rospy.Time.now() - start
                while self.d_sonar < sonar_exit and not self.cancel_requested() and time_to_wait <= rospy.Duration(5):
                    rospy.loginfo("Sonar waiting for exit")
                    rospy.sleep(0.5)
                    time_to_wait = rospy.Time.now()- start
                if time_to_wait > rospy.Duration(5):
                    rospy.loginfo("Sonar below exit threshold (%f) %f", sonar_exit, self.d_sonar)
                    self.stop()
                    return False
            
            # Speed control
            if d_yaw and abs(twist.angular.z) < v_yaw_max:
                twist.angular.z += v_yaw_max/self.SLOPE_IT * d_yaw_rad/abs(d_yaw_rad)
            if gap_goal > 0:
                twist.angular.z = 0.6*twist.angular.z + 0.4*gap_error*10
            
            x_sens = 1
            if d_x:
                x_sens = d_x/abs(d_x)

            if d_x and abs(twist.linear.x) < v_x_max:
                twist.linear.x += v_x_max/self.SLOPE_IT * x_sens
            if sonar_stop > 0 and sonar_stop > self.d_sonar:
                twist.linear.x = 0
                rospy.loginfo("Blocage du au sonar")
            if gap_goal > 0:
                reduce_speed_gap_error = 0.8*abs(gap_error)/gap_max_error
                twist.linear.x *= 1-min(1, abs(gap_error)/0.02)

            self.cmd_pub.publish(twist)
            t = rospy.Time.now() - self.start_time
            rospy.loginfo("%d.%d - x: %.1f cm (%.1f), yaw: %.1f deg (%.1f), gap: %.1f cm, cmd_vel: %.1f cm/s,%.1f deg/s, sonar: %.1f cm (%.1f)", 
                    t.secs,
                    t.nsecs,
                    100*math.sqrt((self.position.x - x_start)**2 + (self.position.y - y_start)**2)*x_sens, 
                    100*d_x,
                    math.degrees(self.principal_angle(self.yaw - yaw_start)),
                    d_yaw,
                    100*gap_error, 
                    100*twist.linear.x, 
                    180*twist.angular.z/pi, 
                    self.d_sonar*100,
                    d_sonar_min*100)
            rospy.sleep(self.SPEED_DT)
            
        self.stop()

    def stop(self):
        twist = Twist()
        self.cmd_pub.publish(twist)
        rospy.sleep(0.2) # Minimum delay for PID to purge (disable when speed is nul)
	rospy.sleep(self.visual_delay)
        rospy.loginfo("Robot stopped")

    def principal_angle(self, a):
        return (a + pi)%(2*pi) - pi

    def show_text(self, text, fps=50, repeat=1):
        self.eye_pub.publish(Eye(text=text, fps=fps, repeat=repeat))
        
    def wait_ils_off(self):
        rgap = 0;
        ils_loop_count = 0.0001 # Div/0 is cancel is requested previously
        ils_count = 0
        while not rospy.is_shutdown() and ils_count < 5:
            rgap += self.gap
            ils_loop_count += 1
            if not self.ils:
                ils_count += 1
            self.show_text("-O-")
            rospy.sleep(0.1)
        return rgap / ils_loop_count
    
    def wait_ils_on(self):
        ils_count = 0
        while not rospy.is_shutdown() and ils_count < 5:
            if self.ils:
                ils_count += 1
            rospy.sleep(0.1)

    def set_side(self, name):
        self.color = name
        self.side_pub.publish(self.color == "orange")

    def start(self):
        self.start_time = rospy.Time.now()

    def cancel_requested(self):
        try:
            ellapsed_time = rospy.Time.now() - self.start_time
        except AttributeError:
            return self.ils or rospy.is_shutdown()
        else:
            if ellapsed_time.secs > 95:
                rospy.loginfo("Match time out")
            return ellapsed_time.secs > 95 or self.ils or rospy.is_shutdown()

    def change_side(self):
        if self.color == "vert":
            self.set_side("orange")
        else:
            self.set_side("vert")

if __name__=="__main__":
    try:
	robot = Homologation(0)
        
        while not rospy.is_shutdown():
            while not robot.is_ready() and not rospy.is_shutdown() :
                rospy.loginfo("Waiting for all subscribers")
                rospy.sleep(0.2)
            rospy.loginfo("All subscribers ready")

            robot.set_side("vert")
            rospy.loginfo("Attente tirette")
            robot.wait_ils_on()
            robot.close_arm() 
            rospy.loginfo("Tirette OK")
            
            side_choosen = False
            rgap = 0.05
            while not side_choosen and not rospy.is_shutdown():
                rospy.loginfo("Calibration distance suivi bordure en attendant retrait de la tirette")
                rgap = robot.wait_ils_off()
                rospy.loginfo("Tirette retiree, attente 2 secondes pour changement de camp")  
                rospy.sleep(2)
                if robot.ils:
                    rospy.loginfo("Changement sens")
                    robot.change_side()
                else:
                    side_choosen = True
            
            score = 5
            slow_speed = 0.08
            robot.start()

            if robot.color == "vert":
                
                # On tangeante la tirette
                robot.move(d_x=0.18, gap_goal = rgap)
                    
                # On charge les balles
                robot.move(d_x=0.155, v_x_max=slow_speed, gap_goal = rgap)
                for i in range(1):
                    robot.move(d_x=-0.09, v_x_max=slow_speed, gap_goal = rgap)
                    robot.move(d_x=0.09,  v_x_max=slow_speed, gap_goal = rgap)
                
                score += 10

                # On sort de dessous le recuperateur
                robot.move(d_x=0.12, v_x_max=slow_speed, sonar_stop=-1)
                robot.move(d_yaw=-90, sonar_min_exit=True)
                robot.move(d_yaw=90)

                # On avance jusqu'a l'abeille et on la pousse
                if robot.move(d_x=2, gap_goal = rgap, gap_max_error=0.04, sonar_exit=0.10):
                    robot.move(d_x=-0.1 )
                    robot.open_arm()
                    robot.move(d_x=0.1)
                    robot.move(d_x=0.10, sonar_stop=-1)
                    rospy.sleep(0.3)
                    robot.move(d_x=-0.10, sonar_stop=-1)
                    robot.close_arm()
                    rospy.sleep(0.3)
                    score += 50

                    # On va tirer les balles
                    robot.move(d_yaw=140)
                    robot.move(d_x=1 )
                    robot.move(d_yaw=30)
                    robot.send_balls(8)
                    score += 20

            else:
                # On va lancer l'abeille en evitant les cubes
                robot.move(d_yaw=-90)
                robot.move(d_x=0.34)
                robot.move(d_yaw=90)
                robot.move(d_x=1.0)
                robot.move(d_yaw=90)
                robot.move(d_x=1, sonar_exit=0.10, sonar_stop=-1)
                robot.move(d_x=0.15, sonar_stop=-1)
                robot.move(d_x=-0.03, v_x_max=slow_speed, sonar_stop=-1)
                robot.move(d_yaw=-75)

                robot.move(d_x=-0.1)
                robot.open_arm()
                rospy.sleep(0.5)
                robot.move(d_x=0.6, sonar_exit=0.16, sonar_stop=-1)
                robot.move(d_x=0.18, sonar_stop=-1)
                robot.move(d_x=-0.15, sonar_stop=-1)
                robot.close_arm()
                rospy.sleep(0.2)

                score += 50
                
                robot.move(d_yaw=179)
                #robot.move(d_x=0.12, sonar_stop=-1)
                #robot.move(d_x=-0.03, sonar_stop=-1)
                #robot.move(d_yaw=80)
                #robot.move(d_x=0.12, sonar_stop=-1)

                # robot.move(d_x=2, v_x_max=0.2, gap_goal = rgap)
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
            
            while not robot.ils and not rospy.is_shutdown():
                robot.show_text("%d pts" % (score), 20, 1)
                rospy.sleep(1)
            
    except KeyboardInterrupt:
        pass
