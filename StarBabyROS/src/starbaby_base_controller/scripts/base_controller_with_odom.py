#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (c) 2018 Julien BAYLE

"""
    simple_base_controller_with_odom
   
    On Twist message received, updates wheel speed goals for the PID controllers
    On left wheel encoder counter message received, updates last wheel counter
    On right wheel encoder counter message received, computes and publishes
    odom, TF and actual wheel speed.

    This code works with the following Arduino board :
    https://github.com/julienbayle/starbaby_arduino_motors

    This robot has only one encoder per wheel. So wheel direction is unknown.
    Wheel speed control is delegated to http://wiki.ros.org/pid node.
    Odometry and wheel speed control is enabled only when the robot is ordered
    to move (nothing is done if the robot is pushed by hand or by another robot)

    Copyright (C) 2018 Julien BAYLE
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   
"""

import rospy
import roslib
roslib.load_manifest('starbaby_base_controller')
from math import sin, cos, pi
from numpy import array

from geometry_msgs.msg import Quaternion, Twist, Vector3, Point, Pose
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler 
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Bool

class simpleBaseControlerWithOdom:

    def __init__(self):
        rospy.init_node("simple_base_controller_with_odom")
        self.node_name = rospy.get_name()
        rospy.loginfo("Simple base controller with odometry  %s started" % self.node_name)
        
        # Rate
        self.rate = rospy.get_param('rate', 5) 

        # Wheel diameter : 78 mm
        self.wheel_radius = float(rospy.get_param('wheel_radius', 0.039))  
        
        # Reduction gear : 1:70
        # 1 motor turn = 4 encoder ticks
        # 1 wheel turn = 70 * 4 ticks = 280 ticks
        # 1 wheel turn = Pi * 78 mm = 0.245 meter
        # tick per meter = 280 / 0.245 = 1143
        self.tick_per_meter = float(rospy.get_param('ticks_per_meter', 1143))  

        # Distance between wheels
        self.base_width = float(rospy.get_param('~base_width', 0.175))

        # Name of the base frame of the robot
        self.base_frame_id = rospy.get_param('~base_frame_id','base_link') 

        # Name of the odometry reference frame
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') 

        # Encoder boundaries
        self.encoder_min = rospy.get_param('encoder_min', -32768)
        self.encoder_max = rospy.get_param('encoder_max', 32767)
       
        # Wheel rotation direction for each wheel 
        # 1 if motor control is enabled and direction is forward
        # 0 if motor control is disabled (no twist). Unable to count as direction is unknown
        # -1 if motor control is enabled and direction is backward
        self.wheel_direction = {"left": 0.0, "right": 0.0}  
        
        # Subscribers
        rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
        self.target_speed = {}    # Target speed for each wheel in m/s

        rospy.Subscriber("/left_wheel/counter", Int16, self.leftWheelCounterCallback)
        rospy.Subscriber("/right_wheel/counter", Int16, self.rightWheelCounterCallback)
        self.prev_counter = {}    # Last encoder counter value for each wheel in ticks
        self.d_wheel = {}         # Last distance travelled by each wheel in meters

        # Publishers
        self.odomBroadcaster = TransformBroadcaster()
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.x = self.y = self.theta = 0

        self.leftGoalSpeedPub = rospy.Publisher("/left_wheel/goal_speed", Float64, queue_size=10)
        self.leftSpeedPub = rospy.Publisher("/left_wheel/speed", Float64, queue_size=10)
        self.leftPIDEnablePub = rospy.Publisher("/left_wheel/pid_enable", Bool, queue_size=10)
        self.leftPWMPub = rospy.Publisher("/left_wheel/pwm", Float64, queue_size=10)

        self.rightGoalSpeedPub = rospy.Publisher("/right_wheel/goal_speed", Float64, queue_size=10)
        self.rightSpeedPub = rospy.Publisher("/right_wheel/speed", Float64, queue_size=10)
        self.rightPIDEnablePub = rospy.Publisher("/right_wheel/pid_enable", Bool, queue_size=10)
        self.rightPWMPub = rospy.Publisher("/right_wheel/pwm", Float64, queue_size=10)
        
    def spin(self):
        # Keep node active
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            r.sleep()
    
    def updateWheelPosition(self, wheel, msg):
        counter = msg.data
        
        # First mesure is used as a starting point
        if wheel not in self.prev_counter :
            self.prev_counter[wheel] = counter

        travel_in_ticks = counter - self.prev_counter[wheel]

        # Counter has been incremented and has exceeded encoder_max
        if self.prev_counter[wheel] > counter :
            travel_in_ticks += self.encoder_max - self.encoder_min

        self.d_wheel[wheel] = self.wheel_direction[wheel] * travel_in_ticks / self.tick_per_meter
        self.prev_counter[wheel] = counter

    def leftWheelCounterCallback(self, msg):
        self.updateWheelPosition("left", msg)

    def rightWheelCounterCallback(self, msg):
        self.updateWheelPosition("right", msg)
        self.computeAndPublishOdomAndWheelsSpeed()

    # Converts twist to a target wheel linear speeds order for PID control
    # Determines wheel rotation direction
    def twistCallback(self, msg):
        # Extracts Twist message data
        v_x = float(msg.linear.x)
        v_theta = float(msg.angular.z)

        # Checks dy is zero
        if(msg.linear.y != 0) :
            rospy.logwarn("Robot is differential drive, dy speed should be zero")
       
        # Some physics :
        # v_x = (v_wheel_left + v_wheel_right) / 2
        # v_theta = (v_wheel_right - v_wheel_left) / distance_between_wheels
        # 
        # Then :
        # v_wheel_right = v_theta * distance_between_wheels + v_wheel_left
        # v_wheel_left = 2 * v_x - v_wheel_right
        # 
        # Conclusion :
        # v_wheel_right = v_theta * distance_between_wheels / 2 + v_x
        # v_wheel_left = - v_theta * distance_between_wheels / 2 + v_x
        self.target_speed["right"] =  v_theta * self.base_width / 2 + v_x
        self.target_speed["left"] = -1.0 * v_theta * self.base_width / 2 + v_x

        self.leftGoalSpeedPub.publish(self.target_speed["left"])
        self.rightGoalSpeedPub.publish(self.target_speed["right"])

        if self.target_speed["left"] > 1e-5:
            self.wheel_direction["left"] = 1.0
        elif self.target_speed["left"] < -1e-5:
            self.wheel_direction["left"] = -1.0
        else:
            self.wheel_direction["left"] = 0.0
            self.leftPWMPub.publish(0)
        self.leftPIDEnablePub.publish(self.wheel_direction["left"] != 0)
        
        if self.target_speed["right"] > 1e-5:
            self.wheel_direction["right"] = 1.0
        elif self.target_speed["right"] < -1e-5:
            self.wheel_direction["right"] = -1.0
        else:
            self.wheel_direction["right"] = 0.0
            self.rightPWMPub.publish(0)
        self.rightPIDEnablePub.publish(self.wheel_direction["right"] != 0)

    def computeAndPublishOdomAndWheelsSpeed(self):
        now = rospy.Time.now()

        try:
            d_t = (now - self.lastOdomTime).to_sec()
                      
            # distance traveled is the average of the two wheels
            d_x = ( self.d_wheel["left"] + self.d_wheel["right"] ) / 2
            
            # spin could be approximated for small angle (result is in radian)
            d_theta = ( self.d_wheel["right"] - self.d_wheel["left"] ) / self.base_width
            
            # calculate velocities in robot base frame
            v_x = d_x / d_t
            v_theta = d_theta / d_t
                      
            # calculate robot position in odom frame
            self.x = self.x + ( cos( self.theta ) * d_x )
            self.y = self.y + ( sin( self.theta ) * d_x )
            self.theta = self.theta + d_theta
            
            # publish the odom and TF
            
            odom_quat = quaternion_from_euler(0, 0, self.theta)
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, self.wheel_radius),
                odom_quat,
                now,
                self.base_frame_id, 
                self.odom_frame_id  
            )
                
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose = Pose(
                    Point(self.x, self.y, self.wheel_radius), 
                    Quaternion(*odom_quat))
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist = Twist(
                    Vector3(v_x, 0, 0), 
                    Vector3(0, 0, v_theta))
            self.odomPub.publish(odom)

            self.leftSpeedPub.publish(self.d_wheel["left"] / d_t)
            self.rightSpeedPub.publish(self.d_wheel["right"] / d_t)
        
        except AttributeError as e:
            pass # First mesure is used as a starting point and ignored
        
        self.lastOdomTime = now
    
if __name__ == '__main__':
    """ main """
    
    try:
        simpleBaseControlerWithOdom = simpleBaseControlerWithOdom()
        simpleBaseControlerWithOdom.spin()
    except rospy.ROSInterruptException:
        pass
