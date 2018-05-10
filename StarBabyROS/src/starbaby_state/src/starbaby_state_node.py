#!/usr/bin/env python

import roslib; roslib.load_manifest('starbaby_state')
import rospy
import smach
import smach_ros

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

mode_auto = True 

cmd_vel_publisher = None

def teleop_cmd_vel_callback(data):
    global mode_auto, cmd_vel_publisher
    if (not mode_auto):
        cmd_vel_publisher.publish(data)
        
def auto_cmd_vel_callback(data):
    global mode_auto, cmd_vel_publisher
    if (mode_auto):
        cmd_vel_publisher.publish(data)
        
def manual_mode_auto_handler(ud, msg):
    global mode_auto
    mode_auto = msg.data
    return not msg.data

def auto_mode_auto_handler(ud, msg):
    global mode_auto
    mode_auto = msg.data
    return msg.data

def main():
    rospy.init_node('starbaby_state_node')
    
    # Init publishers
    global cmd_vel_publisher
    cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    # Create subscribers
    rospy.Subscriber("teleop_cmd_vel", Twist, teleop_cmd_vel_callback)
    rospy.Subscriber("auto_cmd_vel", Twist, auto_cmd_vel_callback)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['DONE'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('MANUAL', smach_ros.MonitorState("/starbaby/mode_auto", Bool, manual_mode_auto_handler), 
                               transitions={'invalid':'AUTO', 'valid':'MANUAL', 'preempted':'MANUAL'})
        smach.StateMachine.add('AUTO', smach_ros.MonitorState("/starbaby/mode_auto", Bool, auto_mode_auto_handler), 
                               transitions={'invalid':'MANUAL', 'valid':'AUTO', 'preempted':'AUTO'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/starbaby')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
