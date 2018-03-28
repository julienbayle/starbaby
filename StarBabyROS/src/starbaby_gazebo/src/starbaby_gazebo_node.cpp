/*
 * starbaby_gazebo_node.cpp
 *
 *  Created on: 28 mars 2018
 *      Author: Yann BOURRIGAULT
 */

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

ros::Publisher mobile_base_controller_cmd_vel_publisher;

void cmd_vel_handler(const geometry_msgs::TwistConstPtr& msg)
{
	mobile_base_controller_cmd_vel_publisher.publish(msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "starbaby_gazebo_node");

	ros::NodeHandle nh;

	mobile_base_controller_cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>("mobile_base_controller/cmd_vel", 1);

	ros::Subscriber cmd_vel_subscriber = nh.subscribe("cmd_vel", 1, cmd_vel_handler);

	ros::spin();

	return 0;
}
