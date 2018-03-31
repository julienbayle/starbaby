/*
 * starbaby_gazebo_node.cpp
 *
 *  Created on: 28 mars 2018
 *      Author: Yann BOURRIGAULT
 */

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

ros::Publisher mobile_base_controller_cmd_vel_publisher;
ros::Publisher odom_publisher;

void cmd_vel_handler(const geometry_msgs::TwistConstPtr& msg)
{
	mobile_base_controller_cmd_vel_publisher.publish(msg);
}

void odom_handler(const nav_msgs::OdometryConstPtr& msg)
{
	odom_publisher.publish(msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "starbaby_gazebo_node");

	ros::NodeHandle nh;

	// Create publishers
	mobile_base_controller_cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>("mobile_base_controller/cmd_vel", 1);
	odom_publisher = nh.advertise<nav_msgs::Odometry>("odom", 1);

	// Create subscribers
	ros::Subscriber cmd_vel_subscriber = nh.subscribe("cmd_vel", 1, cmd_vel_handler);
	ros::Subscriber mobile_base_controller_odom_subscriber = nh.subscribe("mobile_base_controller/odom", 1, odom_handler);

	ros::spin();

	return 0;
}
