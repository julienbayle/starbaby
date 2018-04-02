#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"

ros::Publisher cmd_vel_teleop_publisher;
ros::Publisher mode_auto_publisher;
ros::Publisher side_publisher;

// note on plain values:
// buttons are either 0 or 1
// button axes go from 0 to -1
// stick axes go from 0 to +/-1

#define XBOX_BUTTON_A 0
#define XBOX_BUTTON_B 1
#define XBOX_BUTTON_X 2
#define XBOX_BUTTON_Y 3
#define XBOX_BUTTON_LB 4
#define XBOX_BUTTON_RB 5
#define XBOX_BUTTON_BACK 6
#define XBOX_BUTTON_START 7
#define XBOX_BUTTON_POWER 8
#define XBOX_BUTTON_STICK_LEFT 9
#define XBOX_BUTTON_STICK_RIGHT 10
#define XBOX_BUTTON_CROSS_LEFT 11
#define XBOX_BUTTON_CROSS_RIGHT 12
#define XBOX_BUTTON_CROSS_UP 13
#define XBOX_BUTTON_CROSS_DOWN 14

#define XBOX_AXIS_LEFT_RIGHT_STICK_LEFT 0
#define XBOX_AXIS_UP_DOWN_STICK_LEFT 1
#define XBOX_AXIS_LT 2
#define XBOX_AXIS_LEFT_RIGHT_STICK_RIGHT 3
#define XBOX_AXIS_UP_DOWN_STICK_RIGHT 4
#define XBOX_AXIS_RT 5

double max_linear_speed;
double max_angular_speed;

void joy_handler(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
	geometry_msgs::Twist twist;

	twist.linear.x = joy_msg->axes[XBOX_AXIS_UP_DOWN_STICK_LEFT] * max_linear_speed;
	twist.angular.z = joy_msg->axes[XBOX_AXIS_LEFT_RIGHT_STICK_RIGHT] * max_angular_speed;

	if (joy_msg->buttons[XBOX_BUTTON_START]) {
		std_msgs::Bool bool_msg;
		bool_msg.data = true;
		mode_auto_publisher.publish(bool_msg);
	}
	if (joy_msg->buttons[XBOX_BUTTON_BACK]) {
		std_msgs::Bool bool_msg;
		bool_msg.data = false;
		mode_auto_publisher.publish(bool_msg);
	}

	if (joy_msg->buttons[XBOX_BUTTON_Y]) {
		std_msgs::Bool bool_msg;
		bool_msg.data = true;
		side_publisher.publish(bool_msg);
	}
	if (joy_msg->buttons[XBOX_BUTTON_A]) {
		std_msgs::Bool bool_msg;
		bool_msg.data = false;
		side_publisher.publish(bool_msg);
	}

	cmd_vel_teleop_publisher.publish(twist);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "starbaby_teleop_xbox_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    nh_priv.param("max_linear_speed", max_linear_speed, 0.3);
    nh_priv.param("max_angular_speed", max_angular_speed, 3.0);

    cmd_vel_teleop_publisher = nh.advertise<geometry_msgs::Twist>("teleop_cmd_vel", 10);//cmd_vel_teleop
    mode_auto_publisher = nh.advertise<std_msgs::Bool>("mode_auto", 1, true);
    side_publisher = nh.advertise<std_msgs::Bool>("is_orange", 1, true);

    ros::Subscriber joy_subscriber = nh.subscribe<sensor_msgs::Joy>("joy", 10, joy_handler);

    ros::spin();
}
