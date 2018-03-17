#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

ros::Publisher cmd_vel_teleop_publisher;

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

void joy_handler(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  geometry_msgs::Twist twist;

  double ratio;
  if (joy_msg->buttons[XBOX_BUTTON_LB] && joy_msg->buttons[XBOX_BUTTON_RB]) {
    ratio = 0.75;
  } else if (joy_msg->buttons[XBOX_BUTTON_RB]) {
    ratio = 1.0;
  } else if (joy_msg->buttons[XBOX_BUTTON_LB]) {
    ratio = 0.5;
  } else {
    ratio = 0.25;
  }

  //if (joy_msg->axes[XBOX_AXIS_LT] < 0 && joy_msg->axes[XBOX_AXIS_RT] < 0) {
    twist.linear.x = joy_msg->axes[XBOX_AXIS_UP_DOWN_STICK_LEFT] * ratio;
    twist.angular.z = joy_msg->axes[XBOX_AXIS_LEFT_RIGHT_STICK_RIGHT] * ratio;
  /*} else {
    twist.linear.x = 0;
    twist.angular.z = 0;
  }*/

  cmd_vel_teleop_publisher.publish(twist);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "starbaby_teleop_xbox_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    cmd_vel_teleop_publisher = nh.advertise<geometry_msgs::Twist>("mobile_base_controller/cmd_vel", 10);//cmd_vel_teleop

    ros::Subscriber joy_subscriber = nh.subscribe<sensor_msgs::Joy>("joy", 10, joy_handler);

    ros::spin();
}
