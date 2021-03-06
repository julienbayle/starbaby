#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <starbaby_calibrate/CalibrateAction.h>
#include <starbaby_launcher/LauncherAction.h>

ros::Publisher cmd_vel_teleop_publisher;
ros::Publisher mode_auto_publisher;
ros::Publisher side_publisher;
ros::Publisher laser_active_publisher;
ros::Publisher launcher_publisher;

actionlib::SimpleActionClient<starbaby_calibrate::CalibrateAction> *ac;
actionlib::SimpleActionClient<starbaby_launcher::LauncherAction> *ac_launcher;

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
bool   calibrating = false;
bool  last_mode = true;
int   button_b_anti_repeat = 0;
int   button_a_anti_repeat = 0;
int   button_lb_anti_repeat = 0;
int   launcher_last_speed = 230;
bool   lidar_active = true;
float  axis_rt_memory = 0;

void joy_handler(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
	geometry_msgs::Twist twist;
	if(joy_msg->buttons[XBOX_BUTTON_RB]) {
		twist.linear.x = joy_msg->axes[XBOX_AXIS_UP_DOWN_STICK_LEFT] * max_linear_speed;
		twist.angular.z = joy_msg->axes[XBOX_AXIS_LEFT_RIGHT_STICK_RIGHT] * max_angular_speed;
	}
	else {
		twist.linear.x = joy_msg->axes[XBOX_AXIS_UP_DOWN_STICK_LEFT] * max_linear_speed / 2;
		twist.angular.z = joy_msg->axes[XBOX_AXIS_LEFT_RIGHT_STICK_RIGHT] * max_angular_speed / 2;
	}
	cmd_vel_teleop_publisher.publish(twist);

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
		bool_msg.data = !last_mode;
		last_mode = !last_mode;
		side_publisher.publish(bool_msg);
	}
	if (joy_msg->buttons[XBOX_BUTTON_A] && !button_a_anti_repeat ) {
		button_a_anti_repeat = 10;
		starbaby_launcher::LauncherGoal goal;
		goal.nb_balls = 1;
		goal.speed = 0;
		ac_launcher->sendGoal(goal);
	}
        
	if(!joy_msg->buttons[XBOX_BUTTON_A] && button_a_anti_repeat > 0) {
		button_a_anti_repeat--;
	}


	if (joy_msg->buttons[XBOX_BUTTON_B] && !joy_msg->buttons[XBOX_BUTTON_X] && !button_b_anti_repeat) {
		button_b_anti_repeat = 10;
		starbaby_launcher::LauncherGoal goal;
		goal.nb_balls = 0;
		goal.speed = launcher_last_speed;
		if (launcher_last_speed == 250) { launcher_last_speed = 0; }
		else if (launcher_last_speed == 240) { launcher_last_speed = 250; }
		else if (launcher_last_speed == 230) { launcher_last_speed = 240; }
		else if (launcher_last_speed == 0) { launcher_last_speed = 230; }
			
		ac_launcher->sendGoal(goal);
	}

        if(!joy_msg->buttons[XBOX_BUTTON_B] && button_b_anti_repeat > 0) {
		button_b_anti_repeat--;
	}

	if (joy_msg->buttons[XBOX_BUTTON_X] && joy_msg->buttons[XBOX_BUTTON_B] && !calibrating) {
		calibrating = true;
		starbaby_calibrate::CalibrateGoal goal;
		ac->sendGoal(goal);
	}
	
	if (joy_msg->buttons[XBOX_BUTTON_X] && joy_msg->buttons[XBOX_BUTTON_Y] && !calibrating) {
		calibrating = true;
		ac->cancelAllGoals();
	}

	if (!joy_msg->buttons[XBOX_BUTTON_X]) {
		calibrating = false;
	}
	
	if (joy_msg->buttons[XBOX_BUTTON_LB] && !button_lb_anti_repeat) {
                button_lb_anti_repeat = 10;
		lidar_active = !lidar_active;
		std_msgs::Bool bool_msg;
                bool_msg.data = lidar_active;
                laser_active_publisher.publish(bool_msg);
        }

	if (!joy_msg->buttons[XBOX_BUTTON_LB] && button_lb_anti_repeat > 0) {
		button_lb_anti_repeat--;
	}

        if(joy_msg->axes[XBOX_AXIS_RT] != axis_rt_memory) {
		std_msgs::Float64 pwm;
        	pwm.data = -255*joy_msg->axes[XBOX_AXIS_RT];
        	if(pwm.data < 0) {
			pwm.data = 0;
		}
        	launcher_publisher.publish(pwm);
		axis_rt_memory = joy_msg->axes[XBOX_AXIS_RT];
	}
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "starbaby_teleop_xbox_node");
    ROS_INFO("Starting starbaby_teleop_xbox_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    nh_priv.param("max_linear_speed", max_linear_speed, 0.19);
    nh_priv.param("max_angular_speed", max_angular_speed, 1.8);
    
    cmd_vel_teleop_publisher = nh.advertise<geometry_msgs::Twist>("teleop_cmd_vel", 10);//cmd_vel_teleop
    mode_auto_publisher = nh.advertise<std_msgs::Bool>("mode_auto", 1, true);
    side_publisher = nh.advertise<std_msgs::Bool>("is_orange", 1, true);
    launcher_publisher = nh.advertise<std_msgs::Float64>("/launcher/pwm", 1); // launcher PWM 
    ros::Subscriber joy_subscriber = nh.subscribe<sensor_msgs::Joy>("joy", 10, joy_handler);
    
    ac = new actionlib::SimpleActionClient<starbaby_calibrate::CalibrateAction>(nh, "/starbaby_calibrate", true);
    ROS_INFO("Waiting for calibration action server to start.");
    ac->waitForServer(); //will wait for infinite time
    ROS_INFO("Calibration action server started");
    
    ac_launcher = new actionlib::SimpleActionClient<starbaby_launcher::LauncherAction>(nh, "/starbaby_launcher", true);
    ROS_INFO("Waiting for launcher action server to start.");
    ac_launcher->waitForServer(); //will wait for infinite time
    ROS_INFO("Launcher action server started");

    laser_active_publisher = nh.advertise<std_msgs::Bool>("laser_active", 1, true);

    ros::spin();
}
