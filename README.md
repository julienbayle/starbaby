StarBaby
================================


## Introduction

This code is the main program of the StarBaby robot for Eurobot 2018.
It is based on ROS kinetic and a Raspberry Pi 3

## Configure a development computer

### Install ROS

```bash
sudo apt-get update
sudo apt-get install \
	ros-kinetic-desktop-full \ 
	ros-kinetic-gazebo-plugins \ 
	ros-kinetic-hector-gazebo-plugins \ 
	ros-kinetic-gazebo-ros-control
rosdep init
sudo rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Configure ROS workspace for the project

```bash
git clone https://github.com/julienbayle/starbaby
cd starbaby/StarBabyROS/
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

### Connect an XBOX 360 receiver

Connect your XBOX 360 reciever to your computer.

Turn on your XBOX 360 pad, wait it be available on /dev/input/js0

### Run Gazebo robot simulation

```bash
roslaunch starbaby_gazebo starbaby_gazebo.launch
```

By default, use the pad to control the robot.

To activate the automatic mode, push the "start" button on the pad.

Then send a 2D nav goal via RVIZ to the robot and the navigation stack does the magic.

To take back the control with the pad, push the "back" button 

### Run the code using real ROBOT  

```bash
export ROS_MASTER=http://192.168.0.17:11311
export ROS_IP=192.168.0.10

TODO TODO

```

	
