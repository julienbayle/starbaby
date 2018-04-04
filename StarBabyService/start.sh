#!/bin/bash

BASE=$(dirname "$0")
IP=`ip addr | grep 'inet .* wlan0' | awk '{print $2}' | cut -f1 -d'/'`

source $BASE/../StarBabyROS/devel/setup.bash
export ROS_IP=$IP
export ROS_MASTER=http://$IP:11311

echo Starting ROS Starbaby on $IP
roslaunch --pid=/tmp/starbaby.pid starbaby starbaby.launch & 
