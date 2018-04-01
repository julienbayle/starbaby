StarBaby
================================


## Introduction

This code is the main program of the StarBaby robot for Eurobot 2018.
It is based on ROS kinetic and a Raspberry Pi 3

## Installation on Raspberry Pi 3

Ubuntu 16.04 has been selected because this Linux distribution is compatible
With raspberry Pi 3 and ROS ARM packages are available.

On raspbian (previously used), rosdep does not find all packages. 
So a lot of packages must be installed and compiled by hand. 
It's possible but painful.

Note : This installation is a very long process (About two to three hours) 

### Ubuntu 16.04

Download ubuntu 16.04 image from https://wiki.ubuntu.com/ARM/RaspberryPi
Then, write the image into an SD CARD (Here from Mac OS) :

```bash
brew install xz
diskutil umountDisk /dev/disk2
xzcat ubuntu-16.04-preinstalled-server-armhf+raspi3.img.xz | sudo dd bs=4m of=/dev/rdisk2
```

Then put the SD card into the Raspberry and turn it on.
Connect an ethernet cable to the Raspberry.
Find your raspberry IP on your network using nmap or other method.
Then connect to it via ssh (default user/password is ubuntu/ubuntu)
Define a new new password on first connexion then connect again.

Do not upgrade the Raspberry ! 
Never use "upgrade" or "dist-upgrade" as it will corrupt your system.
(this is because Raspberry Pi3 image is not supported by canonical yet)

### Network and WIFI connexion

First, configure hostname. 
Without this, roscore will not start as ubuntu (defaut hostname) is unknown.
In /etc/hosts, replace :

```
127.0.0.1 localhost
```

By :

```
127.0.0.1 localhost ubuntu
```

Next, add WIFI support.
WPAsupplicant allows to use multiple WIFI networks.

```bash
sudo su
apt-get update
apt-get install -y wpasupplicant
echo "auto lo" > /etc/network/interfaces
echo "iface lo inet loopback" >> /etc/network/interfaces
echo "auto wlan0" >> /etc/network/interfaces
echo "allow-hotplug wlan0" >> /etc/network/interfaces
echo "iface wlan0 inet dhcp" >> /etc/network/interfaces
echo "wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf" >> /etc/network/interfaces
echo "auto eth0" >> /etc/network/interfaces
echo "iface eth0 inet dhcp" >> /etc/network/interfaces
```

Create a new file /etc/wpa_supplicant/wpa_supplicant.conf

```
country=FR
update_config=1

network={
        id_str="A"
	ssid="WIFI_A_ID"
        psk="WIFI_A_PASS"
}

network={
        id_str="B"
	ssid="WIFI_B_ID"
        psk="WIFI_B_PASS"
}
```

Then start the WIFI interface :

```bash
sudo ifup wlan0
sudo halt
```

Unplug the raspberry from ethernet.
Wait for about 10 seconds then start the device.
Wait about 1 or 2 minutes.
Check that you can connect by WIFI to your raspberry.

### ROS installation

Source : http://wiki.ros.org/Installation/UbuntuARM

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install -y ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### StarBaby ROS packages

```bash
git clone https://github.com/julienbayle/starbaby
cd starbaby/StarBabyROS/src
git clone https://github.com/julienbayle/xv_11_laser_motor_control.git
git clone https://github.com/julienbayle/xv_11_laser_driver.git
rosdep install --from-paths src --ignore-src --rosdistro kinetic -r -y
catkin_make
source devel/setup.bash
```


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

	
