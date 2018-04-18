# Installation

This package procides 3 services :
 * calibrate IMU : Put the robot in front of a wall (50 cm, nothin 1 meter around) and execute this service.
   the robot will turn 10 times and provides IMU calibration data.
   IMU node configuration could be ajusted using these.

 * calibrate Odometry : Put the robot in front of a wall (about 50 cm, nothing 2 meters around) and execute thie service.
   the robot will move for about 1 minute and provides Odometru calibration data.
   Odometry configuration counld be ajusted using these

 * PID test : validate PID value are suitables
