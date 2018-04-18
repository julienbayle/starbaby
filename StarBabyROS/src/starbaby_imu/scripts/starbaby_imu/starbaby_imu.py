#!/usr/bin/env python

import rospy
import math
from itg3200.ITG3200 import ITG3200
from sensor_msgs.msg import Imu

class imuPublisher:

    def __init__(self, rate, biais_gz, to_degree_per_second_gz):
        rospy.init_node("starbaby_imu")
        self.node_name = rospy.get_name()
        rospy.loginfo("IMU publisher  %s started" % self.node_name)

	self.sensor = ITG3200() 
        self.imu_pub = rospy.Publisher('imu', Imu, queue_size=10)
        self.rate = rate
        self.biais_gz = biais_gz
        self.to_degree_per_second_gz = to_degree_per_second_gz

    def publishImu(self, data):
        gx, gy, gz = data
        now = rospy.Time.now()

        imu = Imu(header=rospy.Header(frame_id="imu_link"))
        imu.header.stamp = now
        imu.orientation.x = 0
        imu.orientation.y = 0
        imu.orientation.z = 0
        imu.orientation.w = 0
        imu.linear_acceleration.x = 0
        imu.linear_acceleration.y = 0
        imu.linear_acceleration.z = 0
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz * self.to_degree_per_second_gz - self.biais_gz

        self.imu_pub.publish(imu)

    def run(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            data = self.sensor.read_data()
            self.publishImu(data)
            r.sleep()


if __name__ == '__main__':
    rate = rospy.get_param("rate", 50)
    biais_gz = rospy.get_param("biais_gz", 0.0)
    to_degree_per_second_gz = rospy.get_param("to_degree_per_second_gz", 1/14.0)
    imu_publisher = imuPublisher(rate, biais_gz, to_degree_per_second_gz)
    imu_publisher.run()
