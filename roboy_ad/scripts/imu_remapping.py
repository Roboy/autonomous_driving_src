#!/usr/bin/env python
# Software License Agreement (BSD License)

## Simple IMU signal conversion node taking input from sbg_driver
## custom message type and publishing linear acceleration to 
## ROS /imu topic in standard IMU message format

import rospy
from sensor_msgs.msg import Imu
from sbg_driver.msg import SbgImuData

class Converter:
	def __init__(self):

		# set up ROS node
		rospy.init_node('imu_data_converter', anonymous=False)
		self.pub = rospy.Publisher('imu', Imu, queue_size=10)
		rospy.Subscriber('imu_data', SbgImuData, self.mapping)
		rospy.spin()

	def mapping(self, SBG_IMU_msg):

		# remap data from SBG_IMU_msg to ROS_IMU_msg and publish
		ROS_IMU_msg = Imu()
		ROS_IMU_msg.linear_acceleration = SBG_IMU_msg.accel
		ROS_IMU_msg.angular_velocity = SBG_IMU_msg.gyro

		self.pub.publish(ROS_IMU_msg)

if __name__ == '__main__':
	try:
		Converter()
	except rospy.ROSInterruptException:
		pass




