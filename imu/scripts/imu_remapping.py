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
        rospy.init_node('imu_data_converter', anonymous=False)
    	self.pub = rospy.Publisher('imu', Imu, queue_size=10)
    	rospy.Subscriber('imu_data', SbgImuData, self.callback)
    	rospy.spin()

    def callback(self, msg):
    	ROS_IMU_msg = self.mapping(msg)

    def mapping(self, SBG_IMU_msg):
            ROS_IMU_msg = Imu()
	    ROS_IMU_msg.linear_acceleration = SBG_IMU_msg.accel
	    ROS_IMU_msg.angular_velocity = SBG_IMU_msg.gyro

	    print("SBG Acceleration: ", SBG_IMU_msg.accel)
	    print("ROS Acceleration: ", ROS_IMU_msg.linear_acceleration)
	    print("SBG Gyro: ", SBG_IMU_msg.gyro)
	    print("ROS Gyro: ", ROS_IMU_msg.angular_velocity)

	    self.pub.publish(ROS_IMU_msg)

if __name__ == '__main__':
    try:
	Converter()
    except rospy.ROSInterruptException:
        pass







