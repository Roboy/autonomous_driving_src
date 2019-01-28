#!/usr/bin/env python
# Software License Agreement (BSD License)

## Simple IMU signal conversion node taking input from sbg_driver
## custom message type and publishing linear acceleration to 
## ROS /imu topic in standard IMU message format

import rospy
from sensor_msgs.msg import Imu
from sbg_driver.msg import SbgImuData

def IMU_Converter():

    rospy.init_node('imu_data_converter', anonymous=False)
    pub = rospy.Publisher('imu', Imu, queue_size=10)

    def mapping(SBG_IMU_msg):
            ROS_IMU_msg = Imu()
	    ROS_IMU_msg.linear_acceleration = SBG_IMU_msg.accel
	    ROS_IMU_msg.angular_velocity = SBG_IMU_msg.gyro

	    print("SBG Acceleration: ", SBG_IMU_msg.accel)
	    print("ROS Acceleration: ", ROS_IMU_msg.linear_acceleration)
	    print("SBG Gyro: ", SBG_IMU_msg.gyro)
	    print("ROS Gyro: ", ROS_IMU_msg.angular_velocity)

	    pub.publish(ROS_IMU_msg)


    def callback(msg):
    	global SBG_IMU_msg
    	SBG_IMU_msg = msg

    	ROS_IMU_msg = mapping(SBG_IMU_msg)


    rospy.Subscriber('imu_data', SbgImuData, callback)

    rospy.spin()




if __name__ == '__main__':
    try:
        IMU_Converter()
    except rospy.ROSInterruptException:
        pass







