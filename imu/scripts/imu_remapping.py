#!/usr/bin/env python
# Software License Agreement (BSD License)

## Simple IMU signal conversion node taking input from sbg_driver
## custom message type and publishing linear acceleration to 
## ROS /imu topic in standard IMU message format

import rospy
from sensor_msgs.msg import Imu
from sbg_driver.msg import SbgImuData

def callback(msg):
    try:
        print("Acceleration: ", msg.accel)
    except:
        pass
    try:
        print("Acceleration: ", msg.linear_acceleration)
    except:
        pass

    #print("Gyro: ", msg.gyro)
#    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def mapping(SBG_IMU_msg):
    
    ROS_IMU_msg.linear_acceleration = SBG_IMU_msg.accel
    #ROS_IMU_msg.acceleration = SBG_IMU_msg.gyro
    callback(ROS_IMU_msg)

    return ROS_IMU_msg

def IMU_publisher(ROS_IMU_msg, pub):
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo(ROS_IMU_msg)
        pub.publish(ROS_IMU_msg)
        rate.sleep()

def IMU_Converter():

    # init node, subscriber, publisher and keep them running
    rospy.init_node('imu_data_converter', anonymous=False)
    rospy.Subscriber('imu_data', SbgImuData, callback)
    pub = rospy.Publisher('imu', Imu, queue_size=10)
    rospy.spin()

    # map from SBG driver to ROS message
    ROS_IMU_msg = mapping(SBG_IMU_msg)

    # Publish converted signal
    IMU_publisher(ROS_IMU_msg, pub)

if __name__ == '__main__':
    try:
        IMU_Converter()
    except rospy.ROSInterruptException:
        pass







