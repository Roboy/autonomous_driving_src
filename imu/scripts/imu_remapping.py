#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from sensor_msgs.msg import Imu
from sbg_driver.msg import SbgImuData

def callback(msg):
    try:
        print("Acceleration: ", msg.accel)
    except:
        print("Acceleration: ", msg.linear_acceleration)

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
    rospy.init_node('imu_conversion', anonymous=False)
    rospy.Subscriber('imu_data', SbgImuData, callback)
    pub = rospy.Publisher('imu', Imu, queue_size=10)
    rospy.spin()

    # map from SBG driver to ROS message
    ROS_IMU_msg = mapping(SBG_IMU_msg)

    # Publish converted signal
    IMU_publisher(ROS_IMU_msg, pub)


#def talker():
#    pub = rospy.Publisher('chatter', String, queue_size=10)
    #rospy.init_node('talker', anonymous=True)
#    rate = rospy.Rate(10) # 10hz
#    while not rospy.is_shutdown():
#        hello_str = "hello world %s" % rospy.get_time()
#        rospy.loginfo(hello_str)
#        pub.publish(hello_str)
#        rate.sleep()

if __name__ == '__main__':
    try:
        IMU_Converter()
    except rospy.ROSInterruptException:
        pass







