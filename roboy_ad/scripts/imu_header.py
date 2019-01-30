#!/usr/bin/env python

## change /imu frame_id to 'imu'

import rospy
import rosbag
from sensor_msgs.msg import Imu
from sbg_driver.msg import SbgImuData

with rosbag.Bag('output.bag', 'w') as outbag:

	for topic, msg, t in rosbag.Bag('/home/christoph/data/2019_01_29/2019-01-29-20-02-07.bag').read_messages():
		#print('bla')
	# This also replaces tf timestamps under the assumption 
	# that all transforms in the message share the same timestamp
			#if topic == "/tf" and msg.transforms:
		if topic == "/imu": #and msg.Imu:
			#outbag.write(topic, msg, msg.transforms[0].header.frame_id)
			msg.header.frame_id = 'imu'
			outbag.write(topic, msg, t)
		else:
			#pass
			outbag.write(topic, msg, t)




