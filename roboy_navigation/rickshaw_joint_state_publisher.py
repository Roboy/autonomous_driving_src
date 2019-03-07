#!/usr/bin/python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


class RickshawJointStatePublisher:

    def __init__(self):
        rospy.Subscriber("/smooth_angle", Float32, self.angle_callback)
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        rospy.init_node('rickshaw_joint_state_publisher', anonymous=True)

    def send_joint_state_msg(self, angle):
        joint_msg = JointState()
        joint_msg.header.frame_id = ""
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.name = ['joint_front', 'joint_wheel_back', 'joint_wheel_left', 'joint_wheel_right']
        joint_msg.position = [angle, 0.0, 0.0, 0.0]
        joint_msg.velocity = [0.0, 0.0, 0.0, 0.0]
        joint_msg.effort = [0.0, 0.0, 0.0, 0.0]
        self.pub.publish(joint_msg)
        print(joint_msg)

    def angle_callback(self, msg):
        angle = msg.data
        self.send_joint_state_msg(angle)

    def start(self):
        rospy.spin()

if __name__ == '__main__':
    RickshawJointStatePublisher().start()










