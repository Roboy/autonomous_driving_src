#!/usr/bin/python
"""
A controller that listens to cmd_vel topics and forwards commands to the
tricycle model running in gazebo. Model's definition can be found in
navigation/gazebo/tricycle.urdf
"""
import rospy
from std_msgs.msg import Float64MultiArray, Float64
from geometry_msgs.msg import Twist
import random


EPS = 1e-04

class TricycleControlGazebo:

    def __init__(self):
        pass

    def start(self):
        rospy.init_node('tricycle_control_gazebo', log_level=rospy.DEBUG)
        rear_wheels_pub = rospy.Publisher(
            '/tricycle_rear_wheels_controller/command', Float64MultiArray,
            queue_size=10)
        front_wheel_pub = rospy.Publisher(
            '/tricycle_front_wheel_controller/command', Float64,
            queue_size=10)

        def handle_velocity(twist):
            vx, vy = twist.linear.x, twist.linear.y
            if vy > EPS:
                rospy.logerror('Non-holomonic velocity found vy=%.2f', vy)
            phi = twist.angular.z
            rospy.logdebug('Command(vel=%.2f, steering=%.2f) received', vx, phi)
            front_wheel_pub.publish(phi)
            lin_vel = Float64MultiArray(data=[vx, vx])
            rear_wheels_pub.publish(lin_vel)

        rospy.Subscriber('cmd_vel', Twist, handle_velocity, queue_size=1)
        rospy.spin()

if __name__ == '__main__':
    controller = TricycleControlGazebo()
    controller.start()
