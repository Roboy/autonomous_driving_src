#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

EPS = 1e-04

class SteeringController:

    def update_steering(self, target_steering):
        # TODO(melkonyan): implement steering control
        pass


class MotorController:
    """
    Binary controller. When given a non-zero target speed, it will turn on the motor.
    Otherwise
    """
    def __init__(self):
        self.publisher = rospy.Publisher('/roboy/control/GPIO', Bool, queue_size=1)

    def update_speed(self, target_speed):
        if target_speed > EPS:
            self.publisher.publish(True)
        else:
            self.publisher.publish(False)


class ControlNode:
    """
    A ROS node that receives cmd_vel commands and converts them to commands for rickshaw.
    """
    def __init__(self):
        self.steering = SteeringController()
        self.motor = MotorController()

    def start(self):
        rospy.init_node('rickshaw_control', log_level=rospy.DEBUG)
        def handle_velocity(twist):
            vx, vy = twist.linear.x, twist.linear.y
            if vy > EPS:
                rospy.logerror('Non-holomonic velocity found vy=%.2f', vy)
            phi = twist.angular.z
            rospy.logdebug('Command(vel=%.2f, steering=%.2f) received', vx, phi)
            self.steering.update_steering(phi)
            self.motor.update_speed(vx)

        rospy.Subscriber('cmd_vel', Twist, handle_velocity, queue_size=1)
        rospy.spin()


if __name__ == '__main__':
    controller = ControlNode()
    controller.start()